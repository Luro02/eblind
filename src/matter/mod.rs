mod window_covering_handler;
pub use window_covering_handler::*;
mod position;
pub use position::*;

use core::ffi::CStr;
use core::pin::pin;
use core::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

use esp_idf_matter::matter::dm::clusters::desc::{self, ClusterHandler as _, DescHandler};
use esp_idf_matter::matter::dm::devices::test::{TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET};
use esp_idf_matter::matter::dm::{
    Async, Dataver, DeviceType, EmptyHandler, Endpoint, EpClMatcher, Node,
};
use esp_idf_matter::matter::utils::init::InitMaybeUninit;
use esp_idf_matter::wireless::{EspMatterWifi, EspWifiMatterStack};
use rs_matter::{clusters, devices};

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::modem::Modem;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, EspNvsPartition, NvsPartitionId};
use esp_idf_svc::timer::EspTaskTimerService;

use crate::environment::Environment;
use crate::executor::Executor;
use crate::matter::window_covering::ClusterAsyncHandler as _;
use crate::movement::StepperController;
use crate::storage::Storage;
use crate::utils::yield_for;
use crate::EmbassyMutex;

use static_cell::StaticCell;

// TODO: are these sizes necessary?
const MATTER_STACK_SIZE: usize = 80 * 1024; // 40 KiB
const BUMP_SIZE: usize = 26_000;

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[embassy_executor::task]
async fn task(
    is_ready: Arc<AtomicBool>,
    controller: Arc<EmbassyMutex<StepperController<'static>>>,
    modem: Modem<'static>,
    sysloop: EspSystemEventLoop,
    timer: EspTaskTimerService,
    nvs: EspDefaultNvsPartition,
    storage: Arc<Storage>,
) {
    /*// Wait until the system is calibrated before starting the matter task:
    while !is_ready.load(Ordering::Relaxed) {
        yield_for(Duration::from_millis(50)).await;
    }*/

    if let Err(error) = matter(controller, modem, sysloop, timer, nvs, storage).await {
        log::error!("[Matter Task] Error: {:?}", error);
    }
}

pub fn spawn_matter_task(
    is_ready: Arc<AtomicBool>,
    controller: Arc<EmbassyMutex<StepperController<'static>>>,
    mut modem: Modem<'static>,
    sysloop: EspSystemEventLoop,
    timer: EspTaskTimerService,
    nvs: EspDefaultNvsPartition,
    storage: Arc<Storage>,
) -> anyhow::Result<std::thread::JoinHandle<()>> {
    // TODO: Why is this needed?
    ThreadSpawnConfiguration::set(&ThreadSpawnConfiguration {
        name: Some(CStr::from_bytes_until_nul(b"matter\0")?),
        ..Default::default()
    })?;

    // TODO: is it still necessary to spawn a dedicated thread for Matter?
    let handle = std::thread::Builder::new()
        .stack_size(MATTER_STACK_SIZE)
        .spawn(|| {
            /*esp_idf_svc::bt::reduce_bt_memory(unsafe { modem.reborrow() }).unwrap();

            log::info!("Starting Matter task...");
            let executor = EXECUTOR.init(Executor::new());
            executor.run(|spawner| {
                spawner.must_spawn(task(
                    is_ready, controller, modem, sysloop, timer, nvs, storage,
                ))
            });*/
        })?;

    Ok(handle)
}

fn clear_matter_nvs<T: NvsPartitionId>(nvs: EspNvsPartition<T>) -> anyhow::Result<EspNvs<T>> {
    const NAMESPACE: &str = "esp-idf-matter";
    let esp_nvs = EspNvs::new(nvs, NAMESPACE, true)?;

    esp_nvs.erase_all()?;

    Ok(esp_nvs)
}

async fn matter(
    controller: Arc<EmbassyMutex<StepperController<'static>>>,
    modem: Modem<'_>,
    sysloop: EspSystemEventLoop,
    timer: EspTaskTimerService,
    nvs: EspDefaultNvsPartition,
    storage: Arc<Storage>,
) -> anyhow::Result<()> {
    log::info!("Starting Matter example...");

    // Initialize the Matter stack (can be done only once),
    // as we'll run it in this thread
    let stack = MATTER_STACK
        .uninit()
        .init_with(EspWifiMatterStack::init_default(
            &TEST_DEV_DET,
            TEST_DEV_COMM,
            &TEST_DEV_ATT,
        ));

    log::info!("Matter stack initialized");

    let window_covering_handler = WindowCoveringHandler::new(
        Dataver::new_rand(stack.matter().rand()),
        controller,
        storage,
    )?
    .adapt();

    // Chain our endpoint clusters with the
    // (root) Endpoint 0 system clusters in the final handler
    let handler = EmptyHandler
        // The window covering handler is on endpoint 1
        .chain(
            EpClMatcher::new(
                Some(WINDOW_COVERING_ENDPOINT_ID),
                Some(WindowCoveringHandler::CLUSTER.id),
            ),
            &window_covering_handler,
        )
        // Each Endpoint needs a Descriptor cluster too
        // Just use the one that `rs-matter` provides out of the box
        .chain(
            EpClMatcher::new(
                Some(WINDOW_COVERING_ENDPOINT_ID),
                Some(DescHandler::CLUSTER.id),
            ),
            Async(desc::DescHandler::new(Dataver::new_rand(stack.matter().rand())).adapt()),
        );

    // Run the Matter stack with our handler
    // Using `pin!` is completely optional, but saves some memory due to `rustc`
    // not being very intelligent w.r.t. stack usage in async functions
    //
    // NOTE: When testing initially, use the `DummyKVBlobStore` to make sure device
    // commissioning works fine with your controller. Once you confirm, you can enable
    // the `EspKvBlobStore` to persist the Matter state in NVS.
    if Environment::should_clear_stored_data() {
        clear_matter_nvs(nvs.clone())?;
    }

    let store = stack
        .create_persist_with_comm_window(esp_idf_matter::persist::EspKvBlobStore::new_default(
            nvs.clone(),
        )?)
        .await
        .map_err(|err| anyhow::anyhow!("matter error: {}", err))?;

    //let store = stack.create_shared_store(esp_idf_matter::stack::persist::DummyKvBlobStore);

    let wifi = EspMatterWifi::new_with_builtin_mdns(modem, sysloop, timer, nvs, stack);
    // TODO: Implement printing pairing QR code?/save it for being able to print it later?

    // let mut buffer = vec![0u8; 4096];
    //
    //let commission_data = stack.matter().dev_comm().compute_pairing_code();

    // Print the pairing code and QR code to the console
    /*rs_matter::pairing::print_pairing_code_and_qr(
        stack.matter().dev_det(),
        stack.matter().dev_comm(),
        DiscoveryCapabilities::BLE | DiscoveryCapabilities::IP,
        &mut buffer,
    )?;*/

    // TODO: Is run_coex really the issue?
    pin!(stack.run(
        // The Matter stack needs the Wifi/BLE modem peripheral
        wifi,
        // The Matter stack needs a persister to store its state
        &store,
        // Our `AsyncHandler` + `AsyncMetadata` impl
        (NODE, handler),
        // No user future to run
        (),
    ))
    .await
    .map_err(|err| anyhow::anyhow!("matter error: {}", err))?;

    Ok(())
}

/// The Matter stack is allocated statically to avoid
/// program stack blowups.
/// It is also a mandatory requirement when the `WifiBle` stack variation is used.
static MATTER_STACK: StaticCell<EspWifiMatterStack<BUMP_SIZE, ()>> = StaticCell::new();

/// Endpoint 0 (the root endpoint) always runs
/// the hidden Matter system clusters, so we pick ID=1
const WINDOW_COVERING_ENDPOINT_ID: u16 = 1;

const DEV_TYPE_WINDOW_COVERING: DeviceType = DeviceType {
    dtype: 0x0202,
    drev: 2, // TODO: maybe should be 3?
};

/// The Matter Window Covering device Node
const NODE: Node = Node {
    id: 0,
    endpoints: &[
        EspWifiMatterStack::<0, ()>::root_endpoint(),
        Endpoint {
            id: WINDOW_COVERING_ENDPOINT_ID,
            device_types: devices!(DEV_TYPE_WINDOW_COVERING),
            clusters: clusters!(DescHandler::CLUSTER, WindowCoveringHandler::CLUSTER),
        },
    ],
};
