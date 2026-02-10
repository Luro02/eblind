mod position;
pub use position::*;

mod window_covering_handler;
pub use window_covering_handler::*;

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::pin::pin;
use core::sync::atomic::{AtomicBool, Ordering};

use esp_backtrace as _;
use esp_hal::peripherals::{BT, WIFI};

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Duration;
use log::info;

use rs_matter_embassy::epoch::epoch;
use rs_matter_embassy::matter::BasicCommData;
use rs_matter_embassy::matter::dm::clusters::basic_info::BasicInfoConfig;
use rs_matter_embassy::matter::dm::clusters::desc::{ClusterHandler as _, DescHandler};
use rs_matter_embassy::matter::dm::devices::test::TEST_DEV_ATT;
use rs_matter_embassy::matter::dm::{
    Async, Dataver, DeviceType, EmptyHandler, Endpoint, EpClMatcher, Node,
};
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::{clusters, devices};
use rs_matter_embassy::stack::rand::reseeding_csprng;
use rs_matter_embassy::wireless::esp::EspWifiDriver;
use rs_matter_embassy::wireless::{EmbassyWifi, EmbassyWifiMatterStack};
use rs_matter_embassy::matter::crypto::{Crypto, default_crypto};
use rs_matter_embassy::matter::dm::devices::test::DAC_PRIVKEY;

use crate::EmbassyMutex;
use crate::environment::Environment;
use crate::matter::window_covering::ClusterAsyncHandler as _;
use crate::movement::StepperController;
use crate::storage::{Storage, StoragePartition};
use crate::yield_for;

pub type MatterError = rs_matter_embassy::matter::error::Error;
pub type MatterErrorCode = rs_matter_embassy::matter::error::ErrorCode;

macro_rules! mk_static {
    ($t:ty) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        STATIC_CELL.uninit()
    }};
}

/// The amount of memory for allocating all `rs-matter-stack` futures created during
/// the execution of the `run*` methods.
/// This does NOT include the rest of the Matter stack.
///
/// The futures of `rs-matter-stack` created during the execution of the `run*` methods
/// are allocated in a special way using a small bump allocator which results
/// in a much lower memory usage by those.
///
/// If - for your platform - this size is not enough, increase it until
/// the program runs without panics during the stack initialization.
//const BUMP_SIZE: usize = 16_500;
const BUMP_SIZE: usize = 32_000;

pub fn spawn_matter_task(
    spawner: Spawner,
    is_ready: Arc<AtomicBool>,
    wifi: WIFI<'static>,
    bt: BT<'static>,
    controller: Arc<EmbassyMutex<StepperController<'static>>>,
    storage: Storage<'static>,
) -> anyhow::Result<()> {
    log::info!("Starting Matter task...");
    spawner.must_spawn(task(is_ready, wifi, bt, controller, storage));

    Ok(())
}

#[embassy_executor::task]
async fn task(
    is_ready: Arc<AtomicBool>,
    wifi: WIFI<'static>,
    bt: BT<'static>,
    controller: Arc<EmbassyMutex<StepperController<'static>>>,
    storage: Storage<'static>,
) {
    // Wait until the system is calibrated before starting the matter task:
    while !is_ready.load(Ordering::Relaxed) {
        yield_for(Duration::from_millis(50)).await;
    }

    if let Err(error) = matter(wifi, bt, controller, storage).await {
        log::error!("[Matter Task] Error: {error:?}");
    }
}

const KEY_OFFSET: u16 = 0x50;

impl<'d> rs_matter_embassy::stack::persist::KvBlobStore for Storage<'d> {
    /// Load a BLOB with the specified key from the storage.
    ///
    /// # Arguments
    /// - `key` - the key of the BLOB
    /// - `buf` - a buffer that the `KvBlobStore` implementation might use for its own purposes
    /// - `cb` - a callback that will be called with the loaded data is available
    ///   or with `None` if the BLOB does not exist.
    async fn load<F>(&mut self, key: u16, _buf: &mut [u8], cb: F) -> Result<(), MatterError>
    where
        F: FnOnce(Option<&[u8]>) -> Result<(), MatterError>,
    {
        let key = key + KEY_OFFSET;

        let buffer = Storage::load::<Vec<u8>>(self, key).await.map_err(|err| {
            log::error!("MatterStorage load error: {err:?}");
            MatterError::new(MatterErrorCode::StdIoError)
        })?;

        if buffer.is_none() {
            log::warn!("storage.load({key}): unknown key");
        }

        cb(buffer.as_deref())
    }

    /// Store a BLOB with the specified key in the storage.
    ///
    /// # Arguments
    /// - `key` - the key of the BLOB
    /// - `buf` - a buffer that the `KvBlobStore` implementation might use for its own purposes
    /// - `cb` - a callback that will be called with a buffer that the implementation
    ///   should fill with the data to be stored.
    async fn store<F>(&mut self, key: u16, buf: &mut [u8], cb: F) -> Result<(), MatterError>
    where
        F: FnOnce(&mut [u8]) -> Result<usize, MatterError>,
    {
        let key = key + KEY_OFFSET;
        let len = cb(buf)?;

        log::info!("storage.store({key}, {len} bytes)");
        Storage::store(self, key, &buf[..len])
            .await
            .map_err(|err| {
                log::error!("MatterStorage load error: {err:?}");
                MatterError::new(MatterErrorCode::StdIoError)
            })?;

        Ok(())
    }

    /// Remove a BLOB with the specified key from the storage.
    ///
    /// # Arguments
    /// - `key` - the key of the BLOB
    /// - `buf` - a buffer that the `KvBlobStore` implementation might use for its own purposes
    async fn remove(&mut self, key: u16, _buf: &mut [u8]) -> Result<(), MatterError> {
        let key = key + KEY_OFFSET;

        log::info!("storage.remove({key})");
        self.delete(key).await.map_err(|err| {
            log::error!("MatterStorage load error: {err:?}");
            MatterError::new(MatterErrorCode::StdIoError)
        })
    }
}

async fn matter(
    wifi: WIFI<'_>,
    bt: BT<'_>,
    controller: Arc<EmbassyMutex<StepperController<'static>>>,
    storage: Storage<'static>,
) -> anyhow::Result<()> {
    info!("Starting...");

    let init = esp_radio::init().unwrap();

    // == Step 2: ==
    // Allocate the Matter stack.
    // For MCUs, it is best to allocate it statically, so as to avoid program stack blowups (its memory footprint is ~ 35 to 50KB).
    // It is also (currently) a mandatory requirement when the wireless stack variation is used.
    let stack = mk_static!(EmbassyWifiMatterStack::<BUMP_SIZE, ()>).init_with(
        EmbassyWifiMatterStack::init(&DEV_DET, DEV_COMM, &TEST_DEV_ATT, epoch),
    );

    let crypto = default_crypto::<NoopRawMutex, _>(
        reseeding_csprng(esp_hal::rng::Trng::try_new().unwrap(), 1000).unwrap(),
        DAC_PRIVKEY,
    );

    let mut weak_rand = crypto.weak_rand().unwrap();


    // == Step 3: ==
    // Our "light" on-off cluster.
    // It will toggle the light state every 5 seconds
    let window_covering_handler = WindowCoveringHandler::new(
        Dataver::new_rand(&mut weak_rand),
        controller,
        storage.clone(),
    )
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
            Async(DescHandler::new(Dataver::new_rand(&mut weak_rand)).adapt()),
        );

    // Create the persister & load any previously saved state
    // `EmbassyPersist`+`EmbassyKvBlobStore` saves to a user-supplied NOR Flash region
    // However, for this demo and for simplicity, we use a dummy persister that does nothing
    let persist = stack
        .create_persist_with_comm_window(
            &crypto,
            Storage::new(storage.flash_access(), StoragePartition::Matter).await?,
        )
        .await
        .unwrap();

    // == Step 4: ==
    // Run the Matter stack with our handler
    // Using `pin!` is completely optional, but reduces the size of the final future
    //
    // This step can be repeated in that the stack can be stopped and started multiple times, as needed.
    let matter = pin!(stack.run(
        // The Matter stack needs to instantiate an `embassy-net` `Driver` and `Controller`
        EmbassyWifi::new(
            EspWifiDriver::new(&init, wifi, bt),
            weak_rand,
            true, /* use a random BLE address */
            stack
        ),
        // The Matter stack needs a persister to store its state
        &persist,
        // The crypto provider
        &crypto,
        // Our `AsyncHandler` + `AsyncMetadata` impl
        (NODE, handler),
        // No user future to run
        (),
    ));

    // Run Matter
    matter.await.unwrap();

    Ok(())
}

/// Endpoint 0 (the root endpoint) always runs
/// the hidden Matter system clusters, so we pick ID=1
const WINDOW_COVERING_ENDPOINT_ID: u16 = 1;

const DEV_TYPE_WINDOW_COVERING: DeviceType = DeviceType {
    dtype: 0x0202,
    drev: 3,
};

/// The Matter Window Covering device Node
const NODE: Node = Node {
    id: 0,
    endpoints: &[
        EmbassyWifiMatterStack::<0, ()>::root_endpoint(),
        Endpoint {
            id: WINDOW_COVERING_ENDPOINT_ID,
            device_types: devices!(DEV_TYPE_WINDOW_COVERING),
            clusters: clusters!(DescHandler::CLUSTER, WindowCoveringHandler::CLUSTER),
        },
    ],
};

const DEV_DET: BasicInfoConfig = BasicInfoConfig {
    vid: TEST_VID,
    pid: TEST_PID,
    hw_ver: 1,
    hw_ver_str: "1",
    sw_ver: 1,
    sw_ver_str: "1",
    serial_no: Environment::matter_serial_number(),
    product_name: "eblind",
    vendor_name: "",
    device_name: "eblind",
    ..BasicInfoConfig::new()
};

const fn matter_seed_u16() -> u16 {
    let mut result: u16 = 0;

    let mut remainder = Environment::matter_seed();
    while remainder != 0 {
        result = result.wrapping_add((remainder & 0xFFFF) as u16);
        remainder >>= 16;
    }

    result
}

// TODO: Use properly generated keys
const DEV_COMM: BasicCommData = 
rs_matter_embassy::matter::dm::devices::test::TEST_DEV_COMM
/*BasicCommData {
    password: (matter_seed_u16().wrapping_add(0x22BF) as u32)
        .to_le_bytes()
        .into(),
    discriminator: matter_seed_u16().wrapping_add(0x3750),
}*/;

/// Test Vendor ID
/// Matches what chip-tool tests expect
const TEST_VID: u16 = 0xfff1;
/// Test Product ID
/// Matches what chip-tool tests expect
const TEST_PID: u16 = 0x8001;
