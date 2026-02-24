use alloc::sync::Arc;
use alloc::vec::Vec;
use core::cell::RefCell;
use core::str;

use bincode::{Decode, Encode};
use critical_section::Mutex;

use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use esp_bootloader_esp_idf::partitions::{
    DataPartitionSubType, Error as PartitionError, PARTITION_TABLE_MAX_LEN, PartitionType,
    read_partition_table,
};
use esp_hal::peripherals::FLASH as Flash;
use esp_nvs::platform::Crc;
use esp_nvs::{Key, Nvs};
use esp_storage::FlashStorage;

use crate::EmbassyMutex;
use crate::movement::UnitPosition;

// TODO: Prevent accidental writes when NVS is erased and we want to reset

/// Allows shared access to [`FlashStorage`].
///
/// This struct synchronizes access to the underlying flash storage,
/// so that multiple tasks can safely read from and write to it concurrently.
#[derive(Debug, Clone)]
pub struct FlashAccess<'d> {
    storage: Arc<Mutex<RefCell<FlashStorage<'d>>>>,
    can_write: Arc<Mutex<RefCell<bool>>>,
}

impl<'d> FlashAccess<'d> {
    pub fn new(flash: Flash<'d>) -> Self {
        let flash_storage = FlashStorage::new(flash);
        Self {
            storage: Arc::new(Mutex::new(RefCell::new(flash_storage))),
            can_write: Arc::new(Mutex::new(RefCell::new(true))),
        }
    }

    /// This executes the provided closure only if writing is currently allowed, otherwise does nothing.
    pub async fn write_maybe<T>(&self, f: impl AsyncFnOnce() -> T, default: T) -> T {
        let can_write = critical_section::with(|cs| *self.can_write.borrow_ref_mut(cs));

        if can_write { f().await } else { default }
    }

    pub fn disable_writes(&self) {
        critical_section::with(|cs| {
            *self.can_write.borrow_ref_mut(cs) = false;
        });
    }

    pub fn can_write(&self) -> bool {
        critical_section::with(|cs| *self.can_write.borrow_ref_mut(cs))
    }
}

impl<'d> ErrorType for FlashAccess<'d> {
    type Error = <FlashStorage<'d> as ErrorType>::Error;
}

impl<'d> ReadNorFlash for FlashAccess<'d> {
    const READ_SIZE: usize = <FlashStorage<'d> as ReadNorFlash>::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        critical_section::with(|cs| self.storage.borrow_ref_mut(cs).read(offset, bytes))
    }

    fn capacity(&self) -> usize {
        critical_section::with(|cs| self.storage.borrow_ref_mut(cs).capacity())
    }
}

impl<'d> NorFlash for FlashAccess<'d> {
    const WRITE_SIZE: usize = <FlashStorage<'d> as NorFlash>::WRITE_SIZE;
    const ERASE_SIZE: usize = <FlashStorage<'d> as NorFlash>::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        critical_section::with(|cs| self.storage.borrow_ref_mut(cs).erase(from, to))
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        critical_section::with(|cs| self.storage.borrow_ref_mut(cs).write(offset, bytes))
    }
}

impl<'d> embedded_storage::ReadStorage for FlashAccess<'d> {
    type Error = <FlashStorage<'d> as embedded_storage::ReadStorage>::Error;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        critical_section::with(|cs| {
            <FlashStorage<'d> as embedded_storage::ReadStorage>::read(
                &mut *self.storage.borrow_ref_mut(cs),
                offset,
                bytes,
            )
        })
    }

    fn capacity(&self) -> usize {
        critical_section::with(|cs| {
            <FlashStorage<'d> as embedded_storage::ReadStorage>::capacity(
                &*self.storage.borrow_ref_mut(cs),
            )
        })
    }
}

impl<'d> embedded_storage::Storage for FlashAccess<'d> {
    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        critical_section::with(|cs| {
            <FlashStorage<'d> as embedded_storage::Storage>::write(
                &mut *self.storage.borrow_ref_mut(cs),
                offset,
                bytes,
            )
        })
    }
}

impl<'d> Crc for FlashAccess<'d> {
    fn crc32(init: u32, data: &[u8]) -> u32 {
        // TODO: replace with delegation to that once esp-nvs is updated
        // FlashStorage::crc32(init, data)
        esp_hal::rom::crc::crc32_le(init, data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StoragePartition {
    Nvs,
    Matter,
}

impl StoragePartition {
    pub const fn all() -> [Self; 2] {
        [Self::Nvs, Self::Matter]
    }

    pub const fn name(&self) -> &str {
        match self {
            Self::Nvs => "nvs",
            Self::Matter => "matter",
        }
    }
}

#[derive(Clone)]
pub struct Storage<'d> {
    nvs: Arc<EmbassyMutex<Nvs<FlashAccess<'d>>>>,
    namespace: Key,
    flash_access: FlashAccess<'d>,
}

fn choose_partition(
    flash: &mut impl embedded_storage::Storage,
    name: &str,
) -> anyhow::Result<(usize, usize)> {
    let mut pt_buf = [0u8; PARTITION_TABLE_MAX_LEN];
    let table = read_partition_table(flash, &mut pt_buf)?;

    for entry in table.iter() {
        if entry.partition_type() != PartitionType::Data(DataPartitionSubType::Nvs)
            || entry.label_as_str() != name
        {
            continue;
        }

        log::info!(
            "Using NVS partition '{}' at offset {:#X}, length {:#X}",
            entry.label_as_str(),
            entry.offset(),
            entry.len(),
        );

        return Ok((entry.offset() as usize, entry.len() as usize));
    }

    Err(PartitionError::Invalid.into())
}

impl<'d> Storage<'d> {
    const BINCODE_CONFIG: bincode::config::Configuration = bincode::config::standard();

    pub async fn new(
        mut flash_access: FlashAccess<'d>,
        partition: StoragePartition,
    ) -> anyhow::Result<Self> {
        let (offset, len) = choose_partition(&mut flash_access, partition.name())?;

        let nvs = Nvs::new(offset, len, flash_access.clone())
            .map_err(|err| anyhow::anyhow!("Failed to open NVS: {err:?}"))?;

        let partition_name = partition.name();
        let result = Self {
            nvs: Arc::new(EmbassyMutex::new(nvs)),
            namespace: Key::from_str(partition_name),
            flash_access,
        };

        // Test storing and loading a value
        result.store("init", true).await?;
        debug_assert_eq!(result.load::<bool>("init").await?, Some(true));

        Ok(result)
    }

    /// Locks the storage, preventing any writes until the program is restarted.
    ///
    /// Before it locks, it waits for any ongoing write operations to finish.
    pub async fn lock(&self) {
        let _ = self.nvs.lock().await;

        self.flash_access.disable_writes();
    }

    pub fn flash_access(&self) -> FlashAccess<'d> {
        self.flash_access.clone()
    }

    pub async fn reset_partitions(&self, partitions: &[StoragePartition]) -> anyhow::Result<()> {
        let mut flash_access = self.flash_access();

        for partition in partitions {
            let partition_name = partition.name();
            log::info!("Erasing partition '{partition_name}'");
            let (offset, len) = choose_partition(&mut self.flash_access.clone(), partition_name)?;

            flash_access
                .erase(offset as u32, (offset + len) as u32)
                .map_err(|err| {
                    anyhow::anyhow!("Failed to erase partition '{partition_name}': {err:?}")
                })?;
        }

        log::info!("Flash Access: can_write={}", flash_access.can_write());
        // Test storing and loading a value
        self.store("init", true).await?;
        debug_assert_eq!(self.load::<bool>("init").await?, Some(true));

        Ok(())
    }

    fn namespace(&self) -> Key {
        self.namespace
    }

    pub async fn store<V>(&self, name: impl ToStorageKey, value: V) -> anyhow::Result<()>
    where
        V: Encode,
    {
        self.store_in(self.namespace, name.to_key(), value).await
    }

    async fn store_in<V>(&self, namespace: Key, name: Key, value: V) -> anyhow::Result<()>
    where
        V: Encode,
    {
        self.flash_access
            .write_maybe(
                async || {
                    // TODO: Delegate to the set methods of nvs instead of always serializing to a vec through bincode
                    // It is more efficient for primitive types like a bool

                    let data = bincode::encode_to_vec(value, Self::BINCODE_CONFIG)
                        .map_err(|err| anyhow::anyhow!("failed to serialize value: {err}"))?;

                    self.nvs
                        .lock()
                        .await
                        .set(&namespace, &name, data.as_slice())
                        .map_err(|err| anyhow::anyhow!("failed to store key: {err:?}"))?;

                    Ok(())
                },
                Ok(()),
            )
            .await
    }

    pub async fn load<V: Decode<()>>(&self, name: impl ToStorageKey) -> anyhow::Result<Option<V>> {
        self.load_from(self.namespace, name.to_key()).await
    }

    async fn load_from<V: Decode<()>>(
        &self,
        namespace: Key,
        name: Key,
    ) -> anyhow::Result<Option<V>> {
        let data: Vec<u8> = {
            match self.nvs.lock().await.get(&namespace, &name) {
                Ok(data) => data,
                Err(esp_nvs::error::Error::KeyNotFound) => {
                    return Ok(None);
                }
                Err(err) => {
                    return Err(anyhow::anyhow!("failed to load key: {err:?}"));
                }
            }
        };

        Ok(Some(
            bincode::decode_from_slice(&data, Self::BINCODE_CONFIG)
                .map_err(|err| anyhow::anyhow!("failed to deserialize value: {err}"))?
                .0,
        ))
    }

    pub async fn delete(&self, name: impl ToStorageKey) -> anyhow::Result<()> {
        self.nvs
            .lock()
            .await
            .delete(&self.namespace(), &name.to_key())
            .map_err(|err| anyhow::anyhow!("failed to delete key: {err:?}"))?;

        Ok(())
    }

    pub async fn current_position(&self) -> anyhow::Result<Option<UnitPosition>> {
        self.load("cur_pos").await
    }

    pub async fn max_position(&self) -> anyhow::Result<Option<UnitPosition>> {
        self.load("max_pos").await
    }

    pub async fn set_current_position(&self, value: UnitPosition) -> anyhow::Result<()> {
        self.store("cur_pos", value).await
    }

    pub async fn set_max_position(&self, value: UnitPosition) -> anyhow::Result<()> {
        self.store("max_pos", value).await
    }

    pub async fn number_of_actuations(&self) -> anyhow::Result<Option<u16>> {
        self.load("act-cnt").await
    }

    pub async fn set_number_of_actuations(&self, value: u16) -> anyhow::Result<()> {
        self.store("act-cnt", value).await
    }
}

pub trait ToStorageKey {
    fn to_key(&self) -> Key;
}

impl<T: ToStorageKey + ?Sized> ToStorageKey for &T {
    fn to_key(&self) -> Key {
        (*self).to_key()
    }
}

impl ToStorageKey for str {
    fn to_key(&self) -> Key {
        Key::from_str(self)
    }
}

impl ToStorageKey for u16 {
    fn to_key(&self) -> Key {
        Key::from_array(&u16::to_be_bytes(*self))
    }
}
