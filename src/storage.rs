use bincode::{Decode, Encode};

use esp_idf_hal::sys::EspError;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};

use crate::environment::Environment;
use crate::movement::UnitPosition;

const FIRMWARE_NAMESPACE: &str = "window-covering";

pub struct Storage(EspNvs<NvsDefault>, EspDefaultNvsPartition);

impl Storage {
    const BINCODE_CONFIG: bincode::config::Configuration = bincode::config::standard();

    pub fn new(partition: EspDefaultNvsPartition) -> Result<Self, EspError> {
        let nvs = EspNvs::new(partition.clone(), FIRMWARE_NAMESPACE, true)?;
        if Environment::should_clear_stored_data() {
            nvs.erase_all()?;
            log::info!("Cleared NVS data in namespace '{}'", FIRMWARE_NAMESPACE);
        }

        // For set_blob the following restrictions apply:
        // - max key size is 16 - 1 characters, must not be empty
        // - length of binary value is 508_000 bytes or (97.6% of the partition size - 4000) bytes, whichever is lower

        Ok(Self(nvs, partition))
    }

    pub fn reset(&self) -> Result<(), EspError> {
        self.0.erase_all()?;
        //log::info!("Reset NVS data in namespace '{}'", FIRMWARE_NAMESPACE);
        Ok(())
    }

    pub fn store<V>(&self, name: &str, value: V) -> anyhow::Result<()>
    where
        V: Encode,
    {
        let data = bincode::encode_to_vec(value, Self::BINCODE_CONFIG)?;

        self.0.set_blob(name, &data)?;

        Ok(())
    }

    pub fn load<V: Decode<()>>(&self, name: &str) -> anyhow::Result<Option<V>> {
        // first fetch the size of the blob:
        let Some(required_size) = self.0.blob_len(name)? else {
            return Ok(None);
        };

        let mut data = vec![0u8; required_size];
        self.0.get_blob(name, &mut data)?;

        Ok(Some(
            bincode::decode_from_slice(&data, Self::BINCODE_CONFIG)?.0,
        ))
    }

    pub fn current_position(&self) -> anyhow::Result<Option<UnitPosition>> {
        self.load("cur_pos")
    }

    pub fn max_position(&self) -> anyhow::Result<Option<UnitPosition>> {
        self.load("max_pos")
    }

    pub fn set_current_position(&self, value: UnitPosition) -> anyhow::Result<()> {
        self.store("cur_pos", value)
    }

    pub fn set_max_position(&self, value: UnitPosition) -> anyhow::Result<()> {
        self.store("max_pos", value)
    }

    pub fn number_of_actuations(&self) -> Result<Option<u16>, EspError> {
        self.0.get_u16("act-cnt")
    }

    pub fn set_number_of_actuations(&self, value: u16) -> Result<(), EspError> {
        self.0.set_u16("act-cnt", value)
    }

    pub fn clear_matter_store(&self) -> Result<(), EspError> {
        const NAMESPACE: &str = "esp-idf-matter";
        let esp_nvs = EspNvs::new(self.1.clone(), NAMESPACE, true)?;

        esp_nvs.erase_all()?;

        Ok(())
    }
}
