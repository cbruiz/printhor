//! Machine metadata information
use crate::hwa;
use hwa::HwiContract;

/// The Machine metadata info structure
pub struct MachineInfo {
    pub firmware_name: &'static str,
    pub firmware_version: &'static str,
    pub firmware_url: &'static str,
    pub machine_type: &'static str,
    pub machine_board: &'static str,
    pub machine_processor: &'static str,
    pub machine_uuid: &'static str,
    pub extruder_count: u8,
}

impl MachineInfo {
    pub const fn new() -> Self {
        Self {
            firmware_name: "PrinThor",
            firmware_version: env!("CARGO_PKG_VERSION"),
            firmware_url: "https://github.com/cbruiz/printhor",
            machine_type: hwa::Contract::MACHINE_TYPE,
            machine_board: hwa::Contract::MACHINE_BOARD,
            machine_processor: hwa::Contract::MACHINE_PROCESSOR,
            machine_uuid: "00000000-0000-0000-0000-000000000000",
            extruder_count: 1,
        }
    }
}
/// Public static read-only instance of the machine metadata
pub static MACHINE_INFO: MachineInfo = MachineInfo::new();
