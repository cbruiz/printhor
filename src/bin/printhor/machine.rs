use crate::hwa;
#[allow(unused)]
pub(crate) struct MachineInfo
{
    pub(crate) firmware_name: & 'static str,
    pub(crate) firmware_version: & 'static str,
    pub(crate) firmware_url: & 'static str,
    pub(crate) machine_type: & 'static str,
    pub(crate) machine_board: & 'static str,
    pub(crate) machine_processor: & 'static str,
    pub(crate) machine_uuid: & 'static str,
    pub(crate) extruder_count: u8,
}

impl MachineInfo {
    #[allow(unused)]
    pub(crate) const fn new() -> Self {
        Self {
            firmware_name: "PrinThor",
            firmware_version: env!("CARGO_PKG_VERSION"),
            firmware_url: "https://github.com/cbruiz/printhor",
            machine_type: hwa::MACHINE_TYPE,
            machine_board: hwa::MACHINE_BOARD,
            machine_processor: hwa::MACHINE_PROCESSOR,
            machine_uuid: "00000000-0000-0000-0000-000000000000",
            extruder_count: 1,
        }
    }
}
pub(crate) static MACHINE_INFO: MachineInfo = MachineInfo::new();
