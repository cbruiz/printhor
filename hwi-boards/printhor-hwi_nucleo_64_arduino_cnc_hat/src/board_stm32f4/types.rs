use printhor_hwa_common as hwa;

//#region "General controllers lock types"
pub type EventBusPubSubMutexType = hwa::NoopRawMutexType;
pub type EventBusLockType = hwa::NoopRawMutexType;
//#endregion

//#region "HWI Shared controllers lock types"
pub type WatchDogLockType = hwa::NoopRawMutexType;
pub type SerialPort1TxLockType = hwa::NoopRawMutexType;
//#endregion

//#region "General controllers locking strategy customization"
pub type EventBusMutexStrategy = hwa::AsyncStandardStrategy<EventBusLockType, hwa::EventBusChannelController<EventBusPubSubMutexType>>;
pub type WatchDogMutexStrategy = hwa::AsyncStandardStrategy<WatchDogLockType, super::device::Watchdog>;
//#endregion

//#region "Shared controllers locking strategy customization"
pub type SerialPort1TxMutexStrategy = hwa::AsyncStandardStrategy<SerialPort1TxLockType, super::device::SerialPort1Tx>;
//#endregion