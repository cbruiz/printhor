use printhor_hwa_common as hwa;

//#region "General controllers lock types"
pub type EventBusPubSubMutexType = hwa::NoopMutex;
pub type EventBusLockType = hwa::NoopMutex;
//#endregion

//#region "HWI Shared controllers lock types"
pub type WatchDogLockType = hwa::NoopMutex;
pub type SerialPort1TxLockType = hwa::NoopMutex;
//#endregion

//#region "General controllers locking strategy customization"
pub type EventBusMutexStrategy = hwa::NotHoldable<EventBusLockType, hwa::EventBusChannelController<EventBusPubSubMutexType>>;
pub type WatchDogMutexStrategy = hwa::NotHoldable<WatchDogLockType, super::device::Watchdog>;
//#endregion

//#region "Shared controllers locking strategy customization"
pub type SerialPort1TxMutexStrategy = hwa::NotHoldable<SerialPort1TxLockType, super::device::SerialPort1Tx>;
//#endregion