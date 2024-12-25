use printhor_hwa_common as hwa;

//#region "General controllers lock types"
pub type EventBusPubSubMutexType = hwa::AsyncNoopMutexType;
pub type EventBusLockType = hwa::AsyncNoopMutexType;
//#endregion

//#region "HWI Shared controllers lock types"
pub type WatchDogLockType = hwa::AsyncNoopMutexType;

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-motion", feature = "with-hot-end", feature="with-hot-bed"))] {
        pub type DeferChannelMutexType = hwa::AsyncNoopMutexType;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-usb")] {
        pub type SerialUsbTxLockType = hwa::AsyncNoopMutexType;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-1")] {
        pub type SerialPort1TxLockType = hwa::AsyncNoopMutexType;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-2")] {
        pub type SerialPort2TxLockType = hwa::AsyncNoopMutexType;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        pub type MotionPinsMutexType = hwa::SyncCsMutexType;
        pub type MotionRingBufferMutexType = hwa::AsyncNoopMutexType;
        pub type MotionSignalMutexType = hwa::AsyncNoopMutexType;
        pub type MotionConfigMutexType = hwa::AsyncNoopMutexType;
        pub type MotionStatusMutexType = hwa::AsyncNoopMutexType;
        pub type MotionDriverMutexType = hwa::AsyncNoopMutexType;
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-probe", feature = "with-hot-end",feature = "with-hot-bed",
        feature = "with-laser", feature = "with-fan-layer", feature = "with-fan-extra-1"))] {
        pub type Pwm1LockType = hwa::SyncNoopMutexType;
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-hot-end",feature = "with-hot-bed"))] {
        pub type Adc1LockType = hwa::AsyncNoopMutexType;
    }
}

//#endregion

//#region "General controllers locking strategy customization"
pub type EventBusMutexStrategy = hwa::AsyncStandardStrategy<EventBusLockType, hwa::EventBusChannelController<EventBusPubSubMutexType>>;
pub type WatchDogMutexStrategy = hwa::AsyncStandardStrategy<WatchDogLockType, super::device::WatchDog>;
//#endregion

//#region "Shared controllers locking strategy customization"

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-usb")] {
        pub type SerialUsbTxMutexStrategy = hwa::AsyncStandardStrategy<SerialUsbTxLockType, super::device::SerialUsbTx>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-1")] {
        pub type SerialPort1TxMutexStrategy = hwa::AsyncStandardStrategy<SerialPort1TxLockType, super::device::SerialPort1Tx>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-2")] {
        pub type SerialPort2TxMutexStrategy = hwa::AsyncStandardStrategy<SerialPort2TxLockType, super::device::SerialPort2Tx>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        pub type MotionPinsMutexStrategy = hwa::SyncStandardStrategy<MotionPinsMutexType, super::device::MotionPins>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-ps-on")] {
        pub type PSOnLockType = hwa::SyncNoopMutexType;
        pub type PSOnMutexStrategy = hwa::SyncStandardStrategy<PSOnLockType, super::device::PsOnPin>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-probe", feature = "with-hot-end",feature = "with-hot-bed",
        feature = "with-laser", feature = "with-fan-layer", feature = "with-fan-extra-1"))] {
        pub type Pwm1MutexStrategy = hwa::SyncStandardStrategy<Pwm1LockType, super::device::Pwm1>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-probe")] {
        pub type ProbeMutexStrategy = Pwm1MutexStrategy;
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-hot-end",feature = "with-hot-bed"))] {
        pub type Adc1MutexStrategy = hwa::AsyncStandardStrategy<Adc1LockType, super::device::Adc1>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-hot-end")] {
        pub type HotEndAdcMutexStrategy = Adc1MutexStrategy;
        pub type HotEndPwmMutexStrategy = Pwm1MutexStrategy;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-hot-bed")] {
        pub type HotBedAdcMutexStrategy = Adc1MutexStrategy;
        pub type HotBedPwmMutexStrategy = Pwm1MutexStrategy;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-fan-layer")] {
        pub type FanLayerPwmMutexStrategy = Pwm1MutexStrategy;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-sd-card")] {
        pub type SDCardBlockDevice = super::device::SDCardBlockDevice;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-print-job")] {
        pub type PrinterControllerSignalMutexType = hwa::AsyncNoopMutexType;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-laser")] {
        pub type LaserPwmMutexStrategy = Pwm1MutexStrategy;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-fan-extra-1")] {
        pub type FanExtra1PwmMutexStrategy = Pwm1MutexStrategy;
    }
}

//#endregion