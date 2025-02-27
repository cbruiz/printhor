use printhor_hwa_common as hwa;

//#region "General controllers lock types"
pub type EventBusPubSubMutexType = hwa::AsyncNoopMutexType;
pub type EventBusLockType = hwa::AsyncNoopMutexType;
//#endregion

//#region "HWI Shared controllers lock types"
pub type WatchDogLockType = hwa::AsyncNoopMutexType;

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-usb")] {
        pub type SerialUsbTxLockType = hwa::AsyncNoopMutexType;
        pub type SerialUsbTxMutexStrategy = hwa::AsyncStandardStrategy<SerialUsbTxLockType, super::device::SerialUsbSender>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-1")] {
        pub type SerialPort1TxLockType = hwa::AsyncNoopMutexType;
        pub type SerialPort1TxMutexStrategy = hwa::AsyncStandardStrategy<SerialPort1TxLockType, super::device::SerialPort1Tx>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-2")] {
        pub type SerialPort2TxLockType = hwa::AsyncNoopMutexType;
        pub type SerialPort2TxMutexStrategy = hwa::AsyncStandardStrategy<SerialPort2TxLockType, super::device::SerialPort2Tx>;
    }
}
//#endregion

//#region "General controllers locking strategy customization"
pub type EventBusMutexStrategy = hwa::AsyncStandardStrategy<EventBusLockType, hwa::EventBusChannelController<EventBusPubSubMutexType>>;
pub type WatchDogMutexStrategy = hwa::AsyncStandardStrategy<WatchDogLockType, super::device::Watchdog>;
//#endregion

//#region "Shared controllers locking strategy customization"
cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-1")] {
        pub type SerialPort1TxMutexStrategy = hwa::AsyncStandardStrategy<SerialPort1TxLockType, super::device::SerialPort1Tx>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-motion", feature = "with-hot-end", feature = "with-hot-bed"))] {
        pub type DeferChannelMutexType = hwa::AsyncNoopMutexType;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        pub type StepActuatorMutexType = hwa::SyncCsMutexType;
        pub type MotionSignalMutexType = hwa::AsyncNoopMutexType;
        pub type MotionRingBufferMutexType = hwa::AsyncNoopMutexType;
        pub type MotionConfigMutexType = hwa::AsyncCsMutexType;
        pub type MotionStatusMutexType = hwa::AsyncNoopMutexType;
        pub type MotionDriverMutexType = hwa::AsyncNoopMutexType;

        pub type StepActuatorMuxtexStrategy = hwa::SyncStandardStrategy<StepActuatorMutexType, super::device::StepActuator>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(all(feature = "with-motion", feature = "with-motion-broadcast"))] {
        pub type MotionBroadcastChannelMutexType = hwa::AsyncCsMutexType;

        pub type MotionSenderMutexType = I2cMutexType;

        pub type MotionSenderMutexStrategy = hwa::AsyncStandardStrategy<MotionSenderMutexType, super::device::MotionSender>;
    }
}


cfg_if::cfg_if! {
    if #[cfg(feature = "with-ps-on")] {
        pub type PSOnLockType = hwa::SyncNoopMutexType;
        pub type PSOnMutexStrategy = hwa::SyncStandardStrategy<PSOnLockType, super::device::PsOnPin>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-spi")] {
        pub type Spi1MutexType = hwa::AsyncNoopMutexType;
        pub type Spi1MutexStrategyType = hwa::AsyncHoldableStrategy<Spi1MutexType, super::device::Spi>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-i2c")] {
        pub type I2cMutexType = hwa::AsyncCsMutexType;
        pub type I2cMutexStrategyType = hwa::AsyncStandardStrategy<I2cMutexType, super::device::I2c>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-laser")] {
        pub type LaserMutexType = hwa::SyncCsMutexType;
        pub type LaserPwmMutexStrategy = hwa::SyncStandardStrategy<LaserMutexType, super::device::PwmLaser>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-probe")] {
        pub type ProbeMutexType = hwa::SyncCsMutexType;
        pub type ProbePwmMutexStrategy = hwa::SyncStandardStrategy<ProbeMutexType, super::device::PwmProbe>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature="with-hot-end", feature = "with-hot-bed"))] {
        pub type HotEndHotBedAdcMutexType = hwa::SyncCsMutexType;
        pub type HotEndHotBedAdcMutexStrategy = hwa::AsyncStandardStrategy<HotEndHotBedAdcMutexType, super::device::HotEndHotBedAdc>;

        pub type HotEndHotBedPwmMutexType = hwa::SyncNoopMutexType;
        pub type HotEndHotBedPwmMutexStrategy = hwa::SyncStandardStrategy<HotEndHotBedPwmMutexType, super::device::HotEndHotBedPwm>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-hot-end")] {
        pub type HotEndAdcMutexStrategy = HotEndHotBedAdcMutexStrategy;
        pub type HotEndPwmMutexStrategy = HotEndHotBedPwmMutexStrategy;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-hot-bed")] {
        pub type HotBedAdcMutexStrategy = HotEndHotBedAdcMutexStrategy;
        pub type HotBedPwmMutexStrategy = HotEndHotBedPwmMutexStrategy;
    }
}
//#endregion