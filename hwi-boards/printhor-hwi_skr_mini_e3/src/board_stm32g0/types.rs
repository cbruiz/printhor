//! Types for SKR Mini E3 V3 (STM32G0B1RE)
use printhor_hwa_common as hwa;

//#region "General controllers lock types"
pub type EventBusPubSubMutexType = hwa::AsyncNoopMutexType;
pub type EventBusLockType = hwa::AsyncNoopMutexType;
//#endregion

//#region "HWI Shared controllers lock types"
pub type WatchDogLockType = hwa::AsyncNoopMutexType;

//#endregion

//#region "General controllers locking strategy customization"
pub type EventBusMutexStrategy = hwa::AsyncStandardStrategy<EventBusLockType, hwa::EventBusChannelController<EventBusPubSubMutexType>>;
pub type WatchDogMutexStrategy = hwa::AsyncStandardStrategy<WatchDogLockType, crate::board::device::Watchdog>;
//#endregion

//#region "Shared controllers locking strategy customization"

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-usb")] {
        pub type SerialUsbTxLockType = hwa::AsyncNoopMutexType;
        pub type SerialUsbTxMutexStrategy = hwa::AsyncStandardStrategy<SerialUsbTxLockType, crate::board::io::serial_usb::SerialUsbDeviceSender>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-1")] {
        pub type SerialPort1TxLockType = hwa::AsyncNoopMutexType;
        pub type SerialPort1TxMutexStrategy = hwa::AsyncStandardStrategy<SerialPort1TxLockType, crate::board::device::SerialPort1Tx>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-2")] {
        pub type SerialPort2TxLockType = hwa::AsyncNoopMutexType;
        pub type SerialPort2TxMutexStrategy = hwa::AsyncStandardStrategy<SerialPort2TxLockType, crate::board::device::SerialPort2Tx>;
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

        pub type StepActuatorMuxtexStrategy = hwa::SyncStandardStrategy<StepActuatorMutexType, crate::board::device::StepActuator>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-trinamic")] {
        pub type TrinamicUartDevice = crate::board::device::TrinamicUartDevice;
    }
}

cfg_if::cfg_if! {
    if #[cfg(all(feature = "with-motion", feature = "with-motion-broadcast"))] {
        pub type MotionBroadcastChannelMutexType = hwa::AsyncCsMutexType;

        pub type MotionSenderMutexType = I2cMutexType;

        pub type MotionSenderMutexStrategy = hwa::AsyncStandardStrategy<MotionSenderMutexType, crate::board::device::MotionSender>;
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
        pub type Spi1MutexStrategyType = hwa::AsyncHoldableStrategy<Spi1MutexType, crate::board::device::Spi>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-i2c")] {
        pub type I2cMutexType = hwa::AsyncCsMutexType;
        pub type I2cMutexStrategyType = hwa::AsyncStandardStrategy<I2cMutexType, crate::board::device::I2c>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-sd-card")] {
        //pub type Spi1MutexType = hwa::AsyncNoopMutexType;
        pub type SDCardBlockDevice = hwa::sd_card_spi::SPIAdapter<
            Spi1MutexStrategyType,
            super::device::SpiCardCSPin,
        >;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-print-job")] {
        pub type PrinterControllerSignalMutexType = hwa::AsyncNoopMutexType;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-laser")] {
        pub type LaserMutexType = hwa::SyncCsMutexType;
        pub type LaserPwmMutexStrategy = hwa::SyncStandardStrategy<LaserMutexType, crate::board::device::PwmLaser>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-probe")] {
        pub type ProbeMutexType = hwa::SyncCsMutexType;
        pub type ProbePwmMutexStrategy = hwa::SyncStandardStrategy<ProbeMutexType, crate::board::device::PwmProbe>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature="with-hot-end", feature = "with-hot-bed"))] {
        pub type HotEndHotBedAdcMutexType = hwa::SyncCsMutexType;
        pub type HotEndHotBedAdcMutexStrategy = hwa::AsyncStandardStrategy<HotEndHotBedAdcMutexType, crate::board::device::HotEndHotBedAdc>;

        pub type HotEndHotBedPwmMutexType = hwa::SyncNoopMutexType;
        pub type HotEndHotBedPwmMutexStrategy = hwa::SyncStandardStrategy<HotEndHotBedPwmMutexType, crate::board::device::HotEndHotBedPwm>;
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