#[allow(unused)]
use crate::control;
///! This module contains the peripheral and devices export required for ALL HWI boards
use crate::hwa;
use hwa::HwiContract;
//#region "The core and mandatory devices/peripherals"

pub type EventBus = hwa::GenericEventBus<
    <hwa::Contract as HwiContract>::EventBusMutexStrategy,
    <hwa::Contract as HwiContract>::EventBusPubSubMutexType,
>;

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-motion", feature = "with-hot-end", feature = "with-hot-bed"))] {
        pub type DeferChannel = hwa::GenericDeferChannel<
            <hwa::Contract as HwiContract>::DeferChannelMutexType
        >;
    }
}

cfg_if::cfg_if! {
    if #[cfg(all(feature = "with-motion", feature = "with-motion-broadcast"))] {
        pub type MotionBroadcastChannel = hwa::GenericMotionBroadcastChannel<
            <hwa::Contract as HwiContract>::MotionBroadcastChannelMutexType
        >;

        pub type MotionSender = hwa::StaticAsyncController<
            <hwa::Contract as HwiContract>::MotionSenderMutexStrategy
        >;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-trinamic")] {
        pub type TrinamicUart = <hwa::Contract as HwiContract>::TrinamicUartDevice;
    }
}

pub type WatchDogController =
    hwa::StaticAsyncController<<hwa::Contract as HwiContract>::WatchDogMutexStrategy>;

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {

        pub type MotionPinsMutexStrategy = <hwa::Contract as HwiContract>::MotionPinsMutexStrategy;
        pub type HwaMotionPins = hwa::StaticSyncController<MotionPinsMutexStrategy>;

        pub type MotionSignalMutexType = <hwa::Contract as HwiContract>::MotionSignalMutexType;

        pub type MotionRingBufferMutexStrategy = hwa::AsyncStandardStrategy<
            <hwa::Contract as HwiContract>::MotionRingBufferMutexType,
            hwa::controllers::motion::RingBuffer
        >;

        pub type MotionRingBuffer = hwa::StaticAsyncController<MotionRingBufferMutexStrategy>;

        pub type MotionConfigMutexStrategy = hwa::SyncStandardStrategy<
            <hwa::Contract as HwiContract>::MotionConfigMutexType,
            hwa::controllers::MotionConfigContent
        >;

        pub type MotionConfig = hwa::StaticSyncController<MotionConfigMutexStrategy>;

        pub type MotionStatusMutexStrategy = hwa::SyncStandardStrategy<
            <hwa::Contract as HwiContract>::MotionStatusMutexType,
            hwa::controllers::MotionStatusContent
        >;
        pub type MotionStatus = hwa::StaticSyncController<MotionStatusMutexStrategy>;

        pub type MotionDriverMutexStrategy = hwa::AsyncStandardStrategy<
            <hwa::Contract as HwiContract>::MotionDriverMutexType,
            hwa::drivers::MotionDriver
        >;

        pub type MotionDriver = hwa::StaticAsyncController<MotionDriverMutexStrategy>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-usb")] {
        pub type SerialUsbInputStream = <hwa::Contract as HwiContract>::SerialUsbRx;
        pub type SerialUsbTxController = hwa::StaticAsyncController<<hwa::Contract as HwiContract>::SerialUsbTx>;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-1")] {
        pub type SerialPort1TxController = hwa::StaticAsyncController<<hwa::Contract as HwiContract>::SerialPort1Tx>;
        pub type SerialPort1InputStream = <hwa::Contract as HwiContract>::SerialPort1Rx;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-2")] {
        pub type SerialPort2TxController = hwa::StaticAsyncController<<hwa::Contract as HwiContract>::SerialPort2Tx>;
        pub type SerialPort2InputStream = <hwa::Contract as HwiContract>::SerialPort2Rx;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-i2c")] {
        pub type I2CMotionMutexStrategy = <hwa::Contract as HwiContract>::I2cMotionMutexStrategy;
        pub type I2CMotionController = hwa::StaticSyncController<I2CMotionMutexStrategy>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-ps-on")] {
        pub type PSOnController = hwa::StaticSyncController<
            <hwa::Contract as HwiContract>::PSOnMutexStrategy
        >;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-probe")] {
        // A nested controller
        pub type InnerProbeController = hwa::controllers::GenericServoController<
            <hwa::Contract as HwiContract>::ProbePwm
        >;

        pub type ProbeControllerMutexStrategy = hwa::AsyncStandardStrategy<
                hwa::AsyncNoopMutexType,
                InnerProbeController,
            >;

        pub type ProbeController = hwa::StaticAsyncController<
            ProbeControllerMutexStrategy
        >;

    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-hot-end")] {

        pub type HotEndPwmController = hwa::controllers::GenericPwmController<
            <hwa::Contract as HwiContract>::HotEndPwm
        >;
        pub type HotEndAdcController = hwa::controllers::GenericAdcController<
            <hwa::Contract as HwiContract>::HotEndAdc,
        >;

        pub type HotEndControllerMutexStrategy = hwa::AsyncStandardStrategy<
            hwa::AsyncNoopMutexType,
            hwa::controllers::HeaterController<
                <hwa::Contract as HwiContract>::HotEndAdc,
                <hwa::Contract as HwiContract>::HotEndPwm,
            >
        >;
        pub type HotEndController = hwa::StaticAsyncController<HotEndControllerMutexStrategy>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-hot-bed")] {

        pub type HotBedPwmController = hwa::controllers::GenericPwmController<
            <hwa::Contract as HwiContract>::HotBedPwm
        >;
        pub type HotBedAdcController = hwa::controllers::GenericAdcController<
            <hwa::Contract as HwiContract>::HotBedAdc,
        >;

        pub type HotBedControllerMutexStrategy = hwa::AsyncStandardStrategy<
            hwa::AsyncNoopMutexType,
            hwa::controllers::HeaterController<
                <hwa::Contract as HwiContract>::HotBedAdc,
                <hwa::Contract as HwiContract>::HotBedPwm,
            >
        >;
        pub type HotBedController = hwa::StaticAsyncController<HotBedControllerMutexStrategy>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-fan-layer")] {
        // A nested controller
        pub type InnerFanLayerController = hwa::controllers::GenericPwmController<
            <hwa::Contract as HwiContract>::FanLayerPwm
        >;

        pub type FanLayerControllerMutexStrategy = hwa::AsyncStandardStrategy<
            hwa::AsyncNoopMutexType,
            InnerFanLayerController,
        >;

        pub type FanLayerController = hwa::StaticAsyncController<
            FanLayerControllerMutexStrategy
        >;

    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-laser")] {

        // A nested controller
        pub type InnerLaserController = hwa::controllers::GenericPwmController<
            <hwa::Contract as HwiContract>::LaserPwm
        >;

        pub type LaserControllerMutexStrategy = hwa::AsyncStandardStrategy<
            hwa::AsyncNoopMutexType,
            InnerLaserController,
        >;

        pub type LaserController = hwa::StaticAsyncController<
            LaserControllerMutexStrategy
        >;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-fan-extra-1")] {

        // A nested controller
        pub type InnerFanExtra1Controller = hwa::controllers::GenericPwmController<
            <hwa::Contract as HwiContract>::FanExtra1Pwm
        >;

        pub type FanExtra1ControllerMutexStrategy = hwa::AsyncStandardStrategy<
            hwa::AsyncNoopMutexType,
            InnerFanExtra1Controller,
        >;

        pub type FanExtra1Controller = hwa::StaticAsyncController<
            FanExtra1ControllerMutexStrategy
        >;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-sd-card")] {
        // A nested controller

        pub type SDCardInnerController = hwa::controllers::GenericSDCardController<
            <hwa::Contract as HwiContract>::SDCardBlockDevice,
            {<hwa::Contract as HwiContract>::SD_CARD_MAX_DIRS},
            {<hwa::Contract as HwiContract>::SD_CARD_MAX_FILES},
            {<hwa::Contract as HwiContract>::SD_CARD_MAX_DIRS + <hwa::Contract as HwiContract>::SD_CARD_MAX_FILES},
        >;

        pub type SDCardControllerMutexStrategy = hwa::AsyncStandardStrategy<
            hwa::AsyncNoopMutexType,
            SDCardInnerController,
        >;

        pub type SDCardController = hwa::controllers::GenericSDCardController<
            <hwa::Contract as HwiContract>::SDCardBlockDevice,
            {<hwa::Contract as HwiContract>::SD_CARD_MAX_DIRS},
            {<hwa::Contract as HwiContract>::SD_CARD_MAX_FILES},
            {<hwa::Contract as HwiContract>::SD_CARD_MAX_DIRS + <hwa::Contract as HwiContract>::SD_CARD_MAX_FILES},
        >;

    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-print-job")] {
        // A nested controller

        pub type PrinterControllerSignalMutexType = <hwa::Contract as HwiContract>::PrinterControllerSignalMutexType;

        pub type SDCardLineParser = control::GCodeLineParser<
            hwa::controllers::sd_card_controller::SDCardStream<
                <hwa::Contract as HwiContract>::SDCardBlockDevice,
                {<hwa::Contract as HwiContract>::SD_CARD_MAX_DIRS},
                {<hwa::Contract as HwiContract>::SD_CARD_MAX_FILES},
                {<hwa::Contract as HwiContract>::SD_CARD_MAX_DIRS + <hwa::Contract as HwiContract>::SD_CARD_MAX_FILES}
            >
        >;
    }
}
