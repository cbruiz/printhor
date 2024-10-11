///! This module contains the peripheral and devices export required for ALL HWI boards
use crate::hwa;
use hwa::HwiContract;
#[allow(unused)]
use crate::control;

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

pub type WatchDogController = hwa::StaticAsyncController<
    <hwa::Contract as HwiContract>::WatchDogMutexStrategy
>;

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

        pub type MotionStatusMutexStrategy = hwa::SyncStandardStrategy<
            <hwa::Contract as HwiContract>::MotionStatusMutexType,
            hwa::controllers::MotionStatusContent
        >;
        pub type MotionStatus = hwa::StaticAsyncController<MotionStatusMutexStrategy>;

        pub type MotionDriverMutexStrategy = hwa::AsyncStandardStrategy<
            <hwa::Contract as HwiContract>::MotionDriverMutexType,
            hwa::drivers::MotionDriver
        >;

        pub type MotionDriver = hwa::StaticAsyncController<MotionDriverMutexStrategy>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-fan-layer")] {

        pub type FanLayerController = hwa::StaticAsyncController<
            hwa::hwi::FanLayerControllerMutexStrategyType<
                hwa::controllers::PwmController<
                    hwa::hwi::PwmFanLayerMutexStrategyType<device::PwmFanLayer>
                >
            >
        >;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-fan-extra-1")] {
        pub type FanExtra1Device = hwi::device::PwmFanExtra1;
        pub type FanExtra1MutexStrategy = hwi::FanExtra1ControllerMutexStrategyType<
            hwa::controllers::PwmController<
                hwi::PwmFanExtra1MutexStrategyType<FanExtra1Device>
            >
        >;
        pub type FanExtra1Controller = hwa::StaticAsyncController<FanExtra1MutexStrategy>;
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
    if #[cfg(feature = "with-ps-on")] {
        pub type PSOnController = hwa::StaticSyncController<
            <hwa::Contract as HwiContract>::PSOnMutexStrategy
        >;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-laser")] {
        // A nested controller
        type _LaserDeviceMutexStrategy_ = hwi::PwmLaserMutexStrategyType<hwi::device::PwmLaser>;
        pub type _LaserControllerMutexStrategy_ = hwi::LaserControllerMutexStrategyType<
            hwa::controllers::PwmController<
                _LaserDeviceMutexStrategy_
            >
        >;
        pub type LaserController = hwa::StaticAsyncController<_LaserControllerMutexStrategy_>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-probe")] {
        // A nested controller
        type _ProbeDeviceMutexStrategy_ = hwi::PwmProbeMutexStrategyType<hwi::device::PwmProbe>;
        pub type _ProbeControllerMutexStrategy_ = hwi::ProbeServoControllerMutexStrategyType<
            hwa::controllers::ServoController<
                _ProbeDeviceMutexStrategy_
            >
        >;
        pub type ProbeController = hwa::StaticAsyncController<_ProbeControllerMutexStrategy_>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-hot-end")] {
        //hwa::StaticAsyncController<hwa::HotBedControllerMutexStrategyType<hwa::controllers::HotBedController>>
        /*pub type HotBedController = HeaterController<
    hwa::AdcHotBedMutexType,
    hwa::PwmHotBedMutexType,
    hwa::device::AdcHotBedPeripheral,
    hwa::device::AdcHotBedPin,
    hwa::device::PwmHotBed,
>;
        */
        // A complex nested controller
        type _HotEndAdcDeviceMutexStrategy_ = hwi::AdcHotEndMutexStrategyType<hwi::device::AdcHotEnd>;
        type _HotEndPwmDeviceMutexStrategy_ = hwi::PwmHotEndMutexStrategyType<hwi::device::PwmHotEnd>;
        type _HotEndAdcPin_ =  hwi::device::AdcHotEndPin;

        pub type _HotEndControllerMutexStrategy_ = hwi::HotEndControllerMutexStrategyType<
            hwa::controllers::HeaterController<
                _HotEndAdcDeviceMutexStrategy_,_HotEndPwmDeviceMutexStrategy_,_HotEndAdcPin_
            >
        >;
        pub type HotEndController = hwa::StaticAsyncController<_HotEndControllerMutexStrategy_>;


    }
}
