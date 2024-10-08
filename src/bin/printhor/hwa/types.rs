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

pub type WatchDogController =
    hwa::StaticController<<hwa::Contract as HwiContract>::WatchDogMutexStrategy>;

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {

        pub type MotionRingBuffer = hwa::StaticController<
            hwi::MotionRingBufferMutexStrategyType<hwa::controllers::motion::RingBuffer>
        >;

        pub type MotionRingBuffer = hwa::StaticController<
            hwi::MotionRingBufferMutexStrategyType<hwa::controllers::motion::RingBuffer>
        >;

        pub type MotionSignalMutexType = hwi::MotionSignalMutexType;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-fan-layer")] {

        pub type FanLayerController = hwa::StaticController<
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
        pub type FanExtra1Controller = hwa::StaticController<FanExtra1MutexStrategy>;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-usb")] {
        pub type SerialUsbInputStream = <hwa::Contract as HwiContract>::SerialUsbRx;
        pub type SerialUsbTxController = hwa::StaticController<<hwa::Contract as HwiContract>::SerialUsbTx>;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-1")] {
        pub type SerialPort1TxController = hwa::StaticController<<hwa::Contract as HwiContract>::SerialPort1Tx>;
        pub type SerialPort1InputStream = <hwa::Contract as HwiContract>::SerialPort1Rx;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-2")] {
        type _SerialPort2MutexStrategy_ = hwi::SerialPort2MutexStrategyType<hwi::device::SerialPort2TxDevice>;
        pub type SerialPort2InputStream = hwi::device::SerialPort2InputStream;
        pub type SerialPort2TxController = hwa::StaticController<_SerialPort2MutexStrategy_>;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-ps-on")] {
        type _PsOnMutexStrategy_ = hwi::PSOnMutexStrategyType<hwi::device::PsOnPin>;
        pub type PsOnController = hwa::StaticController<_PsOnMutexStrategy_>;
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
        pub type LaserController = hwa::StaticController<_LaserControllerMutexStrategy_>;
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
        pub type ProbeController = hwa::StaticController<_ProbeControllerMutexStrategy_>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-hot-end")] {
        //hwa::StaticController<hwa::HotBedControllerMutexStrategyType<hwa::controllers::HotBedController>>
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
        pub type HotEndController = hwa::StaticController<_HotEndControllerMutexStrategy_>;


    }
}
