//! A minimal context for s-plot

use crate::hwa;
use hwa::HwiContract;

#[allow(unused)]
use hwa::RawHwiResource;

pub(crate) struct SplotContext {
    #[allow(unused)]
    pub event_bus: hwa::types::EventBus,
    pub motion_planner: hwa::controllers::MotionPlanner,
}

pub(crate) async fn init_splot(spawner: embassy_executor::Spawner) -> SplotContext {
    type EventBusMutexStrategyType = <hwa::Contract as HwiContract>::EventBusMutexStrategy;
    type EventBusPubSubMutexType = <hwa::Contract as HwiContract>::EventBusPubSubMutexType;

    let event_bus: hwa::GenericEventBus<EventBusMutexStrategyType, EventBusPubSubMutexType> =
        hwa::GenericEventBus::new(hwa::make_static_async_controller!(
            "EventBus",
            EventBusMutexStrategyType,
            hwa::EventBusChannelController::new(hwa::make_static_ref!(
                "EventBusChannel",
                hwa::EventBusPubSubType<EventBusPubSubMutexType>,
                hwa::EventBusPubSubType::new(),
            )),
        ));

    event_bus
        .publish_event(hwa::EventStatus::containing(hwa::EventFlags::SYS_BOOTING))
        .await;

    let _context = hwa::Contract::init(spawner).await;

    //#region "Setup defer channel (if hotends or motion are enabled)"

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "with-motion", feature = "with-hot-end", feature = "with-hot-bed"))] {
            type DeferChannelMutexType = <hwa::Contract as HwiContract>::DeferChannelMutexType;
            let _defer_channel: hwa::GenericDeferChannel<DeferChannelMutexType> = {
                hwa::GenericDeferChannel::new(hwa::make_static_ref!(
                    "DeferChannel",
                    hwa::DeferChannelChannelType<DeferChannelMutexType>,
                    hwa::DeferChannelChannelType::new()
                ))
            };
        }
    }

    //#endregion

    cfg_if::cfg_if! {
        if #[cfg(all(feature = "with-motion", feature = "with-motion-broadcast"))] {
            type MotionBroadcastChannelMutexType = <hwa::Contract as HwiContract>::MotionBroadcastChannelMutexType;
            let _motion_broadcast_channel: hwa::GenericMotionBroadcastChannel<MotionBroadcastChannelMutexType> = {
                hwa::GenericMotionBroadcastChannel::new(hwa::make_static_ref!(
                    "MotionBroadcastChannel",
                    hwa::MotionBroadcastChannelType<MotionBroadcastChannelMutexType>,
                    hwa::MotionBroadcastChannelType::new()
                ))
            };
        }
    }

    //#region "Init Probe controller (if with-probe is set)"

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-probe")] {

            let probe_controller = hwa::make_static_async_controller!(
                "ProbeController",
                hwa::types::ProbeControllerMutexStrategy,
                hwa::types::InnerProbeController::new(
                    _context.probe_pwm,
                    _context.probe_pwm_channel.take(),
                )
            );
        }
    }

    //#endregion

    //#region "Init HotEnd controller (if with-hot-end is set)"

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-hot-end")] {
            let hot_end_controller: hwa::types::HotEndController = hwa::make_static_async_controller!(
                "HotEndController",
                hwa::types::HotEndControllerMutexStrategy,
                hwa::controllers::HeaterController::new(
                    hwa::types::HotEndAdcController::new(
                        _context.hot_end_adc,
                        _context.hot_end_adc_pin.take(),
                        <hwa::Contract as HwiContract>::HOT_END_ADC_V_REF_DEFAULT_SAMPLE,
                    ),
                    hwa::types::HotEndPwmController::new(
                        _context.hot_end_pwm,
                        _context.hot_end_pwm_channel.take(),
                    ),
                    <hwa::Contract as HwiContract>::HOT_END_THERM_BETA,
                    <hwa::Contract as HwiContract>::HOT_END_THERM_NOMINAL_RESISTANCE,
                    <hwa::Contract as HwiContract>::HOT_END_THERM_PULL_UP_RESISTANCE,
                    _defer_channel.clone(),
                    hwa::DeferAction::HotEndTemperature,
                    hwa::EventFlags::HOT_END_TEMP_OK,
                )
            );
            hot_end_controller.lock().await
                .init(<hwa::Contract as HwiContract>::HOT_END_ADC_V_REF_DEFAULT_MV).await;
        }
    }

    //#endregion

    //#region "Init HotBed controller (if with-hot-bed is set)"

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-hot-bed")] {
            let hot_bed_controller: hwa::types::HotBedController = hwa::make_static_async_controller!(
                "HotBedController",
                hwa::types::HotBedControllerMutexStrategy,
                hwa::controllers::HeaterController::new(
                    hwa::types::HotBedAdcController::new(
                        _context.hot_bed_adc,
                        _context.hot_bed_adc_pin.take(),
                        <hwa::Contract as HwiContract>::HOT_BED_ADC_V_REF_DEFAULT_SAMPLE,
                    ),
                    hwa::types::HotBedPwmController::new(
                        _context.hot_bed_pwm,
                        _context.hot_bed_pwm_channel.take(),
                    ),
                    <hwa::Contract as HwiContract>::HOT_BED_THERM_BETA,
                    <hwa::Contract as HwiContract>::HOT_BED_THERM_NOMINAL_RESISTANCE,
                    <hwa::Contract as HwiContract>::HOT_BED_THERM_PULL_UP_RESISTANCE,
                    _defer_channel.clone(),
                    hwa::DeferAction::HotBedTemperature,
                    hwa::EventFlags::HOT_BED_TEMP_OK,
                )
            );
            hot_bed_controller.lock().await
                .init(<hwa::Contract as HwiContract>::HOT_BED_ADC_V_REF_DEFAULT_MV).await;
        }
    }

    //#endregion

    //#region "Init Laser controller (if with-laser is set)"

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-laser")] {

            let laser_controller = hwa::make_static_async_controller!(
                "LaserController",
                hwa::types::LaserControllerMutexStrategy,
                hwa::types::InnerLaserController::new(
                    _context.laser_pwm,
                    _context.laser_pwm_channel.take(),
                )
            );
        }
    }

    //#endregion

    //#region "Init FanLayer controller (if with-fan-layer is set)"

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-fan-layer")] {

            let fan_layer_controller = hwa::make_static_async_controller!(
                "FanLayerController",
                hwa::types::FanLayerControllerMutexStrategy,
                hwa::types::InnerFanLayerController::new(
                    _context.fan_layer_pwm,
                    _context.fan_layer_pwm_channel.take(),
                )
            );
        }
    }

    //#endregion

    //#region "Init FanExtra controller (if with-fan-extra-1 is set)"

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-fan-extra-1")] {

            let fan_extra1_controller = hwa::make_static_async_controller!(
                "FanExtra1Controller",
                hwa::types::FanExtra1ControllerMutexStrategy,
                hwa::types::InnerFanExtra1Controller::new(
                    _context.fan_extra1_pwm,
                    _context.fan_extra1_pwm_channel.take(),
                )
            );
        }
    }

    //#endregion

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {
            let step_actuator = hwa::controllers::StepActuatorController::new(
                hwa::make_static_sync_controller!(
                    "StepActuator",
                    hwa::types::StepActuatorMutexStrategy,
                    _context.motion_pins,
                ),
                #[cfg(feature = "with-motion-broadcast")]
                _motion_broadcast_channel.clone(),
            );

            step_actuator.disable_steppers(hwa::CoordSel::all_axis());
            step_actuator.set_forward_direction(hwa::CoordSel::all_axis(), hwa::CoordSel::all_axis());

            let motion_config = hwa::controllers::MotionConfig::new(hwa::make_static_sync_controller!(
                "MotionConfig",
                hwa::types::MotionConfigMutexStrategy,
                hwa::controllers::MotionConfigContent::new()
            ));
            #[cfg(feature = "with-trinamic")]
            let trinamic_controller = hwa::controllers::TrinamicController::new(
                _context.trinamic_uart, motion_config.clone()
            );
            let motion_status = hwa::controllers::MotionStatus::new(hwa::make_static_sync_controller!(
                "MotionStatus",
                hwa::types::MotionStatusMutexStrategy,
                hwa::controllers::MotionStatusContent::new()
            ));
            let motion_planner = {
                let motion_driver = hwa::make_static_async_controller!(
                    "MotionDriver",
                    hwa::types::MotionDriverMutexStrategy,
                    hwa::drivers::MotionDriver::new(
                        step_actuator,
                        #[cfg(feature = "with-trinamic")]
                        trinamic_controller,
                        #[cfg(feature = "with-probe")]
                        probe_controller.clone(),
                        #[cfg(feature = "with-fan-layer")]
                        fan_layer_controller.clone(),
                        #[cfg(feature = "with-fan-extra-1")]
                        fan_extra1_controller.clone(),
                        #[cfg(feature = "with-laser")]
                        laser_controller.clone(),
                    )
                );

                hwa::controllers::MotionPlanner::new(
                    _defer_channel.clone(),
                    motion_config.clone(),
                    motion_status.clone(),
                    motion_driver,
                )
            };
        }
    }

    SplotContext {
        event_bus,
        motion_planner,
    }
}