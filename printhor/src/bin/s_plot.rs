extern crate alloc;
extern crate core;

pub mod control;
pub mod helpers;
pub mod hwa;

use crate::hwa::controllers;
use hwa::HwiContract;
#[allow(unused)]
use hwa::RawHwiResource;

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    printhor_main(spawner, false).await;
}

pub fn initialization_error() {
    let msg = "Unable to start because SYS_ALARM raised at startup. Giving up...";
    hwa::error!("{}", msg);
    panic!("{}", msg);
}

/// A minimal context for s-plot
struct SplotContext {
    #[allow(unused)]
    pub event_bus: hwa::types::EventBus,
    pub motion_planner: hwa::controllers::MotionPlanner,
}

async fn printhor_main(spawner: embassy_executor::Spawner, _keep_feeding: bool) {
    hwa::Contract::init_logger();
    hwa::Contract::init_heap();

    let c = init_splot(spawner).await;

    let event_bus = c.event_bus;
    let motion_planner = c.motion_planner;

    motion_planner.start(&event_bus).await;

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {

            // Max speed
            motion_planner.motion_config().set_max_speed(hwa::Contract::DEFAULT_MAX_SPEED_PS);

            motion_planner.motion_config().set_max_accel(hwa::Contract::DEFAULT_MAX_ACCEL_PS);

            motion_planner.motion_config().set_max_jerk(hwa::Contract::DEFAULT_MAX_JERK_PS);

            motion_planner.motion_config().set_default_travel_speed(hwa::Contract::DEFAULT_TRAVEL_SPEED_PS);

            motion_planner.motion_config().set_space_units_per_world_unit(hwa::Contract::DEFAULT_UNITS_PER_WU);

            motion_planner.motion_config().set_micro_steps_per_axis(
                hwa::Contract::DEFAULT_MICRO_STEPS_PER_AXIS
            );

            motion_planner.motion_config().set_world_center(
                hwa::Contract::DEFAULT_WORLD_CENTER_WU
            );

            motion_planner.motion_config().set_world_size(
                hwa::Contract::DEFAULT_WORLD_SIZE_WU
            );

            motion_planner.motion_config().set_nozzle_offset(
                hwa::make_vector_real!(x=0.0, y=0.0,z=0.0)
            );

            motion_planner.motion_config().set_probe_offset(
                hwa::make_vector_real!(x=0.0, y=0.0,z=0.0)
            );

            motion_planner.motion_config().set_flow_rate(100);
            motion_planner.motion_config().set_speed_rate(100);

            // Compute min speed. Really useful because of discretion effects
            motion_planner.motion_config().compute_min_speed();


            // Make homing unneeded
            hwa::info!("Virtually homing");
            {
                let position = hwa::controllers::Position::new_with_world_projection(
                    &hwa::Contract::DEFAULT_WORLD_HOMING_POINT_WU,
                );
                motion_planner
                    .motion_status()
                    .update_last_planned_position(0, &position);
                motion_planner
                    .motion_status()
                    .update_current_position(0, &position);
            }
            #[cfg(all(feature = "native", feature = "with-ps-on"))]
            {
                hwa::info!("Virtually powering on");
                event_bus.publish_event(hwa::EventStatus::containing(hwa::EventFlags::ATX_ON)).await;
            }
        }
    }

    // Starting motion logic

    //hwa::info!("Segment from [0, 0, 0] to [1, 0, 0] at max mm/s");
    let _r = motion_planner
        .plan(
            hwa::CommChannel::Internal,
            &control::GCodeCmd::new(
                1,
                Some(1),
                control::GCodeValue::G0(hwa::make_vector_real!(x = 1.0).into()),
            ),
            false,
            &event_bus,
        )
        .await;

    //hwa::info!("Segment from [1, 0, 0] to [2, 0, 0] at max mm/s");
    let _r = motion_planner
        .plan(
            hwa::CommChannel::Internal,
            &control::GCodeCmd::new(
                2,
                Some(2),
                control::GCodeValue::G0(hwa::make_vector_real!(x = 2.0).into()),
            ),
            false,
            &event_bus,
        )
        .await;

    //hwa::info!("Segment from [2, 0, 0] to [3, 0, 0] at max mm/s");
    let _r = motion_planner
        .plan(
            hwa::CommChannel::Internal,
            &control::GCodeCmd::new(
                3,
                Some(3),
                control::GCodeValue::G0(hwa::make_vector_real!(x = 3.0).into()),
            ),
            false,
            &event_bus,
        )
        .await;

    let _r = motion_planner
        .plan(
            hwa::CommChannel::Internal,
            &control::GCodeCmd::new(
                4,
                Some(4),
                control::GCodeValue::G0(hwa::make_vector_real!(x = 4.0).into()),
            ),
            false,
            &event_bus,
        )
        .await;

    //hwa::info!("Segment from [4, 0, 0] to [5.0, 0.0, 0] at max mm/s");
    let _r = motion_planner
        .plan(
            hwa::CommChannel::Internal,
            &control::GCodeCmd::new(
                5,
                Some(5),
                control::GCodeValue::G0(hwa::make_vector_real!(x = 5.0).into()),
            ),
            false,
            &event_bus,
        )
        .await;

    //let mut total_disp = hwa::math::Real::zero();
    //let mut total_disp_discrete = hwa::math::Real::zero();
    //let mut s_id = 0;
    loop {
        match motion_planner.next_plan(&event_bus).await {
            controllers::motion::ExecPlan::Segment(_segment, _channel, _order_num) => {
                hwa::info!("Segment {:?}", _order_num);
            }
            _ => {}
        }
        motion_planner.consume_current_plan(&event_bus).await;
        if motion_planner.num_queued().await == 0 {
            break;
        }
    }

    #[cfg(not(test))]
    std::process::exit(0);
}

async fn init_splot(spawner: embassy_executor::Spawner) -> SplotContext {
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

#[cfg(feature = "s-plot-bin")]
#[cfg(test)]
mod test {
    use super::*;
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use printhor_hwa_common::PersistentState;
    use std::marker::PhantomData;
    use std::sync::{Condvar, Mutex};

    struct Signaler {
        mutex: Mutex<bool>,
        condvar: Condvar,
    }

    impl Signaler {
        fn new() -> Self {
            Self {
                mutex: Mutex::new(false),
                condvar: Condvar::new(),
            }
        }

        fn wait(&self) {
            let mut signaled = self.mutex.lock().unwrap();
            while !*signaled {
                signaled = self.condvar.wait(signaled).unwrap();
            }
            *signaled = false;
        }
    }

    pub static SPLOT_STATE: PersistentState<CriticalSectionRawMutex, bool> = PersistentState::new();

    pub struct MockedExecutor {
        inner: embassy_executor::raw::Executor,
        not_send: PhantomData<*mut ()>,
        signaler: &'static Signaler,
    }

    impl MockedExecutor {
        /// Create a new Executor.
        pub fn new() -> Self {
            let signaler = Box::leak(Box::new(Signaler::new()));
            Self {
                inner: embassy_executor::raw::Executor::new(signaler as *mut Signaler as *mut ()),
                not_send: PhantomData,
                signaler,
            }
        }

        pub fn run(&'static mut self, init: impl FnOnce(embassy_executor::Spawner)) {
            init(self.inner.spawner());

            loop {
                unsafe { self.inner.poll() };
                if SPLOT_STATE.signaled() {
                    break;
                }
                self.signaler.wait()
            }
        }
    }

    /// The integration test entry-point
    #[embassy_executor::task(pool_size = 1)]
    async fn mocked_splot_main(spawner: embassy_executor::Spawner) {
        printhor_main(spawner, false).await;
        SPLOT_STATE.signal(true);
    }

    #[test]
    fn splot_test() {
        let executor = hwa::make_static_ref!("Executor", MockedExecutor, MockedExecutor::new());
        // 2. Spawn the [mocked_main] task
        executor.run(|spawner| {
            let _tk = spawner.must_spawn(mocked_splot_main(spawner));
        });
    }
}
