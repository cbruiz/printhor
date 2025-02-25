#![allow(stable_features)]
#![cfg_attr(not(feature = "native"), no_std)]
#![cfg_attr(not(feature = "native"), no_main)]
#![doc = include_str!("../../../design/architecture.md")]

extern crate alloc;
extern crate core;
pub mod control;
pub mod helpers;
pub mod hwa;

#[allow(unused)]
use hwa::{Contract, HwiContract, RawHwiResource};

use hwa::math;
#[allow(unused)]
use math::{Real, TVector};
use printhor_hwa_common::CoordSel;

//noinspection RsUnresolvedReference
#[cfg_attr(doc, aquamarine::aquamarine)]
///! Program entry point
#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    printhor_main(spawner, true).await;
}

pub async fn printhor_main(spawner: embassy_executor::Spawner, keep_feeding: bool) {
    hwa::Contract::init_logger();
    hwa::info!("Log ready. Starting...");

    hwa::Contract::init_heap();
    hwa::info!(
        "Initializing {} {}",
        hwa::Contract::FIRMWARE_NAME,
        hwa::Contract::FIRMWARE_VERSION
    );

    let wdt = match sys_start(spawner).await {
        Ok(wdt) => wdt,
        Err(_) => {
            hwa::error!("Error starting system");
            panic!("Unable to start");
        }
    };

    // The core of the main function is just to periodically feed watchdog
    if keep_feeding {
        let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_secs(2));
        loop {
            {
                wdt.lock().await.pet();
            }
            ticker.next().await;
        }
    }
}

async fn sys_start(
    spawner: embassy_executor::Spawner,
) -> Result<hwa::types::WatchDogController, ()> {
    //#region "Init contract"

    let context = hwa::Contract::init(spawner).await;

    hwa::info!(
        "HWI setup completed. Allocated {} bytes for context.",
        core::mem::size_of_val(&context)
    );
    hwa::info!(
        "Vector<Real> size: {} bytes",
        core::mem::size_of::<TVector<Real>>()
    );

    //#endregion

    //#region "Setup event bus"

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

    //#endregion

    //#region "Setup defer channel (if hotends or motion are enabled)"

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "with-motion", feature = "with-hot-end", feature = "with-hot-bed"))] {
            type DeferChannelMutexType = <hwa::Contract as HwiContract>::DeferChannelMutexType;
            let defer_channel: hwa::GenericDeferChannel<DeferChannelMutexType> = {
                hwa::GenericDeferChannel::new(hwa::make_static_ref!(
                    "DeferChannel",
                    hwa::DeferChannelChannelType<DeferChannelMutexType>,
                    hwa::DeferChannelChannelType::new()
                ))
            };
        }
    }

    //#endregion

    //#region "Setup motion broadcast channel (if with-motion-broadcast is enabled)"

    cfg_if::cfg_if! {
        if #[cfg(all(feature = "with-motion", feature = "with-motion-broadcast"))] {
            type MotionBroadcastChannelMutexType = <hwa::Contract as HwiContract>::MotionBroadcastChannelMutexType;
            let motion_broadcast_channel: hwa::GenericMotionBroadcastChannel<MotionBroadcastChannelMutexType> = {
                hwa::GenericMotionBroadcastChannel::new(hwa::make_static_ref!(
                    "MotionBroadcastChannel",
                    hwa::MotionBroadcastChannelType<MotionBroadcastChannelMutexType>,
                    hwa::MotionBroadcastChannelType::new()
                ))
            };
        }
    }

    //#endregion

    //#region "Setup System WatchDog"

    let sys_watch_dog = context.sys_watch_dog.clone();
    sys_watch_dog.lock().await.unleash();

    //#endregion

    //#region "Init controllers and spawn Tasks"
    cfg_if::cfg_if! {
        if #[cfg(any(feature="with-motion-stepper", feature="with-motion-broadcast"))] {
            Contract::setup_ticker()
        }
    }
    //#endregion

    //#region "Init controllers and spawn Tasks"

    if init_controllers_and_spawn_tasks(
        spawner,
        event_bus.clone(),
        #[cfg(any(
            feature = "with-motion",
            feature = "with-hot-end",
            feature = "with-hot-bed"
        ))]
        defer_channel,
        #[cfg(all(feature = "with-motion", feature = "with-motion-broadcast"))]
        motion_broadcast_channel,
        context,
        sys_watch_dog.clone(),
    )
    .await
    .is_ok()
    {
        event_bus
            .publish_event(hwa::EventStatus::not_containing(
                hwa::EventFlags::SYS_BOOTING,
            ))
            .await;

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion")] {
                let mut event_suscriber = event_bus.subscriber().await;
                if event_suscriber.ft_wait_until(hwa::EventFlags::MOV_QUEUE_EMPTY).await.is_err() {
                    initialization_error()
                }
            }
        }

        hwa::info!(
            "Tasks spawned. Allocated {} bytes for shared state. Firing SYS_READY.",
            hwa::Contract::stack_reservation_current_size(),
        );
        event_bus
            .publish_event(hwa::EventStatus::containing(hwa::EventFlags::SYS_READY))
            .await;
        Ok(sys_watch_dog)
    } else {
        event_bus
            .publish_event(hwa::EventStatus::containing(
                hwa::EventFlags::SYS_BOOT_FAILURE,
            ))
            .await;
        hwa::error!("Unable start. Any task launch failed");
        Err(())
    }

    //#endregion
}

async fn init_controllers_and_spawn_tasks(
    spawner: embassy_executor::Spawner,
    event_bus: hwa::types::EventBus,
    #[cfg(any(
        feature = "with-motion",
        feature = "with-hot-end",
        feature = "with-hot-bed"
    ))]
    _defer_channel: hwa::types::DeferChannel,
    #[cfg(all(feature = "with-motion", feature = "with-motion-broadcast"))]
    _motion_broadcast_channel: hwa::types::MotionBroadcastChannel,
    _context: hwa::HwiContext<hwa::Contract>,
    _wd: hwa::types::WatchDogController,
) -> Result<(), ()> {
    //#region "Init printer controller (if with-print-job is set)"

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-print-job")] {
            let printer_controller = hwa::controllers::PrinterController::new(event_bus.clone());
        }
    }

    //#endregion

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

    //#region "Init SD-Card controller (if with-sd-card is set)"

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-sd-card")] {

            type SDManager = hwa::AsyncStandardStrategy<
                    hwa::AsyncNoopMutexType,
                    hwa::sd_card::SDStateManager<
                        <hwa::Contract as HwiContract>::SDCardBlockDevice,
                        {<hwa::Contract as HwiContract>::SD_CARD_MAX_DIRS},
                        {<hwa::Contract as HwiContract>::SD_CARD_MAX_FILES},
                        {<hwa::Contract as HwiContract>::SD_CARD_MAX_DIRS + <hwa::Contract as HwiContract>::SD_CARD_MAX_FILES},
                    >
                >;

            let card = hwa::sd_card::SDStateManager::new(
                _context.sd_card_block_device, 0
            ).await;

            let sd_card_controller = hwa::controllers::GenericSDCardController::new(
                hwa::make_static_async_controller!(
                    "SDCardManager",
                    SDManager,
                    card
                )
            ).await;
        }
    }

    //#endregion

    //#region "Init motion Controllers (if with-motion is set)"

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
            
            step_actuator.disable_steppers(CoordSel::all_axis());
            step_actuator.set_forward_direction(CoordSel::all_axis(), CoordSel::all_axis());
            
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

    //#endregion

    //#region "Init Gcode processor"

    let processor: hwa::GCodeProcessor = hwa::GCodeProcessor::new(
        event_bus.clone(),
        #[cfg(feature = "with-serial-usb")]
        _context.serial_usb_tx,
        #[cfg(feature = "with-serial-port-1")]
        _context.serial_port_1_tx,
        #[cfg(feature = "with-serial-port-2")]
        _context.serial_port_2_tx,
        #[cfg(feature = "with-motion")]
        motion_planner,
        #[cfg(feature = "with-ps-on")]
        _context.ps_on,
        #[cfg(feature = "with-probe")]
        probe_controller,
        #[cfg(feature = "with-hot-end")]
        hot_end_controller.clone(),
        #[cfg(feature = "with-hot-bed")]
        hot_bed_controller.clone(),
        #[cfg(feature = "with-fan-layer")]
        fan_layer_controller.clone(),
        #[cfg(feature = "with-fan-extra-1")]
        fan_extra1_controller.clone(),
        #[cfg(feature = "with-laser")]
        laser_controller,
    );

    //#endregion

    //#region "Configure Motion defaults (if with-motion is set)"

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {

            // Max speed
            motion_config.set_max_speed(hwa::Contract::DEFAULT_MAX_SPEED_PS);

            motion_config.set_max_accel(hwa::Contract::DEFAULT_MAX_ACCEL_PS);

            motion_config.set_max_jerk(hwa::Contract::DEFAULT_MAX_JERK_PS);

            motion_config.set_default_travel_speed(hwa::Contract::DEFAULT_TRAVEL_SPEED_PS);

            motion_config.set_space_units_per_world_unit(hwa::Contract::DEFAULT_UNITS_PER_WU);

            motion_config.set_micro_steps_per_axis(
                hwa::Contract::DEFAULT_MICRO_STEPS_PER_AXIS
            );

            motion_config.set_world_center(
                hwa::Contract::DEFAULT_WORLD_CENTER_WU
            );

            motion_config.set_world_size(
                hwa::Contract::DEFAULT_WORLD_SIZE_WU
            );

            motion_config.set_nozzle_offset(
                hwa::make_vector_real!(x=0.0, y=0.0,z=0.0)
            );

            motion_config.set_probe_offset(
                hwa::make_vector_real!(x=0.0, y=0.0,z=0.0)
            );

            motion_config.set_flow_rate(100);
            motion_config.set_speed_rate(100);

            // Compute min speed. Really useful because of discretion effects
            motion_config.compute_min_speed();


            // Make homing unneeded
            hwa::warn!("Virtually homing");
            {
                let pos = hwa::controllers::Position::new_with_world_projection(
                    &Contract::DEFAULT_WORLD_HOMING_POINT_WU
                );
                motion_status.update_last_planned_position(0, &pos);
                motion_status.update_current_position(0, &pos);
            }
            #[cfg(all(feature = "native", feature = "with-ps-on"))]
            {
                hwa::warn!("Virtually powering on");
                event_bus.publish_event(hwa::EventStatus::containing(hwa::EventFlags::ATX_ON)).await;
            }
        }
    }

    //#endregion

    hwa::info!("HWA setup completed. Spawning tasks...");

    //#region "Spawn print-job task (if with-print-job is set)"

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-print-job")] {
            spawner.spawn(
                control::task_print_job::task_print_job(
                    processor.clone(),
                    printer_controller.clone(),
                    sd_card_controller.clone(),
                )
            ).map_err(|_| ())?;
        }
    }

    //#endregion

    //#region "Spawn motion tasks (if with-motion and with-motion-broadcast are set)"

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {
            spawner.spawn(
                control::task_stepper::task_stepper(
                    event_bus.clone(),
                    processor.motion_planner.clone(),
                    _wd,
                )
            ).map_err(|_| ())?;

            cfg_if::cfg_if! {
                if #[cfg(feature = "with-motion-broadcast")] {
                    hwa::Contract::launch_high_priotity(
                        _context.high_priority_core,
                        control::task_motion_broadcast::task_motion_broadcast(
                            _motion_broadcast_channel,
                            motion_config,
                            _context.motion_sender
                        )
                    ).map_err(|_| ())?;
                }
            }
        }
    }

    //#endregion

    //#region "Spawn temperature task (if with-hot-end or with-hot-bed are set)"

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))] {
            spawner.spawn(
                control::task_temperature::task_temperature(
                    event_bus.clone(),
                    #[cfg(feature = "with-hot-end")]
                    hot_end_controller,
                    #[cfg(feature = "with-hot-bed")]
                    hot_bed_controller,
                )
            ).map_err(|_| ())?;
        }
    }

    //#endregion

    //#region "Spawn defer-event task (if motion or temperature features are set)"

    cfg_if::cfg_if! {
        if #[cfg(any(
            feature = "with-motion", feature = "with-hot-end", feature = "with-hot-bed"
        ))] {
            spawner.spawn(
                control::task_defer::task_defer(processor.clone(), _defer_channel)
            ).map_err(|_| ())?;
        }
    }

    //#endregion

    //#region "Spawn control task"

    spawner
        .spawn(control::task_control::task_control(
            processor.clone(),
            control::GCodeMultiplexedInputStream::new(
                #[cfg(feature = "with-serial-usb")]
                _context.serial_usb_rx_stream,
                #[cfg(feature = "with-serial-port-1")]
                _context.serial_port_1_rx_stream,
                #[cfg(feature = "with-serial-port-2")]
                _context.serial_port_2_rx_stream,
            ),
            #[cfg(feature = "with-print-job")]
            printer_controller.clone(),
            #[cfg(feature = "with-sd-card")]
            sd_card_controller.clone(),
        ))
        .map_err(|_| ())?;

    //#endregion

    //#region "Spawn integration task (if integration-test is set)"

    cfg_if::cfg_if! {
        if #[cfg(any(test, feature = "integration-test"))] {
            spawner.spawn(
                control::task_integration::task_integration(
                    processor.clone(),
                    #[cfg(feature = "with-sd-card")]
                    sd_card_controller.clone(),
                    #[cfg(feature = "with-print-job")]
                    printer_controller.clone()
                )
            ).map_err(|_| ())?;
        }
    }

    //#endregion

    Ok(())
}

pub fn initialization_error() {
    let msg = "Unable to start because SYS_ALARM raised at startup. Giving up...";
    hwa::error!("{}", msg);
    panic!("{}", msg);
}

//#region "Panic handling"

cfg_if::cfg_if! {
    if #[cfg(not(feature = "native"))] {
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-defmt")] {
                #[allow(unused)]
                use defmt_rtt as _;
                #[allow(unused)]
                use panic_probe as _;
            }
            else {
                #[panic_handler]
                fn panic(_info: &core::panic::PanicInfo) -> ! {
                    loop {}
                }
            }
        }
    }
}

//#endregion
