#![allow(stable_features)]
#![cfg_attr(not(feature = "native"), no_std)]
#![cfg_attr(not(feature = "native"), no_main)]
#![doc = include_str!("../../../design/architecture.md")]

extern crate alloc;
extern crate core;
pub mod control;
pub mod helpers;
pub mod hwa;
pub mod math;

#[allow(unused)]
use hwa::Contract;
use hwa::HwiContract;
#[allow(unused)]
use hwa::RawHwiResource;

//noinspection RsUnresolvedReference
#[cfg_attr(doc, aquamarine::aquamarine)]
///! Program entry point
#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    printhor_main(spawner, true).await;
    unreachable!("Main should not end");
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
    let context = hwa::Contract::init(spawner).await;

    hwa::info!(
        "HWI setup completed. Allocated {} bytes for context.",
        core::mem::size_of_val(&context)
    );

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

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "with-motion", feature = "with-hot-end", feature = "with-hot-bed"))] {
            type DeferChannelMutexType = <hwa::Contract as HwiContract>::DeferChannelMutexType;
            let defer_channel: hwa::GenericDeferChannel<DeferChannelMutexType> = {
                type DeferChannelMutexType = <hwa::Contract as HwiContract>::DeferChannelMutexType;
                hwa::GenericDeferChannel::new(hwa::make_static_ref!(
                    "DeferChannel",
                    hwa::DeferChannelChannelType<DeferChannelMutexType>,
                    hwa::DeferChannelChannelType::new()
                ))
            };
        }
    }

    let sys_watch_dog = context.sys_watch_dog.clone();
    sys_watch_dog.lock().await.unleash();

    //#endregion

    //#region "Task Spawning"

    if spawn_tasks(
        spawner,
        event_bus.clone(),
        #[cfg(any(
            feature = "with-motion",
            feature = "with-hot-end",
            feature = "with-hot-bed"
        ))]
        defer_channel,
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
    //#enregion
}

async fn spawn_tasks(
    spawner: embassy_executor::Spawner,
    event_bus: hwa::types::EventBus,
    #[cfg(any(
        feature = "with-motion",
        feature = "with-hot-end",
        feature = "with-hot-bed"
    ))]
    _defer_channel: hwa::types::DeferChannel,
    _context: hwa::HwiContext<hwa::Contract>,
    _wd: hwa::types::WatchDogController,
) -> Result<(), ()> {
    #[cfg(feature = "with-print-job")]
    let printer_controller = hwa::controllers::PrinterController::new(event_bus.clone());

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

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-hot-end")] {
            let hot_end_controller: hwa::types::HotEndController = hwa::make_static_async_controller!(
                "HotEndController",
                hwa::types::HotEndControllerMutexStrategy,
                hwa::controllers::HeaterController::new(
                    hwa::types::HotEndAdcController::new(
                        _context.hot_end_adc,
                        _context.hot_end_adc_pin.take(),
                        <Contract as HwiContract>::HOT_END_ADC_V_REF_DEFAULT_SAMPLE,
                    ),
                    hwa::types::HotEndPwmController::new(
                        _context.hot_end_pwm,
                        _context.hot_end_pwm_channel.take(),
                    ),
                    <Contract as HwiContract>::HOT_END_THERM_BETA,
                    <Contract as HwiContract>::HOT_END_THERM_NOMINAL_RESISTANCE,
                    <Contract as HwiContract>::HOT_END_THERM_PULL_UP_RESISTANCE,
                    _defer_channel.clone(),
                    hwa::DeferAction::HotEndTemperature,
                    hwa::EventFlags::HOT_END_TEMP_OK,
                )
            );
            hot_end_controller.lock().await
                .init(<Contract as HwiContract>::HOT_END_ADC_V_REF_DEFAULT_MV).await;
        }
    }
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-hot-bed")] {
            let hot_bed_controller: hwa::types::HotBedController = hwa::make_static_async_controller!(
                "HotBedController",
                hwa::types::HotBedControllerMutexStrategy,
                hwa::controllers::HeaterController::new(
                    hwa::types::HotBedAdcController::new(
                        _context.hot_bed_adc,
                        _context.hot_bed_adc_pin.take(),
                        <Contract as HwiContract>::HOT_BED_ADC_V_REF_DEFAULT_SAMPLE,
                    ),
                    hwa::types::HotBedPwmController::new(
                        _context.hot_bed_pwm,
                        _context.hot_bed_pwm_channel.take(),
                    ),
                    <Contract as HwiContract>::HOT_BED_THERM_BETA,
                    <Contract as HwiContract>::HOT_BED_THERM_NOMINAL_RESISTANCE,
                    <Contract as HwiContract>::HOT_BED_THERM_PULL_UP_RESISTANCE,
                    _defer_channel.clone(),
                    hwa::DeferAction::HotBedTemperature,
                    hwa::EventFlags::HOT_BED_TEMP_OK,
                )
            );
            hot_bed_controller.lock().await
                .init(<Contract as HwiContract>::HOT_BED_ADC_V_REF_DEFAULT_MV).await;
        }
    }

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

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-sd-card")] {

            type SDManager = hwa::AsyncStandardStrategy<
                    hwa::AsyncNoopMutexType,
                    hwa::sd_card::SDStateManager<
                        <Contract as HwiContract>::SDCardBlockDevice,
                        {<Contract as HwiContract>::SD_CARD_MAX_DIRS},
                        {<Contract as HwiContract>::SD_CARD_MAX_FILES},
                        {<Contract as HwiContract>::SD_CARD_MAX_DIRS + <Contract as HwiContract>::SD_CARD_MAX_FILES},
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


            /*

            let mut sd_card_controller: hwa::types::SDCardController = {
                hwa::make_static_async_controller!(
                    "SDCardController",
                    hwa::types::SDCardControllerMutexStrategy,
                    c
                )
            };

            let cc = sd_card_controller.lock().await;
            cc.list_dir("/").await;

             */
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {
            let motion_pins = hwa::controllers::MotionPins::new(hwa::make_static_sync_controller!(
                "MotionPins",
                hwa::types::MotionPinsMutexStrategy,
                _context.motion_pins,
            ));
            let motion_config = hwa::controllers::MotionConfig::new(hwa::make_static_sync_controller!(
                "MotionConfig",
                hwa::types::MotionConfigMutexStrategy,
                hwa::controllers::MotionConfigContent::new()
            ));
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
                        motion_pins,
                        #[cfg(feature = "with-trinamic")]
                        motion_config.clone(),
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

    let processor: hwa::GCodeProcessor = hwa::GCodeProcessor::new(
        event_bus.clone(),
        #[cfg(feature = "with-serial-usb")]
        _context.serial_usb_tx,
        #[cfg(feature = "with-serial-port-1")]
        _context.serial_port1_tx,
        #[cfg(feature = "with-serial-port-2")]
        _context.serial_port2_tx,
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

    #[cfg(feature = "with-motion")]
    {
        motion_config.set_max_speed(math::TVector::from_coords(
            Some(600),
            Some(600),
            Some(100),
            Some(600),
        ));
        motion_config.set_max_accel(math::TVector::from_coords(
            Some(9800),
            Some(9800),
            Some(3200),
            Some(9800),
        ));
        motion_config.set_max_jerk(math::TVector::from_coords(
            Some(19600),
            Some(19600),
            Some(6400),
            Some(19600),
        ));
        motion_config.set_default_travel_speed(600);
        // Homing unneeded
        motion_status.set_last_planned_position(&math::TVector::zero());

        cfg_if::cfg_if! {
            if #[cfg(feature = "native")] {
                motion_config.set_units_per_mm(math::Real::new(10, 0), math::Real::new(10, 0), math::Real::new(50, 0), math::Real::new(50, 0));
                motion_config.set_micro_steps_per_axis(8, 8, 8, 8);
            }
            else {
                motion_config.set_units_per_mm(math::Real::new(10, 0), math::Real::new(10, 0), math::Real::new(50, 0), math::Real::new(50, 0));
                motion_config.set_micro_steps_per_axis(8, 8, 8, 8);
            }
        }

        motion_config.set_machine_bounds(200, 200, 200);
        motion_config.set_flow_rate(100);
        motion_config.set_speed_rate(100);

        spawner
            .spawn(control::task_stepper::task_stepper(
                event_bus.clone(),
                processor.motion_planner.clone(),
                _wd,
            ))
            .map_err(|_| ())?;
    }

    #[cfg(any(test, feature = "integration-test"))]
    spawner
        .spawn(control::task_integration::task_integration(
            processor.clone(),
            #[cfg(feature = "with-sd-card")]
            sd_card_controller.clone(),
            #[cfg(feature = "with-print-job")]
            printer_controller.clone(),
        ))
        .map_err(|_| ())?;

    hwa::info!("HWA setup completed. Spawning tasks...");
    spawner
        .spawn(control::task_control::task_control(
            processor.clone(),
            control::GCodeMultiplexedInputStream::new(
                #[cfg(feature = "with-serial-usb")]
                _context.serial_usb_rx_stream,
                #[cfg(feature = "with-serial-port-1")]
                _context.serial_port1_rx_stream,
                #[cfg(feature = "with-serial-port-2")]
                _context.serial_port2_rx_stream,
            ),
            #[cfg(feature = "with-print-job")]
            printer_controller.clone(),
            #[cfg(feature = "with-sd-card")]
            sd_card_controller.clone(),
        ))
        .map_err(|_| ())?;

    #[cfg(feature = "with-print-job")]
    spawner
        .spawn(control::task_print_job::task_print_job(
            processor.clone(),
            printer_controller,
            sd_card_controller,
        ))
        .map_err(|_| ())?;

    #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
    spawner
        .spawn(control::task_temperature::task_temperature(
            event_bus.clone(),
            #[cfg(feature = "with-hot-end")]
            hot_end_controller,
            #[cfg(feature = "with-hot-bed")]
            hot_bed_controller,
        ))
        .map_err(|_| ())?;

    #[cfg(any(
        feature = "with-motion",
        feature = "with-hot-end",
        feature = "with-hot-bed"
    ))]
    spawner
        .spawn(control::task_defer::task_defer(processor, _defer_channel))
        .map_err(|_| ())?;

    Ok(())
}

pub fn initialization_error() {
    let msg = "Unable to start because SYS_ALARM raised at startup. Giving up...";
    hwa::error!("{}", msg);
    panic!("{}", msg);
}
cfg_if::cfg_if! {
    if #[cfg(not(feature = "native"))] {
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-defmt")] {
                #[allow(unused)]
                #[cfg(feature = "with-defmt")]
                use defmt_rtt as _;
                #[allow(unused)]
                #[cfg(feature = "with-defmt")]
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
