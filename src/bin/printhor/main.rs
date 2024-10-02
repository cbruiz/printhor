#![allow(stable_features)]
#![cfg_attr(not(feature = "native"), no_std)]
#![cfg_attr(not(feature = "native"), no_main)]
extern crate alloc;
extern crate core;
pub mod control;
pub mod helpers;
pub mod hwa;
mod hwi;
pub mod machine;
pub mod math;
pub mod tgeo;

pub use tgeo::TVector;

use crate::control::task_control::ControlTaskControllers;
use crate::control::GCodeProcessorParams;
#[cfg(feature = "with-sdcard")]
use crate::hwa::controllers::CardController;
#[cfg(feature = "with-hot-bed")]
use crate::hwa::controllers::HotbedPwmController;
#[cfg(feature = "with-hot-end")]
use crate::hwa::controllers::HotEndPwmController;
#[cfg(feature = "with-print-job")]
use crate::hwa::controllers::PrinterController;
#[cfg(feature = "with-motion")]
use crate::hwa::drivers::MotionDriver;
#[cfg(feature = "with-motion")]
use hwa::controllers::{MotionConfig, MotionPlanner};
use hwa::GCodeProcessor;
use hwa::{Controllers, IODevices, MotionDevices, PwmDevices, SysDevices};

//noinspection RsUnresolvedReference
/// Entry point
#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    printhor_main(spawner, true).await;
    unreachable!("Main should not end");
}

pub async fn printhor_main(spawner: embassy_executor::Spawner, keep_feeding: bool) {
    hwa::init_logger();
    hwa::info!("Init");

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
        let _t0 = embassy_time::Instant::now();
        loop {
            //let _r = event_bus.get_status().await;
            //hwa::info!("STATUS: {:?}", _r);
            //hwa::info!("watchdog feed at {}", _t0.elapsed().as_micros());
            {
                wdt.lock().await.pet();
            }
            ticker.next().await;
        }
    }
}

async fn sys_start(
    spawner: embassy_executor::Spawner,
) -> Result<hwa::StaticController<hwa::WatchdogMutexType, hwa::device::Watchdog>, ()> {
    hwa::info!("Init printhor {}", machine::MACHINE_INFO.firmware_version);
    let peripherals = hwa::init();
    hwa::debug!("Peripherals initialized");

    let context = hwa::setup(spawner, peripherals).await;
    hwa::info!(
        "HWI setup completed. Allocated {} bytes for context.",
        core::mem::size_of_val(&context)
    );

    let event_bus: hwa::EventBusController<hwa::EventbusMutexType, hwa::EventBusChannelMutexType> =
        hwa::EventBusController::new(hwa::make_static_controller!(
            "EventBusChannelController",
            hwa::EventbusMutexType,
            hwa::EventBusChannelController<hwa::EventBusChannelMutexType>,
            hwa::EventBusChannelController::new(hwa::make_static_ref!(
                "EventBusChannel",
                hwa::EventBusPubSubType<hwa::EventBusChannelMutexType>,
                hwa::EventBusPubSubType::new()
            ))
        ));

    event_bus
        .publish_event(hwa::EventStatus::containing(hwa::EventFlags::SYS_BOOTING))
        .await;

    let defer_channel: hwa::DeferChannel<hwa::DeferChannelMutexType> =
        hwa::DeferChannel::new(hwa::make_static_ref!(
            "DeferChannel",
            hwa::DeferChannelChannelType<hwa::DeferChannelMutexType>,
            hwa::DeferChannelChannelType::new()
        ));

    let wdt = context.controllers.sys_watchdog.clone();
    wdt.lock().await.unleash();

    if spawn_tasks(
        spawner,
        event_bus.clone(),
        defer_channel,
        context.controllers,
        context.sys_devices,
        context.io_devices,
        context.motion,
        context.pwm,
        wdt.clone(),
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
            hwa::mem::stack_reservation_current_size(),
        );
        event_bus
            .publish_event(hwa::EventStatus::containing(hwa::EventFlags::SYS_READY))
            .await;
        Ok(wdt)
    } else {
        event_bus
            .publish_event(hwa::EventStatus::containing(
                hwa::EventFlags::SYS_BOOT_FAILURE,
            ))
            .await;
        hwa::error!("Unable start. Any task launch failed");
        Err(())
    }
}

async fn spawn_tasks(
    spawner: embassy_executor::Spawner,
    event_bus: hwa::EventBusController<hwa::EventbusMutexType, hwa::EventBusChannelMutexType>,
    _defer_channel: hwa::DeferChannel<hwa::DeferChannelMutexType>,
    _controllers: Controllers,
    _sys_devices: SysDevices,
    _io_devices: IODevices,
    _motion_device: MotionDevices,
    _pwm_devices: PwmDevices,
    _wd: hwa::StaticController<hwa::WatchdogMutexType, hwa::device::Watchdog>,
) -> Result<(), ()> {
    #[cfg(all(feature = "with-sdcard", feature = "sdcard-uses-spi"))]
    let sdcard_adapter =
        hwa::adapters::SPIAdapter::new(_io_devices.sdcard_device, _io_devices.sdcard_cs_pin);

    #[cfg(all(feature = "with-sdcard", not(feature = "sdcard-uses-spi")))]
    let sdcard_adapter = _io_devices.sdcard_device;

    #[cfg(feature = "with-sdcard")]
    let sdcard_controller = CardController::new(sdcard_adapter).await;

    #[cfg(feature = "with-print-job")]
    let printer_controller = PrinterController::new(event_bus.clone());

    #[cfg(feature = "with-hot-end")]
    let hot_end_pwm = HotEndPwmController::new(
        _pwm_devices.hot_end.power_pwm.clone(),
        _pwm_devices.hot_end.power_channel,
    );
    #[cfg(feature = "with-hot-bed")]
    let hot_bed_pwm = HotbedPwmController::new(
        _pwm_devices.hot_bed.power_pwm.clone(),
        _pwm_devices.hot_bed.power_channel,
    );

    #[cfg(feature = "with-probe")]
    cfg_if::cfg_if! {
        if #[cfg(feature = "with-probe")] {
            type ServoControllerType = hwa::controllers::ServoController<hwa::ProbeMutexType>;
            let probe_controller = hwa::make_static_controller!(
                "ProbeServoController",
                hwa::ServoControllerMutexType,
                ServoControllerType,
                ServoControllerType::new(
                    _pwm_devices.probe.power_pwm,
                    _pwm_devices.probe.power_channel,
                )
            );
        }
    }


    #[cfg(feature = "with-fan-layer")]
    let fan_layer_controller = hwa::make_static_controller!(
        "FanLayerController",
        hwa::FanLayerControllerMutexType,
        hwa::controllers::FanLayerPwmController,
        hwa::controllers::FanLayerPwmController::new(
            _pwm_devices.fan_layer.power_pwm,
            _pwm_devices.fan_layer.power_channel,
        )
    );

    #[cfg(feature = "with-fan-extra-1")]
    let fan_extra_1_controller = hwa::make_static_controller!(
        "FanExtra1Controller",
        hwa::FanExtra1ControllerMutexType,
        hwa::controllers::FanExtra1PwmController,
        hwa::controllers::FanExtra1PwmController::new(
            _pwm_devices.fan_extra_1.power_pwm,
            _pwm_devices.fan_extra_1.power_channel,
        )
    );

    #[cfg(feature = "with-laser")]
    let laser_controller = hwa::make_static_controller!(
        "LaserController",
        hwa::LaserControllerMutexType,
        hwa::controllers::LaserPwmController,
        hwa::controllers::LaserPwmController::new(
            _pwm_devices.laser.power_pwm,
                _pwm_devices.laser.power_channel,
        )
    );

    #[cfg(feature = "with-hot-end")]
    let hot_end_controller = hwa::make_static_controller!(
        "HotEndController",
        hwa::HotEndControllerMutexType,
        hwa::controllers::HotEndController,
        hwa::controllers::HotEndController::new(
            _pwm_devices.hot_end.temp_adc.clone(),
            _pwm_devices.hot_end.temp_pin,
            hot_end_pwm,
            _defer_channel.clone(),
            _pwm_devices.hot_end.thermistor_properties,
        )
    );
    #[cfg(feature = "with-hot-end")]
    hot_end_controller.lock().await.init().await;

    #[cfg(feature = "with-hot-bed")]
    let hot_bed_controller = hwa::make_static_controller!(
        "HotBedController",
        hwa::HotBedControllerMutexType,
        hwa::controllers::HotBedController,
        hwa::controllers::HotBedController::new(
            _pwm_devices.hot_bed.temp_adc.clone(),
            _pwm_devices.hot_bed.temp_pin,
            hot_bed_pwm,
            _defer_channel.clone(),
            _pwm_devices.hot_bed.thermistor_properties,
        )
    );
    #[cfg(feature = "with-hot-bed")]
    hot_bed_controller.lock().await.init().await;

    #[cfg(feature = "with-motion")]
    let motion_planner = {
        let motion_config = hwa::make_static_controller!(
            "MotionConfig",
            hwa::MotionConfigMutexType,
            MotionConfig,
            MotionConfig::new()
        );

        let motion_driver = hwa::make_static_controller!(
            "MotionDriver",
            hwa::MotionDriverMutexType,
            MotionDriver,
            MotionDriver::new(hwa::drivers::MotionDriverParams {
                motion_device: _motion_device.motion_devices,
                #[cfg(feature = "with-trinamic")]
                motion_config: motion_config.clone(),
                #[cfg(feature = "with-probe")]
                probe_controller: probe_controller.clone(),
                #[cfg(feature = "with-fan-layer")]
                fan_layer_controller: fan_layer_controller.clone(),
                #[cfg(feature = "with-fan-extra-1")]
                fan_extra_1_controller: fan_extra_1_controller.clone(),
                #[cfg(feature = "with-laser")]
                laser_controller: laser_controller.clone(),
            })
        );

        MotionPlanner::new(_defer_channel, motion_config, motion_driver)
    };

    let processor: GCodeProcessor = GCodeProcessor::new(GCodeProcessorParams {
        event_bus: event_bus.clone(),
        #[cfg(feature = "with-motion")]
        motion_planner,
        #[cfg(feature = "with-serial-usb")]
        serial_usb_tx: _controllers.serial_usb_tx,
        #[cfg(feature = "with-serial-port-1")]
        serial_port1_tx: _controllers.serial_port1_tx,
        #[cfg(feature = "with-serial-port-2")]
        serial_port2_tx: _controllers.serial_port2_tx,
        #[cfg(feature = "with-ps-on")]
        ps_on: _sys_devices.ps_on,
        #[cfg(feature = "with-probe")]
        probe: probe_controller,
        #[cfg(feature = "with-hot-end")]
        hot_end: hot_end_controller.clone(),
        #[cfg(feature = "with-hot-bed")]
        hot_bed: hot_bed_controller.clone(),
        #[cfg(feature = "with-fan-layer")]
        fan_layer: fan_layer_controller.clone(),
        #[cfg(feature = "with-fan-extra-1")]
        fan_extra_1: fan_extra_1_controller.clone(),
        #[cfg(feature = "with-laser")]
        laser: laser_controller,
    });

    hwa::info!("HWA setup completed.");

    #[cfg(feature = "with-motion")]
    {
        processor
            .motion_planner
            .set_max_speed(tgeo::TVector::from_coords(
                Some(100),
                Some(100),
                Some(50),
                Some(100),
            ))
            .await;
        processor
            .motion_planner
            .set_max_accel(tgeo::TVector::from_coords(
                Some(3000),
                Some(3000),
                Some(100),
                Some(3000),
            ))
            .await;
        processor
            .motion_planner
            .set_max_jerk(tgeo::TVector::from_coords(
                Some(6000),
                Some(6000),
                Some(200),
                Some(6000),
            ))
            .await;
        processor.motion_planner.set_default_travel_speed(100).await;
        // Homing unneeded
        processor
            .motion_planner
            .set_last_planned_pos(&tgeo::TVector::zero())
            .await;

        cfg_if::cfg_if! {
            if #[cfg(feature = "native")] {
                processor.motion_planner.set_steps_per_mm(math::Real::new(10, 0), math::Real::new(10, 0), math::Real::new(50, 0), math::Real::new(50, 0)).await;
                processor.motion_planner.set_usteps(8, 8, 8, 8).await;
            }
            else {
                processor.motion_planner.set_steps_per_mm(math::Real::new(10, 0), math::Real::new(10, 0), math::Real::new(50, 0), math::Real::new(50, 0)).await;
                processor.motion_planner.set_usteps(8, 8, 8, 8).await;
            }
        }

        processor
            .motion_planner
            .set_machine_bounds(200, 200, 200)
            .await;
        processor.motion_planner.set_flow_rate(100).await;
        processor.motion_planner.set_speed_rate(100).await;

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
            control::task_integration::IntegrationaskParams {
                processor: processor.clone(),
                #[cfg(feature = "with-sdcard")]
                card_controller: sdcard_controller.clone(),
                #[cfg(feature = "with-print-job")]
                printer_controller: printer_controller.clone(),
            },
        ))
        .map_err(|_| ())?;

    spawner
        .spawn(control::task_control::task_control(
            processor.clone(),
            control::GCodeMultiplexedInputStream::new(
                #[cfg(feature = "with-serial-usb")]
                _io_devices.serial_usb_rx_stream,
                #[cfg(feature = "with-serial-port-1")]
                _io_devices.serial_port1_rx_stream,
                #[cfg(feature = "with-serial-port-2")]
                _io_devices.serial_port2_rx_stream,
            ),
            ControlTaskControllers {
                #[cfg(feature = "with-print-job")]
                printer_controller: printer_controller.clone(),
                #[cfg(feature = "with-sdcard")]
                card_controller: sdcard_controller.clone(),
            },
        ))
        .map_err(|_| ())?;

    #[cfg(feature = "with-print-job")]
    spawner
        .spawn(control::task_print_job::task_print_job(
            processor.clone(),
            printer_controller,
            sdcard_controller,
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

    #[cfg(feature = "with-motion")]
    spawner
        .spawn(control::task_defer::task_defer(processor))
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
