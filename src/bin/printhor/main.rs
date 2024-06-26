#![cfg_attr(not(feature = "native"), no_std)]
#![cfg_attr(not(feature = "native"), no_main)]
#![allow(stable_features)]
#![allow(nonstandard_style)]
#![cfg_attr(feature="nightly", feature(type_alias_impl_trait))]

extern crate alloc;
extern crate core;
mod hwi;
mod hwa;
pub(crate) mod helpers;
pub(crate) mod control;
pub(crate) mod machine;
pub(crate) mod sync;
#[cfg(feature = "with-display")]
pub(crate) mod display;
pub mod tgeo;
pub mod math;

use crate::control::task_control::ControlTaskControllers;
use embassy_executor::Spawner;
#[cfg(any(feature = "with-probe", feature = "with-hot-bed", feature = "with-hot-end", feature = "with-fan-layer", feature = "with-fan-extra-1", feature = "with-laser"))]
use printhor_hwa_common::{ControllerMutex, ControllerRef};
#[allow(unused)]
use hwa::{DeferChannelRef, EventBusRef, TrackedStaticCell};
use hwa::{Controllers, SysDevices, IODevices, MotionDevices, PwmDevices};
#[allow(unused)]
use printhor_hwa_common::{EventStatus, EventFlags};
use hwa::{GCodeProcessor};
#[cfg(feature = "with-motion")]
use hwa::controllers::{MotionPlanner, MotionPlannerRef, MotionConfig};
use crate::control::GCodeProcessorParams;
#[cfg(feature = "with-sdcard")]
use crate::hwa::controllers::CardController;
#[cfg(feature = "with-hot-end")]
use crate::hwa::controllers::HotendPwmController;
#[cfg(feature = "with-hot-bed")]
use crate::hwa::controllers::HotbedPwmController;
#[cfg(feature = "with-printjob")]
use crate::hwa::controllers::PrinterController;

//noinspection RsUnresolvedReference
/// Entry point
#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) -> ! {

    hwa::info!("Init");
    hwa::init_logger();

    hwa::info!("Init printhor {}", machine::MACHINE_INFO.firmware_version);
    let peripherals = hwa::init();
    hwa::debug!("Peripherals initialized");

    let context = hwa::setup(spawner, peripherals).await;
    hwa::info!("HWI setup completed. Allocated {} bytes for context.", core::mem::size_of_val(&context));

    let event_bus: EventBusRef = printhor_hwa_common::init_event_bus::<{hwa::MAX_STATIC_MEMORY}>();

    event_bus.publish_event(
        EventStatus::containing(EventFlags::SYS_BOOTING)
    ).await;

    let defer_channel: DeferChannelRef = printhor_hwa_common::init_defer_channel::<{hwa::MAX_STATIC_MEMORY}>();

    let wdt = context.controllers.sys_watchdog.clone();
    wdt.lock().await.unleash();

    if spawn_tasks(spawner, event_bus.clone(), defer_channel,
                   context.controllers,
                   context.sys_devices,
                   context.io_devices,
                   context.motion,
                   context.pwm,
                   wdt.clone()
    ).await.is_ok() {
        event_bus.publish_event(
            EventStatus::not_containing(EventFlags::SYS_BOOTING)
        ).await;

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion")] {
                let mut event_suscriber = event_bus.subscriber().await;
                if event_suscriber.ft_wait_until(EventFlags::MOV_QUEUE_EMPTY).await.is_err() {
                    initialization_error()
                }
            }
        }

        hwa::info!("Tasks spawned. Allocated {} bytes for shared state. Firing SYS_READY.",
            crate::hwa::mem::stack_reservation_current_size(),
        );
        event_bus.publish_event(
            EventStatus::containing(EventFlags::SYS_READY)
        ).await;
    }
    else {
        event_bus.publish_event(
            EventStatus::containing(EventFlags::SYS_BOOT_FAILURE)
        ).await;
        hwa::error!("Unable start. Any task launch failed");
    }

    let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_secs(2));
    let _t0 = embassy_time::Instant::now();
    loop {
        let _r = event_bus.get_status().await;
        //hwa::info!("STATUS: {:?}", _r);
        //hwa::info!("watchdog feed at {}", _t0.elapsed().as_micros());
        {
            wdt.lock().await.pet();
        }
        ticker.next().await;
    }

}

#[inline(never)]
async fn spawn_tasks(spawner: Spawner, event_bus: EventBusRef, _defer_channel: DeferChannelRef,
                     _controllers: Controllers,
                     _sys_devices: SysDevices,
                     _io_devices: IODevices,
                     _motion_device: MotionDevices,
                     _pwm_devices: PwmDevices, _wd: hwa::WatchdogRef
) -> Result<(), ()> {

    #[cfg(all(feature = "with-sdcard", feature = "sdcard-uses-spi"))]
        let sdcard_adapter = hwa::adapters::SPIAdapter::new(_io_devices.sdcard_device, _io_devices.sdcard_cs_pin);

    #[cfg(all(feature = "with-sdcard", not(feature = "sdcard-uses-spi")))]
        let sdcard_adapter = _io_devices.sdcard_device;

    #[cfg(feature = "with-sdcard")]
        let sdcard_controller = CardController::new(
        sdcard_adapter
    ).await;

    #[cfg(feature = "with-printjob")]
        let printer_controller = PrinterController::new(event_bus.clone());

    #[cfg(feature = "with-hot-end")]
        let hotend_pwm = HotendPwmController::new(
        _pwm_devices.hotend.power_pwm.clone(),
        _pwm_devices.hotend.power_channel,
    );
    #[cfg(feature = "with-hot-bed")]
        let hotbed_pwm = HotbedPwmController::new(
        _pwm_devices.hotbed.power_pwm.clone(),
        _pwm_devices.hotbed.power_channel,
    );

    #[cfg(feature = "with-probe")]
        let probe_controller = {
        static PROBE_CONTROLLER_INST: TrackedStaticCell<ControllerMutex<hwa::controllers::ServoController>> = TrackedStaticCell::new();
        ControllerRef::new(
            PROBE_CONTROLLER_INST.init::<{hwa::MAX_STATIC_MEMORY}>("ProbeServoController",
                                       ControllerMutex::new(
                                           hwa::controllers::ServoController::new(
                                               _pwm_devices.probe.power_pwm,
                                               _pwm_devices.probe.power_channel,
                                           )
                                       )
            )
        )
    };

    #[cfg(feature = "with-fan-layer")]
        let fan_layer_controller = {
        static LAYER_CONTROLLER_INST: TrackedStaticCell<ControllerMutex<hwa::controllers::FanLayerPwmController>> = TrackedStaticCell::new();
        ControllerRef::new(
            LAYER_CONTROLLER_INST.init::<{hwa::MAX_STATIC_MEMORY}>("FanLayerController",
                                       ControllerMutex::new(
                                           hwa::controllers::FanLayerPwmController::new(
                                               _pwm_devices.fan_layer.power_pwm,
                                               _pwm_devices.fan_layer.power_channel,
                                           )
                                       )
            )
        )
    };

    #[cfg(feature = "with-fan-extra-1")]
        let fan_extra_1_controller = {
        static FAN_EXTRA_1_CONTROLLER_INST: TrackedStaticCell<ControllerMutex<hwa::controllers::FanExtra1PwmController>> = TrackedStaticCell::new();
        ControllerRef::new(
            FAN_EXTRA_1_CONTROLLER_INST.init::<{hwa::MAX_STATIC_MEMORY}>("FanExtra1Controller",
                                      ControllerMutex::new(
                                          hwa::controllers::FanExtra1PwmController::new(
                                              _pwm_devices.fan_extra_1.power_pwm,
                                              _pwm_devices.fan_extra_1.power_channel,
                                          )
                                      )
            )
        )
    };

    #[cfg(feature = "with-laser")]
        let laser_controller = {
        static LASER_CONTROLLER_INST: TrackedStaticCell<ControllerMutex<hwa::controllers::LaserPwmController>> = TrackedStaticCell::new();
        ControllerRef::new(
            LASER_CONTROLLER_INST.init::<{hwa::MAX_STATIC_MEMORY}>("LaserController",
                                       ControllerMutex::new(
                                           hwa::controllers::LaserPwmController::new(
                                               _pwm_devices.laser.power_pwm,
                                               _pwm_devices.laser.power_channel,
                                           ))
            ))
    };

    #[cfg(feature = "with-hot-end")]
    let hotend_controller = {
        static HOTEND_CONTROLLER_INST: TrackedStaticCell<ControllerMutex<hwa::controllers::HotendController>> = TrackedStaticCell::new();
        ControllerRef::new(
            HOTEND_CONTROLLER_INST.init::<{hwa::MAX_STATIC_MEMORY}>("HotEndController",
                                        ControllerMutex::new(
                                            hwa::controllers::HotendController::new(
                                                _pwm_devices.hotend.temp_adc.clone(),
                                                _pwm_devices.hotend.temp_pin,
                                                hotend_pwm,
                                                _defer_channel.clone(),
                                                _pwm_devices.hotend.thermistor_properties,
                                            )
                                        )
            )
        )
    };
    #[cfg(feature = "with-hot-end")]
    hotend_controller.lock().await.init().await;

    #[cfg(feature = "with-hot-bed")]
    let hotbed_controller = {
        static HOTBED_CONTROLLER_INST: TrackedStaticCell<ControllerMutex<hwa::controllers::HotbedController>> = TrackedStaticCell::new();
        ControllerRef::new(
            HOTBED_CONTROLLER_INST.init::<{hwa::MAX_STATIC_MEMORY}>("HotbedController",
                                        ControllerMutex::new(
                                            hwa::controllers::HotbedController::new(
                                                _pwm_devices.hotbed.temp_adc,
                                                _pwm_devices.hotbed.temp_pin,
                                                hotbed_pwm,
                                                _defer_channel.clone(),
                                                _pwm_devices.hotbed.thermistor_properties,
                                            )
                                        )
            )
        )
    };
    #[cfg(feature = "with-hot-bed")]
    hotbed_controller.lock().await.init().await;

    #[cfg(feature = "with-motion")]
    let motion_planer = {
        static MCS: TrackedStaticCell<hwa::ControllerMutex<MotionConfig>> = TrackedStaticCell::new();
        let motion_config = hwa::ControllerRef::new(
            MCS.init::<{hwa::MAX_STATIC_MEMORY}>("MotionConfig",
            hwa::ControllerMutex::new(MotionConfig::new())
            )
        );
        static MPS : TrackedStaticCell<MotionPlanner> = TrackedStaticCell::new();
        MotionPlannerRef::new(
            MPS.init::<{hwa::MAX_STATIC_MEMORY}>("MotionPlanner",
                     MotionPlanner::new(
                        event_bus.clone(),
                        _defer_channel.clone(),
                        motion_config.clone(),
                        hwa::drivers::MotionDriver::new(hwa::drivers::MotionDriverParams{
                            motion_device: _motion_device.motion_devices,
                            motion_config,
                            #[cfg(feature = "with-probe")]
                            probe_controller: probe_controller.clone(),
                            #[cfg(feature = "with-fan-layer")]
                            fan_layer_controller: fan_layer_controller.clone(),
                            #[cfg(feature = "with-fan-extra-1")]
                            fan_extra_1_controller: fan_extra_1_controller.clone(),
                            #[cfg(feature = "with-laser")]
                            laser_controller: laser_controller.clone(),
                        }),
                    )
            )
        )
    };

    let processor: GCodeProcessor = GCodeProcessor::new(GCodeProcessorParams {
        event_bus: event_bus.clone(),
        #[cfg(feature = "with-motion")]
        motion_planner: motion_planer.clone(),
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
        hotend: hotend_controller.clone(),
        #[cfg(feature = "with-hot-bed")]
        hotbed: hotbed_controller.clone(),
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
        motion_planer.set_max_speed(tgeo::TVector::from_coords(Some(3000), Some(3000), Some(100), Some(3000))).await;
        motion_planer.set_max_accel(tgeo::TVector::from_coords(Some(9000), Some(9000), Some(9000), Some(9000))).await;
        motion_planer.set_max_jerk(tgeo::TVector::from_coords(Some(12000), Some(12000), Some(12000), Some(12000))).await;
        motion_planer.set_default_travel_speed(1000).await;

        cfg_if::cfg_if! {
            if #[cfg(feature = "native")] {
                motion_planer.set_steps_per_mm(math::Real::new(2, 0), math::Real::new(2, 0), math::Real::new(2, 0), math::Real::new(2, 0)).await;
                motion_planer.set_usteps(4, 8, 16, 8).await;
            }
            else {
                motion_planer.set_steps_per_mm(math::Real::new(80, 0), math::Real::new(160, 0), math::Real::new(400, 0), math::Real::new(2, 0)).await;
                motion_planer.set_usteps(1, 2, 4, 1).await;
            }
        }

        motion_planer.set_machine_bounds(200, 200, 200).await;
        motion_planer.set_flow_rate(100).await;
        motion_planer.set_speed_rate(100).await;

        cfg_if::cfg_if! {
            if #[cfg(feature = "threaded")] {
                hwa::launch_high_priotity( _sys_devices.task_stepper_core,
                                   spawner,
                                   control::task_stepper::task_stepper(motion_planer, _wd)
                ).and_then(|_| {
                    hwa::debug!("stepper_start() spawned");
                    Ok(())
                })?;
            }
            else {
                spawner.spawn(control::task_stepper::task_stepper(motion_planer, _wd)).map_err(|_| ())?;
            }
        }
    }

    #[cfg(feature = "integration-test")]
    spawner.spawn(control::task_integration::task_integration(
        control::task_integration::IntegrationaskParams {
            processor: processor.clone(),
            #[cfg(feature = "with-printjob")]
            printer_controller: printer_controller.clone(),
        }
    )).map_err(|_| ())?;

    spawner.spawn(control::task_control::task_control(
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
            #[cfg(feature="with-printjob")]
            printer_controller: printer_controller.clone(),
            #[cfg(feature="with-sdcard")]
            card_controller: sdcard_controller.clone(),
        }
    )).map_err(|_| ())?;

    #[cfg(feature = "with-printjob")]
    spawner.spawn(control::task_printjob::task_printjob(
        processor.clone(),
        printer_controller,
        sdcard_controller,
    )).map_err(|_| ())?;

    #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
    spawner.spawn(control::task_temperature::task_temperature(
        event_bus.clone(),
        #[cfg(feature = "with-hot-end")]
        hotend_controller,
        #[cfg(feature = "with-hot-bed")]
        hotbed_controller,
    )).map_err(|_| ())?;

    #[cfg(feature = "with-display")]
    spawner.spawn(display::display_task::display_task(
        _io_devices.display_device,
        event_bus.clone()
    )).map_err(|_| ())?;

    #[cfg(feature = "with-motion")]
    spawner.spawn(control::task_defer::task_defer(
        processor,
    )).map_err(|_| ())?;

    Ok(())
}


pub fn initialization_error() {
    let msg = "Unable to start because SYS_ALARM raised at startup. Giving up...";
    hwa::error!("{}", msg);
    panic!("{}", msg);
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-defmt")] {
        #[allow(unused)]
        #[cfg(feature = "with-defmt")]
        use defmt_rtt as _;
        #[allow(unused)]
        #[cfg(feature = "with-defmt")]
        use panic_probe as _;
    }
}
