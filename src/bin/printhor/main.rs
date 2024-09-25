#![allow(stable_features)]
#![cfg_attr(not(feature = "native"), no_std)]
#![cfg_attr(not(feature = "native"), no_main)]
extern crate alloc;
extern crate core;
pub mod control;
#[cfg(feature = "with-display")]
pub mod display;
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
use crate::hwa::controllers::HotendPwmController;
#[cfg(feature = "with-printjob")]
use crate::hwa::controllers::PrinterController;
#[cfg(feature = "with-motion")]
use crate::hwa::drivers::MotionDriver;
use embassy_executor::Spawner;
#[cfg(feature = "with-motion")]
use hwa::controllers::{MotionConfig, MotionPlanner, MotionPlannerRef};
use hwa::GCodeProcessor;
use hwa::{Controllers, IODevices, MotionDevices, PwmDevices, SysDevices};

//noinspection RsUnresolvedReference
/// Entry point
#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) -> ! {

    hwa::info!("Init");
    hwa::init_logger();

    let wdt = match sys_start(spawner).await {
        Ok(wdt) => wdt,
        Err(_) => {
            hwa::error!("Error starting system");
            panic!("Unable to start");
        }
    };

    // The core of the main function is just to periodically feed watchdog

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

async fn sys_start(spawner: embassy_executor::Spawner) -> Result<hwa::StandardControllerRef<hwa::device::Watchdog>, ()> {

    hwa::info!("Init printhor {}", machine::MACHINE_INFO.firmware_version);
    let peripherals = hwa::init();
    hwa::debug!("Peripherals initialized");

    let context = hwa::setup(spawner, peripherals).await;
    hwa::info!(
        "HWI setup completed. Allocated {} bytes for context.",
        core::mem::size_of_val(&context)
    );

    let event_bus: hwa::EventBusRef =
        printhor_hwa_common::init_event_bus::<{ hwa::MAX_STATIC_MEMORY }>();

    event_bus
        .publish_event(hwa::EventStatus::containing(hwa::EventFlags::SYS_BOOTING))
        .await;

    let defer_channel: hwa::DeferChannelRef =
        printhor_hwa_common::init_defer_channel::<{ hwa::MAX_STATIC_MEMORY }>();

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
            .publish_event(hwa::EventStatus::not_containing(hwa::EventFlags::SYS_BOOTING))
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
            crate::hwa::mem::stack_reservation_current_size(),
        );
        event_bus
            .publish_event(hwa::EventStatus::containing(hwa::EventFlags::SYS_READY))
            .await;
        Ok(wdt)
    } else {
        event_bus
            .publish_event(hwa::EventStatus::containing(hwa::EventFlags::SYS_BOOT_FAILURE))
            .await;
        hwa::error!("Unable start. Any task launch failed");
        Err(())
    }

}

async fn spawn_tasks(
    spawner: Spawner,
    event_bus: hwa::EventBusRef,
    _defer_channel: hwa::DeferChannelRef,
    _controllers: Controllers,
    _sys_devices: SysDevices,
    _io_devices: IODevices,
    _motion_device: MotionDevices,
    _pwm_devices: PwmDevices,
    _wd: hwa::WatchdogRef,
) -> Result<(), ()> {

    #[cfg(all(feature = "with-sdcard", feature = "sdcard-uses-spi"))]
    let sdcard_adapter =
        hwa::adapters::SPIAdapter::new(_io_devices.sdcard_device, _io_devices.sdcard_cs_pin);

    #[cfg(all(feature = "with-sdcard", not(feature = "sdcard-uses-spi")))]
    let sdcard_adapter = _io_devices.sdcard_device;

    #[cfg(feature = "with-sdcard")]
    let sdcard_controller = CardController::new(sdcard_adapter).await;

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
        #[cfg_attr(not(target_arch = "aarch64"), link_section = ".bss")]
        #[cfg_attr(target_arch = "aarch64", link_section = "__DATA,.bss")]
        static PROBE_CONTROLLER_INST: hwa::TrackedStaticCell<
            printhor_hwa_common::InterruptControllerMutex<hwa::controllers::ServoController>,
        > = hwa::TrackedStaticCell::new();
        hwa::ControllerRef::new(PROBE_CONTROLLER_INST.init::<{ hwa::MAX_STATIC_MEMORY }>(
            "ProbeServoController",
            printhor_hwa_common::InterruptControllerMutex::new(
                hwa::controllers::ServoController::new(
                    _pwm_devices.probe.power_pwm,
                    _pwm_devices.probe.power_channel,
                ),
            ),
        ))
    };

    #[cfg(feature = "with-fan-layer")]
    let fan_layer_controller = {
        #[cfg_attr(not(target_arch = "aarch64"), link_section = ".bss")]
#[cfg_attr(target_arch = "aarch64", link_section = "__DATA,.bss")]
        static LAYER_CONTROLLER_INST: hwa::TrackedStaticCell<
            printhor_hwa_common::InterruptControllerMutex<hwa::controllers::FanLayerPwmController>,
        > = hwa::TrackedStaticCell::new();
        hwa::ControllerRef::new(LAYER_CONTROLLER_INST.init::<{ hwa::MAX_STATIC_MEMORY }>(
            "FanLayerController",
            printhor_hwa_common::ControllerMutex::new(
                hwa::controllers::FanLayerPwmController::new(
                    _pwm_devices.fan_layer.power_pwm,
                    _pwm_devices.fan_layer.power_channel,
                ),
            ),
        ))
    };

    #[cfg(feature = "with-fan-extra-1")]
    let fan_extra_1_controller = {
        #[cfg_attr(not(target_arch = "aarch64"), link_section = ".bss")]
#[cfg_attr(target_arch = "aarch64", link_section = "__DATA,.bss")]
        static FAN_EXTRA_1_CONTROLLER_INST: hwa::TrackedStaticCell<
            printhor_hwa_common::InterruptControllerMutex<hwa::controllers::FanExtra1PwmController>,
        > = hwa::TrackedStaticCell::new();
        hwa::ControllerRef::new(
            FAN_EXTRA_1_CONTROLLER_INST.init::<{ hwa::MAX_STATIC_MEMORY }>(
                "FanExtra1Controller",
                hwa::ControllerMutex::new(hwa::controllers::FanExtra1PwmController::new(
                    _pwm_devices.fan_extra_1.power_pwm,
                    _pwm_devices.fan_extra_1.power_channel,
                )),
            ),
        )
    };

    #[cfg(feature = "with-laser")]
    let laser_controller = {
        #[cfg_attr(not(target_arch = "aarch64"), link_section = ".bss")]
#[cfg_attr(target_arch = "aarch64", link_section = "__DATA,.bss")]
        static LASER_CONTROLLER_INST: hwa::TrackedStaticCell<
            hwa::InterruptControllerMutex<hwa::controllers::LaserPwmController>,
        > = hwa::TrackedStaticCell::new();
        hwa::ControllerRef::new(LASER_CONTROLLER_INST.init::<{ hwa::MAX_STATIC_MEMORY }>(
            "LaserController",
            hwa::ControllerMutex::new(hwa::controllers::LaserPwmController::new(
                _pwm_devices.laser.power_pwm,
                _pwm_devices.laser.power_channel,
            )),
        ))
    };

    #[cfg(feature = "with-hot-end")]
    let hotend_controller = {
        #[cfg_attr(not(target_arch = "aarch64"), link_section = ".bss")]
#[cfg_attr(target_arch = "aarch64", link_section = "__DATA,.bss")]
        static HOTEND_CONTROLLER_INST: hwa::TrackedStaticCell<
            hwa::InterruptControllerMutex<hwa::controllers::HotendController>,
        > = hwa::TrackedStaticCell::new();
        hwa::ControllerRef::new(HOTEND_CONTROLLER_INST.init::<{ hwa::MAX_STATIC_MEMORY }>(
            "HotEndController",
            hwa::ControllerMutex::new(hwa::controllers::HotendController::new(
                _pwm_devices.hotend.temp_adc.clone(),
                _pwm_devices.hotend.temp_pin,
                hotend_pwm,
                _defer_channel.clone(),
                _pwm_devices.hotend.thermistor_properties,
            )),
        ))
    };
    #[cfg(feature = "with-hot-end")]
    hotend_controller.lock().await.init().await;

    #[cfg(feature = "with-hot-bed")]
    let hotbed_controller = {
        #[cfg_attr(not(target_arch = "aarch64"), link_section = ".bss")]
#[cfg_attr(target_arch = "aarch64", link_section = "__DATA,.bss")]
        static HOTBED_CONTROLLER_INST: hwa::TrackedStaticCell<
            hwa::InterruptControllerMutex<hwa::controllers::HotbedController>,
        > = hwa::TrackedStaticCell::new();
        hwa::ControllerRef::new(HOTBED_CONTROLLER_INST.init::<{ hwa::MAX_STATIC_MEMORY }>(
            "HotbedController",
            hwa::ControllerMutex::new(hwa::controllers::HotbedController::new(
                _pwm_devices.hotbed.temp_adc,
                _pwm_devices.hotbed.temp_pin,
                hotbed_pwm,
                _defer_channel.clone(),
                _pwm_devices.hotbed.thermistor_properties,
            )),
        ))
    };
    #[cfg(feature = "with-hot-bed")]
    hotbed_controller.lock().await.init().await;

    #[cfg(feature = "with-motion")]
    let motion_planer = {
        #[cfg_attr(not(target_arch = "aarch64"), link_section = ".bss")]
#[cfg_attr(target_arch = "aarch64", link_section = "__DATA,.bss")]
        static MCS: hwa::TrackedStaticCell<hwa::InterruptControllerMutex<MotionConfig>> =
            hwa::TrackedStaticCell::new();
        let motion_config: printhor_hwa_common::InterruptControllerRef<MotionConfig> =
            hwa::ControllerRef::new(MCS.init::<{ hwa::MAX_STATIC_MEMORY }>(
                "MotionConfig",
                hwa::ControllerMutex::new(MotionConfig::new()),
            ));

        #[cfg_attr(not(target_arch = "aarch64"), link_section = ".bss")]
#[cfg_attr(target_arch = "aarch64", link_section = "__DATA,.bss")]
        static MDS: hwa::TrackedStaticCell<
            hwa::ControllerMutex<hwa::InterruptControllerMutexType, MotionDriver>,
        > = hwa::TrackedStaticCell::new();
        let motion_driver: printhor_hwa_common::InterruptControllerRef<MotionDriver> =
            hwa::ControllerRef::new(MDS.init::<{ hwa::MAX_STATIC_MEMORY }>(
                "MotionDriver",
                hwa::ControllerMutex::new(MotionDriver::new(hwa::drivers::MotionDriverParams {
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
                })),
            ));
        #[cfg_attr(not(target_arch = "aarch64"), link_section = ".bss")]
#[cfg_attr(target_arch = "aarch64", link_section = "__DATA,.bss")]
        static MPS: hwa::TrackedStaticCell<MotionPlanner> = hwa::TrackedStaticCell::new();
        MotionPlannerRef::new(MPS.init::<{ hwa::MAX_STATIC_MEMORY }>(
            "MotionPlanner",
            MotionPlanner::new(_defer_channel.clone(), motion_config.clone(), motion_driver),
        ))
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
        motion_planer
            .set_max_speed(tgeo::TVector::from_coords(
                Some(100),
                Some(100),
                Some(50),
                Some(100),
            ))
            .await;
        motion_planer
            .set_max_accel(tgeo::TVector::from_coords(
                Some(3000),
                Some(3000),
                Some(100),
                Some(3000),
            ))
            .await;
        motion_planer
            .set_max_jerk(tgeo::TVector::from_coords(
                Some(6000),
                Some(6000),
                Some(200),
                Some(6000),
            ))
            .await;
        motion_planer.set_default_travel_speed(100).await;
        // Homing unneeded
        motion_planer.set_last_planned_pos(&tgeo::TVector::zero()).await;

        cfg_if::cfg_if! {
            if #[cfg(feature = "native")] {
                motion_planer.set_steps_per_mm(math::Real::new(10, 0), math::Real::new(10, 0), math::Real::new(50, 0), math::Real::new(50, 0)).await;
                motion_planer.set_usteps(8, 8, 8, 8).await;
            }
            else {
                motion_planer.set_steps_per_mm(math::Real::new(10, 0), math::Real::new(10, 0), math::Real::new(50, 0), math::Real::new(50, 0)).await;
                motion_planer.set_usteps(8, 8, 8, 8).await;
            }
        }

        motion_planer.set_machine_bounds(200, 200, 200).await;
        motion_planer.set_flow_rate(100).await;
        motion_planer.set_speed_rate(100).await;

        spawner
            .spawn(control::task_stepper::task_stepper(
                event_bus.clone(),
                motion_planer.clone(),
                _wd,
            ))
            .map_err(|_| ())?;
    }

    #[cfg(feature = "integration-test")]
    spawner
        .spawn(control::task_integration::task_integration(
            control::task_integration::IntegrationaskParams {
                processor: processor.clone(),
                #[cfg(feature = "with-printjob")]
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
                #[cfg(feature = "with-printjob")]
                printer_controller: printer_controller.clone(),
                #[cfg(feature = "with-sdcard")]
                card_controller: sdcard_controller.clone(),
            },
        ))
        .map_err(|_| ())?;

    #[cfg(feature = "with-printjob")]
    spawner
        .spawn(control::task_printjob::task_printjob(
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
            hotend_controller,
            #[cfg(feature = "with-hot-bed")]
            hotbed_controller,
        ))
        .map_err(|_| ())?;

    #[cfg(feature = "with-display")]
    spawner
        .spawn(display::display_task::display_task(
            _io_devices.display_device,
            event_bus.clone(),
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
    if #[cfg(feature = "with-defmt")] {
        #[allow(unused)]
        #[cfg(feature = "with-defmt")]
        use defmt_rtt as _;
        #[allow(unused)]
        #[cfg(feature = "with-defmt")]
        use panic_probe as _;
    }
}
