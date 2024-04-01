use embassy_time::Duration;
use crate::hwa;
#[allow(unused)]
use crate::control::*;
#[allow(unused)]
use crate::math;
#[allow(unused)]
use crate::math::Real;
#[allow(unused)]
use printhor_hwa_common::{CommChannel, EventBusSubscriber, EventStatus, EventFlags};

pub struct IntegrationaskParams {
    pub processor: hwa::GCodeProcessor,
    #[cfg(feature = "with-printjob")]
    pub printer_controller: hwa::controllers::PrinterController,
}
#[embassy_executor::task(pool_size=1)]
pub async fn task_integration(mut params: IntegrationaskParams)
{

    #[allow(unused)]
    let expect_immediate = |res| match res {
        CodeExecutionSuccess::OK | CodeExecutionSuccess::CONSUMED => Ok(CodeExecutionSuccess::OK),
        CodeExecutionSuccess::QUEUED => Ok(CodeExecutionSuccess::OK),
        CodeExecutionSuccess::DEFERRED(_) => Err(CodeExecutionFailure::ERR),
    };
    #[allow(unused)]
    let expect_deferred = |res| match res {
        CodeExecutionSuccess::OK | CodeExecutionSuccess::CONSUMED => Err(CodeExecutionFailure::ERR),
        CodeExecutionSuccess::QUEUED => Err(CodeExecutionFailure::ERR),
        CodeExecutionSuccess::DEFERRED(evt) => Ok(evt),
    };

    let event_bus = params.processor.event_bus.clone();
    let mut subscriber: EventBusSubscriber<'static> = hwa::task_allocations::init_integration_subscriber(event_bus).await;

    subscriber.ft_wait_until(EventFlags::SYS_READY).await.unwrap();
    hwa::info!("D; integration_task started");

    // Set endstops up for simulation for homing to be completed properly
    {
        let mut dg = params.processor.motion_planner.motion_driver.lock().await;
        dg.pins.x_endstop_pin.set_high();
        dg.pins.y_endstop_pin.set_high();
        dg.pins.z_endstop_pin.set_high();
    }

    #[cfg(feature = "integration-test-m100")]
    {
        hwa::info!("Testing M100");
        if params.processor.execute(CommChannel::Internal, &GCode::M100, true).await.and_then(expect_immediate).is_ok() {
            hwa::info!("-- M100 OK");
        }
        else {
            hwa::error!("M100: Unexpected result");
        }
    }

    #[cfg(feature = "integration-test-power-on")]
    {
        hwa::info!("Testing M80");
        if params.processor.execute(CommChannel::Internal, &GCode::M80, false).await.and_then(expect_immediate).is_err() {
            hwa::error!("M80: Unexpected result");
        }
        else {
            hwa::info!("-- M80 OK");
        }
    }

    #[cfg(feature = "integration-test-trinamic")]
    {
        hwa::info!("Testing M502");
        if params.processor.execute(CommChannel::Internal, &GCode::M502, false).await.and_then(expect_immediate).is_err() {
            hwa::error!("M502: Unexpected result");
        }
        else {
            hwa::info!("-- M502 OK");
        }
    }

    #[cfg(feature = "integration-test-homing")]
    {

        let homing_gcode = GCode::G28(
            XYZE {
                ln: None,
                x: None,
                y: None,
                z: None,
                e: None,
            }
        );

        hwa::info!("Testing G28");
        let t0 = embassy_time::Instant::now();
        if let Some(evt) = params.processor.execute(CommChannel::Internal, &homing_gcode, false).await.and_then(expect_deferred).ok()
        {
            if subscriber.ft_wait_for(evt).await.is_err() {
                hwa::error!("Unexpected state for G28");
            }
            hwa::info!("-- G28 OK (took: {} ms)", t0.elapsed().as_millis());
        } else {
            hwa::error!("Unexpected state for G28");
        }
    }

    #[cfg(feature = "integration-test-reset-pos")]
    {

        let set_pos_gcode = GCode::G92(
            XYZE {
                ln: None,
                x: Some(math::ZERO),
                y: Some(math::ZERO),
                z: Some(math::ZERO),
                e: Some(math::ZERO),
            }
        );

        hwa::info!("Testing G92");
        if params.processor.execute(CommChannel::Internal, &set_pos_gcode, false).await.and_then(expect_immediate).is_ok() {
            hwa::info!("-- G92 OK");
        } else {
            hwa::error!("Unexpected state for G92");
        }
    }

    #[cfg(feature = "integration-test-move-ortho")]
    {
        hwa::info!("Testing G1 Ortho");

        let g1_code = GCode::G1(crate::control::XYZEFS {
            ln: None,
            x: Some(Real::new(10, 0)),
            y: None,
            z: None,
            e: None,
            f: None,
            s: None
        });

        if let Some(_evt) = params.processor.execute(CommChannel::Internal, &g1_code, false).await.and_then(expect_immediate).ok() {
            hwa::trace!("-- G1 OK");
        } else {
            hwa::error!("G1 Unexpected state");
        }
    }

    #[cfg(feature = "integration-test-move-oblique")]
    {
        hwa::info!("Testing G1 Oblique");

        let g1_code = GCode::G1(crate::control::XYZEFS {
            ln: None,
            x: Some(Real::new(20, 0)),
            y: Some(Real::new(20, 0)),
            z: None,
            e: Some(Real::new(25, 1)),
            f: None,
            s: None
        });

        if let Some(_evt) = params.processor.execute(CommChannel::Internal, &g1_code, false).await.and_then(expect_immediate).ok() {
            hwa::info!("-- G1 Oblique OK");
        } else {
            hwa::error!("G1 Unexpected state");
        }
    }

    #[cfg(feature = "integration-test-move-boundaries-1")]
    {
        hwa::info!("Testing G1 retract");

        let g1_code = GCode::G1(crate::control::XYZEFS {
            ln: None,
            x: None,
            y: None,
            z: None,
            e: Some(Real::new(-310, 2)),
            f: None,
            s: None
        });

        if let Some(_evt) = params.processor.execute(&g1_code, false).await.and_then(expect_immediate).ok() {
            hwa::info!("-- G1 retract OK");
        } else {
            hwa::error!("G1 Unexpected state");
        }
        hwa::info!("Testing G1 retract");
    }

    #[cfg(feature = "integration-test-move-boundaries")]
    {
        let g1_code = GCode::G1(crate::control::XYZEFS {
            ln: None,
            x: Some(Real::new(7414, 2)),
            y: Some(Real::new(9066, 2)),
            z: None,
            e: Some(Real::new(335, 4)),
            f: None,
            s: None
        });

        if let Some(_evt) = params.processor.execute(CommChannel::Internal, &g1_code, false).await.and_then(expect_immediate).ok() {
            hwa::info!("-- G1 retract OK");
        } else {
            hwa::error!("G1 Unexpected state");
        }
    }

    #[cfg(feature = "integration-test-dwell")]
    {
        hwa::info!("Testing G4");
        if let Some(evt) = params.processor.execute(CommChannel::Internal, &GCode::G4, false).await.and_then(expect_deferred).ok() {
            subscriber.ft_wait_for(evt).await.unwrap();
            hwa::info!("-- G4 OK");
        } else {
            hwa::error!("G4 Unexpected state");
        }
    }

    #[cfg(feature = "integration-test-set-hotend-temp")]
    {
        hwa::info!("Testing M104");
        if !params.processor.execute(CommChannel::Internal, &GCode::M104(S { ln: None, s: Some(Real::new(235, 0)) }), false).await.and_then(expect_immediate).is_ok() {
            hwa::error!("M104: unexpected result");
        }
        hwa::info!("Testing M109 S235");
        if let Some(evt) = params.processor.execute(CommChannel::Internal, &GCode::M109(S{ln: None, s: Some(Real::new(235, 0))}), false).await.and_then(expect_deferred).ok() {
            subscriber.ft_wait_for(evt).await.unwrap();
        }
    }
    #[cfg(feature = "integration-test-laser-engrave")]
    {
        use crate::hwa::controllers::PrinterControllerEvent;

        hwa::info!("Testing GCODE for engraving");
        params.printer_controller.set(PrinterControllerEvent::SetFile(String::from("dir/laser.g"))).await.unwrap();
        match embassy_time::with_timeout(
            embassy_time::Duration::from_secs(5),
            subscriber.ft_wait_for(EventStatus::containing(EventFlags::JOB_FILE_SEL).and_containing(EventFlags::JOB_PAUSED))
        ).await {
            Ok(_) => {
                // command resume (eq: M24)
                params.printer_controller.set(PrinterControllerEvent::Resume).await.unwrap();
                // wait for job completion
                subscriber.ft_wait_for(EventStatus::containing(EventFlags::JOB_COMPLETED)).await.unwrap();
            }
            Err(_) => {
                hwa::error!("Timeout dispatching engraving job");
            }
        }

    }
    #[cfg(feature = "integration-test-benchy")]
    {
        use crate::hwa::controllers::PrinterControllerEvent;

        hwa::info!("Testing GCODE for benchy FDM print");
        params.printer_controller.set(PrinterControllerEvent::SetFile(String::from("benchy.g"))).await.unwrap();
        match embassy_time::with_timeout(
            embassy_time::Duration::from_secs(5),
            subscriber.ft_wait_for(printhor_hwa_common::EventStatus::containing(EventFlags::JOB_FILE_SEL).and_containing(EventFlags::JOB_PAUSED))
        ).await {
            Ok(_) => {
                // command resume (eq: M24)
                params.printer_controller.set(PrinterControllerEvent::Resume).await.unwrap();
                // wait for job completion
                subscriber.ft_wait_for(printhor_hwa_common::EventStatus::containing(EventFlags::JOB_COMPLETED)).await.unwrap();
            }
            Err(_) => {
                hwa::error!("Timeout dispatching benchy job");
            }
        }

    }
    #[cfg(feature = "integration-test-pen")]
    {
        use crate::hwa::controllers::PrinterControllerEvent;

        hwa::info!("Testing GCODE for pen (plotter)");
        params.printer_controller.set(PrinterControllerEvent::SetFile(String::from("dir/pen.g"))).await.unwrap();
        match embassy_time::with_timeout(
            embassy_time::Duration::from_secs(5),
            subscriber.ft_wait_for(printhor_hwa_common::EventStatus::containing(EventFlags::JOB_FILE_SEL).and_containing(EventFlags::JOB_PAUSED))
        ).await {
            Ok(_) => {
                // command resume (eq: M24)
                params.printer_controller.set(PrinterControllerEvent::Resume).await.unwrap();
                // wait for job completion
                subscriber.ft_wait_for(printhor_hwa_common::EventStatus::containing(EventFlags::JOB_COMPLETED)).await.unwrap();
            }
            Err(_) => {
                hwa::error!("Timeout dispatching pen job");
            }
        }

    }
    if subscriber.ft_wait_for(
        EventStatus::not_containing(EventFlags::JOB_PRINTING)
            .and_not_containing(EventFlags::MOVING)
            .and_containing(EventFlags::MOV_QUEUE_EMPTY)
    ).await.is_err() {
        hwa::error!("Unexpected error processing benchy job");
    }
    embassy_time::Timer::after(Duration::from_millis(100)).await;
    hwa::info!("D; Integration task END");
    #[cfg(feature = "native")]
    std::process::exit(0)
}

