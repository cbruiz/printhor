use crate::hwa;
use crate::control::GCode;
use crate::math::Real;
use crate::ctrl::*;
use printhor_hwa_common::EventBusSubscriber;

pub struct IntegrationaskParams {
    pub processor: hwa::GCodeProcessor,
    #[cfg(feature = "with-printjob")]
    pub printer_controller: hwa::controllers::PrinterController,
}
#[embassy_executor::task(pool_size=1)]
pub(crate) async fn integration_task(mut params: IntegrationaskParams)
{
    hwa::info!("D; integration_task started");

    let expect_immediate = |res| match res {
        CodeExecutionSuccess::OK => Ok(CodeExecutionSuccess::OK),
        CodeExecutionSuccess::QUEUED => Ok(CodeExecutionSuccess::OK),
        CodeExecutionSuccess::DEFERRED(_) => Err(CodeExecutionFailure::ERR),
    };
    let expect_deferred = |res| match res {
        CodeExecutionSuccess::OK => Err(CodeExecutionFailure::ERR),
        CodeExecutionSuccess::QUEUED => Err(CodeExecutionFailure::ERR),
        CodeExecutionSuccess::DEFERRED(evt) => Ok(evt),
    };

    let event_bus = params.processor.event_bus.clone();

    let mut subscriber: EventBusSubscriber<'static> = hwa::task_allocations::init_integration_subscriber(event_bus).await;

    #[cfg(feature = "integration-test-m100")]
    {
        hwa::info!("Testing M100");
        if params.processor.execute(&GCode::M100, true).await.and_then(expect_immediate).is_err() {
            hwa::error!("M100: Unexpected result");
        }
    }

    #[cfg(feature = "integration-test-power-on")]
    {
        hwa::info!("Testing M80");
        if params.processor.execute(&GCode::M80, false).await.and_then(expect_immediate).is_err() {
            hwa::error!("M80: Unexpected result");
        }
        else {
            hwa::info!("-- M80 OK");
        }
    }

    #[cfg(feature = "integration-test-homing")]
    {
        let homing_gcode = GCode::G28(
            crate::control::XYZW {
                ln: None,
                x: None,
                y: None,
                z: None,
                w: None,
            }
        );

        hwa::info!("Testing G28");
        let t0 = embassy_time::Instant::now();
        if let Some(evt) = params.processor.execute(&homing_gcode, false).await.and_then(expect_deferred).ok()
        {
            subscriber.wait_until(evt).await;
            hwa::info!("G28 OK (took: {} ms)", t0.elapsed().as_millis());
        } else {
            hwa::error!("Unexpected state for G28");
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

        if let Some(_evt) = params.processor.execute(&g1_code, false).await.and_then(expect_immediate).ok() {
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
            x: Some(Real::new(10, 0)),
            y: Some(Real::new(5, 0)),
            z: None,
            e: Some(Real::new(1414, 3)),
            f: None,
            s: None
        });

        if let Some(_evt) = params.processor.execute(&g1_code, false).await.and_then(expect_immediate).ok() {
            hwa::info!("-- G1 Oblique OK");
        } else {
            hwa::error!("G1 Unexpected state");
        }
    }

    #[cfg(feature = "integration-test-dwell")]
    {
        hwa::info!("Testing G4");
        if let Some(evt) = params.processor.execute(&GCode::G4, false).await.and_then(expect_deferred).ok() {
            subscriber.wait_until(evt).await;
            hwa::info!("-- G4 OK");
        } else {
            hwa::error!("G4 Unexpected state");
        }
    }

    #[cfg(feature = "integration-test-set-hotend-temp")]
    {
        hwa::info!("Testing M104");
        if !params.processor.execute(&GCode::M104(S { ln: None, s: Some(Real::new(235, 0)) }), false).await.and_then(expect_immediate).is_ok() {
            hwa::error!("M104: unexpected result");
        }
        hwa::info!("Testing M109 S235");
        if let Some(evt) = params.processor.execute(&GCode::M109(S{ln: None, s: Some(Real::new(235, 0))}), false).await.and_then(expect_deferred).ok() {
            subscriber.wait_until(evt).await;
        }
    }
    hwa::info!("D; Integration task END");
}
