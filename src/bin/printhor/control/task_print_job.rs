//! TODO: This feature is still incomplete
use crate::control::{GCodeLineParser, GCodeLineParserError};
use crate::hwa;
use embassy_time::Duration;
use embassy_time::{Instant, Timer};

use crate::control::GCodeCmd;
use crate::control::{CodeExecutionFailure, CodeExecutionSuccess};
use hwa::controllers::sdcard_controller::SDCardError;
use hwa::controllers::sdcard_controller::SDCardStream;
use hwa::controllers::PrinterController;
use hwa::controllers::PrinterControllerEvent;
use printhor_hwa_common::EventBusSubscriber;
use printhor_hwa_common::EventFlags;
use printhor_hwa_common::{CommChannel, EventStatus};

#[allow(unused_mut)]
#[allow(unreachable_patterns)]
#[embassy_executor::task(pool_size = 1)]
pub async fn task_print_job(
    mut processor: hwa::GCodeProcessor,
    mut printer_controller: PrinterController,
    mut card_controller: hwa::controllers::CardController,
) {
    let mut subscriber: EventBusSubscriber<'static> =
        hwa::task_allocations::init_printer_subscriber(processor.event_bus.clone()).await;

    hwa::info!("[task_print_job] Waiting for SYS_BOOTING");
    // Pauses this task until system is ready
    match subscriber.ft_wait_for(EventStatus::not_containing(EventFlags::SYS_BOOTING)).await
    {
        Ok(_) => hwa::info!("[task_print_job] Got SYS_BOOTING. Continuing."),
        Err(_) => crate::initialization_error(),
    }

    hwa::debug!(
        "SDCardStream takes {} bytes",
        core::mem::size_of::<SDCardStream>()
    );

    let mut print_job_parser = None;
    let mut job_time = Duration::from_ticks(0);

    loop {
        let _ = subscriber
            .ft_wait_for(EventStatus::not_containing(EventFlags::SYS_ALARM))
            .await;

        match embassy_time::with_timeout(
            Duration::from_secs(30),
            printer_controller.consume(),
        ).await {
            Err(_) => {
                #[cfg(feature = "trace-commands")]
                hwa::info!("[task_print_job]] Timeout");
                #[cfg(test)]
                if crate::control::task_integration::INTEGRATION_STATUS.signaled() {
                    hwa::info!("[task_print_job] Ending gracefully");
                    return ()
                }
            }
            Ok(PrinterControllerEvent::SetFile(channel, file_path)) => {
                #[cfg(feature = "trace-commands")]
                hwa::info!("[task_print_job] SetFile: {}", file_path.as_str());
                job_time = Duration::from_ticks(0);
                print_job_parser = match card_controller.new_stream(file_path.as_str()).await {
                    Ok(stream) => {
                        let parser = GCodeLineParser::new(stream);
                        processor.write(channel, "ok\n").await;
                        Some(parser)
                    },
                    Err(_e) => {
                        let error_msg = match _e {
                            SDCardError::NoSuchVolume => {
                                "SetFile: Card not ready"
                            }
                            SDCardError::NotFound => {
                                "SetFile: File not found"
                            }
                            _ => {
                                "SetFile: Internal error"
                            }
                        };
                        hwa::error!("{}", error_msg);
                        processor.write(channel, alloc::format!("error; {}\n", error_msg).as_str()).await;
                        continue;
                    }
                };
                processor
                    .event_bus
                    .publish_event(EventStatus::containing(
                        EventFlags::JOB_PAUSED,
                    ))
                    .await;
            }
            Ok(PrinterControllerEvent::Resume(channel)) => {
                let mut interrupted = false;
                let mut fatal_error = false;
                let job_t0 = Instant::now();
                loop {
                    match gcode_pull(&mut print_job_parser).await {
                        Ok(gcode) => {
                            #[cfg(feature="trace-commands")]
                            hwa::info!("Executing {}", gcode);
                            match processor.execute(CommChannel::Internal, &gcode, true).await {
                                Ok(CodeExecutionSuccess::OK) => {
                                    hwa::debug!("Response: OK {} (I)", gcode);
                                }
                                Ok(CodeExecutionSuccess::CONSUMED) => {
                                    hwa::debug!("Response: OK {} (C)", gcode);
                                }
                                Ok(CodeExecutionSuccess::QUEUED) => {
                                    hwa::debug!("Response: OK {} (Q)", gcode);
                                }
                                Ok(CodeExecutionSuccess::DEFERRED(_status)) => {
                                    hwa::debug!("Deferred {} (D)", gcode);
                                    if subscriber.ft_wait_for(_status).await.is_err() {
                                        // Must pause. Recoverable when SYS_ALARM go down
                                        break;
                                    } else {
                                        hwa::debug!("Response: OK {} (D)", gcode);
                                    }
                                }
                                Err(CodeExecutionFailure::BUSY) => {
                                    fatal_error = true;
                                    hwa::error!("Response: BUSY {}", gcode);
                                    break;
                                }
                                Err(
                                    CodeExecutionFailure::ERR
                                    | CodeExecutionFailure::NumericalError,
                                ) => {
                                    fatal_error = true;
                                    hwa::error!("Response: ERR {}", gcode);
                                    break;
                                }
                                Err(CodeExecutionFailure::NotYetImplemented) => {
                                    hwa::warn!(
                                        "Ignoring GCode not implemented {}", gcode
                                    );
                                }
                                Err(CodeExecutionFailure::HomingRequired) => {
                                    fatal_error = true;
                                    hwa::error!(
                                        "Unexpected HomingRequired before {}", gcode
                                    );
                                    break;
                                }
                                Err(CodeExecutionFailure::PowerRequired) => {
                                    fatal_error = true;
                                    hwa::error!(
                                        "Unexpected PowerRequired before {}", gcode
                                    );
                                    break;
                                }
                            }
                        }
                        Err(error) => {
                            let current_line = print_job_parser
                                .as_ref()
                                .map_or(0, |parser| parser.get_line());
                            match error {
                                GCodeLineParserError::EOF  => {
                                    // EOF
                                    break;
                                }
                                GCodeLineParserError::FatalError => {
                                    // Fatal
                                    fatal_error = true;
                                    hwa::error!("Fatal error at line {}", current_line);
                                    break;
                                }
                                GCodeLineParserError::GCodeNotImplemented(_ln, _gcode) => {
                                    processor.write(channel, alloc::format!("Ignoring gcode not supported: {} at {}\n", _gcode, current_line).as_str()).await;
                                    hwa::warn!(
                                        "Ignoring GCode not supported {} at line {}", _gcode.as_str(), current_line
                                    );
                                }
                                GCodeLineParserError::ParseError(_ln) => {
                                    hwa::warn!("Parse error at {}", current_line);
                                }
                                _e => {
                                    hwa::warn!("Internal error at {} :  {:?}", current_line, _e);
                                    fatal_error = true;
                                    break;
                                }
                            }
                        }
                    }
                    // Peek new event to see if job is cancelled
                    if printer_controller.signaled().await {
                        interrupted = true;
                        break;
                    }
                }
                if subscriber
                    .ft_wait_until(EventFlags::MOV_QUEUE_EMPTY)
                    .await
                    .is_err()
                {
                    hwa::warn!("SYS_ALARM raised");
                }
                job_time += job_t0.elapsed();
                if fatal_error {
                    printer_controller
                        .set(PrinterControllerEvent::Abort(channel))
                        .await
                        .unwrap();
                } else if !interrupted {
                    processor
                        .event_bus
                        .publish_event(EventStatus::containing(EventFlags::JOB_COMPLETED))
                        .await;
                    // Job completed or cancelled
                    match print_job_parser.take() {
                        None => {}
                        Some(mut p) => p.close().await,
                    }
                    hwa::info!(
                        "Job completed in {:03} seconds",
                        job_time.as_millis() as f64 / 1000.0
                    );
                }
            }
            Ok(PrinterControllerEvent::Abort(channel)) => {
                processor.write(channel, "echo: Job aborted\n").await;
                match print_job_parser.take() {
                    None => {}
                    Some(mut p) => p.close().await,
                }
                hwa::debug!("File done");
            }
            Ok(_) => {
                hwa::warn!("unexpected event");
                continue;
            }
        }
        Timer::after(Duration::from_secs(2)).await;
    }
}

async fn gcode_pull(
    print_job_parser: &mut Option<GCodeLineParser<SDCardStream>>,
) -> Result<GCodeCmd, GCodeLineParserError> {
    match print_job_parser.as_mut() {
        Some(parser) => {
            parser.next_gcode().await.map(|mut gc| {
                gc.order_num = parser.get_line();
                gc
            })
        }
        None => Err(GCodeLineParserError::FatalError)
    }

}
