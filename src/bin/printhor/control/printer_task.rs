//! TODO: This feature is still incomplete
use crate::hwa;
use crate::control::parser::{GCodeLineParser, GCodeLineParserError};
use embassy_time::Timer;
use embassy_time::Duration;

use crate::hwa::controllers::PrinterController;
use crate::hwa::controllers::PrinterControllerEvent;
use crate::hwa::controllers::sdcard_controller::SDCardStream;
use crate::hwa::controllers::sdcard_controller::SDCardError;
use printhor_hwa_common::EventStatus;
use printhor_hwa_common::EventFlags;
use printhor_hwa_common::EventBusSubscriber;
use crate::control::GCode;
use crate::ctrl::{CodeExecutionFailure, CodeExecutionSuccess};

#[allow(unused_mut)]
#[allow(unreachable_patterns)]
#[embassy_executor::task(pool_size=1)]
pub(crate) async fn printer_task(
    mut processor: hwa::GCodeProcessor,
    mut printer_controller: PrinterController,
    mut card_controller: hwa::controllers::CardController,
) -> ! {

    let mut subscriber: EventBusSubscriber<'static> = hwa::task_allocations::init_printer_subscriber(processor.event_bus.clone()).await;
    subscriber.wait_for(EventStatus::not_containing(EventFlags::SYS_BOOTING)).await;

    hwa::debug!("printer_task started");
    hwa::debug!("SDCardStream takes {} bytes", core::mem::size_of::<SDCardStream>());

    let mut print_job_parser = None;
    let mut current_line = 0;

    loop {
        hwa::debug!("Waiting for event");
        match printer_controller.consume().await {
            PrinterControllerEvent::SetFile(file_path) => {
                hwa::info!("SetFile: {}", file_path.as_str());
                current_line = 0;
                print_job_parser = match card_controller.new_stream(file_path.as_str()).await {
                    Ok(stream) => {
                        Some(GCodeLineParser::new(stream))
                    }
                    Err(_e) => {
                        match _e {
                            SDCardError::NoSuchVolume => {
                                hwa::error!("SetFile: Card not ready");
                            }
                            SDCardError::NotFound => {
                                hwa::error!("SetFile: File not found");
                            }
                            _ => {
                                hwa::error!("SetFile: Internal error {:?}", _e);
                            }
                        }
                        processor.event_bus.publish_event(EventStatus::not_containing(EventFlags::JOB_FILE_SEL)).await;
                        continue;
                    }
                };
                processor.event_bus.publish_event(EventStatus::containing(EventFlags::JOB_FILE_SEL | EventFlags::JOB_PAUSED)).await;

            }
            PrinterControllerEvent::Resume => {
                let mut interrupted = false;
                let mut fatal_error = false;
                loop {
                    current_line += 1;
                    match gcode_pull(&mut print_job_parser).await {
                        Ok(Some(gcode)) => {
                            hwa::debug!("Line {}: Executing {}", current_line, gcode);
                            match processor.execute(&gcode, true).await {
                                Ok(CodeExecutionSuccess::OK) => {
                                    hwa::debug!("Line {}: OK {}", current_line, gcode);
                                }
                                Ok(CodeExecutionSuccess::QUEUED) => {
                                    hwa::debug!("Line {}: QUEUED {}", current_line, gcode);
                                }
                                Ok(CodeExecutionSuccess::DEFERRED(_status)) => {
                                    hwa::debug!("Line {}: DEFERRED {}", current_line, gcode);
                                }
                                Err(CodeExecutionFailure::BUSY) => {
                                    fatal_error = true;
                                    hwa::error!("Line {}: BUSY {}", current_line, gcode);
                                    break;
                                }
                                Err(CodeExecutionFailure::ERR | CodeExecutionFailure::NumericalError) => {
                                    fatal_error = true;
                                    hwa::error!("Line {} ERR {}", current_line, gcode);
                                    break;
                                }
                                Err(CodeExecutionFailure::NotYetImplemented) => {
                                    hwa::warn!("Line {}: Ignoring GCode not implemented {}", current_line, gcode);
                                }
                                Err(CodeExecutionFailure::HomingRequired) => {
                                    fatal_error = true;
                                    hwa::error!("Line {}: Unexpected HomingRequired before {}", current_line, gcode);
                                    break;
                                }
                            }
                        }
                        Ok(None) => { // EOF
                            break;
                        }
                        Err(GCodeLineParserError::FatalError) => { // Fatal
                            hwa::error!("Line {}: Fatal error", current_line);
                            break;
                        }
                        Err(GCodeLineParserError::GCodeNotImplemented(_ln, _gcode)) => { // Fatal
                            hwa::warn!("Line {}: Ignoring GCode not supported {}", current_line, _gcode.as_str());
                        }
                        Err(_e) => {
                            hwa::warn!("Line {}: Internal error: {:?}", current_line, _e);
                            break;
                        }
                    }
                    // Peek new event to see if job is cancelled
                    if printer_controller.signaled().await {
                        interrupted = true;
                        break;
                    }
                }
                if fatal_error {
                    printer_controller.set(PrinterControllerEvent::Abort).await.unwrap();
                }
                else if !interrupted {
                    processor.event_bus.publish_event(EventStatus::containing(EventFlags::JOB_COMPLETED)).await;
                    // Job completed or cancelled
                    match print_job_parser.take() {
                        None => {}
                        Some(mut p) => {
                            p.close().await
                        }
                    }
                    current_line = 0;
                    hwa::info!("Job completed");
                }
            }
            PrinterControllerEvent::Abort => {
                match print_job_parser.take() {
                    None => {}
                    Some(mut p) => {
                        p.close().await
                    }
                }
                current_line = 0;
                hwa::info!("file done");
            }
            _ => {
                hwa::debug!("unexpected event");
                continue
            }
        }
        Timer::after(Duration::from_secs(2)).await;
    }
}

#[inline]
async fn gcode_pull(print_job_parser: &mut Option<GCodeLineParser<SDCardStream>>) -> Result<Option<GCode>, GCodeLineParserError> {
    print_job_parser.as_mut().ok_or(GCodeLineParserError::FatalError)?.next_gcode().await
}