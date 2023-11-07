//! TODO: This feature is still incomplete
use crate::{hwa};
use crate::control::parser::{GCodeLineParser, GCodeLineParserError};
use embassy_time::Timer;
use embassy_time::Duration;
use crate::ctrl::*;

use crate::hwa::controllers::PrinterController;
use crate::hwa::controllers::PrinterControllerEvent;
use crate::hwa::controllers::sdcard_controller::SDCardStream;
use crate::hwa::controllers::sdcard_controller::SDCardError;
use printhor_hwa_common::EventStatus;
use printhor_hwa_common::EventFlags;
use printhor_hwa_common::EventBusSubscriber;

#[allow(unused_mut)]
#[allow(unreachable_patterns)]
#[embassy_executor::task(pool_size=1)]
pub(crate) async fn printer_task(
    mut processor: hwa::GCodeProcessor,
    mut printer_controller: PrinterController,
    mut card_controller: hwa::controllers::CardController,
) -> ! {

    const ABORT_ON_FAIL: bool = false;
    let mut subscriber: EventBusSubscriber<'static> = hwa::task_allocations::init_printer_subscriber(processor.event_bus.clone()).await;
    subscriber.wait_until(EventStatus::not_containing(EventFlags::SYS_BOOTING)).await;

    hwa::debug!("printer_task started");
    hwa::debug!("SDCardStream takes {} bytes", core::mem::size_of::<SDCardStream>());

    loop {
        hwa::debug!("Waiting for event");
        match printer_controller.wait().await {
            PrinterControllerEvent::PrintFile(file_path) => {
                hwa::info!("Printing {}.\n", file_path.as_str());
                processor.write("Q. (M24)\n").await;
                let mut print_job_parser = match card_controller.new_stream(file_path.as_str()).await {
                    Ok(stream) => {
                        GCodeLineParser::new(stream)
                    }
                    Err(_e) => {
                        match _e {
                            SDCardError::NoSuchVolume => {
                                processor.write("E. M24 (card not ready)\n").await;
                                continue;
                            }
                            SDCardError::NotFound => {
                                processor.write("E. M24 (file not found)\n").await;
                                continue;
                            }
                            _ => {
                                processor.write("E. M24 (Internal error)\n").await;
                                continue;
                            }
                        }
                    }
                };

                let mut num_gcodes_processed: u32 = 0u32;

                loop {
                    match print_job_parser.next_gcode().await {
                        Err(_error) => {
                            match _error {
                                GCodeLineParserError::ParseError(line) => {
                                    let s = alloc::format!("E. M24 (Parse error at line {}. Ignored)\n", line);
                                    processor.write(s.as_str()).await;
                                }
                                GCodeLineParserError::GCodeNotImplemented(_ln, _gc) => {
                                    let s = alloc::format!("E. {} (Not Implemented at line {})\n", _gc, _ln);
                                    processor.write(s.as_str()).await;
                                    if ABORT_ON_FAIL {
                                        break;
                                    }
                                }
                            }
                            continue;
                        }
                        Ok(result) => {
                            match result {
                                None => { // EOF
                                    let summary = alloc::format!("ok; M24 done. {} gcodes processed\n", num_gcodes_processed);
                                    processor.write(summary.as_str()).await;
                                    break;
                                }
                                Some(gc) => {
                                    num_gcodes_processed += 1;
                                    match gc {
                                        _ => {
                                            hwa::debug!("Executing {}", gc);
                                            match processor.execute(&gc, true).await {
                                                Ok(CodeExecutionSuccess::OK) | Ok(CodeExecutionSuccess::QUEUED) => {

                                                },
                                                Ok(CodeExecutionSuccess::DEFERRED(state)) => {
                                                    subscriber.wait_until(state).await;
                                                },
                                                Err(CodeExecutionFailure::BUSY) => {
                                                    hwa::error!("E. (ExecError BUSY) at line {}", print_job_parser.current_line());
                                                    if ABORT_ON_FAIL {
                                                        break;
                                                    }
                                                },
                                                Err(CodeExecutionFailure::ERR) => {
                                                    hwa::debug!("E. (ExecError ERR)");
                                                },
                                                Err(CodeExecutionFailure::NumericalError) => {
                                                    hwa::debug!("E. (NumericalError ERR)");
                                                },
                                                Err(CodeExecutionFailure::NotYetImplemented) => {
                                                    hwa::debug!("E. (NotImplemented)");
                                                },
                                                Err(CodeExecutionFailure::HomingRequired) => {
                                                    hwa::debug!("E. (Homing required)");
                                                },
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                print_job_parser.close().await;
                hwa::info!("file done");

            }
            PrinterControllerEvent::Abort => {

            }
            _ => {
                hwa::debug!("unexpected event");
                continue
            }
        }
        Timer::after(Duration::from_secs(2)).await;
    }
}