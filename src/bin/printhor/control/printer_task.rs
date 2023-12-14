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

#[allow(unused_mut)]
#[allow(unreachable_patterns)]
#[embassy_executor::task(pool_size=1)]
pub(crate) async fn printer_task(
    mut processor: hwa::GCodeProcessor,
    mut printer_controller: PrinterController,
    mut card_controller: hwa::controllers::CardController,
) -> ! {

    let mut subscriber: EventBusSubscriber<'static> = hwa::task_allocations::init_printer_subscriber(processor.event_bus.clone()).await;
    subscriber.wait_until(EventStatus::not_containing(EventFlags::SYS_BOOTING)).await;

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
                loop {
                    current_line += 1;
                    match gcode_pull(&mut print_job_parser).await {
                        Ok(Some(_gcode)) => {
                            hwa::info!("Processing {}", current_line);
                            // TODO: Process something
                            embassy_time::Timer::after_secs(5).await;
                        }
                        Ok(None) => { // EOF
                            break;
                        }
                        Err(GCodeLineParserError::FatalError) => { // Fatal
                            hwa::error!("Fatal error at line {}", current_line);
                            break;
                        }
                        Err(GCodeLineParserError::GCodeNotImplemented(_ln, _gcode)) => { // Fatal
                            hwa::warn!("Ignoring GCode at line {} ({}): Not implemented", current_line, _gcode.as_str());
                        }
                        Err(_e) => {
                            hwa::warn!("Error procesing GCode at line {} ({:?})", current_line, _e);
                            break;
                        }
                    }
                    // Peek new event to see if job is cancelled
                    if printer_controller.signaled().await {
                        interrupted = true;
                        break;
                    }
                }
                if !interrupted {
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