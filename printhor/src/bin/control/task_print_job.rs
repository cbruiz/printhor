//! This module contains functionality related to the execution of print jobs in a 3D printer.
//! It leverages asynchronous tasks and event-driven programming to manage and control the
//! sequence of actions required to process G-code commands from an SD card and relay them to the printer controller.
//!
//! The task `task_print_job` is defined with the main goal of managing the print job execution. It initializes
//! controllers, waits for system readiness, handles SD card operations, and processes G-code lines. Key events such
//! as setting a new file for printing, resuming a print job, and handling timeouts are meticulously managed.
//!
//! The workflow involves:
//! 1. **Initialization**: The task sets up an event subscriber and waits for the system to finish booting.
//! 2. **Event Handling**: It enters a loop where it waits for the system to be free of alarms (`SYS_ALARM`), then tries
//!    to consume any commands from the printer controller.
//! 3. **G-code Processing**: When a new file is set for printing, it parses the G-code lines asynchronously.
//! 4. **Error Management**: Handles various errors such as file not found on the SD card, or any fatal errors during G-code execution.
//! 5. **State Updates**: Publishes events to signal the state of the print job (e.g., `JOB_PAUSED`, `JOB_COMPLETED`).
//!
//! This approach ensures robust handling of print job execution, making sure that the 3D printer can efficiently
//! and reliably process and execute the G-code commands.
use crate::control;
use crate::hwa;
use embassy_time::Duration;
use embassy_time::{Instant, Timer};

use crate::control::GCodeValue;
use control::{CodeExecutionFailure, CodeExecutionSuccess};
use control::{GCodeCmd, GCodeLineParser, GCodeLineParserError};
use hwa::controllers::PrinterController;
use hwa::controllers::PrinterControllerEvent;
use hwa::sd_card::SDCardError;
use hwa::{CommChannel, EventFlags, EventStatus};

/// The `task_print_job` function manages the execution of a 3D printer's print job.
///
/// # Arguments
/// * `processor` - The GCode processor responsible for handling the GCode commands.
/// * `printer_controller` - The controller for the printer to interface with the hardware.
/// * `card_controller` - The controller for the SD card where GCode files are stored.
///
/// # Description
/// This async function achieves the following tasks:
/// 1. **Initialization**: Sets up an event subscriber and waits for the system to finish booting.
/// 2. **Event Handling**: Enters a loop where it waits for the system to be free of alarms (`SYS_ALARM`) and tries to consume any commands from the printer controller.
/// 3. **G-code Processing**: When a new file is set for printing, it parses the G-code lines asynchronously.
/// 4. **Error Management**: Handles various errors such as file not found on the SD card, or any fatal errors during G-code execution.
/// 5. **State Updates**: Publishes events to signal the state of the print job (e.g., `JOB_PAUSED`, `JOB_COMPLETED`).
///
/// The robust handling of print job execution ensures that the 3D printer can efficiently and reliably process and execute the G-code commands.
/// FIXME: JOB_PAUSE status flag remains!!
#[allow(unused_mut)]
#[embassy_executor::task(pool_size = 1)]
pub async fn task_print_job(
    mut processor: hwa::GCodeProcessor,
    mut printer_controller: PrinterController,
    mut card_controller: hwa::types::SDCardController,
) {
    let event_bus = processor.event_bus.clone();
    let mut subscriber = event_bus.subscriber().await;

    hwa::info!("[task_print_job] Waiting for SYS_BOOTING");
    // Pauses this task until system is ready
    match subscriber
        .ft_wait_for(EventStatus::not_containing(EventFlags::SYS_BOOTING))
        .await
    {
        Ok(_) => hwa::info!("[task_print_job] Got SYS_BOOTING. Continuing."),
        Err(_) => crate::initialization_error(),
    }

    let mut print_job_parser = None;
    let mut job_time = Duration::from_ticks(0);

    loop {
        let _ = subscriber
            .ft_wait_for(EventStatus::not_containing(EventFlags::SYS_ALARM))
            .await;

        match embassy_time::with_timeout(Duration::from_secs(10), printer_controller.consume())
            .await
        {
            Err(_) => {
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [task_print_job] Timeout");
                #[cfg(test)]
                if control::task_integration::INTEGRATION_STATUS.signaled() {
                    hwa::info!("[task_print_job] Ending gracefully");
                    return ();
                }
            }
            Ok(PrinterControllerEvent::SetFile(channel, file_path)) => {
                #[cfg(feature = "trace-commands")]
                hwa::info!(
                    "[trace-commands] [task_print_job] SetFile: {}",
                    file_path.as_str()
                );
                job_time = Duration::from_ticks(0);
                print_job_parser = match card_controller.new_stream(file_path.as_str()).await {
                    Ok(stream) => {
                        let parser = GCodeLineParser::new(stream);
                        processor.write(channel, "File opened: ").await;
                        processor.write(channel, file_path.as_str()).await;
                        processor.write(channel, "\nFile selected\n").await;
                        Some(parser)
                    }
                    Err(_e) => {
                        let error_msg = match _e {
                            SDCardError::NoSuchVolume => "SetFile: Card not ready",
                            SDCardError::NotFound => "SetFile: File not found",
                            _ => "SetFile: Internal error",
                        };
                        hwa::error!("{}", error_msg);
                        processor
                            .write(channel, alloc::format!("error; {}\n", error_msg).as_str())
                            .await;
                        continue;
                    }
                };
                processor
                    .event_bus
                    .publish_event(
                        EventStatus::containing(EventFlags::JOB_PAUSED)
                            .and_not_containing(EventFlags::JOB_COMPLETED),
                    )
                    .await;
            }
            Ok(PrinterControllerEvent::Resume(channel)) => {
                let mut interrupted = false;
                let mut fatal_error = false;
                let job_t0 = Instant::now();
                loop {
                    match gcode_pull(&mut print_job_parser).await {
                        Ok(gcode) => {
                            match &gcode.value {
                                GCodeValue::M2 => {
                                    match printer_controller
                                        .set(PrinterControllerEvent::Abort(channel))
                                        .await
                                    {
                                        Ok(_f) => {
                                            processor
                                                .write(channel, "echo: Print job terminated\n")
                                                .await;
                                            break;
                                        }
                                        Err(_e) => {
                                            let s = alloc::format!(
                                                "echo: M2: Unable to terminate: {:?}\n",
                                                _e
                                            );
                                            processor.write(channel, s.as_str()).await;
                                            fatal_error = true;
                                            hwa::error!("Response: ERR {}", gcode);
                                            break;
                                        }
                                    }
                                }
                                _ => {
                                    #[cfg(feature = "trace-commands")]
                                    hwa::info!("[trace-commands] Executing {}", gcode);

                                    cfg_if::cfg_if! {
                                        if #[cfg(feature = "trace-commands")] {
                                            const CHANNEL: CommChannel = CommChannel::Internal;
                                        }
                                        else {
                                            const CHANNEL: CommChannel = CommChannel::Sink;
                                        }
                                    }

                                    match processor.execute(CHANNEL, &gcode, true).await {
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
                                            hwa::warn!("Ignoring GCode not implemented {}", gcode);
                                        }
                                        Err(CodeExecutionFailure::HomingRequired) => {
                                            fatal_error = true;
                                            hwa::error!(
                                                "Unexpected HomingRequired before {}",
                                                gcode
                                            );
                                            break;
                                        }
                                        Err(CodeExecutionFailure::PowerRequired) => {
                                            fatal_error = true;
                                            hwa::error!(
                                                "Unexpected PowerRequired before {}",
                                                gcode
                                            );
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                        Err(error) => {
                            let current_line = print_job_parser
                                .as_ref()
                                .map_or(0, |parser| parser.get_line());
                            match error {
                                GCodeLineParserError::EOF => {
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
                                    processor
                                        .write(
                                            channel,
                                            alloc::format!(
                                                "Ignoring gcode not supported: {} at {}\n",
                                                _gcode,
                                                current_line
                                            )
                                            .as_str(),
                                        )
                                        .await;
                                    hwa::warn!(
                                        "Ignoring GCode not supported {} at line {}",
                                        _gcode.as_str(),
                                        current_line
                                    );
                                }
                                GCodeLineParserError::ParseError(_ln) => {
                                    hwa::warn!("Parse error at {}", current_line);
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
                        .publish_event(
                            EventStatus::containing(EventFlags::JOB_COMPLETED)
                                .and_not_containing(EventFlags::JOB_PAUSED),
                        )
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
    print_job_parser: &mut Option<hwa::types::SDCardLineParser>,
) -> Result<GCodeCmd, GCodeLineParserError> {
    match print_job_parser.as_mut() {
        Some(parser) => parser.next_gcode().await.map(|mut gc| {
            gc.order_num = parser.get_line();
            gc
        }),
        None => Err(GCodeLineParserError::FatalError),
    }
}
