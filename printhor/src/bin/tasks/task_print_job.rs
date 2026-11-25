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

use crate::{hwa, processing};

use hwa::controllers::PrinterController;
use hwa::controllers::PrinterControllerEvent;
use hwa::controllers::JOB_PROGRESS;
use hwa::sd_card::SDCardError;
use hwa::{CommChannel, EventFlags, EventStatus};
use processing::{CodeExecutionFailure, CodeExecutionSuccess, GCodeCmd, GCodeValue};

use embassy_time::Duration;
use embassy_time::Instant;

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
///
/// Job status flags: `JOB_PAUSED` while a file is selected or the job is paused, `JOB_PRINTING`
/// while running and `JOB_COMPLETED` when the end of the file is reached.
#[allow(unused_mut)]
#[embassy_executor::task(pool_size = 1)]
pub async fn task_print_job(
    mut processor: processing::GCodeProcessor,
    mut printer_controller: PrinterController,
    mut card_controller: hwa::types::SDCardController,
) {
    let event_bus = processor.event_bus.clone();
    let mut subscriber = event_bus.subscriber().await;

    hwa::info!("[task_print_job] Waiting for SYS_BOOTING");
    // Pauses this task until system is ready
    crate::helpers::wait_for_flags(&event_bus, EventFlags::SYS_BOOTING, false).await;
    hwa::info!("[task_print_job] Got SYS_BOOTING. Continuing.");

    let mut print_job_parser: Option<hwa::types::SDCardLineParser> = None;
    // Last M27 S auto-report
    let mut last_report = Instant::now();
    let mut job_time = Duration::from_ticks(0);

    loop {
        let _ = subscriber
            .ft_wait_for(EventStatus::not_containing(EventFlags::SYS_ALARM))
            .await;

        // Woken up at least every M27 S interval, for the auto-report
        let idle_timeout = match JOB_PROGRESS.autoreport() {
            Some((secs, _)) => Duration::from_secs(u64::from(secs)).min(Duration::from_secs(10)),
            None => Duration::from_secs(10),
        };
        match embassy_time::with_timeout(idle_timeout, printer_controller.consume()).await {
            Err(_) => {
                autoreport(&processor, &mut last_report).await;
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [task_print_job] Timeout");
                #[cfg(test)]
                if crate::tasks::task_integration::INTEGRATION_STATUS.signaled() {
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
                // A previously selected file is closed before selecting the new one
                close_job_file(&mut print_job_parser).await;
                JOB_PROGRESS.clear();
                match card_controller.new_stream(file_path.as_str()).await {
                    Ok(mut stream) => {
                        // M27/M73 progress: the stream publishes its position from now on
                        JOB_PROGRESS.start(stream.length());
                        stream.track_progress();
                        // Moved in place before awaiting: the (large) parser is never held twice
                        // in the task state
                        print_job_parser = Some(processing::GCodeLineParser::new(
                            processing::GCodeInputStream::PrintJob(stream),
                        ));
                    }
                    Err(_e) => {
                        let error_msg = match _e {
                            SDCardError::NoSuchVolume => "SetFile: Card not ready",
                            SDCardError::NotFound => "SetFile: File not found",
                            _ => "SetFile: Internal error",
                        };
                        hwa::error!("{}", error_msg);
                        processor
                            .write(channel, crate::processing::LineBuf::<192>::from_fmt(format_args!("error; {}\n", error_msg)).as_str())
                            .await;
                        // Nothing selected: back to Ready, so M24 is rejected instead of
                        // starting a job without a file
                        printer_controller.finish(channel);
                        publish_job_idle(&processor).await;
                        continue;
                    }
                }
                // Marlin format: hosts read the size to compute the progress
                processor.write(channel, "File opened: ").await;
                processor.write(channel, file_path.as_str()).await;
                processor
                    .write_u32(
                        channel,
                        " Size: ",
                        JOB_PROGRESS.sd_bytes().map_or(0, |(_, s)| s),
                        "\nFile selected\n",
                    )
                    .await;
                processor
                    .event_bus
                    .publish_event(
                        EventStatus::containing(EventFlags::JOB_PAUSED)
                            .and_not_containing(EventFlags::JOB_COMPLETED),
                    )
                    .await;
            }
            Ok(PrinterControllerEvent::Resume(channel)) => {
                // A stale event (e.g. M24 followed by M524 before it was consumed): the status
                // is the reference
                if !printer_controller.is_printing().await {
                    hwa::debug!("[task-print-job] Stale Resume ignored");
                    continue;
                }
                if print_job_parser.is_none() {
                    // Should not happen (the controller rejects M24 with no file selected), but
                    // it is recoverable: report it and go back to Ready
                    hwa::error!("[task-print-job] Resume with no file selected");
                    processor.write(channel, "error; No file selected\n").await;
                    printer_controller.finish(channel);
                    publish_job_idle(&processor).await;
                    continue;
                }
                processor
                    .event_bus
                    .publish_event(
                        EventStatus::containing(EventFlags::JOB_PRINTING)
                            .and_not_containing(EventFlags::JOB_PAUSED)
                            .and_not_containing(EventFlags::JOB_COMPLETED),
                    )
                    .await;
                let mut interrupted = false;
                let mut fatal_error = false;
                let job_t0 = Instant::now();
                loop {
                    match gcode_pull(&mut print_job_parser).await {
                        Ok(gcode) => {
                            match &gcode.value {
                                GCodeValue::M2 => {
                                    // End of program: the job ends here (as at EOF)
                                    processor
                                        .write(channel, "echo: Print job terminated\n")
                                        .await;
                                    break;
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
                                            hwa::debug!(
                                                "[task-print-job] Response: OK {} (I)",
                                                gcode
                                            );
                                        }
                                        Ok(CodeExecutionSuccess::CONSUMED) => {
                                            hwa::debug!(
                                                "[task-print-job] Response: OK {} (C)",
                                                gcode
                                            );
                                        }
                                        Ok(CodeExecutionSuccess::QUEUED) => {
                                            hwa::debug!(
                                                "[task-print-job] Response: OK {} (Q)",
                                                gcode
                                            );
                                        }
                                        Ok(CodeExecutionSuccess::DEFERRED(_status)) => {
                                            hwa::debug!("[task-print-job] Deferred {} (D)", gcode);
                                            // Interruptible: M524/M25 must not wait for
                                            // a condition that may never come (e.g. heating)
                                            let waited = embassy_futures::select::select(
                                                subscriber.ft_wait_for(_status),
                                                wait_not_printing(&printer_controller),
                                            )
                                            .await;
                                            if let embassy_futures::select::Either::Second(_) =
                                                waited
                                            {
                                                interrupted = true;
                                                break;
                                            }
                                            if let embassy_futures::select::Either::First(Err(_)) =
                                                waited
                                            {
                                                // SYS_ALARM (machine halted): the job is aborted
                                                hwa::error!(
                                                    "[task-print-job] SYS_ALARM raised. Aborting job"
                                                );
                                                fatal_error = true;
                                                break;
                                            } else {
                                                hwa::debug!(
                                                    "[task-print-job] Response: OK {} (D)",
                                                    gcode
                                                );
                                            }
                                        }
                                        Err(CodeExecutionFailure::BUSY) => {
                                            fatal_error = true;
                                            hwa::error!(
                                                "[task-print-job] Response: BUSY {}",
                                                gcode
                                            );
                                            break;
                                        }
                                        Err(
                                            CodeExecutionFailure::ERR
                                            | CodeExecutionFailure::NumericalError,
                                        ) => {
                                            fatal_error = true;
                                            hwa::error!("[task-print-job] Response: ERR {}", gcode);
                                            break;
                                        }
                                        Err(CodeExecutionFailure::NotYetImplemented) => {
                                            hwa::warn!(
                                                "[task-print-job] Ignoring GCode not implemented {}",
                                                gcode
                                            );
                                        }
                                        Err(CodeExecutionFailure::HomingRequired) => {
                                            fatal_error = true;
                                            hwa::error!(
                                                "[task-print-job] Unexpected HomingRequired before {}",
                                                gcode
                                            );
                                            break;
                                        }
                                        Err(CodeExecutionFailure::PowerRequired) => {
                                            fatal_error = true;
                                            hwa::error!(
                                                "[task-print-job] Unexpected PowerRequired before {}",
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
                                processing::GCodeLineParserError::EOF => {
                                    // EOF
                                    break;
                                }
                                processing::GCodeLineParserError::FatalError => {
                                    // Fatal
                                    fatal_error = true;
                                    hwa::error!(
                                        "[task-print-job] Fatal error at line {}",
                                        current_line
                                    );
                                    break;
                                }
                                processing::GCodeLineParserError::GCodeNotImplemented(
                                    _ln,
                                    _gcode,
                                ) => {
                                    processor
                                        .write(
                                            channel,
                                            crate::processing::LineBuf::<192>::from_fmt(format_args!(
                                                "[task-print-job] Ignoring gcode not supported: {} at {}\n",
                                                _gcode,
                                                current_line
                                            ))
                                            .as_str(),
                                        )
                                        .await;
                                    hwa::warn!(
                                        "[task-print-job] Ignoring GCode not supported {} at line {}",
                                        _gcode.as_str(),
                                        current_line
                                    );
                                }
                                processing::GCodeLineParserError::ParseError(_ln, _why) => {
                                    hwa::warn!(
                                        "[task-print-job] Parse error at line {}: {} (skipped)",
                                        current_line,
                                        _why
                                    );
                                }
                                // A file line with a checksum (e.g. written with M28 by a host
                                // that numbers its lines): skipped as any line not parsed
                                processing::GCodeLineParserError::BadChecksum(_ln, _computed, _received) => {
                                    hwa::warn!(
                                        "[task-print-job] Checksum mismatch at line {} (received {}, computed {}): skipped",
                                        current_line,
                                        _received,
                                        _computed
                                    );
                                }
                                // The line number check is only enabled for host channels
                                processing::GCodeLineParserError::LineNumberMismatch(_) => {}
                            }
                        }
                    }
                    // Paused (M25) or aborted (M524): the status changes before the event is
                    // consumed, so the job stops right after the current command
                    if !printer_controller.is_printing().await {
                        interrupted = true;
                        break;
                    }
                    autoreport(&processor, &mut last_report).await;
                }
                // An error caused by an intentional interruption (e.g. a move cut by M524's quick
                // stop) is not a job failure
                if fatal_error && !printer_controller.is_printing().await {
                    fatal_error = false;
                    interrupted = true;
                }
                if subscriber
                    .ft_wait_until(EventFlags::MOV_QUEUE_EMPTY)
                    .await
                    .is_err()
                {
                    hwa::warn!("[task-print-job] SYS_ALARM raised");
                }
                job_time += job_t0.elapsed();
                if fatal_error {
                    // Ended by the job itself: no event is involved (it could be rejected if the
                    // status changed meanwhile), the job is just closed and the machine is idle
                    let line = print_job_parser.as_ref().map_or(0, |p| p.get_line());
                    printer_controller.finish(channel);
                    publish_job_idle(&processor).await;
                    JOB_PROGRESS.clear();
                    close_job_file(&mut print_job_parser).await;
                    processor
                        .write(
                            channel,
                            crate::processing::LineBuf::<192>::from_fmt(format_args!("echo: Job aborted (error at line {})\n", line)).as_str(),
                        )
                        .await;
                } else if !interrupted {
                    processor
                        .event_bus
                        .publish_event(
                            EventStatus::containing(EventFlags::JOB_COMPLETED)
                                .and_not_containing(EventFlags::JOB_PAUSED)
                                .and_not_containing(EventFlags::JOB_PRINTING),
                        )
                        .await;
                    printer_controller.finish(channel);
                    JOB_PROGRESS.clear();
                    processor.write(channel, "Done printing file\n").await;
                    // Job completed or cancelled
                    // Closing the stream releases the file (and its directories) on the card
                    close_job_file(&mut print_job_parser).await;
                    hwa::info!(
                        "[task-print-job] Job completed in {} ms",
                        job_time.as_millis()
                    );
                }
            }
            Ok(PrinterControllerEvent::Pause(channel)) => {
                // The job loop was interrupted. The parser is kept so that M24 resumes from here.
                // A stale event (already resumed or aborted) does not change the flags
                if !matches!(
                    printer_controller.status().await,
                    hwa::controllers::PrinterControllerStatus::Paused(_)
                ) {
                    continue;
                }
                processor
                    .event_bus
                    .publish_event(
                        EventStatus::containing(EventFlags::JOB_PAUSED)
                            .and_not_containing(EventFlags::JOB_PRINTING),
                    )
                    .await;
                let _ = channel;
            }
            Ok(PrinterControllerEvent::Abort(channel)) => {
                // The aborted file is closed in any case. The flags only when nothing new was
                // selected meanwhile (M524 quickly followed by M23)
                if matches!(
                    printer_controller.status().await,
                    hwa::controllers::PrinterControllerStatus::Ready(_)
                ) {
                    publish_job_idle(&processor).await;
                }
                JOB_PROGRESS.clear();
                processor.write(channel, "echo: Job aborted\n").await;
                close_job_file(&mut print_job_parser).await;
                hwa::debug!("[task-print-job] File done");
            }
        }
    }
}

/// Sends the `M27` report to the `M27 S` channel when its interval elapsed (no-op when off)
async fn autoreport(processor: &processing::GCodeProcessor, last_report: &mut Instant) {
    if let Some((secs, channel)) = JOB_PROGRESS.autoreport() {
        if last_report.elapsed() >= Duration::from_secs(u64::from(secs)) {
            *last_report = Instant::now();
            processor.write_sd_status(channel).await;
        }
    }
}

/// Closes the job file (if any), releasing its entries on the card.
///
/// The (large) parser is reduced to its closer synchronously, before awaiting: the task state never
/// holds a second copy of the parser while closing.
async fn close_job_file(print_job_parser: &mut Option<hwa::types::SDCardLineParser>) {
    let closer = print_job_parser
        .take()
        .and_then(|parser| match parser.into_stream() {
            processing::GCodeInputStream::PrintJob(stream) => Some(stream.into_closer()),
            // Never: the print job parser is only built on the job stream
            #[allow(unreachable_patterns)]
            _ => None,
        });
    if let Some(closer) = closer {
        closer.close().await;
    }
}

/// Job flags when no job is selected nor running
async fn publish_job_idle(processor: &processing::GCodeProcessor) {
    processor
        .event_bus
        .publish_event(
            EventStatus::not_containing(EventFlags::JOB_PAUSED)
                .and_not_containing(EventFlags::JOB_PRINTING),
        )
        .await;
}

/// Completes when the job is not meant to be running any more (paused or aborted)
async fn wait_not_printing(printer_controller: &PrinterController) {
    while printer_controller.is_printing().await {
        embassy_time::Timer::after(Duration::from_millis(50)).await;
    }
}

async fn gcode_pull(
    print_job_parser: &mut Option<hwa::types::SDCardLineParser>,
) -> Result<GCodeCmd, processing::GCodeLineParserError> {
    match print_job_parser.as_mut() {
        Some(parser) => parser
            .next_gcode(CommChannel::Internal)
            .await
            .map(|mut gc| {
                gc.order_num = parser.get_line();
                gc
            }),
        None => Err(processing::GCodeLineParserError::FatalError),
    }
}
