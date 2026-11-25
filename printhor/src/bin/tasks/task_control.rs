//! Task control
use crate::{hwa, processing};

#[cfg(feature = "with-sd-card")]
use crate::hwa::controllers::sd_card_controller::SDEntryType;
use async_gcode::AsyncParserState;
use embassy_time::{Duration, TimeoutError};

#[allow(unused)]
use hwa::{EventFlags, EventStatus};

cfg_if::cfg_if! {
    if #[cfg(feature = "with-print-job")] {
        cfg_if::cfg_if! {
            if #[cfg(not(feature = "with-sd-card"))] {
                compiler_error("Feature with-print-job requires with-sd-card")
            }
        }
    }
}

// Marlin hosts/displays only recognise `Error:` (capitalised, colon) as an error marker
#[cfg(feature = "marlin-compat")]
const ERR_PREFIX: &str = "Error:";
#[cfg(not(feature = "marlin-compat"))]
const ERR_PREFIX: &str = "error;";
#[cfg(feature = "marlin-compat")]
const ERR_PARSER: &str = "Error:ParserError\n";
#[cfg(not(feature = "marlin-compat"))]
const ERR_PARSER: &str = "error; (ParserError)\n";
#[cfg(feature = "marlin-compat")]
const ERR_INTERNAL: &str = "Error:Internal error\n";
#[cfg(not(feature = "marlin-compat"))]
const ERR_INTERNAL: &str = "error; Internal error\n";

/// Answers a line that could not be parsed: the host resends it when it numbers its lines
async fn report_parse_error(
    processor: &mut processing::GCodeProcessor,
    channel: hwa::CommChannel,
    gcode_input_stream: &processing::GCodeMultiplexedInputStream,
) {
    processor.write(channel, ERR_PARSER).await;
    if let Some(last_line) = gcode_input_stream.get_gcode_line(channel) {
        let s = processing::LineBuf::<32>::from_fmt(format_args!("Resend: {}\n", last_line.wrapping_add(1)));
        processor.write(channel, s.as_str()).await;
    }
    processor.write_ok(channel).await;
}

/// Answers a line with a wrong checksum as Marlin: the host resends the line after the last one
/// accepted
async fn report_bad_checksum(
    processor: &mut processing::GCodeProcessor,
    channel: hwa::CommChannel,
    gcode_input_stream: &processing::GCodeMultiplexedInputStream,
) {
    match gcode_input_stream.get_gcode_line(channel) {
        Some(last_line) => {
            let s = processing::LineBuf::<96>::from_fmt(format_args!(
                "Error:checksum mismatch, Last Line: {}\nResend: {}\n",
                last_line as i32,
                last_line.wrapping_add(1)
            ));
            processor.write(channel, s.as_str()).await;
        }
        None => processor.write(channel, "Error:checksum mismatch\n").await,
    }
    processor.write_ok(channel).await;
}

/// Tracks, per channel, whether its stream already delivered something (a G-code or an error).
///
/// The first parser error of a channel that has not delivered anything yet is discarded silently:
/// it is the line noise of a peer starting up (e.g. a BTT TFT emits a framing error and a NUL byte
/// when it boots, and would show the `Error:` reply as an error popup).
#[derive(Default)]
struct StreamStartFilter {
    started: u8,
}

impl StreamStartFilter {
    const fn bit(channel: hwa::CommChannel) -> u8 {
        match channel {
            #[cfg(feature = "with-serial-usb")]
            hwa::CommChannel::SerialUsb => 1 << 0,
            #[cfg(feature = "with-serial-port-1")]
            hwa::CommChannel::SerialPort1 => 1 << 1,
            #[cfg(feature = "with-serial-port-2")]
            hwa::CommChannel::SerialPort2 => 1 << 2,
            hwa::CommChannel::Internal => 1 << 3,
            hwa::CommChannel::Sink => 1 << 4,
        }
    }

    /// Marks the channel as started. Returns whether it already was
    fn start(&mut self, channel: hwa::CommChannel) -> bool {
        let bit = Self::bit(channel);
        let was_started = self.started & bit != 0;
        self.started |= bit;
        was_started
    }
}

/// A file being written (M28): the lines of `channel` go to it until M29
#[cfg(feature = "with-sd-card")]
struct SDWriting {
    path: hwa::types::SDCardPath,
}

#[cfg(feature = "with-sd-card")]
impl SDWriting {
    /// Flushes and closes the file
    async fn close(mut self, card_controller: &mut hwa::types::SDCardController) {
        card_controller.close_path(&mut self.path).await;
    }
}

/// M28: creates the file and stores the next lines of the channel in it (see [handle_raw_line])
#[cfg(feature = "with-sd-card")]
async fn start_writing(
    processor: &mut processing::GCodeProcessor,
    channel: hwa::CommChannel,
    file: Option<&str>,
    gcode_input_stream: &mut processing::GCodeMultiplexedInputStream,
    card_controller: &mut hwa::types::SDCardController,
    #[cfg(feature = "with-print-job")] printer_controller: &mut hwa::controllers::PrinterController,
    writing: &mut Option<SDWriting>,
) -> processing::CodeExecutionResult {
    let file = file.unwrap_or("").trim();
    #[cfg(feature = "with-print-job")]
    if printer_controller.is_printing().await {
        processor.write(channel, "echo: M28: Not allowed while printing\n").await;
        return Err(processing::CodeExecutionFailure::ERR);
    }
    if writing.is_some() || file.is_empty() {
        processor.write(channel, "echo: M28: Already writing a file, or no file name\n").await;
        return Err(processing::CodeExecutionFailure::ERR);
    }
    match card_controller.create_file(file).await {
        Ok(path) => {
            gcode_input_stream.start_capture(channel);
            writing.replace(SDWriting { path });
            let s = processing::LineBuf::<128>::from_fmt(format_args!("Writing to file: {}\n", file));
            processor.write(channel, s.as_str()).await;
            Ok(processing::CodeExecutionSuccess::OK)
        }
        Err(_e) => {
            let s = processing::LineBuf::<128>::from_fmt(format_args!("open failed, File: {}. ({:?})\n", file, _e));
            processor.write(channel, s.as_str()).await;
            Err(processing::CodeExecutionFailure::ERR)
        }
    }
}

/// A line of the channel writing a file (M28): stored, or M29 (ends the file)
#[cfg(feature = "with-sd-card")]
async fn handle_raw_line(
    processor: &mut processing::GCodeProcessor,
    channel: hwa::CommChannel,
    line: Result<core::ops::Range<usize>, processing::GCodeLineParserError>,
    gcode_input_stream: &mut processing::GCodeMultiplexedInputStream,
    card_controller: &mut hwa::types::SDCardController,
    writing: &mut Option<SDWriting>,
) {
    let range = match line {
        Ok(range) => range,
        Err(processing::GCodeLineParserError::LineNumberMismatch(last_line)) => {
            hwa::warn!("[{:?}] Line number out of sequence: line not stored", channel);
            let s = processing::LineBuf::<128>::from_fmt(format_args!(
                "Error:Line Number is not Last Line Number+1, Last Line: {}\nResend: {}\n",
                last_line as i32,
                last_line.wrapping_add(1)
            ));
            processor.write(channel, s.as_str()).await;
            processor.write_ok(channel).await;
            return;
        }
        Err(processing::GCodeLineParserError::EOF) => return,
        Err(processing::GCodeLineParserError::BadChecksum(_ln, _computed, _received)) => {
            hwa::warn!(
                "[{:?}] Checksum mismatch (received {}, computed {}): line not stored",
                channel,
                _received,
                _computed
            );
            report_bad_checksum(processor, channel, gcode_input_stream).await;
            return;
        }
        Err(processing::GCodeLineParserError::ParseError(_ln, _why)) => {
            // Longer than a line can be, or unreadable: the host resends it
            hwa::warn!("[{:?}] Parse error: {}: line not stored", channel, _why);
            report_parse_error(processor, channel, gcode_input_stream).await;
            return;
        }
        Err(_other) => {
            hwa::warn!("[{:?}] Unexpected error: line not stored", channel);
            report_parse_error(processor, channel, gcode_input_stream).await;
            return;
        }
    };
    if range.is_empty() {
        // An empty line: nothing to store, and no answer (as when executing)
        return;
    }
    let raw = gcode_input_stream.raw_line(range);
    // M29 (not M290..): the end of the file
    let is_m29 = raw.len() >= 3
        && raw[0].eq_ignore_ascii_case(&b'm')
        && &raw[1..3] == b"29"
        && raw.get(3).is_none_or(|b| !b.is_ascii_digit() && *b != b'.');
    let Some(current) = writing.as_ref() else {
        gcode_input_stream.stop_capture();
        return;
    };
    if is_m29 {
        gcode_input_stream.stop_capture();
        if let Some(current) = writing.take() {
            current.close(card_controller).await;
        }
        processor.write(channel, "Done saving file.\n").await;
        processor.write_ok(channel).await;
        return;
    }
    let file = match current.path.last() {
        Some(file) => *file,
        None => return,
    };
    // The line, then its end of line
    let mut stored = card_controller.write(file, raw).await;
    if stored.is_ok() {
        stored = card_controller.write(file, b"\n").await;
    }
    match stored {
        Ok(()) => processor.write_ok(channel).await,
        Err(_e) => {
            // The file is closed (what was written is kept) and the lines are executed again
            gcode_input_stream.stop_capture();
            if let Some(current) = writing.take() {
                current.close(card_controller).await;
            }
            let s = processing::LineBuf::<64>::from_fmt(format_args!("{} SD write failed: {:?}\n", ERR_PREFIX, _e));
            processor.write(channel, s.as_str()).await;
            processor.write_ok(channel).await;
        }
    }
}

pub async fn do_with_timeout<F: core::future::Future>(
    _timeout: Duration,
    fut: F,
) -> Result<F::Output, TimeoutError> {
    //Ok(fut.await)
    embassy_time::with_timeout(_timeout, fut).await
}

#[embassy_executor::task(pool_size = 1)]
pub async fn task_control(
    mut processor: processing::GCodeProcessor,
    mut gcode_input_stream: processing::GCodeMultiplexedInputStream,
    #[cfg(feature = "with-print-job")] mut printer_controller: hwa::controllers::PrinterController,
    #[cfg(feature = "with-sd-card")] mut card_controller: hwa::types::SDCardController,
) {
    // Event bus subscriber (only used to report its status on a host channel timeout)
    #[cfg(any(
        feature = "with-serial-usb",
        feature = "with-serial-port-1",
        feature = "with-serial-port-2"
    ))]
    let mut subscriber = processor.event_bus.clone().subscriber().await;

    hwa::info!("[task_control] Waiting for SYS_READY");
    // Pauses this task until system is ready (also in alarm mode: M999 must be accepted)
    crate::helpers::wait_for_flags(&processor.event_bus, hwa::EventFlags::SYS_READY, true).await;
    hwa::info!("[task_control] Got SYS_READY. Continuing.");

    let mut stream_start = StreamStartFilter::default();
    #[cfg(feature = "with-sd-card")]
    let mut writing: Option<SDWriting> = None;

    // The task loop
    loop {
        let input = match do_with_timeout(Duration::from_secs(30), gcode_input_stream.next_gcode()).await {
            Err(e) => Err(e),
            #[cfg(feature = "with-sd-card")]
            Ok((processing::InputEvent::Raw(line), channel)) => {
                stream_start.start(channel);
                handle_raw_line(&mut processor, channel, line, &mut gcode_input_stream,
                    &mut card_controller, &mut writing).await;
                continue;
            }
            Ok((processing::InputEvent::GCode(result), channel)) => Ok((result, channel)),
        };
        match input {
            // Timeout
            Err(_) => {
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-serial-usb")] {
                        manage_timeout(&mut processor, &mut subscriber, &mut gcode_input_stream,
                            hwa::CommChannel::SerialUsb).await;
                    }
                }
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-serial-port-1")] {
                        manage_timeout(&mut processor, &mut subscriber, &mut gcode_input_stream,
                            hwa::CommChannel::SerialPort1).await;
                    }
                }
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-serial-port-2")] {
                        manage_timeout(&mut processor, &mut subscriber, &mut gcode_input_stream,
                            hwa::CommChannel::SerialPort2).await;
                    }
                }
                #[cfg(test)]
                if crate::tasks::task_integration::INTEGRATION_STATUS.signaled() {
                    hwa::info!("[task_control] Ending gracefully");
                    return ();
                }
            }
            Ok((Err(processing::GCodeLineParserError::ParseError(_ln, _why)), channel))
                if !stream_start.start(channel) =>
            {
                hwa::warn!("[{:?}] First stream error ignored (peer start-up noise)", channel);
            }
            Ok((Err(processing::GCodeLineParserError::ParseError(_ln, _why)), channel)) => {
                hwa::warn!("[{:?}] Parse error: {}", channel, _why);
                report_parse_error(&mut processor, channel, &gcode_input_stream).await;
            }
            Ok((Err(processing::GCodeLineParserError::BadChecksum(_ln, _computed, _received)), channel)) => {
                stream_start.start(channel);
                hwa::warn!("[{:?}] Checksum mismatch (received {}, computed {})", channel, _received, _computed);
                report_bad_checksum(&mut processor, channel, &gcode_input_stream).await;
            }
            Ok((Err(processing::GCodeLineParserError::LineNumberMismatch(last_line)), channel)) => {
                stream_start.start(channel);
                hwa::warn!("[{:?}] Line number out of sequence", channel);
                let s = crate::processing::LineBuf::<192>::from_fmt(format_args!(
                    "Error:Line Number is not Last Line Number+1, Last Line: {}\nResend: {}\n",
                    last_line as i32,
                    last_line.wrapping_add(1)
                ));
                processor.write(channel, s.as_str()).await;
                processor.write_ok(channel).await;
            }
            Ok((
                Err(processing::GCodeLineParserError::GCodeNotImplemented(_ln, _gcode_name)),
                channel,
            )) => {
                stream_start.start(channel);
                hwa::error!("GCode {} (NotImplemented)", _gcode_name.as_str());
                #[cfg(feature = "marlin-compat")]
                let s = crate::processing::LineBuf::<192>::from_fmt(format_args!("echo:Unknown command: \"{}\"\n", _gcode_name));
                #[cfg(not(feature = "marlin-compat"))]
                let s = crate::processing::LineBuf::<192>::from_fmt(format_args!("error; {} (NotImplemented)\n", _gcode_name));
                processor.write(channel, s.as_str()).await;
                processor.write_ok(channel).await;
            }
            Ok((Err(processing::GCodeLineParserError::FatalError), channel)) => {
                hwa::error!("[{:?}] GCode N/A (Internal error)", channel);
                let s = ERR_INTERNAL;
                processor.write(channel, s).await;
                processor.write_ok(channel).await;
            }
            Ok((Err(processing::GCodeLineParserError::EOF), _channel)) => {
                hwa::warn!("{:?} Got EOF", _channel);
                //embassy_time::Timer::after_secs(1).await; // Avoid respawn too fast
            }
            #[cfg(feature = "with-sd-card")]
            Ok((Ok(gc), channel)) if matches!(gc.value, processing::GCodeValue::M28(_)) => {
                stream_start.start(channel);
                let file = match &gc.value {
                    processing::GCodeValue::M28(file) => file.as_deref(),
                    _ => None,
                };
                let response = start_writing(&mut processor, channel, file, &mut gcode_input_stream,
                    &mut card_controller,
                    #[cfg(feature = "with-print-job")] &mut printer_controller,
                    &mut writing).await;
                report(response, &gc, channel, &mut processor).await;
            }
            Ok((Ok(gc), channel)) => {
                stream_start.start(channel);
                hwa::debug!("{:?} got {:?}", channel, gc);
                let response = execute(
                    &mut processor,
                    channel,
                    &gc,
                    #[cfg(feature = "with-sd-card")]
                    &mut card_controller,
                    #[cfg(feature = "with-print-job")]
                    &mut printer_controller,
                )
                .await;
                report(response, &gc, channel, &mut processor).await;
            }
        }
    }
}

/// Without host channels it is not used. Only `dead_code` is allowed: the whole `unused` group
/// would also silence `unused_must_use`, which reports a future not awaited (such as a `reset`)
#[allow(dead_code)]
async fn manage_timeout<M>(
    processor: &mut processing::GCodeProcessor,
    subscriber: &mut hwa::EventBusSubscriber<'static, M>,
    gcode_input_stream: &mut processing::GCodeMultiplexedInputStream,
    comm_channel: hwa::CommChannel,
) where
    M: hwa::AsyncRawMutex,
{
    let current_parser_state = gcode_input_stream.get_state(comm_channel);

    #[cfg(feature = "trace-commands")]
    hwa::info!(
        "[trace-commands] [task_control] Timeout at {:?}: [State: {:?} Line: {} TaggedLine: {:?}]",
        comm_channel,
        crate::processing::LineBuf::<192>::from_fmt(format_args!("{:?}", current_parser_state)).as_str(),
        gcode_input_stream.get_line(comm_channel),
        gcode_input_stream.get_gcode_line(comm_channel),
    );

    match current_parser_state {
        // Parser is currently ready to accept a gcode. Nothing to do
        AsyncParserState::Start(_) => {
            gcode_input_stream.reset(comm_channel).await;
        }
        // Parser is at any other intermediate state
        _ => {
            // Notify there is a timeout on [CommChannel::Internal] channel
            #[cfg(feature = "trace-commands")]
            processor
                .write(hwa::CommChannel::Internal, "echo: Control timeout.\n")
                .await;
            // Dumps the relevant status for a more easy troubleshooting
            // Heap-free (truncated when too long)
            use core::fmt::Write;
            let mut z3 = processing::LineBuf::<192>::new();
            let _ = write!(z3, "echo: EventBus status: {:?}\n", subscriber.get_status().await);
            processor
                .write(hwa::CommChannel::Internal, z3.as_str())
                .await;

            let mut z = processing::LineBuf::<192>::new();
            let _ = write!(
                z,
                "echo: Will reset channel: {:?} state was: {:?}\n",
                comm_channel,
                current_parser_state
            );
            processor
                .write(hwa::CommChannel::Internal, z.as_str())
                .await;
            gcode_input_stream.reset(comm_channel).await;
            // Submit an err to give up the intermediate state.
            processor.write(comm_channel, "err\n").await;
        }
    }
    #[cfg(feature = "trace-commands")]
    processor
        .write(comm_channel, "echo: Control timeout.\n")
        .await;
}

/*
Process and consume gcode commands
A set of commands will be directly executed
Remaining will be delegated to the processor.
 */
pub async fn execute(
    processor: &mut processing::GCodeProcessor,
    channel: hwa::CommChannel,
    gc: &processing::GCodeCmd,
    #[cfg(feature = "with-sd-card")] _card_controller: &mut hwa::types::SDCardController,
    #[cfg(feature = "with-print-job")]
    _printer_controller: &mut hwa::controllers::PrinterController,
) -> processing::CodeExecutionResult {
    match &gc.value {
        processing::GCodeValue::Nop => {
            hwa::error!("Go Nop");
            // Should not happen
            Err(processing::CodeExecutionFailure::ERR)
        }
        #[cfg(feature = "grbl-compat")]
        processing::GCodeValue::Status => {
            // TODO provide GRBL compatibility status
            processor
                .write(
                    channel,
                    "<Idle|MPos:0.000,0.000,0.000|Pn:XP|FS:0,0|WCO:0.000,0.000,0.000>\n",
                )
                .await;
            Ok(processing::CodeExecutionSuccess::CONSUMED)
        }
        #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
        processing::GCodeValue::M2 => {
            match _printer_controller
                .set(hwa::controllers::PrinterControllerEvent::Abort(channel))
                .await
            {
                Ok(_f) => {
                    processor
                        .write(channel, "echo: Print job terminated\n")
                        .await;
                    Ok(processing::CodeExecutionSuccess::OK)
                }
                // Idempotent: no print job (e.g. M2 at the end of a streamed file), answered ok
                Err(hwa::controllers::PrinterControllerError::NoEffect) => {
                    Ok(processing::CodeExecutionSuccess::OK)
                }
                Err(_e) => {
                    let s = crate::processing::LineBuf::<192>::from_fmt(format_args!("echo: M2: Unable to terminate: {:?}\n", _e));
                    processor.write(channel, s.as_str()).await;
                    Err(processing::CodeExecutionFailure::ERR)
                }
            }
        }
        #[cfg(feature = "with-sd-card")]
        processing::GCodeValue::M20(_path, _long_names) => {
            // Borrowed: no copy of the (up to 128 bytes) path in the heap
            let path = _path.as_deref().unwrap_or("/");
            match list_files(processor, channel, _card_controller, path, *_long_names).await {
                Ok(()) => {
                    processor.write(channel, "ok\n").await;
                    Ok(processing::CodeExecutionSuccess::CONSUMED)
                }
                Err(_e) => {
                    let reason = _card_controller.mount_error().await;
                    let s = crate::processing::LineBuf::<192>::from_fmt(format_args!("echo: M20: Unable to list: {:?} {}\n", _e, reason.as_str()));
                    processor.write(channel, s.as_str()).await;
                    Err(processing::CodeExecutionFailure::ERR)
                }
            }
        }
        // The long path of a file or directory (Marlin's format: `/<long name>/...`, each entry by
        // its short name when it has no long one)
        #[cfg(feature = "with-sd-card")]
        processing::GCodeValue::M33(_path) => {
            let path = _path.as_deref().unwrap_or("/");
            match _card_controller.long_path(path).await {
                Ok(long) => {
                    processor.write(channel, long.as_str()).await;
                    processor.write(channel, "\n").await;
                    Ok(processing::CodeExecutionSuccess::OK)
                }
                Err(_e) => {
                    let s = crate::processing::LineBuf::<192>::from_fmt(format_args!("echo: M33: {} not found\n", path));
                    processor.write(channel, s.as_str()).await;
                    Err(processing::CodeExecutionFailure::ERR)
                }
            }
        }
        #[cfg(feature = "with-sd-card")]
        processing::GCodeValue::M21 => match _card_controller.mount().await {
            Ok(()) => {
                processor.write(channel, "echo:SD card ok\n").await;
                Ok(processing::CodeExecutionSuccess::OK)
            }
            Err(_e) => {
                let reason = _card_controller.mount_error().await;
                let s = crate::processing::LineBuf::<128>::from_fmt(format_args!("echo:SD init fail: {}\n", reason.as_str()));
                processor.write(channel, s.as_str()).await;
                Err(processing::CodeExecutionFailure::ERR)
            }
        },
        #[cfg(feature = "with-sd-card")]
        processing::GCodeValue::M22 => match _card_controller.unmount().await {
            Ok(()) => {
                processor.write(channel, "echo:SD card released\n").await;
                Ok(processing::CodeExecutionSuccess::OK)
            }
            Err(_e) => {
                processor.write(channel, "echo: M22: SD card in use\n").await;
                Err(processing::CodeExecutionFailure::ERR)
            }
        },
        // Outside a file being written (see M28) there is nothing to end
        #[cfg(feature = "with-sd-card")]
        processing::GCodeValue::M29 => Ok(processing::CodeExecutionSuccess::OK),
        #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
        processing::GCodeValue::M524 => {
            match _printer_controller
                .set(hwa::controllers::PrinterControllerEvent::Abort(channel))
                .await
            {
                Ok(_) => {
                    // Not printing any more from now on (M27/M73 reports), even before the print
                    // job task closes the file
                    hwa::controllers::JOB_PROGRESS.clear();
                    #[cfg(feature = "with-motion")]
                    processor.quick_stop().await;
                    processor.shutdown_heaters().await;
                    #[cfg(feature = "with-fan-layer")]
                    processor.fan_layer.lock().await.set_power(0);
                    processor.write(channel, "echo: Print job aborted\n").await;
                    Ok(processing::CodeExecutionSuccess::OK)
                }
                // Repeated (or nothing to do): no effect, answered ok as Marlin does
                Err(hwa::controllers::PrinterControllerError::NoEffect) => {
                    processor.write(channel, "echo: M524: No print job to abort\n").await;
                    Ok(processing::CodeExecutionSuccess::OK)
                }
                Err(_e) => {
                    let s = crate::processing::LineBuf::<192>::from_fmt(format_args!("echo: M524: Unable to abort: {:?}\n", _e));
                    processor.write(channel, s.as_str()).await;
                    Err(processing::CodeExecutionFailure::ERR)
                }
            }
        }
        #[cfg(feature = "with-print-job")]
        processing::GCodeValue::M23(file) => {
            let file_name = file.clone().unwrap_or_default();
            // Only checks that it can be opened. Reduced to its closer before awaiting, so the
            // stream (~0.5 KB) is not part of the task state
            let opened = _card_controller
                .new_stream(file_name.as_str())
                .await
                .map(|stream| stream.into_closer());
            match opened {
                Ok(closer) => closer.close().await,
                Err(_e) => {
                    let s = crate::processing::LineBuf::<192>::from_fmt(format_args!("open failed, File: {}.\n", file_name));
                    processor.write(channel, s.as_str()).await;
                    return Err(processing::CodeExecutionFailure::ERR);
                }
            }
            match _printer_controller
                .set(hwa::controllers::PrinterControllerEvent::SetFile(
                    channel,
                    match file {
                        None => alloc::string::String::new(),
                        Some(str) => str.clone(),
                    },
                ))
                .await
            {
                Ok(()) => {
                    if let Some(_f) = file {
                        processor.write(channel, "echo Now fresh file: ").await;
                        processor.write(channel, _f.as_str()).await;
                        processor.write(channel, "\n").await;
                    }
                    processor
                        .event_bus
                        .publish_event(EventStatus::not_containing(EventFlags::JOB_COMPLETED))
                        .await;
                    Ok(processing::CodeExecutionSuccess::OK)
                }
                Err(_e) => {
                    let s = crate::processing::LineBuf::<192>::from_fmt(format_args!("echo: M23: Unable to set: {:?}\n", _e));
                    processor.write(channel, s.as_str()).await;
                    Err(processing::CodeExecutionFailure::ERR)
                }
            }
        }
        // SD print status. `S<secs>` sets the auto-report interval (0 = off), sent by the print
        // job task to this channel
        #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
        processing::GCodeValue::M27(_s) => {
            if let Some(secs) = _s.s {
                let secs = secs.int().clamp(0, i64::from(u16::MAX)) as u16;
                hwa::controllers::JOB_PROGRESS.set_autoreport(secs, channel);
            } else {
                processor.write_sd_status(channel).await;
            }
            Ok(processing::CodeExecutionSuccess::OK)
        }
        #[cfg(feature = "with-print-job")]
        processing::GCodeValue::M24 => {
            match _printer_controller
                .set(hwa::controllers::PrinterControllerEvent::Resume(channel))
                .await
            {
                Ok(_f) => {
                    processor.write(channel, "ok\n").await;
                    processor.write(channel, "echo: Print job resumed\n").await;
                    Ok(processing::CodeExecutionSuccess::CONSUMED)
                }
                // Repeated (or nothing to do): no effect, answered ok as Marlin does
                Err(hwa::controllers::PrinterControllerError::NoEffect) => {
                    processor.write(channel, "echo: M24: Print job already running\n").await;
                    Ok(processing::CodeExecutionSuccess::OK)
                }
                Err(_e) => {
                    let s = crate::processing::LineBuf::<192>::from_fmt(format_args!("echo: M24: Unable to start/resume: {:?}\n", _e));
                    processor.write(channel, s.as_str()).await;
                    Err(processing::CodeExecutionFailure::ERR)
                }
            }
        }
        #[cfg(feature = "with-print-job")]
        processing::GCodeValue::M25 => {
            match _printer_controller
                .set(hwa::controllers::PrinterControllerEvent::Pause(channel))
                .await
            {
                Ok(_f) => {
                    processor.write(channel, "ok\n").await;
                    processor.write(channel, "echo: Print job paused\n").await;
                    Ok(processing::CodeExecutionSuccess::CONSUMED)
                }
                // Repeated (or nothing to do): no effect, answered ok as Marlin does
                Err(hwa::controllers::PrinterControllerError::NoEffect) => {
                    processor.write(channel, "echo: M25: Print job already paused\n").await;
                    Ok(processing::CodeExecutionSuccess::OK)
                }
                Err(_e) => {
                    let s = crate::processing::LineBuf::<192>::from_fmt(format_args!("echo: M25: Unable to pause print job: {:?}\n", _e));
                    processor.write(channel, s.as_str()).await;
                    Err(processing::CodeExecutionFailure::ERR)
                }
            }
        }
        // Otherwise... delegate
        _ => processor.execute(channel, &gc, false).await,
    }
}

/// Max depth of the subdirectories listed by `M20` (each level keeps a directory open)
#[cfg(feature = "with-sd-card")]
const M20_MAX_DEPTH: usize = 4;

/// `M20`: the directory listed (a G-code text, at most 128 bytes) and the short path of the
/// subdirectory being listed below it
#[cfg(feature = "with-sd-card")]
const M20_DIR_LEN: usize = 192;

/// `M20 L`: the long path of the subdirectory being listed
#[cfg(feature = "with-sd-card")]
const M20_LONG_DIR_LEN: usize = 192;

/// `M20`: lists the files of `path` and of its subdirectories (up to [M20_MAX_DEPTH] levels; the
/// hidden ones are not listed), as Marlin does: one per line, `<short path> <size>`, and with
/// `long_names` also its long path (each entry by its long name, or by its short one when it has
/// none): `LONGFO~1/AREALL~1.GCO 1234 Long Folder Name/A really long file name.gcode`.
///
/// A subdirectory whose path does not fit (or that can not be opened) is not listed.
#[cfg(feature = "with-sd-card")]
async fn list_files(
    processor: &mut processing::GCodeProcessor,
    channel: hwa::CommChannel,
    card_controller: &mut hwa::types::SDCardController,
    path: &str,
    long_names: bool,
) -> Result<(), hwa::sd_card::SDCardError> {
    use processing::LineBuf;
    // The directory being listed, from the root: `path` and the subdirectories below it
    let mut dir = LineBuf::<M20_DIR_LEN>::from(path);
    if !dir.as_str().ends_with('/') {
        dir.push_str("/");
    }
    let base = dir.len();
    if !dir.as_str().ends_with('/') {
        return Err(hwa::sd_card::SDCardError::NotFound);
    }
    let mut long_dir = LineBuf::<M20_LONG_DIR_LEN>::new();
    // The directories being listed, with the lengths of both paths before their names
    let mut stack: heapless::Vec<_, { M20_MAX_DEPTH + 1 }> = heapless::Vec::new();
    let _ = stack.push((card_controller.dir_iterator(dir.as_str()).await?, base, 0usize));
    processor.write(channel, "Begin file list\n").await;
    while let Some((it, _, _)) = stack.last_mut() {
        match it.next().await {
            Ok(Some(entry)) if entry.hidden => {}
            Ok(Some(entry)) => match entry.entry_type {
                SDEntryType::FILE => {
                    let mut size = LineBuf::<12>::new();
                    size.push_str(" ");
                    size.push_u32(entry.size);
                    processor.write(channel, dir.as_str().get(base..).unwrap_or("")).await;
                    processor.write(channel, entry.name.as_str()).await;
                    processor.write(channel, size.as_str()).await;
                    if long_names {
                        processor.write(channel, " ").await;
                        processor.write(channel, long_dir.as_str()).await;
                        processor.write(channel, match entry.long_name.as_str() {
                            "" => entry.name.as_str(),
                            long => long,
                        }).await;
                    }
                    processor.write(channel, "\n").await;
                }
                SDEntryType::DIRECTORY if stack.len() <= M20_MAX_DEPTH => {
                    let (dir_len, long_len) = (dir.len(), long_dir.len());
                    let long_name = match entry.long_name.as_str() {
                        "" => entry.name.as_str(),
                        long => long,
                    };
                    dir.push_str(entry.name.as_str());
                    dir.push_str("/");
                    long_dir.push_str(long_name);
                    long_dir.push_str("/");
                    // Not listed when a path does not fit
                    let fits = dir.as_str().ends_with('/')
                        && long_dir.as_str().ends_with('/')
                        && dir.len() == dir_len + entry.name.len() + 1;
                    let sub_dir = match fits {
                        true => card_controller.dir_iterator(dir.as_str()).await.ok(),
                        false => None,
                    };
                    let pushed = match sub_dir {
                        Some(sub_dir) => match stack.push((sub_dir, dir_len, long_len)) {
                            Ok(()) => true,
                            Err((mut sub_dir, _, _)) => {
                                sub_dir.close().await;
                                false
                            }
                        },
                        None => false,
                    };
                    if !pushed {
                        hwa::warn!("[M20] Directory not listed: {}", dir.as_str());
                        dir.truncate(dir_len);
                        long_dir.truncate(long_len);
                    }
                }
                SDEntryType::DIRECTORY => {}
            },
            // The end of the directory (or a failure reading it): back to its parent
            end => {
                if end.is_err() {
                    hwa::error!("[M20] Error listing {}", dir.as_str());
                }
                if let Some((mut it, dir_len, long_len)) = stack.pop() {
                    it.close().await;
                    dir.truncate(dir_len);
                    long_dir.truncate(long_len);
                }
            }
        }
    }
    processor.write(channel, "End file list\n").await;
    Ok(())
}

async fn report(
    result: processing::CodeExecutionResult,
    gc: &processing::GCodeCmd,
    channel: hwa::CommChannel,
    processor: &mut processing::GCodeProcessor,
) {
    match result {
        Ok(processing::CodeExecutionSuccess::OK) => {
            cfg_if::cfg_if! {
                if #[cfg(feature="trace-commands")] {
                    let s = crate::processing::LineBuf::<192>::from_fmt(format_args!("ok; [trace-commands] {}\n", gc));
                    processor.write(channel, s.as_str()).await;
                }
                else {
                    processor.write(channel, "ok\n").await;
                }
            }
        }
        Ok(processing::CodeExecutionSuccess::QUEUED) => {
            cfg_if::cfg_if! {
                if #[cfg(feature="trace-commands")] {
                    let s = crate::processing::LineBuf::<192>::from_fmt(format_args!("ok; [trace-commands] promised {}\n", gc));
                    processor.write(channel, s.as_str()).await;
                }
                else {
                    processor.write(channel, "ok\n").await;
                }
            }
        }
        Ok(processing::CodeExecutionSuccess::DEFERRED(_)) => {
            #[cfg(feature = "trace-commands")]
            {
                hwa::debug!("Control not sending (deferred)");
                let s = crate::processing::LineBuf::<192>::from_fmt(format_args!("echo; promised {}\n", gc));
                processor.write(channel, s.as_str()).await;
            }
        }
        Ok(processing::CodeExecutionSuccess::CONSUMED) => {
            #[cfg(feature = "trace-commands")]
            {
                let s = crate::processing::LineBuf::<192>::from_fmt(format_args!("echo; already confirmed {}\n", gc));
                processor.write(channel, s.as_str()).await;
            }
        }
        Err(_e) => {
            hwa::warn!("Control sending ERR for {} ({:?})", gc, _e);
            // Heap-free: the command may carry a text of up to MAX_TEXT_LENGTH (128) bytes,
            // already in the heap. The report is truncated when too long
            use core::fmt::Write;
            let mut s = processing::LineBuf::<192>::new();
            let _ = write!(s, "{} {} ({:?})", ERR_PREFIX, gc, _e);
            processor.write(channel, s.as_str()).await;
            processor.write(channel, "\n").await;
            // Every command gets an `ok`, so that hosts (OctoPrint, Pronterface...) do not stall
            processor.write_ok(channel).await;
        }
    }
}
