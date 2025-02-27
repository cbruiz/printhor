use crate::control;
use crate::hwa;
#[cfg(feature = "with-sd-card")]
use crate::hwa::controllers::sd_card_controller::SDEntryType;
use async_gcode::AsyncParserState;
use embassy_time::{Duration, TimeoutError};
#[allow(unused)]
use printhor_hwa_common::{EventFlags, EventStatus};

cfg_if::cfg_if! {
    if #[cfg(feature = "with-print-job")] {
        cfg_if::cfg_if! {
            if #[cfg(not(feature = "with-sd-card"))] {
                compiler_error("Feature with-print-job requires with-sd-card")
            }
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
    mut processor: hwa::GCodeProcessor,
    mut gcode_input_stream: control::GCodeMultiplexedInputStream,
    #[cfg(feature = "with-print-job")] mut printer_controller: hwa::controllers::PrinterController,
    #[cfg(feature = "with-sd-card")] mut card_controller: hwa::types::SDCardController,
) {
    let ev2 = processor.event_bus.clone();
    let mut subscriber = ev2.subscriber().await;

    // Initialize an event bus subscriber

    hwa::info!("[task_control] Waiting for SYS_READY");
    // Pauses this task until system is ready
    match subscriber
        .ft_wait_for(hwa::EventStatus::containing(hwa::EventFlags::SYS_READY))
        .await
    {
        Ok(_) => hwa::info!("[task_control] Got SYS_READY. Continuing."),
        Err(_) => crate::initialization_error(),
    }

    // The task loop
    loop {
        match do_with_timeout(Duration::from_secs(30), gcode_input_stream.next_gcode()).await {
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
                if control::task_integration::INTEGRATION_STATUS.signaled() {
                    hwa::info!("[task_control] Ending gracefully");
                    return ();
                }
            }
            Ok((Err(control::GCodeLineParserError::ParseError(_x)), channel)) => {
                hwa::error!("[{:?}] GCode N/A ParserError", channel);
                processor.write(channel, "error; (ParserError)\n").await;
            }
            Ok((
                Err(control::GCodeLineParserError::GCodeNotImplemented(_ln, _gcode_name)),
                channel,
            )) => {
                hwa::error!("GCode {} (NotImplemented)", _gcode_name.as_str());
                let s = alloc::format!("error; {} (NotImplemented)\n", _gcode_name);
                processor.write(channel, &s).await;
            }
            Ok((Err(control::GCodeLineParserError::FatalError), channel)) => {
                hwa::error!("[{:?}] GCode N/A (Internal error)", channel);
                let s = "error; Internal error\n";
                processor.write(channel, s).await;
            }
            Ok((Err(control::GCodeLineParserError::EOF), _channel)) => {
                hwa::warn!("{:?} Got EOF", _channel);
                //embassy_time::Timer::after_secs(1).await; // Avoid respawn too fast
            }
            Ok((Ok(gc), channel)) => {
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

#[allow(unused)]
async fn manage_timeout<M>(
    processor: &mut hwa::GCodeProcessor,
    subscriber: &mut hwa::EventBusSubscriber<'static, M>,
    gcode_input_stream: &mut control::GCodeMultiplexedInputStream,
    comm_channel: hwa::CommChannel,
) where
    M: hwa::AsyncRawMutex,
{
    let current_parser_state = gcode_input_stream.get_state(comm_channel);

    #[cfg(feature = "trace-commands")]
    hwa::info!(
        "[trace-commands] [task_control] Timeout at {:?}: [State: {:?} Line: {} TaggedLine: {:?}]",
        comm_channel,
        alloc::format!("{:?}", current_parser_state).as_str(),
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
            let z3 = alloc::format!("echo: EventBus status: {:?}", subscriber.get_status().await);
            processor
                .write(hwa::CommChannel::Internal, z3.as_str())
                .await;

            let z = alloc::format!(
                "echo: Will reset channel: {:?} state was: {:?}\n",
                comm_channel,
                current_parser_state
            );
            processor
                .write(hwa::CommChannel::Internal, z.as_str())
                .await;
            gcode_input_stream.reset(comm_channel);
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
    processor: &mut hwa::GCodeProcessor,
    channel: hwa::CommChannel,
    gc: &control::GCodeCmd,
    #[cfg(feature = "with-sd-card")] _card_controller: &mut hwa::types::SDCardController,
    #[cfg(feature = "with-print-job")]
    _printer_controller: &mut hwa::controllers::PrinterController,
) -> control::CodeExecutionResult {
    match &gc.value {
        control::GCodeValue::Nop => {
            hwa::error!("Go Nop");
            // Should not happen
            Err(control::CodeExecutionFailure::ERR)
        }
        #[cfg(feature = "grbl-compat")]
        control::GCodeValue::Status => {
            // TODO provide GRBL compatibility status
            processor
                .write(
                    channel,
                    "<Idle|MPos:0.000,0.000,0.000|Pn:XP|FS:0,0|WCO:0.000,0.000,0.000>\n",
                )
                .await;
            Ok(control::CodeExecutionSuccess::CONSUMED)
        }
        #[cfg(all(feature = "with-sd-card", feature = "with-print-job"))]
        control::GCodeValue::M2 => {
            match _printer_controller
                .set(hwa::controllers::PrinterControllerEvent::Abort(channel))
                .await
            {
                Ok(_f) => {
                    processor
                        .write(channel, "echo: Print job terminated\n")
                        .await;
                    Ok(control::CodeExecutionSuccess::OK)
                }
                Err(_e) => {
                    let s = alloc::format!("echo: M2: Unable to terminate: {:?}\n", _e);
                    processor.write(channel, s.as_str()).await;
                    Err(control::CodeExecutionFailure::ERR)
                }
            }
        }
        #[cfg(feature = "with-sd-card")]
        control::GCodeValue::M20(_path) => {
            {
                let path = _path.clone().unwrap_or(alloc::string::String::from("/"));
                let mut sent_first = false;
                match _card_controller.dir_iterator(path.as_str()).await {
                    Ok(mut it) => {
                        loop {
                            match it.next().await {
                                Ok(Some(entry)) => match entry.entry_type {
                                    SDEntryType::FILE => {
                                        if !sent_first {
                                            processor.write(channel, "Begin file list\n").await;
                                            sent_first = true;
                                        }
                                        processor.write(channel, entry.name.as_str()).await;
                                        let s = alloc::format!(" {}\n", entry.size);
                                        processor.write(channel, s.as_str()).await;
                                    }
                                    SDEntryType::DIRECTORY => {}
                                },
                                Ok(None) => {
                                    // EOF
                                    processor.write(channel, "End file list\n").await;
                                    break;
                                }
                                Err(_e) => {
                                    if sent_first {
                                        processor.write(channel, "End file list\n").await;
                                    }
                                    processor.write(channel, "err\n").await;
                                    hwa::error!("Error listing files");
                                    break;
                                }
                            }
                        }
                        it.close().await;
                        processor.write(channel, "ok\n").await;
                        Ok(control::CodeExecutionSuccess::CONSUMED)
                    }
                    Err(_e) => {
                        let s = alloc::format!("echo: M20: Unable to list: {:?}\n", _e);
                        processor.write(channel, s.as_str()).await;
                        Err(control::CodeExecutionFailure::ERR)
                    }
                }
            }
        }
        #[cfg(feature = "with-sd-card")]
        control::GCodeValue::M21 => Ok(control::CodeExecutionSuccess::OK),
        #[cfg(feature = "with-print-job")]
        control::GCodeValue::M23(file) => {
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
                    // TODO
                    Ok(control::CodeExecutionSuccess::CONSUMED)
                }
                Err(_e) => {
                    let s = alloc::format!("echo: M23: Unable to set: {:?}\n", _e);
                    processor.write(channel, s.as_str()).await;
                    Err(control::CodeExecutionFailure::ERR)
                }
            }
        }
        #[cfg(feature = "with-print-job")]
        control::GCodeValue::M24 => {
            match _printer_controller
                .set(hwa::controllers::PrinterControllerEvent::Resume(channel))
                .await
            {
                Ok(_f) => {
                    processor.write(channel, "ok\n").await;
                    processor.write(channel, "echo: Print job resumed\n").await;
                    Ok(control::CodeExecutionSuccess::CONSUMED)
                }
                Err(_e) => {
                    let s = alloc::format!("echo: M24: Unable to start/resume: {:?}\n", _e);
                    processor.write(channel, s.as_str()).await;
                    Err(control::CodeExecutionFailure::ERR)
                }
            }
        }
        #[cfg(feature = "with-print-job")]
        control::GCodeValue::M25 => {
            match _printer_controller
                .set(hwa::controllers::PrinterControllerEvent::Pause(channel))
                .await
            {
                Ok(_f) => {
                    processor.write(channel, "ok\n").await;
                    processor.write(channel, "echo: Print job paused\n").await;
                    Ok(control::CodeExecutionSuccess::CONSUMED)
                }
                Err(_e) => {
                    let s = alloc::format!("echo: M25: Unable to pause print job: {:?}\n", _e);
                    processor.write(channel, s.as_str()).await;
                    Err(control::CodeExecutionFailure::ERR)
                }
            }
        }
        // Otherwise... delegate
        _ => processor.execute(channel, &gc, false).await,
    }
}

async fn report(
    result: control::CodeExecutionResult,
    gc: &control::GCodeCmd,
    channel: hwa::CommChannel,
    processor: &mut hwa::GCodeProcessor,
) {
    match result {
        Ok(control::CodeExecutionSuccess::OK) => {
            cfg_if::cfg_if! {
                if #[cfg(feature="trace-commands")] {
                    let s = alloc::format!("ok; [trace-commands] {}\n", gc);
                    processor.write(channel, s.as_str()).await;
                }
                else {
                    processor.write(channel, "ok\n").await;
                }
            }
        }
        Ok(control::CodeExecutionSuccess::QUEUED) => {
            cfg_if::cfg_if! {
                if #[cfg(feature="trace-commands")] {
                    let s = alloc::format!("ok; [trace-commands] promised {}\n", gc);
                    processor.write(channel, s.as_str()).await;
                }
                else {
                    processor.write(channel, "ok\n").await;
                }
            }
        }
        Ok(control::CodeExecutionSuccess::DEFERRED(_)) => {
            #[cfg(feature = "trace-commands")]
            {
                hwa::debug!("Control not sending (deferred)");
                let s = alloc::format!("echo; promised {}\n", gc);
                processor.write(channel, s.as_str()).await;
            }
        }
        Ok(control::CodeExecutionSuccess::CONSUMED) => {
            #[cfg(feature = "trace-commands")]
            {
                let s = alloc::format!("echo; already confirmed {}\n", gc);
                processor.write(channel, s.as_str()).await;
            }
        }
        Err(_e) => {
            hwa::info!("Control sending ERR");
            let s = alloc::format!("error; {} ({:?})\n", gc, _e);
            processor.write(channel, s.as_str()).await;
        }
    }
}
