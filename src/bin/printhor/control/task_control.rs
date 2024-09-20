use async_gcode::AsyncParserState;
use printhor_hwa_common::CommChannel;
use crate::control::CodeExecutionSuccess;
use crate::control::{GCode, GCodeLineParserError, GCodeMultiplexedInputStream};
use crate::hwa;
use hwa::{EventBusSubscriber, EventFlags, EventStatus};
use crate::hwa::GCodeProcessor;

cfg_if::cfg_if! {
    if #[cfg(feature = "with-printjob")] {
        use crate::hwa::controllers::{PrinterController, PrinterControllerEvent};
        cfg_if::cfg_if! {
            if #[cfg(not(feature = "with-sdcard"))] {
                compiler_error("Feature with-printjob requires with-sdcard")
            }
        }
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-sdcard")] {
        use crate::hwa::controllers::sdcard_controller::SDEntryType;
    }
}

pub struct ControlTaskControllers {
    #[cfg(feature = "with-printjob")]
    pub printer_controller: PrinterController,
    #[cfg(feature = "with-sdcard")]
    pub card_controller: hwa::controllers::CardController,
}

#[embassy_executor::task(pool_size = 1)]
pub async fn task_control(
    mut processor: GCodeProcessor,
    mut gcode_input_stream: GCodeMultiplexedInputStream,
    #[allow(unused)]
    mut controllers: ControlTaskControllers,
) {
    // Initialize an event bus subscriber
    let mut subscriber: EventBusSubscriber<'static> =
        hwa::task_allocations::init_control_subscriber(processor.event_bus.clone()).await;

    hwa::info!("task_control waiting for SYS_READY");
    // Pauses this task until system is ready
    match subscriber.ft_wait_for(EventStatus::containing(EventFlags::SYS_READY)).await
    {
        Ok(_) => hwa::info!("task_control got ATX_ON. Continuing."),
        Err(_) => crate::initialization_error(),
    }

    // The task loop
    loop {
        match embassy_time::with_timeout(
            embassy_time::Duration::from_secs(6), gcode_input_stream.next_gcode())
            .await
        {
            // Timeout
            Err(_) => {
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-serial-usb")] {
                        manage_timeout(&mut processor, &mut subscriber, &mut gcode_input_stream,
                            CommChannel::SerialUsb).await;
                    }
                }
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-serial-port-1")] {
                        manage_timeout(&mut processor, &mut subscriber, &mut gcode_input_stream,
                            CommChannel::SerialPort1).await;
                    }
                }
                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-serial-port-2")] {
                        manage_timeout(&mut processor, &mut subscriber, &mut gcode_input_stream,
                            CommChannel::SerialPort2).await;
                    }
                }
            }
            Ok((Err(GCodeLineParserError::ParseError(_x)), channel)) => {
                hwa::error!("[{:?}] GCode N/A ParserError", channel);
                processor.write(channel, "error; (ParserError)\n").await;
            }
            Ok((Err(GCodeLineParserError::GCodeNotImplemented(_ln, _gcode_name)), channel)) => {
                hwa::error!("GCode {} (NotImplemented)", _gcode_name.as_str());
                let s = alloc::format!("error; {} (NotImplemented)\n", _gcode_name);
                processor.write(channel, &s).await;
            }
            Ok((Err(GCodeLineParserError::FatalError), channel)) => {
                hwa::error!("[{:?}] GCode N/A (Internal error)", channel);
                let s = "error; Internal error\n";
                processor.write(channel, s).await;
            }
            Ok((Ok(None), _channel)) => {
                hwa::info!("{:?} Got EOF", _channel);
                //embassy_time::Timer::after_secs(1).await; // Avoid respawn too fast
            }
            Ok((Ok(Some(gc)), channel)) => {
                hwa::debug!("{:?} Got {:?}", channel, gc);
                match gc {
                    GCode::NOP => {
                        // Should not happen
                    }
                    #[cfg(feature = "grbl-compat")]
                    GCode::STATUS => {
                        // TODO provide GRBL compatibility status
                        processor.write(channel, "<Idle|MPos:0.000,0.000,0.000|Pn:XP|FS:0,0|WCO:0.000,0.000,0.000>\n").await;
                    }
                    #[cfg(feature = "with-sdcard")]
                    GCode::M20(path) => {
                        let path = path.unwrap_or(alloc::string::String::from("/"));
                        match controllers.card_controller.list_dir(path.as_str()).await {
                            Ok(mut it) => {
                                loop {
                                    //crate::debug!("will get next");
                                    match it.next().await {
                                        Ok(result) => {
                                            //crate::debug!("got a result");
                                            match result {
                                                Some(entry) => {
                                                    let s = alloc::format!(
                                                        "echo: F\"{}\" {} {} ; M20 \n",
                                                        entry.name,
                                                        match entry.entry_type {
                                                            SDEntryType::FILE => "A",
                                                            SDEntryType::DIRECTORY => "D",
                                                        },
                                                        entry.size
                                                    );
                                                    processor.write(channel, s.as_str()).await;
                                                    //crate::debug!("sent to uart");
                                                }
                                                None => {
                                                    //crate::debug!("got EOF");
                                                    break;
                                                }
                                            }
                                        }
                                        Err(_e) => {
                                            let s = alloc::format!(
                                                "Error; M20; Error listing: {:?}\n",
                                                _e
                                            );
                                            processor.write(channel, s.as_str()).await;
                                        }
                                    }
                                }
                                it.close().await;
                                processor.write(channel, "ok; M20\n").await;
                            }
                            Err(_e) => {
                                let s = alloc::format!("Error; M20 (Unable to list: {:?})\n", _e);
                                processor.write(channel, s.as_str()).await;
                            }
                        }
                    }
                    #[cfg(feature = "with-printjob")]
                    GCode::M23(f) => {
                        match controllers
                            .printer_controller
                            .set(PrinterControllerEvent::SetFile(
                                f.map_or(alloc::string::String::from("default"), |s| {
                                    alloc::string::String::from(s.as_str())
                                }),
                            ))
                            .await
                        {
                            Ok(_f) => {
                                hwa::info!("ok; M23");
                            }
                            Err(_e) => {
                                let s = alloc::format!("E. M23; Unable to set: {:?}\n", _e);
                                processor.write(channel, s.as_str()).await;
                            }
                        }
                    }
                    #[cfg(feature = "with-printjob")]
                    GCode::M24 => {
                        match controllers
                            .printer_controller
                            .set(PrinterControllerEvent::Resume)
                            .await
                        {
                            Ok(_f) => {
                                hwa::info!("ok; M24");
                            }
                            Err(_e) => {
                                let s = alloc::format!("echo: unable to start/resume: {:?}\n", _e);
                                processor.write(channel, s.as_str()).await;
                            }
                        }
                    }
                    #[cfg(feature = "with-printjob")]
                    GCode::M25 => {
                        match controllers
                            .printer_controller
                            .set(PrinterControllerEvent::Pause)
                            .await
                        {
                            Ok(_f) => {
                                hwa::info!("ok. M25");
                            }
                            Err(_e) => {
                                let s = alloc::format!("echo: unable to pause: {:?}\n", _e);
                                processor.write(channel, s.as_str()).await;
                            }
                        }
                    }
                    _ => match processor.execute(channel, &gc, false).await {
                        Ok(CodeExecutionSuccess::OK) => {
                            hwa::debug!("Control sending OK");
                            cfg_if::cfg_if! {
                                if #[cfg(feature="trace-commands")] {
                                    let s = alloc::format!("ok; {}\n", gc.as_ref());
                                    processor.write(channel, s.as_str()).await;
                                }
                                else {
                                    processor.write(channel, "ok\n").await;
                                }
                            }
                        }
                        Ok(CodeExecutionSuccess::QUEUED) => {
                            hwa::debug!("Control sending OK (Q) BEGIN");
                            cfg_if::cfg_if! {
                                if #[cfg(feature="trace-commands")] {
                                    let s = alloc::format!("ok; {} (QUEUED)\n", gc.as_ref());
                                    processor.write(channel, s.as_str()).await;
                                }
                                else {
                                    processor.write(channel, "ok\n").await;
                                }
                            }
                            hwa::debug!("Control sending OK (Q) END");
                        }
                        Ok(CodeExecutionSuccess::DEFERRED(_)) => {
                            hwa::debug!("Control not sending (deferred)");
                        }
                        Ok(CodeExecutionSuccess::CONSUMED) => {
                            hwa::info!("Control not sending (implicitly consumed)");
                        }
                        Err(_e) => {
                            hwa::info!("Control sending ERR");
                            let s = alloc::format!("error; {} ({:?})\n", gc.as_ref(), _e);
                            processor.write(channel, s.as_str()).await;
                        }
                    },
                }
            }
        }
    }
}

async fn manage_timeout(processor: &mut GCodeProcessor,
                        subscriber: &mut EventBusSubscriber<'_>,
                        gcode_input_stream: &mut GCodeMultiplexedInputStream,
                        comm_channel: CommChannel,
) {

    let current_parser_state = gcode_input_stream.get_state(comm_channel);

    match current_parser_state {
        // Parser is currently ready to accept a gcode. Nothing to do
        AsyncParserState::Start(_) => {},
        // Parser is at any other intermediate state
        _ => {
            // Notify there is a timeout on [CommChannel::Internal] channel
            processor.write(CommChannel::Internal, "echo: Control timeout.").await;
            // Dumps the relevant status for a more easy troubleshooting
            let z3 = alloc::format!("echo: EventBus status: {:?}", subscriber.get_status().await);
            processor.write(CommChannel::Internal, z3.as_str()).await;

            let z = alloc::format!("echo: Will reset channel: {:?} state was: {:?}", comm_channel, current_parser_state);
            processor.write(CommChannel::Internal, z.as_str()).await;
            gcode_input_stream.reset(comm_channel);
            // Submit an err to give up the intermediate state.
            processor.write(comm_channel, "err").await;
        }
    }
}
