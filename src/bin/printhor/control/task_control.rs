use crate::hwa;
use hwa::{EventBusSubscriber, EventStatus, EventFlags};
use crate::control::{GCode, GCodeLineParserError, GCodeMultiplexedInputStream};
use crate::control::CodeExecutionSuccess;
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

#[embassy_executor::task(pool_size=1)]
pub async fn task_control(
    _processor: hwa::GCodeProcessor,
    mut _d: GCodeMultiplexedInputStream,
    mut _c : ControlTaskControllers,
) {
    let mut s: EventBusSubscriber<'static> = hwa::task_allocations::init_control_subscriber(_processor.event_bus.clone()).await;

    let mut processor = _processor;

    if s.ft_wait_for(EventStatus::containing(EventFlags::SYS_READY)).await.is_err() {
        crate::initialization_error()
    }
    loop {

        match embassy_time::with_timeout(embassy_time::Duration::from_secs(600), _d.next_gcode()).await {
            // Timeout
            Err(_) => {
                hwa::warn!("task_control: Timeout");
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
                hwa::trace!("{:?} Got EOF", _channel);
                //embassy_time::Timer::after_secs(1).await; // Avoid respawn too fast
            }
            Ok((Ok(Some(gc)), channel)) => {
                match gc {
                    GCode::NOP => {
                        // Should not happen
                    }
                    #[cfg(feature = "grbl-compat")]
                    GCode::STATUS => {
                        // TODO provide GRBL compatibility status
                        processor.write("<Idle|MPos:0.000,0.000,0.000|Pn:XP|FS:0,0|WCO:0.000,0.000,0.000>\n").await;
                    }
                    #[cfg(feature = "with-sdcard")]
                    GCode::M20(path) => {
                        let path = path.unwrap_or(alloc::string::String::from("/"));
                        match _c.card_controller.list_dir(path.as_str()).await {
                            Ok(mut it) => {
                                loop {
                                    //crate::debug!("will get next");
                                    match it.next().await {
                                        Ok(result) => {
                                            //crate::debug!("got a result");
                                            match result {
                                                Some(entry) => {
                                                    let s = alloc::format!("echo: F\"{}\" {} {} ; M20 \n",
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
                                        },
                                        Err(_e) => {
                                            let s = alloc::format!("error; M20; Error listing: {:?}\n", _e);
                                            processor.write(channel, s.as_str()).await;
                                        }
                                    }
                                }
                                it.close().await;
                                processor.write(channel, "ok; M20\n").await;
                            },
                            Err(_e) => {
                                let s = alloc::format!("error; M20 (Unable to list: {:?})\n", _e);
                                processor.write(channel, s.as_str()).await;
                            }
                        }
                    },
                    #[cfg(feature = "with-printjob")]
                    GCode::M23(f) => {
                        match _c.printer_controller.set(
                            PrinterControllerEvent::SetFile(
                                f.map_or(alloc::string::String::from("default"),
                                         |s| { alloc::string::String::from(s.as_str()) }
                                ),
                            )
                        ).await {
                            Ok(_f) => {
                                hwa::info!("ok; M23");
                            }
                            Err(_e) => {
                                let s = alloc::format!("E. M23; Unable to set: {:?}\n", _e);
                                processor.write(channel, s.as_str()).await;
                            }
                        }
                    },
                    #[cfg(feature = "with-printjob")]
                    GCode::M24 => {
                        match _c.printer_controller.set(PrinterControllerEvent::Resume).await {
                            Ok(_f) => {
                                hwa::info!("ok; M24");
                            }
                            Err(_e) => {
                                let s = alloc::format!("echo: unable to start/resume: {:?}\n", _e);
                                processor.write(channel, s.as_str()).await;
                            }
                        }
                    },
                    #[cfg(feature = "with-printjob")]
                    GCode::M25 => {
                        match _c.printer_controller.set(PrinterControllerEvent::Pause).await {
                            Ok(_f) => {
                                hwa::info!("ok. M25");
                            }
                            Err(_e) => {
                                let s = alloc::format!("echo: unable to pause: {:?}\n", _e);
                                processor.write(channel, s.as_str()).await;
                            }
                        }
                    },
                    _ => {
                        match processor.execute(channel, &gc, false).await {
                            Ok(CodeExecutionSuccess::OK) => {
                                hwa::debug!("Control sending OK");
                                let s = alloc::format!("ok; {}\n", gc.as_ref());
                                processor.write(channel, s.as_str()).await;
                            }
                            Ok(CodeExecutionSuccess::QUEUED) => {
                                hwa::debug!("Control sending OK (Q)");
                                let s = alloc::format!("ok; {} (QUEUED)\n", gc.as_ref());
                                processor.write(channel, s.as_str()).await;
                            }
                            Ok(CodeExecutionSuccess::DEFERRED(_)) => {
                                hwa::debug!("Control not sending (deferred)");
                            }
                            Ok(CodeExecutionSuccess::CONSUMED) => {
                                hwa::debug!("Control not sending (implicitly consumed)");
                            }
                            Err(_e) => {
                                hwa::debug!("Control sending ERR");
                                let s = alloc::format!("error; {} ({:?})\n", gc.as_ref(), _e);
                                processor.write(channel, s.as_str()).await;
                            }
                        }
                    }
                }
            }
        }
    }
}
