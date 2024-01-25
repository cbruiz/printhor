use crate::hwa;

use printhor_hwa_common::EventBusSubscriber;
use printhor_hwa_common::EventStatus;
use printhor_hwa_common::EventFlags;
#[allow(unused)]
use crate::control::planner::*;
use crate::control::GCode;
#[cfg(feature = "with-printjob")]
use crate::hwa::controllers::{PrinterController, PrinterControllerEvent};

#[cfg(feature = "with-sdcard")]
use crate::hwa::controllers::sdcard_controller::SDEntryType;

pub struct ControlTaskDevices {
    #[cfg(feature = "with-usbserial")]
    pub usb_serial_rx: hwa::device::USBSerialDeviceInputStream,
    #[cfg(feature = "with-uart-port-1")]
    pub uart_port1_rx_stream: hwa::device::UartPort1RxInputStream,
}

pub struct ControlTaskControllers {
    #[cfg(feature = "with-printjob")]
    pub printer_controller: PrinterController,
    #[cfg(feature = "with-sdcard")]
    pub card_controller: hwa::controllers::CardController,
}

#[embassy_executor::task(pool_size=1)]
pub async fn control_task(
    _processor: hwa::GCodeProcessor,
    _d: ControlTaskDevices,
    mut _c : ControlTaskControllers,
) {
    let mut s: EventBusSubscriber<'static> = hwa::task_allocations::init_control_subscriber(_processor.event_bus.clone()).await;

    // TODO: Design a multiplexing mechanism:
    // future_set[usb, uart_port1, uart_port2, ...] => iterator[channel, gcode]
    #[cfg(feature = "with-usbserial")]
    let mut code_parser = crate::control::parser::GCodeLineParser::new(_d.usb_serial_rx);
    #[cfg(all(not(feature = "with-usbserial"), feature = "with-uart-port-1"))]
    let mut code_parser = crate::control::parser::GCodeLineParser::new(_d.uart_port1_rx_stream);

    let mut _processor = _processor;

    s.wait_for(EventStatus::containing(EventFlags::SYS_READY)).await;
    #[cfg(feature = "with-usbserial")]
    hwa::debug!("Control_task started [USBSerial]");
    #[cfg(feature = "with-uart-port-1")]
    hwa::debug!("Control_task started [UARTPort1]");
    _processor.write("echo: ready for commands\n").await;

    #[cfg(any(feature = "with-usbserial", feature = "with-uart-port-1"))]
    loop {
        match code_parser.next_gcode().await {
            Err(err) => {
                match err {
                    crate::control::parser::GCodeLineParserError::ParseError(_x) => {
                        hwa::error!("GCode N/A ParserError");
                        _processor.write("error; (ParserError)\n").await;
                    }
                    crate::control::parser::GCodeLineParserError::GCodeNotImplemented(_ln, _gcode_name) => {
                        hwa::error!("GCode {} (NotImplemented)\n", _gcode_name.as_str());
                        let s = alloc::format!("error; {} (NotImplemented)\n", _gcode_name);
                        _processor.write(&s).await;
                    }
                    crate::control::parser::GCodeLineParserError::FatalError => {
                        hwa::error!("GCode N/A (Internal error)\n");
                        let s = "error; Internal error\n";
                        _processor.write(s).await;
                    }
                }
            }
            Ok(result) => {
                hwa::trace!("GCODE PARSE OK");
                match result {
                    None => { // EOF. Theoretically impossible
                        embassy_time::Timer::after_secs(10).await; // Avoid respawn too fast
                    }
                    Some(gc) => {
                        match gc {
                            GCode::STATUS => {
                                // TODO provide GRBL compatibility status
                                _processor.write("<Idle|MPos:0.000,0.000,0.000|Pn:XP|FS:0,0|WCO:0.000,0.000,0.000>\n").await;
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
                                                            _processor.write(s.as_str()).await;
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
                                                    _processor.write(s.as_str()).await;
                                                }
                                            }
                                        }
                                        it.close().await;
                                        _processor.write("ok; M20\n").await;
                                    },
                                    Err(_e) => {
                                        let s = alloc::format!("error; M20 (Unable to list: {:?})\n", _e);
                                        _processor.write(s.as_str()).await;
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
                                        _processor.write(s.as_str()).await;
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
                                        _processor.write(s.as_str()).await;
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
                                        _processor.write(s.as_str()).await;
                                    }
                                }
                            },
                            _ => {
                                match _processor.execute(&gc, false).await {
                                    Ok(CodeExecutionSuccess::OK) => {
                                        let s = alloc::format!("ok; {}\n", gc.as_ref());
                                        _processor.write(s.as_str()).await;
                                    }
                                    Ok(CodeExecutionSuccess::QUEUED) => {
                                        let s = alloc::format!("ok; {} (QUEUED)\n", gc.as_ref());
                                        _processor.write(s.as_str()).await;
                                    }
                                    Err(_e) => {
                                        let s = alloc::format!("error; {} ({:?})\n", gc.as_ref(), _e);
                                        _processor.write(s.as_str()).await;
                                    }
                                    _ => { // Deferred
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    #[cfg(not(any(feature = "with-usbserial", feature = "with-uart-port-1")))]
    {
        hwa::warn!("Control task ended");
        _processor.event_bus.publish_event(
            EventStatus::containing(EventFlags::SYS_ALARM)
        ).await;
    }
}
