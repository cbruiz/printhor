///! Input Multiplexer
///!
///! This feature is a bit inefficient and could be improved leveraging macros
///! because currently useless polls are performed in disabled channels
use crate::control::{GCode, GCodeLineParserError};
#[allow(unused)]
use futures_util::future;
use embassy_futures::select::Either3;

// Utility to accept a common gcode stream from multiple sources
pub struct GCodeMultiplexedInputStream {
    #[cfg(feature = "with-serial-usb")]
    serial_usb_line_parser: crate::control::GCodeLineParser<crate::hwa::device::USBSerialDeviceInputStream>,
    #[cfg(feature = "with-serial-port-1")]
    serial_port1_line_parser: crate::control::GCodeLineParser<crate::hwa::device::UartPort1RxInputStream>,
    #[cfg(feature = "with-serial-port-2")]
    serial_port2_line_parser: crate::control::GCodeLineParser<crate::hwa::device::UartPort2RxInputStream>,
}

impl GCodeMultiplexedInputStream {

    pub fn new(
        #[cfg(feature = "with-serial-usb")]
        serial_usb_rx_stream: crate::hwa::device::USBSerialDeviceInputStream,
        #[cfg(feature = "with-serial-port-1")]
        serial_port1_rx_stream: crate::hwa::device::UartPort1RxInputStream,
        #[cfg(feature = "with-serial-port-2")]
        serial_port2_rx_stream: crate::hwa::device::UartPort2RxInputStream,
    ) -> Self {
        Self {
            #[cfg(feature = "with-serial-usb")]
            serial_usb_line_parser: crate::control::GCodeLineParser::new(serial_usb_rx_stream),
            #[cfg(feature = "with-serial-port-1")]
            serial_port1_line_parser: crate::control::GCodeLineParser::new(serial_port1_rx_stream),
            #[cfg(feature = "with-serial-port-2")]
            serial_port2_line_parser: crate::control::GCodeLineParser::new(serial_port2_rx_stream),
        }
    }

    pub async fn next_gcode(&mut self) -> (Result<Option<GCode>, GCodeLineParserError>, crate::hwa::CommChannel) {

        cfg_if::cfg_if! {
            if #[cfg(feature="with-serial-usb")] {
                let f1 = self.serial_usb_line_parser.next_gcode();
            }
            else {
                let f1 = future::pending::<Result<Option<GCode>, GCodeLineParserError>>();
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature="with-serial-port-1")] {
                let f2 = self.serial_port1_line_parser.next_gcode();
            }
            else {
                let f2 = future::pending::<Result<Option<GCode>, GCodeLineParserError>>();
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature="with-serial-port-2")] {
                let f3 = self.serial_port2_line_parser.next_gcode();
            }
            else {
                let f3 = future::pending::<Result<Option<GCode>, GCodeLineParserError>>();
            }
        }

        match embassy_futures::select::select3(f1, f2, f3).await {
            Either3::First(_result) => {
                cfg_if::cfg_if! {
                    if #[cfg(feature="with-serial-usb")] {
                        (_result, crate::hwa::CommChannel::SerialUsb)
                    }
                    else {
                        (Ok(None), crate::hwa::CommChannel::Internal)
                    }
                }
            }
            Either3::Second(_result) => {
                cfg_if::cfg_if! {
                    if #[cfg(feature="with-serial-port-1")] {
                        (_result, crate::hwa::CommChannel::SerialPort1)
                    }
                    else {
                        (Ok(None), crate::hwa::CommChannel::Internal)
                    }
                }
            }
            Either3::Third(_result) => {
                cfg_if::cfg_if! {
                    if #[cfg(feature="with-serial-port-2")] {
                        (_result, crate::hwa::CommChannel::SerialPort2)
                    }
                    else {
                        (Ok(None), crate::hwa::CommChannel::Internal)
                    }
                }
            }
        }
    }
}