///! Input Multiplexer
///!
///! This feature is a bit inefficient and could be improved leveraging macros
///! because currently useless polls are performed in disabled channels
use crate::{hwa, processing};

use embassy_futures::select::Either3;

/// Longest line stored while writing a file (M28): Marlin's `MAX_CMD_SIZE`
#[cfg(feature = "with-sd-card")]
pub const RAW_LINE_SIZE: usize = 96;

/// What a channel delivered
pub enum InputEvent {
    /// A G-code to execute
    GCode(Result<processing::GCodeCmd, processing::GCodeLineParserError>),
    /// A line to store (the channel is writing a file, see [GCodeMultiplexedInputStream::start_capture]):
    /// the range of its content in [GCodeMultiplexedInputStream::raw_line]
    #[cfg(feature = "with-sd-card")]
    Raw(Result<core::ops::Range<usize>, processing::GCodeLineParserError>),
}

/// Reads the next G-code of a channel, or its next raw line when it is given the raw buffer
#[cfg(any(
    feature = "with-serial-usb",
    feature = "with-serial-port-1",
    feature = "with-serial-port-2"
))]
async fn next_of<S>(
    parser: &mut processing::GCodeLineParser<S>,
    channel: hwa::CommChannel,
    #[cfg(feature = "with-sd-card")] raw: Option<&mut [u8]>,
) -> InputEvent
where
    S: async_gcode::ByteStream<Item = Result<u8, async_gcode::Error>>,
{
    #[cfg(feature = "with-sd-card")]
    if let Some(buf) = raw {
        return InputEvent::Raw(parser.next_raw_line(buf).await);
    }
    InputEvent::GCode(parser.next_gcode(channel).await)
}

/// The input stream of every G-code parser: the host channels and the print job (an SD card file).
///
/// All of them share this type so that the G-code parser (~6 KB of flash per stream type) is
/// instantiated once instead of once per stream type. The cost is a `match` per byte and every
/// parser holds the largest stream: with a print job, its stream (the 512 bytes block buffer),
/// ~0.9 KB more of RAM per host channel parser.
#[cfg(any(
    feature = "with-serial-usb",
    feature = "with-serial-port-1",
    feature = "with-serial-port-2",
    feature = "with-print-job"
))]
pub enum GCodeInputStream {
    #[cfg(feature = "with-serial-usb")]
    SerialUsb(hwa::types::SerialUsbInputStream),
    #[cfg(feature = "with-serial-port-1")]
    SerialPort1(hwa::types::SerialPort1InputStream),
    #[cfg(feature = "with-serial-port-2")]
    SerialPort2(hwa::types::SerialPort2InputStream),
    /// The file of the print job
    #[cfg(feature = "with-print-job")]
    PrintJob(hwa::types::PrintJobStream),
}

#[cfg(any(
    feature = "with-serial-usb",
    feature = "with-serial-port-1",
    feature = "with-serial-port-2",
    feature = "with-print-job"
))]
impl async_gcode::ByteStream for GCodeInputStream {
    type Item = Result<u8, async_gcode::Error>;

    async fn next(&mut self) -> Option<Self::Item> {
        match self {
            #[cfg(feature = "with-serial-usb")]
            Self::SerialUsb(s) => s.next().await,
            #[cfg(feature = "with-serial-port-1")]
            Self::SerialPort1(s) => s.next().await,
            #[cfg(feature = "with-serial-port-2")]
            Self::SerialPort2(s) => s.next().await,
            #[cfg(feature = "with-print-job")]
            Self::PrintJob(s) => s.next().await,
        }
    }

    async fn recovery_check(&mut self) {
        match self {
            #[cfg(feature = "with-serial-usb")]
            Self::SerialUsb(s) => s.recovery_check().await,
            #[cfg(feature = "with-serial-port-1")]
            Self::SerialPort1(s) => s.recovery_check().await,
            #[cfg(feature = "with-serial-port-2")]
            Self::SerialPort2(s) => s.recovery_check().await,
            #[cfg(feature = "with-print-job")]
            Self::PrintJob(s) => s.recovery_check().await,
        }
    }
}

// Utility to accept a common gcode stream from multiple sources
pub struct GCodeMultiplexedInputStream {
    /// The channel whose lines are stored instead of executed (M28), if any
    #[cfg(feature = "with-sd-card")]
    capture: Option<hwa::CommChannel>,
    /// The last raw line of the capturing channel
    #[cfg(feature = "with-sd-card")]
    raw: [u8; RAW_LINE_SIZE],
    #[cfg(feature = "with-serial-usb")]
    serial_usb_line_parser: processing::GCodeLineParser<GCodeInputStream>,
    #[cfg(feature = "with-serial-port-1")]
    serial_port1_line_parser: processing::GCodeLineParser<GCodeInputStream>,
    #[cfg(feature = "with-serial-port-2")]
    serial_port2_line_parser: processing::GCodeLineParser<GCodeInputStream>,
}

impl GCodeMultiplexedInputStream {
    pub fn new(
        #[cfg(feature = "with-serial-usb")] serial_usb_rx_stream: hwa::types::SerialUsbInputStream,
        #[cfg(feature = "with-serial-port-1")]
        serial_port1_rx_stream: hwa::types::SerialPort1InputStream,
        #[cfg(feature = "with-serial-port-2")]
        serial_port2_rx_stream: hwa::types::SerialPort2InputStream,
    ) -> Self {
        Self {
            #[cfg(feature = "with-sd-card")]
            capture: None,
            #[cfg(feature = "with-sd-card")]
            raw: [0; RAW_LINE_SIZE],
            #[cfg(feature = "with-serial-usb")]
            serial_usb_line_parser: processing::GCodeLineParser::new(GCodeInputStream::SerialUsb(serial_usb_rx_stream))
                .with_line_number_check(),
            #[cfg(feature = "with-serial-port-1")]
            serial_port1_line_parser: processing::GCodeLineParser::new(GCodeInputStream::SerialPort1(serial_port1_rx_stream))
                .with_line_number_check(),
            #[cfg(feature = "with-serial-port-2")]
            serial_port2_line_parser: processing::GCodeLineParser::new(GCodeInputStream::SerialPort2(serial_port2_rx_stream))
                .with_line_number_check(),
        }
    }

    /// The lines of `channel` are delivered raw ([InputEvent::Raw]) until [Self::stop_capture]
    #[cfg(feature = "with-sd-card")]
    pub fn start_capture(&mut self, channel: hwa::CommChannel) {
        self.capture = Some(channel);
    }

    #[cfg(feature = "with-sd-card")]
    pub fn stop_capture(&mut self) {
        self.capture = None;
    }

    /// The content of the last raw line ([InputEvent::Raw])
    #[cfg(feature = "with-sd-card")]
    pub fn raw_line(&self, range: core::ops::Range<usize>) -> &[u8] {
        self.raw.get(range).unwrap_or(&[])
    }

    pub async fn next_gcode(&mut self) -> (InputEvent, hwa::CommChannel) {
        // The raw buffer goes to the capturing channel only
        #[cfg(feature = "with-sd-card")]
        let (capture, mut raw) = (self.capture, Some(&mut self.raw[..]));
        #[cfg(feature = "with-sd-card")]
        #[allow(unused_mut, unused_variables)]
        let mut raw_for = move |channel: hwa::CommChannel| {
            if capture == Some(channel) { raw.take() } else { None }
        };
        cfg_if::cfg_if! {
            if #[cfg(feature="with-serial-usb")] {
                let f1 = next_of(&mut self.serial_usb_line_parser, hwa::CommChannel::SerialUsb,
                    #[cfg(feature = "with-sd-card")] raw_for(hwa::CommChannel::SerialUsb));
            }
            else {
                let f1 = core::future::pending::<InputEvent>();
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature="with-serial-port-1")] {
                let f2 = next_of(&mut self.serial_port1_line_parser, hwa::CommChannel::SerialPort1,
                    #[cfg(feature = "with-sd-card")] raw_for(hwa::CommChannel::SerialPort1));
            }
            else {
                let f2 = core::future::pending::<InputEvent>();
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature="with-serial-port-2")] {
                let f3 = next_of(&mut self.serial_port2_line_parser, hwa::CommChannel::SerialPort2,
                    #[cfg(feature = "with-sd-card")] raw_for(hwa::CommChannel::SerialPort2));
            }
            else {
                let f3 = core::future::pending::<InputEvent>();
            }
        }

        match embassy_futures::select::select3(f1, f2, f3).await {
            Either3::First(_result) => {
                cfg_if::cfg_if! {
                    if #[cfg(feature="with-serial-usb")] {
                        (_result, hwa::CommChannel::SerialUsb)
                    }
                    else {
                        hwa::error!("Unexpectedly got nothing");
                        (InputEvent::GCode(Err(processing::GCodeLineParserError::EOF)), hwa::CommChannel::Internal)
                    }
                }
            }
            Either3::Second(_result) => {
                cfg_if::cfg_if! {
                    if #[cfg(feature="with-serial-port-1")] {
                        (_result, hwa::CommChannel::SerialPort1)
                    }
                    else {
                        hwa::error!("Unexpectedly got nothing");
                        (InputEvent::GCode(Err(processing::GCodeLineParserError::EOF)), hwa::CommChannel::Internal)
                    }
                }
            }
            Either3::Third(_result) => {
                cfg_if::cfg_if! {
                    if #[cfg(feature="with-serial-port-2")] {
                        (_result, hwa::CommChannel::SerialPort2)
                    }
                    else {
                        hwa::error!("Unexpectedly got nothing");
                        (InputEvent::GCode(Err(processing::GCodeLineParserError::EOF)), hwa::CommChannel::Internal)
                    }
                }
            }
        }
    }

    pub fn get_state(&self, channel: hwa::CommChannel) -> async_gcode::AsyncParserState {
        match channel {
            #[cfg(feature = "with-serial-usb")]
            hwa::CommChannel::SerialUsb => self.serial_usb_line_parser.get_state(),
            #[cfg(feature = "with-serial-port-1")]
            hwa::CommChannel::SerialPort1 => self.serial_port1_line_parser.get_state(),
            #[cfg(feature = "with-serial-port-2")]
            hwa::CommChannel::SerialPort2 => self.serial_port2_line_parser.get_state(),
            _ => async_gcode::AsyncParserState::ErrorRecovery,
        }
    }

    #[cfg(feature = "trace-commands")]
    pub fn get_line(&self, channel: hwa::CommChannel) -> u32 {
        match channel {
            #[cfg(feature = "with-serial-usb")]
            hwa::CommChannel::SerialUsb => self.serial_usb_line_parser.get_line(),
            #[cfg(feature = "with-serial-port-1")]
            hwa::CommChannel::SerialPort1 => self.serial_port1_line_parser.get_line(),
            #[cfg(feature = "with-serial-port-2")]
            hwa::CommChannel::SerialPort2 => self.serial_port2_line_parser.get_line(),
            _ => 0,
        }
    }

    pub fn get_gcode_line(&self, channel: hwa::CommChannel) -> Option<u32> {
        match channel {
            #[cfg(feature = "with-serial-usb")]
            hwa::CommChannel::SerialUsb => self.serial_usb_line_parser.gcode_line(),
            #[cfg(feature = "with-serial-port-1")]
            hwa::CommChannel::SerialPort1 => self.serial_port1_line_parser.gcode_line(),
            #[cfg(feature = "with-serial-port-2")]
            hwa::CommChannel::SerialPort2 => self.serial_port2_line_parser.gcode_line(),
            _ => None,
        }
    }

    pub async fn reset(&mut self, comm_channel: hwa::CommChannel) {
        match comm_channel {
            #[cfg(feature = "with-serial-usb")]
            hwa::CommChannel::SerialUsb => self.serial_usb_line_parser.reset().await,
            #[cfg(feature = "with-serial-port-1")]
            hwa::CommChannel::SerialPort1 => self.serial_port1_line_parser.reset().await,
            #[cfg(feature = "with-serial-port-2")]
            hwa::CommChannel::SerialPort2 => self.serial_port2_line_parser.reset().await,
            _ => {}
        }
    }
}
