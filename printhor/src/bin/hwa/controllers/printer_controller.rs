//! TODO: This feature is still in incubation
use crate::hwa;
use hwa::{CommChannel, PersistentState};

#[cfg_attr(feature = "native", derive(Debug))]
/// Enumeration representing various events that the Printer Controller can handle.
pub enum PrinterControllerEvent {
    /// Event to set a file to be printed.
    /// Contains the file name as an `alloc::string::String`.
    SetFile(CommChannel, alloc::string::String),

    /// Event to resume a paused print job.
    Resume(CommChannel),

    /// Event to pause an ongoing print job.
    Pause(CommChannel),

    /// Event to abort the current print job.
    Abort(CommChannel),
}

#[cfg_attr(feature = "native", derive(Debug))]
#[derive(Copy, Clone)]
/// Enumeration representing the status of the Printer Controller.
pub enum PrinterControllerStatus {
    /// Indicates that the printer is ready to receive print jobs.
    Ready(CommChannel),
    /// Indicates that a print job is currently in progress.
    Printing(CommChannel),
    /// Indicates that a print job is in progress but currently paused.
    Paused(CommChannel),
}

/// Enum representing possible errors that can occur in the Printer Controller.
///
/// Variants:
///
/// - `AlreadyPrinting`:
///     Occurs when attempting to start a new print job while another job is already in progress.
///     To start a new job, the ongoing job must be stopped first.
///
/// - `NotPrinting`:
///     Occurs when attempting to stop a print job when no job is currently running. This generally happens when the printer is in a `Ready` state.
///
/// - `NoEffect`:
///     Occurs when the requested operation has no effect.
///     For example, trying to start a job that is already started, or stop a job that is already stopped.
#[derive(Debug)]
pub enum PrinterControllerError {
    /// Attempted to start a new print job while another is already in progress.
    /// To start a new job, the ongoing job must be stopped first.
    AlreadyPrinting,

    /// Attempted to stop a print job when no job is currently running.
    /// This generally happens when the printer is in a `Ready` state.
    NotPrinting,

    /// The requested operation has no effect.
    /// For example, trying to start a job that is already started, or stop a job that is already stopped.
    NoEffect,

    /// Too many job events pending to be consumed by the print job task: retry later.
    Busy,

    /// Attempted to start/resume (`M24`) with no file selected (`M23`).
    NoFileSelected,
}

/// Pending job events (e.g. `M23` immediately followed by `M24`) are queued, never overwritten.
const PRINTER_CONTROLLER_QUEUE_LEN: usize = 4;

pub type PrinterControllerSignalType = embassy_sync::channel::Channel<
    hwa::types::PrinterControllerSignalMutexType,
    PrinterControllerEvent,
    PRINTER_CONTROLLER_QUEUE_LEN,
>;

/// The `PrinterController` struct represents the controller for managing printer operations.
///
/// This controller maintains the state and communication channels necessary for handling print jobs and events.
///
/// Fields:
///
/// - `channel`:
///     A reference to the signal channel for sending and receiving `PrinterControllerEvent` events.
///     This channel is used to communicate event changes within the printer controller.
///
/// - `event_bus`:
///     A reference to the event bus that facilitates communication between different components or modules that need to respond to printer events.
///     This is typically used for broadcasting events to various parts of the system.
///
/// - `status`:
///     A reference to the status configuration that keeps track of the current state of the printer.
///     The status can be one of `PrinterControllerStatus::Ready`, `PrinterControllerStatus::Printing`, or `PrinterControllerStatus::Paused`.
pub struct PrinterController {
    /// The signal channel for handling printer controller events.
    channel: &'static PrinterControllerSignalType,

    /// The event bus for broadcasting events.
    event_bus: hwa::types::EventBus,

    /// The current status of the printer controller.
    status: &'static PersistentState<
        hwa::types::PrinterControllerSignalMutexType,
        PrinterControllerStatus,
    >,
}

impl PrinterController {
    pub fn new(event_bus: hwa::types::EventBus) -> PrinterController {
        type StatusType =
            PersistentState<hwa::types::PrinterControllerSignalMutexType, PrinterControllerStatus>;
        let status_cfg = StatusType::new();
        status_cfg.signal(PrinterControllerStatus::Ready(CommChannel::Internal));
        let channel = hwa::make_static_ref!(
            "PrinterControllerChannelSignal",
            PrinterControllerSignalType,
            PrinterControllerSignalType::new()
        );
        Self {
            channel,
            event_bus,
            status: hwa::make_static_ref!("PrinterControllerStatus", StatusType, status_cfg),
        }
    }

    pub async fn set(
        &mut self,
        event: PrinterControllerEvent,
    ) -> Result<(), PrinterControllerError> {
        // Checked before the state transition, so a full queue leaves everything untouched. Never
        // waits for room: the print job task itself (the consumer) also sets events (e.g. `M2`)
        if self.channel.is_full() {
            return Err(PrinterControllerError::Busy);
        }
        let update_result = match &event {
            PrinterControllerEvent::SetFile(_channel, _) => {
                hwa::debug!(">> SetFile");
                match &self.status.wait().await {
                    PrinterControllerStatus::Ready(channel) => {
                        hwa::debug!("Ready -> Paused");
                        self.status
                            .signal(PrinterControllerStatus::Paused(*channel));
                        Ok(())
                    }
                    PrinterControllerStatus::Printing(_channel) => {
                        Err(PrinterControllerError::AlreadyPrinting)
                    }
                    PrinterControllerStatus::Paused(_channel) => {
                        Err(PrinterControllerError::AlreadyPrinting)
                    }
                }
            }
            PrinterControllerEvent::Resume(_channel) => {
                hwa::debug!(">> Resume");
                match &self.status.wait().await {
                    // Nothing selected: there is nothing to start
                    PrinterControllerStatus::Ready(_channel) => {
                        Err(PrinterControllerError::NoFileSelected)
                    }
                    PrinterControllerStatus::Printing(_channel) => {
                        Err(PrinterControllerError::NoEffect)
                    }
                    PrinterControllerStatus::Paused(channel) => {
                        hwa::debug!("Paused -> Printing");
                        self.status
                            .signal(PrinterControllerStatus::Printing(*channel));
                        Ok(())
                    }
                }
            }
            PrinterControllerEvent::Pause(_channel) => {
                hwa::debug!(">> Pause");
                match &self.status.wait().await {
                    PrinterControllerStatus::Ready(_channel) => {
                        hwa::debug!("NotPrinting!!");
                        Err(PrinterControllerError::NotPrinting)
                    }
                    PrinterControllerStatus::Printing(channel) => {
                        hwa::debug!("Printing -> Paused");
                        self.status
                            .signal(PrinterControllerStatus::Paused(*channel));
                        Ok(())
                    }
                    PrinterControllerStatus::Paused(_channel) => {
                        Err(PrinterControllerError::NoEffect)
                    }
                }
            }
            PrinterControllerEvent::Abort(channel) => {
                hwa::debug!(">> Abort");
                match &self.status.wait().await {
                    PrinterControllerStatus::Ready(_channel) => {
                        hwa::debug!("NoEffect");
                        Err(PrinterControllerError::NoEffect)
                    }
                    PrinterControllerStatus::Printing(_channel) => {
                        hwa::debug!("Printing -> Ready");
                        self.status.signal(PrinterControllerStatus::Ready(*channel));
                        Ok(())
                    }
                    PrinterControllerStatus::Paused(channel) => {
                        hwa::debug!("Paused -> Ready");
                        self.status.signal(PrinterControllerStatus::Ready(*channel));
                        Ok(())
                    }
                }
            }
        };
        match update_result {
            Ok(()) => {
                // Room was checked above and this is the only producer path
                self.channel
                    .try_send(event)
                    .map_err(|_| PrinterControllerError::Busy)
            }
            Err(_any) => Err(_any),
        }
    }

    /// Marks the current job as finished (Ready), with no event. Called when the job ends normally
    pub fn finish(&self, channel: CommChannel) {
        self.status.signal(PrinterControllerStatus::Ready(channel));
    }

    /// The current status. It is updated synchronously by [Self::set] (before the print job task
    /// consumes the event), so it is the reference for the print job task to react: e.g. a job
    /// loop stops as soon as the status is not `Printing` any more.
    pub async fn status(&self) -> PrinterControllerStatus {
        self.status.wait().await
    }

    /// Whether the job is (still) meant to be running
    pub async fn is_printing(&self) -> bool {
        matches!(self.status.wait().await, PrinterControllerStatus::Printing(_))
    }

    /// Whether a job is selected or running
    pub async fn is_busy(&self) -> bool {
        !matches!(self.status.wait().await, PrinterControllerStatus::Ready(_))
    }

    pub async fn consume(&self) -> PrinterControllerEvent {
        self.channel.receive().await
    }

    pub async fn signaled(&self) -> bool {
        !self.channel.is_empty()
    }
}

impl Clone for PrinterController {
    fn clone(&self) -> Self {
        PrinterController {
            channel: self.channel,
            event_bus: self.event_bus.clone(),
            status: self.status,
        }
    }
}

//#region "Job progress"

use core::sync::atomic::{AtomicU8, AtomicU16, AtomicU32, Ordering};

/// No value set (percentage)
const PERCENT_NONE: u8 = u8::MAX;
/// No value set (remaining minutes)
const REMAINING_NONE: u32 = u32::MAX;

/// Progress of the SD print job (`M27`, `M73`), shared between the tasks without locks nor heap.
///
/// Only plain atomic loads/stores are used (no read-modify-write), which every target supports,
/// Cortex-M0+ included. The fields are independent: a reader may combine a position and a size
/// from slightly different instants, which is harmless for a progress report.
pub struct JobProgress {
    /// Size of the selected file in bytes (0 = no file selected)
    size: AtomicU32,
    /// Bytes of the file consumed by the job
    position: AtomicU32,
    /// Percentage set by the slicer (`M73 P`), or [PERCENT_NONE]
    percent: AtomicU8,
    /// Remaining minutes set by the slicer (`M73 R`), or [REMAINING_NONE]
    remaining_min: AtomicU32,
    /// `M27 S` auto-report interval in seconds (0 = off)
    autoreport_s: AtomicU16,
    /// Channel of the `M27 S` request, where the auto-reports go
    autoreport_channel: critical_section::Mutex<core::cell::Cell<CommChannel>>,
}

impl JobProgress {
    const fn new() -> Self {
        Self {
            size: AtomicU32::new(0),
            position: AtomicU32::new(0),
            percent: AtomicU8::new(PERCENT_NONE),
            remaining_min: AtomicU32::new(REMAINING_NONE),
            autoreport_s: AtomicU16::new(0),
            autoreport_channel: critical_section::Mutex::new(core::cell::Cell::new(
                CommChannel::Internal,
            )),
        }
    }

    /// A file of `size` bytes was selected: the progress starts from zero
    pub fn start(&self, size: u32) {
        self.position.store(0, Ordering::Relaxed);
        self.percent.store(PERCENT_NONE, Ordering::Relaxed);
        self.remaining_min.store(REMAINING_NONE, Ordering::Relaxed);
        self.size.store(size, Ordering::Relaxed);
    }

    /// No file selected any more (job completed, aborted or replaced)
    pub fn clear(&self) {
        self.size.store(0, Ordering::Relaxed);
        self.position.store(0, Ordering::Relaxed);
        self.percent.store(PERCENT_NONE, Ordering::Relaxed);
        self.remaining_min.store(REMAINING_NONE, Ordering::Relaxed);
    }

    #[inline]
    pub fn set_position(&self, position: u32) {
        self.position.store(position, Ordering::Relaxed);
    }

    /// `M73`: the values given by the slicer (each one only when present)
    pub fn set_slicer_progress(&self, percent: Option<u8>, remaining_min: Option<u32>) {
        if let Some(p) = percent {
            self.percent.store(p.min(100), Ordering::Relaxed);
        }
        if let Some(r) = remaining_min {
            self.remaining_min
                .store(r.min(REMAINING_NONE - 1), Ordering::Relaxed);
        }
    }

    /// `(position, size)` while a file is selected
    pub fn sd_bytes(&self) -> Option<(u32, u32)> {
        let size = self.size.load(Ordering::Relaxed);
        match size {
            0 => None,
            _ => Some((self.position.load(Ordering::Relaxed).min(size), size)),
        }
    }

    /// The percentage given by the slicer (`M73`), or else computed from the file position
    pub fn percent(&self) -> Option<u8> {
        match self.percent.load(Ordering::Relaxed) {
            PERCENT_NONE => self
                .sd_bytes()
                .map(|(pos, size)| ((u64::from(pos) * 100) / u64::from(size)) as u8),
            p => Some(p),
        }
    }

    /// The remaining minutes given by the slicer (`M73 R`)
    pub fn remaining_min(&self) -> Option<u32> {
        match self.remaining_min.load(Ordering::Relaxed) {
            REMAINING_NONE => None,
            r => Some(r),
        }
    }

    /// `M27 S<secs>` (0 = off): the reports go to `channel`
    pub fn set_autoreport(&self, secs: u16, channel: CommChannel) {
        critical_section::with(|cs| self.autoreport_channel.borrow(cs).set(channel));
        self.autoreport_s.store(secs, Ordering::Relaxed);
    }

    /// The `M27 S` auto-report interval and channel, when enabled
    pub fn autoreport(&self) -> Option<(u16, CommChannel)> {
        match self.autoreport_s.load(Ordering::Relaxed) {
            0 => None,
            secs => Some((
                secs,
                critical_section::with(|cs| self.autoreport_channel.borrow(cs).get()),
            )),
        }
    }
}

/// The progress of the SD print job
pub static JOB_PROGRESS: JobProgress = JobProgress::new();

//#endregion
