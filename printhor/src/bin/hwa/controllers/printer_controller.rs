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
}

pub type PrinterControllerSignalType = embassy_sync::signal::Signal<
    hwa::types::PrinterControllerSignalMutexType,
    PrinterControllerEvent,
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
                    PrinterControllerStatus::Ready(channel) => {
                        hwa::debug!("Ready -> Printing");
                        self.status
                            .signal(PrinterControllerStatus::Printing(*channel));
                        Ok(())
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
                self.channel.signal(event);
                Ok(())
            }
            Err(_any) => Err(_any),
        }
    }

    pub async fn consume(&self) -> PrinterControllerEvent {
        let evt = self.channel.wait().await;
        self.channel.reset();
        evt
    }

    pub async fn signaled(&self) -> bool {
        self.channel.signaled()
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
