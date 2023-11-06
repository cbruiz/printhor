//! TODO: This feature is still very experimental
//! TODO: Pending to review after intense refactor
use printhor_hwa_common::TrackedStaticCell;
use printhor_hwa_common::EventBusRef;

#[allow(unused)]
#[cfg_attr(feature = "native", derive(Debug))]
pub enum PrinterControllerEvent {
    PrintFile(alloc::string::String),
    Pause,
    Resume,
    Abort,
}

#[allow(unused)]
#[cfg_attr(feature = "native", derive(Debug))]
#[derive(Copy, Clone)]
pub enum PrinterControllerStatus {
    /// Ready to get orders
    Ready,
    /// There is a job work in progress
    Printing,
    /// There is a job work in progress but it's paused
    Paused,
}

#[allow(unused)]
#[derive(Debug)]
pub enum PrinterControllerError {
    /// You wanted to schedule a print job but there is something running. It must be stopped first.
    AlreadyPrinting,
    /// You wanted to stop a print job but there is nothing running.
    NotPrinting,
}

type ChannelMutexType = embassy_sync::blocking_mutex::raw::NoopRawMutex;

pub type PrinterControllerSignalType = embassy_sync::signal::Signal<ChannelMutexType, PrinterControllerEvent>;

pub struct PrinterController {
    channel: &'static PrinterControllerSignalType,
    event_bus: EventBusRef,
    status: PrinterControllerStatus, // TODO: replace by event_bus
}

impl PrinterController {
    pub(crate) fn new(event_bus: EventBusRef) -> PrinterController {
        static SIGNAL_CHANNEL_INST: TrackedStaticCell<PrinterControllerSignalType> = TrackedStaticCell::new();
        let channel = SIGNAL_CHANNEL_INST.init("PrinterController::channel", PrinterControllerSignalType::new());
        PrinterController {
            channel,
            event_bus,
            status: PrinterControllerStatus::Ready,
        }
    }

    #[allow(unused)]
    #[inline]
    pub(crate) fn something_running(&self) -> bool {
        match self.status {
            PrinterControllerStatus::Ready => false,
            PrinterControllerStatus::Printing => true,
            PrinterControllerStatus::Paused => true,
        }
    }

    #[inline]
    pub(crate) fn set(&mut self, event: PrinterControllerEvent) -> Result<(), PrinterControllerError> {
        match self.status {
            PrinterControllerStatus::Ready => {
                self.channel.signal(event);
                Ok(())
            },
            PrinterControllerStatus::Printing => {
                Err(PrinterControllerError::AlreadyPrinting)
            },
            PrinterControllerStatus::Paused => {
                Err(PrinterControllerError::AlreadyPrinting)
            },
        }
    }

    #[inline]
    pub(crate) async fn wait(&self) -> PrinterControllerEvent {
        self.channel.wait().await
    }

    #[allow(unused)]
    #[inline]
    pub(crate) async fn complete(&mut self) {
        self.channel.reset();
        self.status = PrinterControllerStatus::Ready;
    }

    #[allow(unused)]
    pub(crate) async fn abort(&mut self) -> Result<(), PrinterControllerError> {
        match self.status {
            PrinterControllerStatus::Ready => {
                Err(PrinterControllerError::NotPrinting)
            },
            PrinterControllerStatus::Printing => {
                // TODO: Signal abort to task
                self.status = PrinterControllerStatus::Ready;
                Ok(())
            },
            PrinterControllerStatus::Paused => {
                // TODO: Signal abort to task
                self.status = PrinterControllerStatus::Ready;
                Ok(())
            },
        }
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