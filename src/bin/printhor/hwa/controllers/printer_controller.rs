//! TODO: This feature is still in incubation
use printhor_hwa_common::TrackedStaticCell;
use printhor_hwa_common::EventBusRef;
use crate::hwa;
use crate::sync::config::Config;

#[allow(unused)]
#[cfg_attr(feature = "native", derive(Debug))]
pub enum PrinterControllerEvent {
    SetFile(alloc::string::String),
    Resume,
    Pause,
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
    /// The command has no effect (start what is already started, stop what is already stopped...).
    NoEffect,
}

type ChannelMutexType = embassy_sync::blocking_mutex::raw::NoopRawMutex;

pub type PrinterControllerSignalType = embassy_sync::signal::Signal<ChannelMutexType, PrinterControllerEvent>;

pub struct PrinterController {
    channel: &'static PrinterControllerSignalType,
    event_bus: EventBusRef,
    status: &'static Config<ChannelMutexType,PrinterControllerStatus>,
}

impl PrinterController {
    pub(crate) fn new(event_bus: EventBusRef) -> PrinterController {
        static SIGNAL_CHANNEL_INST: TrackedStaticCell<PrinterControllerSignalType> = TrackedStaticCell::new();
        static STATUS_INST: TrackedStaticCell<Config<ChannelMutexType, PrinterControllerStatus>> = TrackedStaticCell::new();

        let channel = SIGNAL_CHANNEL_INST.init("PrinterController::channel", PrinterControllerSignalType::new());
        let status_cfg = Config::new();
        status_cfg.signal(PrinterControllerStatus::Ready);
        let status = STATUS_INST.init("PrinterController::state", status_cfg);
        PrinterController {
            channel,
            event_bus,
            status,
        }
    }

    #[allow(unused)]
    #[inline]
    pub async fn something_running(&self) -> bool {
        match self.status.wait().await {
            PrinterControllerStatus::Ready => false,
            PrinterControllerStatus::Printing => true,
            PrinterControllerStatus::Paused => true,
        }
    }

    pub async fn set(&mut self, event: PrinterControllerEvent) -> Result<(), PrinterControllerError> {
        match &event {
            PrinterControllerEvent::SetFile(_) => {
                hwa::debug!(">> SetFile");
                match &self.status.wait().await {
                    PrinterControllerStatus::Ready => {
                        hwa::debug!("Ready -> Paused");
                        self.status.signal(PrinterControllerStatus::Paused);
                        self.channel.signal(event);
                        Ok(())
                    },
                    PrinterControllerStatus::Printing => {
                        Err(PrinterControllerError::AlreadyPrinting)
                    },
                    PrinterControllerStatus::Paused => {
                        Err(PrinterControllerError::NoEffect)
                    },
                }

            }
            PrinterControllerEvent::Resume => {
                hwa::debug!(">> Resume");
                match &self.status.wait().await {
                    PrinterControllerStatus::Ready => {
                        hwa::debug!("Ready -> Printing");
                        self.status.signal(PrinterControllerStatus::Printing);
                        self.channel.signal(event);
                        Ok(())
                    },
                    PrinterControllerStatus::Printing => {
                        Err(PrinterControllerError::NoEffect)
                    },
                    PrinterControllerStatus::Paused => {
                        hwa::debug!("Paused -> Printing");
                        self.status.signal(PrinterControllerStatus::Printing);
                        self.channel.signal(event);
                        Ok(())
                    },
                }
            }
            PrinterControllerEvent::Pause => {
                hwa::debug!(">> Pause");
                match &self.status.wait().await {
                    PrinterControllerStatus::Ready => {
                        hwa::debug!("NotPrinting!!");
                        Err(PrinterControllerError::NotPrinting)
                    },
                    PrinterControllerStatus::Printing => {
                        hwa::debug!("Printing -> Paused");
                        self.status.signal(PrinterControllerStatus::Paused);
                        self.channel.signal(event);
                        Ok(())
                    },
                    PrinterControllerStatus::Paused => {
                        Err(PrinterControllerError::NoEffect)
                    },
                }
            }
            PrinterControllerEvent::Abort => {
                hwa::debug!(">> Abort");
                match &self.status.wait().await {
                    PrinterControllerStatus::Ready => {
                        hwa::debug!("NoEffect");
                        Err(PrinterControllerError::NoEffect)
                    },
                    PrinterControllerStatus::Printing => {
                        hwa::debug!("Printing -> Ready");
                        self.status.signal(PrinterControllerStatus::Ready);
                        self.channel.signal(event);
                        Ok(())
                    },
                    PrinterControllerStatus::Paused => {
                        hwa::debug!("Paused -> Ready");
                        self.status.signal(PrinterControllerStatus::Ready);
                        self.channel.signal(event);
                        Ok(())
                    },
                }
            }
        }
    }

    #[inline]
    pub(crate) async fn consume(&self) -> PrinterControllerEvent {
        let evt = self.channel.wait().await;
        self.channel.reset();
        evt
    }

    #[inline]
    pub(crate) async fn signaled(&self) -> bool {
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