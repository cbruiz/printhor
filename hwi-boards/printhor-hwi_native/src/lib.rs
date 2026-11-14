#![allow(stable_features)]
mod board;
pub use board::Contract;

pub mod simulation;

/// Hooks for host-side tests of the firmware running on this simulator.
#[cfg(feature = "with-serial-port-1")]
pub mod test_hooks {
    use crate::board::mocked_peripherals::{SERIAL_PIPE, SERIAL_PORT_1_CAPTURE};

    /// Feeds bytes to serial port 1, as if they were typed on the standard input.
    pub async fn serial_port_1_input(bytes: &[u8]) {
        SERIAL_PIPE.write_all(bytes).await;
    }

    /// Starts (or restarts) capturing what the firmware writes to serial port 1.
    pub fn serial_port_1_capture_start() {
        *SERIAL_PORT_1_CAPTURE.lock().unwrap() = Some(String::new());
    }

    /// What the firmware wrote to serial port 1 since the capture started.
    pub fn serial_port_1_captured() -> String {
        SERIAL_PORT_1_CAPTURE.lock().unwrap().clone().unwrap_or_default()
    }

    /// Stops capturing and returns what was captured.
    pub fn serial_port_1_capture_stop() -> String {
        SERIAL_PORT_1_CAPTURE.lock().unwrap().take().unwrap_or_default()
    }

    /// Makes the simulator of this process listen on a USB socket of its own,
    /// `/tmp/printhor-<pid>.sock`, so that simultaneous runs (several test runs, or a test run and
    /// an interactive simulator) never talk to each other's firmware through the shared default
    /// socket. It must be called before the board setup. Returns the path, so the caller can
    /// remove the socket file when done.
    #[cfg(feature = "with-serial-usb")]
    pub fn use_private_usb_socket() -> &'static str {
        crate::board::set_usb_socket_path(format!("/tmp/printhor-{}.sock", std::process::id()))
    }

    /// A host connected to the simulated USB serial port (the Unix socket the simulator listens on).
    #[cfg(feature = "with-serial-usb")]
    pub struct UsbHostClient {
        stream: smol::net::unix::UnixStream,
    }

    #[cfg(feature = "with-serial-usb")]
    impl UsbHostClient {
        /// Connects to the simulated USB serial port, as a host would.
        pub async fn connect() -> std::io::Result<Self> {
            Ok(Self {
                stream: smol::net::unix::UnixStream::connect(crate::board::usb_socket_path()).await?,
            })
        }

        /// Sends text to the firmware.
        pub async fn send(&mut self, text: &str) -> std::io::Result<()> {
            use smol::io::AsyncWriteExt;
            self.stream.write_all(text.as_bytes()).await
        }

        /// Reads what the firmware writes until `needle` appears (plus what follows it within
        /// 100 ms), the connection is closed or the timeout expires.
        pub async fn read_until(&mut self, needle: &str, timeout: embassy_time::Duration) -> String {
            use embassy_time::{Duration, Instant};
            use smol::io::AsyncReadExt;
            let deadline = Instant::now() + timeout;
            let mut grace_end: Option<Instant> = None;
            let mut out = String::new();
            let mut buf = [0u8; 256];
            loop {
                let limit = grace_end.map_or(deadline, |g| g.min(deadline));
                let now = Instant::now();
                if now >= limit {
                    break;
                }
                match embassy_time::with_timeout(limit - now, self.stream.read(&mut buf)).await {
                    Ok(Ok(n)) if n > 0 => {
                        out.push_str(&String::from_utf8_lossy(&buf[..n]));
                        if grace_end.is_none() && out.contains(needle) {
                            grace_end = Some(Instant::now() + Duration::from_millis(100));
                        }
                    }
                    // EOF, error or timeout
                    _ => break,
                }
            }
            out
        }

        /// Sends `line` and returns the response, until `needle` (see [Self::read_until]).
        pub async fn exchange(&mut self, line: &str, needle: &str, timeout: embassy_time::Duration) -> String {
            match self.send(line).await {
                Ok(()) => self.read_until(needle, timeout).await,
                Err(_) => String::new(),
            }
        }

        /// Closes the connection (the firmware drops it and waits for another host).
        pub fn close(self) {
            let _ = self.stream.shutdown(std::net::Shutdown::Both);
        }
    }
}

#[cfg(not(feature = "s-plot-bin"))]
#[cfg(test)]
mod integration_test {
    use printhor_hwa_common as hwa;
    use hwa::HwiContract;
    use std::marker::PhantomData;
    use std::sync::{Condvar, Mutex};
    use printhor_hwa_common::CoordSel;
    use printhor_hwa_common::traits::StepActuatorTrait;
    use crate::Contract;

    pub static TEST_SIGNAL: hwa::PersistentState<hwa::SyncCsMutexType, bool> =
        hwa::PersistentState::new();

    struct Signaler {
        mutex: Mutex<bool>,
        condvar: Condvar,
    }

    impl Signaler {
        fn new() -> Self {
            Self {
                mutex: Mutex::new(false),
                condvar: Condvar::new(),
            }
        }

        fn wait(&self) {
            let mut signaled = self.mutex.lock().unwrap();
            while !*signaled {
                signaled = self.condvar.wait(signaled).unwrap();
            }
            *signaled = false;
        }
    }

    pub struct MockedExecutor {
        inner: embassy_executor::raw::Executor,
        not_send: PhantomData<*mut ()>,
        signaler: &'static Signaler,
    }

    impl MockedExecutor {
        /// Create a new Executor.
        pub fn new() -> Self {
            let signaler = Box::leak(Box::new(Signaler::new()));
            Self {
                inner: embassy_executor::raw::Executor::new(signaler as *mut Signaler as *mut ()),
                not_send: PhantomData,
                signaler,
            }
        }

        pub fn run(&'static mut self, init: impl FnOnce(embassy_executor::Spawner)) {
            init(self.inner.spawner());

            loop {
                unsafe { self.inner.poll() };
                if TEST_SIGNAL.signaled() {
                    break;
                }
                self.signaler.wait()
            }
        }
    }

    async fn do_machine_test(spawner: embassy_executor::Spawner) {
        Contract::init_logger();
        let mut context = crate::Contract::init(spawner).await;

        context.motion_pins.set_enabled(CoordSel::all_axis(), true);
        context.motion_pins.step_high(CoordSel::all_axis());

        #[cfg(feature = "with-e-axis")]
        assert!(context.motion_pins.e_step_pin.is_high());
        #[cfg(feature = "with-x-axis")]
        assert!(context.motion_pins.x_step_pin.is_high());
        #[cfg(feature = "with-y-axis")]
        assert!(context.motion_pins.y_step_pin.is_high());
        #[cfg(feature = "with-z-axis")]
        assert!(context.motion_pins.z_step_pin.is_high());

        context.motion_pins.step_low(CoordSel::all_axis());

        #[cfg(feature = "with-e-axis")]
        assert!(context.motion_pins.e_step_pin.is_low());
        #[cfg(feature = "with-x-axis")]
        assert!(context.motion_pins.x_step_pin.is_low());
        #[cfg(feature = "with-y-axis")]
        assert!(context.motion_pins.y_step_pin.is_low());
        #[cfg(feature = "with-z-axis")]
        assert!(context.motion_pins.z_step_pin.is_low());

        context.motion_pins.step_high(CoordSel::all_axis());

        #[cfg(feature = "with-e-axis")]
        assert!(context.motion_pins.e_step_pin.is_high());
        #[cfg(feature = "with-x-axis")]
        assert!(context.motion_pins.x_step_pin.is_high());
        #[cfg(feature = "with-y-axis")]
        assert!(context.motion_pins.y_step_pin.is_high());
        #[cfg(feature = "with-z-axis")]
        assert!(context.motion_pins.z_step_pin.is_high());
        
        TEST_SIGNAL.signal(true);
    }

    #[test]
    fn machine_test() {
        std::env::set_current_dir(std::env::current_dir().unwrap().parent().unwrap()).unwrap();
        // 1. Init embassy runtime
        let executor = hwa::make_static_ref!("Executor", MockedExecutor, MockedExecutor::new());

        /// The test entry-point
        #[embassy_executor::task(pool_size = 1)]
        async fn mocked_main(spawner: embassy_executor::Spawner) {
            do_machine_test(spawner).await;
            if TEST_SIGNAL.wait().await {
                hwa::info!("Integration task Succeeded");
            } else {
                panic!("Integration task Failed");
            }
        }

        // 2. Spawn the [mocked_main] task
        executor.run(|spawner| {
            let _tk = spawner.spawn(mocked_main(spawner).unwrap());
        });
        hwa::info!("executor gone...");
    }

    #[cfg(test)]
    mod test {
        use printhor_hwa_common as hwa;
        use crate::board::mocked_peripherals;
        use mocked_peripherals::{MockedIOPin, PinState};

        #[test]
        fn test_internals() {
            let _pin_state = hwa::make_static_ref!(
            "GlobalPinState",
            mocked_peripherals::PinsCell<PinState>,
            mocked_peripherals::PinsCell::new(PinState::new())
        );
            let mut pin = MockedIOPin::new(0, _pin_state);
            // Test it does not fail when _pin_state is locked
            pin.set_high();
            assert_eq!(pin.is_high(), true);
            assert_eq!(pin.is_low(), false);
            pin.set_low();
            assert_eq!(pin.is_high(), false);
            assert_eq!(pin.is_low(), true);
            pin.toggle();
            assert_eq!(pin.is_high(), true);
            assert_eq!(pin.is_low(), false);
            pin.toggle();
            assert_eq!(pin.is_high(), false);
            assert_eq!(pin.is_low(), true);
        }
    }
}
