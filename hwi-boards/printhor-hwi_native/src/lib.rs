#![allow(stable_features)]
mod board;
pub use board::Contract;

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
            let _tk = spawner.must_spawn(mocked_main(spawner));
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
