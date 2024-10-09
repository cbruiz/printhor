use crate::hwa;
use crate::hwa::controllers::{MultiTimer, StepPlanner};
use core::cell::RefCell;
use core::future::{poll_fn, Future};
use core::task::{Context, Poll};
use critical_section::Mutex as CsMutex;
use embassy_sync::waitqueue::WakerRegistration;
use printhor_hwa_common::StepperChannel;

/// The size of the timer queue.
const TIMER_QUEUE_SIZE: usize = 4;

/// Represents the state of the soft timer driver.
#[derive(PartialEq)]
pub enum State {
    Idle,
    Duty,
}

/// The driver for the soft timer, managing timed operations for stepper motors.
pub struct SoftTimerDriver {
    /// The current step planner in use.
    current: StepPlanner,

    /// The current state of the driver.
    pub state: State,

    /// The tick count for the current timer.
    tick_count: u32,

    /// The queue of step planners to be executed.
    queue: [Option<StepPlanner>; TIMER_QUEUE_SIZE],

    /// The head index of the queue.
    head: u8,

    /// The tail index of the queue.
    tail: u8,

    /// The number of step planners currently queued.
    num_queued: u8,

    /// Reference to the motion driver.
    pins: Option<hwa::controllers::MotionPins>,

    /// The waker registration for waking up tasks.
    waker: WakerRegistration,

    /// Flags indicating the current enabled stepper channels.
    current_stepper_enable_flags: StepperChannel,

    /// Flags indicating the current forward direction stepper channels.
    current_stepper_dir_fwd_flags: StepperChannel,

    /// The number of pulses generated for each stepper channel (for debugging/testing).
    #[cfg(any(feature = "assert-motion", test))]
    pub pulses: [u32; 4],
}

impl SoftTimerDriver {
    /// Tries to consume the next `StepPlanner` from the queue and setup the current timer.
    ///
    /// This method checks if there are any `StepPlanner` instances queued. If there are, it
    /// dequeues the next one, sets it as the current step planner, and initializes the timer
    /// for the next operation. It also updates the stepper enable and direction flags as
    /// necessary.
    ///
    /// # Returns
    ///
    /// * `true` - If a `StepPlanner` was successfully dequeued and set up.
    /// * `false` - If the queue was empty and there's no work to do.
    ///
    /// # Panics
    ///
    /// This method panics if:
    ///
    /// * The driver instance is not available.
    /// * It is unable to lock the driver.
    /// * It encounters an unexpected state, such as being unable to dequeue a `StepPlanner`.
    ///
    /// # Examples
    ///
    /// ```
    /// // Assuming you have a `SoftTimerDriver` instance named `driver`
    /// let result = driver.try_consume();
    /// ```
    fn try_consume(&mut self) -> bool {
        if self.num_queued > 0 {
            if let Some(timer) = self.queue[self.head as usize].take() {
                self.current = timer;
                self.tick_count = 0;

                match self.pins.as_ref() {
                    None => {
                        unreachable!("Driver instance not set")
                    }
                    Some(pins) => {
                        #[cfg(feature = "debug-signals")]
                        pins.y_dir_pin.set_high();
                        if self.current_stepper_enable_flags
                            != self.current.stepper_enable_flags
                        {
                            pins.enable_steppers(self.current.stepper_enable_flags);
                            self.current_stepper_enable_flags =
                                self.current.stepper_enable_flags;
                        }
                        if self.current_stepper_dir_fwd_flags
                            != self.current.stepper_dir_fwd_flags
                        {
                            pins.set_forward_direction(self.current.stepper_dir_fwd_flags);
                            self.current_stepper_dir_fwd_flags =
                                self.current.stepper_dir_fwd_flags;
                        }
                    }
                }
                hwa::trace!("u-segment dequeued at {}", self.head);
            } else {
                unreachable!("Unable to deque!!!")
            }
            self.state = State::Duty;
            self.head += 1;
            if self.head as usize == self.queue.len() {
                self.head = 0;
            }
            self.num_queued -= 1;
            true
        } else {
            self.state = State::Idle;
            // Quickly return as there is no work to do
            false
        }
    }

    /// Notifies the system that there are queued items to process.
    ///
    /// This method is typically called to wake up any tasks waiting on the
    /// completion of the queue processing. It ensures that the system checks
    /// the queue for any new items to process.
    ///
    /// If the number of queued items is less than `TIMER_QUEUE_SIZE`, it
    /// triggers the waker to continue processing the queue.
    ///
    /// # Examples
    ///
    /// ```
    /// // Assuming you have a `SoftTimerDriver` instance named `driver`
    /// driver.notify();
    /// ```
    pub fn notify(&mut self) {
        if (self.num_queued as usize) < TIMER_QUEUE_SIZE {
            self.waker.wake();
        }
    }

    /// Handles the interrupt routine for the timer.
    ///
    /// This method is called when a timer interrupt occurs. It processes the current state
    /// of the timer and steps through the queue of `StepPlanner` instances. The method
    /// performs the following steps:
    ///
    /// 1. If the current state is idle, it attempts to consume the next item in the queue.
    /// 2. If an item was consumed, it updates the tick count by the stepper planner clock period.
    /// 3. Processes the next step based on the current timer.
    /// 4. If the timer interval has been completed, it attempts to consume the next item
    ///    in the queue and set the state to either Duty or Idle.
    /// 5. Notifies the system about any pending items in the queue.
    ///
    /// # Panics
    ///
    /// This method will panic if:
    ///
    /// * It encounters an unexpected state, such as being unable to process the next step.
    ///
    /// # Examples
    ///
    /// ```
    /// // Assuming you have a `SoftTimerDriver` instance named `driver`
    /// driver.on_interrupt();
    /// ```
    pub fn on_interrupt(&mut self) {
        if self.state == State::Idle {
            if !self.try_consume() {
                self.notify();
                return;
            }
        }
        self.tick_count += crate::control::task_stepper::STEPPER_PLANNER_CLOCK_PERIOD_US;

        // In this point, state is either Duty or something were dequeued
        match self
            .current
            .next(crate::control::task_stepper::STEPPER_PLANNER_CLOCK_PERIOD_US)
        {
            None => {
                panic!("Unexpected state");
            }
            Some(_ch) => {
                self.apply(_ch);
            }
        }

        if self.tick_count >= self.current.interval_width {
            hwa::trace!("u-segment completed in {} us", self.tick_count);
            if self.try_consume() {}
            #[cfg(feature = "debug-signals")]
            match &self.drv {
                None => {
                    unreachable!("Driver instance not set")
                }
                Some(_ref) => match _ref.try_lock() {
                    Ok(mut _drv) => {
                        _drv.pins.y_dir_pin.set_low();
                    }
                    Err(_) => {
                        unreachable!("unable to lock")
                    }
                },
            }
            self.notify();
        }
    }

    /// Applies the current stepper channel.
    ///
    /// This method is responsible for applying the changes to the stepper motor based on the given channel.
    /// It checks if the channel is empty and returns `true` if it is. If the channel is not empty, it attempts
    /// to lock the driver and apply the step through the configured stepper pins.
    ///
    /// # Parameters
    ///
    /// - `_channel`: The stepper channel to apply.
    ///
    /// # Returns
    ///
    /// - `true` if the stepper channel was successfully applied or if the channel is empty.
    /// - `false` if the driver is unavailable.
    ///
    /// If the `pulsed` feature is enabled, this method will attempt to output a pulse to the stepper pin.
    /// Otherwise, it will toggle the stepper pin.
    ///
    /// # Panics
    ///
    /// This method will panic if the driver is set but cannot be locked or if the driver instance is not set when
    /// the "debug-signals" feature is enabled.
    ///
    /// # Examples
    ///
    /// ```
    /// // Assuming a `StepperChannel` instance named `channel` and a mutable reference of `SoftTimerDriver` named `driver`
    /// let is_applied = driver.apply(channel);
    /// ```
    fn apply(&mut self, channels: StepperChannel) -> bool {
        if channels.is_empty() {
            true
        } else {
            match self.pins.as_ref() {
                None => false,
                Some(pins) => {
                    cfg_if::cfg_if! {
                        if #[cfg(feature = "pulsed")] {
                            todo!("Not yet implemented")
                            //pins.step_pin_high(channels);
                            //s_block_for(STEPPER_PULSE_WIDTH_US);
                            //pins.step_pin_high(channels);
                        }
                        else {
                            pins.step_toggle(channels);
                        }
                    }
                    cfg_if::cfg_if! {
                        if #[cfg(feature = "assert-motion")] {
                            if _channel.contains(StepperChannel::X) {
                                self.pulses[0] += 1;
                            }
                            if _channel.contains(StepperChannel::Y) {
                                self.pulses[1] += 1;
                            }
                            if _channel.contains(StepperChannel::Z) {
                                self.pulses[2] += 1;
                            }
                            if _channel.contains(StepperChannel::E) {
                                self.pulses[3] += 1;
                            }
                        }
                    }
                    true
                },
            }
        }
    }
}

/// The `SoftTimer` struct encapsulates a timer mechanism that coordinates
/// stepper motor operations with various functional capabilities based
/// on associated feature flags. It uses a critical section for synchronization
/// of operations reactive to events.
///
/// # Fields
///
/// - `0`: A mutex protected, reference-counted cell containing the soft timer driver.
///
/// # Features
///
/// - `assert-motion`: If enabled, exposes additional debugging capabilities
///   for motion and pulse counts.
/// - `test`: Enables testing mode for the struct.
///
/// # Example
///
/// ```
/// // Instantiating a new SoftTimer.
/// let timer = SoftTimer::new();
///
/// // Setting up the SoftTimer with a motion driver reference.
/// let motion_driver_ref = ...; // Provide a motion driver reference
/// timer.setup(motion_driver_ref);
///
/// // Pushing a multi-timer segment onto the queue.
/// let multi_timer = ...; // Define a multi-timer
/// let stepper_enable_flags = StepperChannel::X; // Enable X channel
/// let stepper_dir_fwd_flags = StepperChannel::X; // Forward direction for X channel
/// let push_future = timer.push(multi_timer, stepper_enable_flags, stepper_dir_fwd_flags);
/// ```
pub struct SoftTimer(pub CsMutex<RefCell<SoftTimerDriver>>);

cfg_if::cfg_if! {
    if #[cfg(any(feature = "assert-motion", test))] {
        pub type FlushResult = crate::math::TVector<u32>;
    }
    else {
        pub type FlushResult = ();
    }
}

impl SoftTimer {
    /// Creates a new instance of `SoftTimer`.
    ///
    /// This constructor initializes a `SoftTimer` with a `SoftTimerDriver`
    /// containing default values for its fields. The `SoftTimerDriver` is
    /// encapsulated within a `CsMutex` and `RefCell` to ensure safe concurrent
    /// access.
    ///
    /// The `SoftTimer` struct coordinates stepper motor operations and ensures
    /// synchronization using critical sections. The fields within the `SoftTimerDriver`
    /// include attributes for step planning, state, queue management, pulse counting,
    /// and driver configuration.
    ///
    /// # Return
    ///
    /// Returns an instance of `SoftTimer`.
    ///
    /// # Example
    ///
    /// ```
    /// // Creating a new SoftTimer instance
    /// let timer = SoftTimer::new();
    /// ```
    pub const fn new() -> Self {
        Self(CsMutex::new(RefCell::new(SoftTimerDriver {
            current: StepPlanner::new(),
            state: State::Idle,
            queue: [None; TIMER_QUEUE_SIZE],
            head: 0,
            tick_count: 0,
            waker: WakerRegistration::new(),
            num_queued: 0,
            tail: 0,
            pins: None,
            current_stepper_enable_flags: StepperChannel::empty(),
            current_stepper_dir_fwd_flags: StepperChannel::empty(),
            #[cfg(any(feature = "assert-motion", test))]
            pulses: [0, 0, 0, 0],
        })))
    }

    /// Sets up the `SoftTimer` instance with a motion driver reference.
    ///
    /// This method configures the `SoftTimer` by associating it with a provided
    /// motion driver reference. It ensures that all subsequent operations can
    /// utilize the provided driver for managing stepper motor operations.
    ///
    /// The setup process is enclosed within a critical section to ensure safe
    /// concurrent access and modification of the internal state.
    ///
    /// # Parameters
    ///
    /// - `_mp`: A motion driver reference that the `SoftTimer` will use for its
    ///   operations.
    ///
    /// # Example
    ///
    /// ```
    /// let timer = SoftTimer::new();
    /// let motion_driver_ref = ...; // Provide a motion driver reference
    /// timer.setup(motion_driver_ref);
    /// ```
    pub fn setup(&self, _pins: hwa::controllers::MotionPins) {
        critical_section::with(|cs| {
            let mut r = self.0.borrow_ref_mut(cs);
            r.pins.replace(_pins);
        })
    }

    /// Pushes a multi-timer segment onto the internal queue.
    ///
    /// This method enqueues a multi-timer segment, along with flags for stepper
    /// motor channel enablement and direction, into the `SoftTimer` queue. The
    /// `push` operation ensures that the stepper motor operations are managed
    /// in sequence with other queued segments.
    ///
    /// The method returns a future that resolves once the operation is completed.
    /// This allows asynchronous handling of the queueing process.
    ///
    /// # Parameters
    ///
    /// - `multi_timer`: The `MultiTimer` instance representing the segment to be queued.
    /// - `stepper_enable_flags`: Flags indicating which stepper motor channels are enabled.
    /// - `stepper_dir_fwd_flags`: Flags indicating the forward direction for the
    ///   stepper motor channels.
    ///
    /// # Example
    ///
    /// ```
    /// let timer = SoftTimer::new();
    /// // Define a multi-timer segment
    /// let multi_timer = ...;
    /// // Enable X channel and set forward direction for X channel
    /// let stepper_enable_flags = StepperChannel::X;
    /// let stepper_dir_fwd_flags = StepperChannel::X;
    /// let push_future = timer.push(multi_timer, stepper_enable_flags, stepper_dir_fwd_flags);
    /// ```
    ///
    /// # Returns
    ///
    /// A future that resolves to `()` once the segment has been successfully queued.
    pub fn push(
        &self,
        multi_timer: MultiTimer,
        stepper_enable_flags: StepperChannel,
        stepper_dir_fwd_flags: StepperChannel,
    ) -> impl Future<Output = ()> + '_ {
        poll_fn(move |cx| {
            self.poll_push(cx, multi_timer, stepper_enable_flags, stepper_dir_fwd_flags)
        })
    }

    /// Resets the internal state of the `SoftTimer`.
    ///
    /// This method clears the current stepper direction and enable flags, effectively
    /// resetting the timer to its initial state. The operation is enclosed within a
    /// critical section to ensure safe concurrent access and modification of the
    /// internal state.
    ///
    /// # Example
    ///
    /// ```
    /// let timer = SoftTimer::new();
    /// timer.reset();
    /// ```
    fn poll_push(
        &self,
        cx: &mut Context<'_>,
        multi_timer: MultiTimer,
        stepper_enable_flags: StepperChannel,
        stepper_dir_fwd_flags: StepperChannel,
    ) -> Poll<()> {
        critical_section::with(|cs| {
            let mut r = self.0.borrow_ref_mut(cs);
            if r.num_queued as usize >= r.queue.len() {
                r.waker.register(cx.waker());
                Poll::Pending
            } else {
                r.num_queued += 1;
                let current_tail = r.tail;
                hwa::trace!("u-segment queued at {}", current_tail);
                r.queue[current_tail as usize] = Some(StepPlanner::from(
                    multi_timer,
                    stepper_enable_flags,
                    stepper_dir_fwd_flags,
                ));
                r.tail = if (r.tail + 1) as usize == r.queue.len() {
                    0
                } else {
                    r.tail + 1
                };
                Poll::Ready(())
            }
        })
    }

    /// Flushes the internal state of the `SoftTimer`, ensuring all queued operations
    /// are completed before future operations commence.
    ///
    /// This method returns a future that will resolve to a `FlushResult`, which indicates
    /// whether the flush operation was successful. The flush operation is performed
    /// within a critical section to ensure safe concurrent access and modification of the
    /// internal state.
    ///
    /// # Returns
    ///
    /// A future that resolves to a `FlushResult` indicating the outcome of the flush operation.
    ///
    /// # Example
    ///
    /// ```
    /// let timer = SoftTimer::new();
    /// let flush_result = timer.flush().await;
    /// match flush_result {
    ///     FlushResult::Success => println!("Flush successful"),
    ///     FlushResult::Incomplete => println!("Flush incomplete"),
    /// }
    /// ```
    #[allow(unused)]
    pub fn flush(&self) -> impl Future<Output = FlushResult> + '_ {
        poll_fn(move |cx| self.poll_flush(cx))
    }

    ///
    /// Publishes a flush event to ensure all queued operations in the `SoftTimer`
    /// are performed before subsequent operations.
    ///
    /// This method initializes a future that will resolve to a `FlushResult`,
    /// which indicates whether the flush operation was completed successfully.
    /// The process is carried out within a critical section to ensure the internal
    /// state is safely accessed and modified concurrently.
    ///
    /// # Returns
    ///
    /// A future resolving to `FlushResult`, detailing the success or failure
    /// of the flush process.
    ///
    /// # Example
    ///
    /// ```
    /// let timer = SoftTimer::new();
    /// let result = timer.flush().await;
    /// match result {
    ///     FlushResult::Success => println!("Flush operation completed successfully."),
    ///     FlushResult::Incomplete => println!("Flush operation was incomplete."),
    /// }
    /// ```
    pub fn reset(&self) {
        critical_section::with(|cs| {
            let mut r = self.0.borrow_ref_mut(cs);
            r.current_stepper_dir_fwd_flags = StepperChannel::UNSET;
            r.current_stepper_dir_fwd_flags = StepperChannel::UNSET;
        });
    }

    /// Polls the current state of the `SoftTimer` to determine if the flush operation
    /// can be completed.
    ///
    /// This method checks the internal state to see if there are any pending queued
    /// operations. If there are, it registers the context's waker and returns
    /// `Poll::Pending`. If the queue is empty, it evaluates the flush operation result
    /// and returns it.
    ///
    /// The operation is performed within a critical section to ensure thread-safe
    /// access and modification of the internal state.
    ///
    /// # Arguments
    ///
    /// * `cx` - A mutable reference to the task's current context, used to register the waker.
    ///
    /// # Returns
    ///
    /// A `Poll` that resolves to `FlushResult` indicating the outcome of the flush operation.
    ///
    /// - `Poll::Pending`: Indicates that there are still queued operations.
    /// - `Poll::Ready(FlushResult)`: Indicates the result of the flush operation.
    fn poll_flush(&self, cx: &mut Context<'_>) -> Poll<FlushResult> {
        critical_section::with(|cs| {
            let mut r = self.0.borrow_ref_mut(cs);
            if r.num_queued > 0 {
                r.waker.register(cx.waker());
                Poll::Pending
            } else {
                cfg_if::cfg_if! {
                    if #[cfg(any(feature = "assert-motion", test))] {
                        let pulses = crate::math::TVector::from_coords(
                            Some(r.pulses[0]), Some(r.pulses[1]), Some(r.pulses[2]), Some(r.pulses[3])
                        );
                        hwa::trace!("Pulses: {:?}", pulses);
                        r.pulses[0] = 0;
                        r.pulses[1] = 0;
                        r.pulses[2] = 0;
                        r.pulses[3] = 0;
                        Poll::Ready(pulses)
                    }
                    else {
                        Poll::Ready(())
                    }
                }
            }
        })
    }
}

/// A global static instance of `SoftTimer` that is initialized and ready to be used.
///
/// This instance is designed for managing and scheduling timed operations,
/// particularly for controlling stepper motors. By using a static instance,
/// you can ensure that there is a single source of truth for the timer state,
/// reducing the complexity associated with managing multiple instances in
/// concurrent or asynchronous contexts.
///
/// # Main Usage
///
/// The `STEP_DRIVER` is primarily utilized by `task_stepper.rs` for coordinating
/// and handling stepper motor tasks. It ensures thread-safe operations by
/// enclosing critical sections within the defined methods:
///
/// ```rust
/// STEP_DRIVER.reset();
/// let result = STEP_DRIVER.flush().await;
/// match result {
///     FlushResult::Success => println!("Flush successful"),
///     FlushResult::Incomplete => println!("Flush incomplete"),
/// }
/// ```
///
/// By leveraging this static instance, `task_stepper.rs` can streamline timing
/// operations across different parts of your application, ensuring efficient
/// and synchronized stepper motor control.
pub static STEP_DRIVER: SoftTimer = SoftTimer::new();

/// Executes a tick operation for the `STEP_DRIVER` static instance.
///
/// This function is marked as `#[no_mangle]` to ensure its symbol name
/// remains unchanged during the compilation process, making it accessible
/// from external code. It is called by the MCU's SysTick interrupt handler
/// to manage timing operations for the `STEP_DRIVER`.
///
/// The function operates within a critical section to safely access and
/// modify the internal state of the `STEP_DRIVER`, handling stepper motor
/// control signals.
///
/// # Features
///
/// - If the `debug-signals` feature is enabled, this function will also
///   manage the state of the `x_dir_pin` by setting it high at the start
///   and low at the end of the tick operation. This can be useful for debugging
///   stepper motor control signals by toggling an extra pin.
///
/// # Safety
///
/// This function uses critical sections to ensure thread-safe access to the
/// `STEP_DRIVER`'s internal state. Critical sections prevent race conditions
/// and ensure that modifications to shared resources are safely managed.
///
/// # Panics
///
/// This function is expected to handle failure scenarios gracefully, such as
/// being unable to lock the driver instance, and does so by guarding the code
/// with `cfg` blocks and `match` constructs to handle different states.
///
/// # Example
///
/// ```rust
/// extern "Rust" fn do_tick();
/// do_tick();
/// ```
///
#[no_mangle]
pub extern "Rust" fn do_tick() {
    critical_section::with(|cs| {
        let mut counter = STEP_DRIVER.0.borrow_ref_mut(cs);
        #[cfg(feature = "debug-signals")]
        match &counter.drv {
            None => {
                //unreachable!("Driver instance not set")
            }
            Some(_ref) => {
                match _ref.try_lock() {
                    Ok(mut _drv) => {
                        _drv.pins.x_dir_pin.set_high();
                    }
                    Err(_e) => {
                        //unreachable!("Unable to lock")
                    }
                }
            }
        }
        counter.on_interrupt();
        #[cfg(feature = "debug-signals")]
        match &counter.drv {
            None => {}
            Some(_ref) => {
                match _ref.try_lock() {
                    Ok(mut _drv) => {
                        _drv.pins.x_dir_pin.set_low();
                    }
                    Err(_e) => {
                        //unreachable!("Unable to lock")
                    }
                }
            }
        }
    });
}
