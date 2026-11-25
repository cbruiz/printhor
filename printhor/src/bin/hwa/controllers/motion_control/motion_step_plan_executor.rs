//! Motion Step plan execution
use crate::hwa;
use crate::hwa::controllers::{MultiTimer, StepPlan};
use core::cell::RefCell;
use core::future::{Future, poll_fn};
use core::task::{Context, Poll};
use critical_section::Mutex as CsMutex;
use embassy_sync::waitqueue::WakerRegistration;

/// The size of the timer queue.
#[const_env::env_item("TIMER_QUEUE_SIZE")]
const TIMER_QUEUE_SIZE: usize = 4;

/// Represents the state of the soft timer driver.
#[derive(PartialEq)]
pub enum StepPlannerExecutorState {
    Idle,
    Duty,
}

/// The driver for the soft timer, managing timed operations for stepper motors.
pub struct StepPlanExecutor {
    /// The current step planner in use.
    current: StepPlan,

    /// The current state of the driver.
    pub state: StepPlannerExecutorState,

    /// The tick count for the current timer.
    tick_count: u32,

    /// The queue of step planners to be executed.
    queue: [Option<StepPlan>; TIMER_QUEUE_SIZE],

    /// The head index of the queue.
    head: u8,

    /// The tail index of the queue.
    tail: u8,

    /// The number of step planners currently queued.
    num_queued: u8,

    /// Reference to the motion driver.
    actuator: Option<hwa::controllers::StepActuatorController>,

    /// The waker registration for waking up tasks.
    waker: WakerRegistration,

    /// Flags indicating the current enabled stepper channels.
    current_stepper_enable_flags: hwa::CoordSel,

    /// Flags indicating the current forward direction stepper channels.
    current_stepper_dir_fwd_flags: hwa::CoordSel,

    /// The number of pulses generated for each stepper channel (for debugging/testing).
    #[cfg(any(feature = "assert-motion", test))]
    pub pulses: hwa::math::TVector<u32>,

    /// The tool outputs (see [SoftTimer::setup_tools])
    tool_outputs: Option<hwa::controllers::ToolOutputs>,

    /// The state of the tool outputs: every tool action applied so far
    applied_tools: hwa::controllers::ToolActions,

    /// Whether an end-stop of the watched plans triggered (see [SoftTimer::begin_watch])
    watch_triggered: bool,

    /// Steps emitted by the watched plans, per motor (see [SoftTimer::begin_watch])
    watch_steps: hwa::math::TVector<u32>,

    /// The laser power (per mille) last set by the executor (see `apply_laser_power`)
    laser_permille: u16,
}

impl StepPlanExecutor {
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
        // Markers (tool actions only) and dropped plans take no time: the loop goes on with the
        // next plan in the same tick
        loop {
            if self.num_queued == 0 {
                self.state = StepPlannerExecutorState::Idle;
                // The motion stopped: so does the laser in dynamic mode (M4)
                self.apply_laser_power(None, true);
                // Quickly return as there is no work to do
                return false;
            }
            let Some(next) = self.queue[self.head as usize].as_ref() else {
                hwa::error!("try_consume: queue slot at head={} is empty with num_queued={}", self.head, self.num_queued);
                return false;
            };
            // Plans without steps (tool actions, the rest of a move stopped by its end-stop) need
            // no actuator
            let steps_nothing = next.is_marker() || (next.watch.is_some() && self.watch_triggered);
            // Checked before dequeuing: the plan stays queued (and accounted) until the actuator
            // is set up, instead of being dropped while still counted in `num_queued`.
            if !steps_nothing && self.actuator.is_none() {
                hwa::error!("try_consume: actuator not initialized");
                return false;
            }
            let Some(plan) = self.queue[self.head as usize].take() else {
                return false;
            };
            self.head += 1;
            if self.head as usize == self.queue.len() {
                self.head = 0;
            }
            self.num_queued -= 1;

            // The rest of a homing (or probing) move whose end-stop already triggered
            if plan.watch.is_some() && self.watch_triggered {
                hwa::trace!("u-segment dropped (end-stop triggered)");
                continue;
            }
            // Tool actions: applied right after the windows queued before them. Not while the
            // motion is being aborted (M410/M112): the queued ones are discarded with the motion
            if !plan.tools.is_empty() && !crate::tasks::task_stepper::motion_aborted() {
                self.apply_tools(&plan.tools);
            }
            if plan.is_marker() {
                continue;
            }

            self.current = plan;
            self.tick_count = 0;
            let Some(pins) = self.actuator.as_ref() else {
                return false;
            };

            cfg_if::cfg_if! {
                if #[cfg(feature = "with-motion-broadcast")] {
                    // The motion driver moves (homing) are not broadcast, as before
                    if self.current.watch.is_none() {
                        if pins.broadcast_channel.try_send(
                            hwa::MotionBroadcastEvent::Delta(self.current.delta)
                        ).is_err() {
                            hwa::warn!("Unable to broadcast: Overflow");
                        }
                        self.current.ref_time_us += self.current.interval_width
                    }
                }
            }

            cfg_if::cfg_if! {
                if #[cfg(feature = "debug-signals")] {
                    use core::ops::BitAnd;
                    let dir_mask = hwa::CoordSel::all_axis().bitand(
                        (hwa::CoordSel::Y | hwa::CoordSel::X).complement()
                    );
                    pins.set_forward_direction(hwa::CoordSel::Y, hwa::CoordSel::Y);
                }
                else {
                    let dir_mask = hwa::CoordSel::all_axis();
                }
            }

            if self.current_stepper_enable_flags != self.current.stepper_enable_flags {
                pins.enable_steppers(self.current.stepper_enable_flags);
                self.current_stepper_enable_flags = self.current.stepper_enable_flags;
            }
            if self.current_stepper_dir_fwd_flags != self.current.stepper_dir_fwd_flags {
                pins.set_forward_direction(self.current.stepper_dir_fwd_flags, dir_mask);
                self.current_stepper_dir_fwd_flags = self.current.stepper_dir_fwd_flags;
            }
            // The laser power of the window (rapids off, M4 dynamic), with its first step
            self.apply_laser_power(self.current.speed_ratio, false);
            hwa::trace!("u-segment dequeued");
            self.state = StepPlannerExecutorState::Duty;
            return true;
        }
    }

    /// Applies tool actions to the outputs (when set up) and to the tool state
    fn apply_tools(&mut self, actions: &hwa::controllers::ToolActions) {
        self.applied_tools.merge(actions);
        if let Some(outputs) = self.tool_outputs.as_mut() {
            outputs.apply(actions);
        }
        if let Some(cutter) = actions.cutter {
            // As set by `ToolOutputs::apply`: off in dynamic mode (M4) until a window sets it
            self.laser_permille = match cutter {
                hwa::controllers::CutterState::Clockwise(power) => u16::from(power) * 10,
                _ => 0,
            };
        }
        #[cfg(all(test, feature = "native"))]
        hwa::controllers::tool_trace::record(actions);
    }

    /// The laser power along the motion (as GRBL laser mode), when a window starts (`stopped`
    /// false, `speed_ratio` of the window) or the motion stops (`stopped`):
    /// * `M3` (constant power): the programmed power over the cutting moves, off over the
    ///   windows without a programmed speed (rapids, homing). Kept while stopped (a dwell burns).
    /// * `M4` (dynamic power): scaled by the speed ratio of the window (see
    ///   [hwa::controllers::dynamic_power_permille]), off over the rapids and homing, and off
    ///   while stopped.
    /// * Off (`M5`): nothing.
    ///
    /// Only written when it changes
    fn apply_laser_power(&mut self, speed_ratio: Option<u16>, stopped: bool) {
        use hwa::controllers::CutterState;
        let permille = match (self.applied_tools.cutter, stopped) {
            (Some(CutterState::CounterClockwise(_)), true) => 0,
            (Some(CutterState::CounterClockwise(power)), false) => {
                hwa::controllers::dynamic_power_permille(power, speed_ratio)
            }
            (Some(CutterState::Clockwise(power)), false) => match speed_ratio {
                Some(_) => u16::from(power) * 10,
                None => 0,
            },
            _ => return,
        };
        if permille != self.laser_permille {
            self.laser_permille = permille;
            if let Some(outputs) = self.tool_outputs.as_mut() {
                outputs.set_laser_permille(permille);
            }
            #[cfg(all(test, feature = "native"))]
            hwa::controllers::tool_trace::record_laser(permille);
        }
    }

    /// One tick of a watched plan (homing, probing): the end-stops are checked before stepping,
    /// so that no step is emitted once one triggered (latency: one tick). Returns the channels
    /// to step
    #[inline]
    fn watch_tick(&mut self, endstops: hwa::CoordSel, channels: hwa::CoordSel) -> hwa::CoordSel {
        if !endstops.is_empty()
            && self
                .actuator
                .as_ref()
                .is_some_and(|pins| pins.end_stop_triggered(endstops))
        {
            hwa::debug!("ENDSTOP TRIGGERED");
            self.watch_triggered = true;
            self.current.cancel();
            return hwa::CoordSel::empty();
        }
        self.watch_steps.increment(channels, 1);
        channels
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
    /// Returns `true` when the tick found nothing to execute (idle)
    #[inline]
    pub fn on_interrupt(&mut self) -> bool {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

        if self.state == StepPlannerExecutorState::Idle {
            if !self.try_consume() {
                self.notify();
                return true;
            }
        }
        self.tick_count += crate::tasks::task_stepper::STEPPER_PLANNER_CLOCK_PERIOD_US;

        // In this point, state is either Duty or something were dequeued
        match self
            .current
            .next(crate::tasks::task_stepper::STEPPER_PLANNER_CLOCK_PERIOD_US)
        {
            // Not expected (the state is Duty or a plan was just dequeued). Never panics in the
            // interrupt: the tick is just skipped
            None => {}
            Some(_ch) => {
                let _ch = match self.current.watch {
                    None => _ch,
                    Some(endstops) => self.watch_tick(endstops, _ch),
                };
                self.apply(_ch);
            }
        }

        // A window is only closed once all its steps were emitted. When a window carries more
        // steps than ticks (feed rate above one step per tick, or rounding at the max feed rate)
        // it is stretched by the few ticks required instead of silently dropping the remaining
        // steps (which used to be lost without being accounted in the position).
        if self.tick_count >= self.current.interval_width && !self.current.has_pending_steps() {
            //hwa::info!("u-segment completed in {} us", self.tick_count);
            if self.try_consume() {}
            cfg_if::cfg_if! {
                if #[cfg(feature = "debug-signals")] {
                    match self.actuator.as_ref() {
                        None => {}
                        Some(pins) => {
                            pins.set_forward_direction(hwa::CoordSel::empty(), hwa::CoordSel::Y);
                        }
                    }
                }
            }
            self.notify();
        }
        false
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
    fn apply(&mut self, channels: hwa::CoordSel) -> bool {
        if channels.is_empty() {
            true
        } else {
            match self.actuator.as_ref() {
                None => false,
                Some(actuator) => {
                    cfg_if::cfg_if! {
                        if #[cfg(feature = "pulsed")] {
                            compile_error!("feature `pulsed` is not yet implemented");
                            //actuator.step_pin_high(channels);
                            //s_block_for(STEPPER_PULSE_WIDTH_US);
                            //actuator.step_pin_high(channels);
                        }
                        else {
                            actuator.step_toggle(channels);
                        }
                    }
                    cfg_if::cfg_if! {
                        if #[cfg(feature = "assert-motion")] {
                            self.pulses.apply(|c, val| {
                                let v = val.unwrap_or(0);
                                if channels.contains(c.into()) {
                                    Some(v + 1)
                                }
                                else {
                                    Some(v)
                                }
                            });
                            true
                        }
                        else {
                            true
                        }
                    }
                }
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
pub struct SoftTimer(pub CsMutex<RefCell<StepPlanExecutor>>);

cfg_if::cfg_if! {
    if #[cfg(any(feature = "assert-motion", test))] {
        pub type FlushResult = hwa::math::TVector<u32>;
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
        Self(CsMutex::new(RefCell::new(StepPlanExecutor {
            current: StepPlan::new(
                #[cfg(feature = "with-motion-broadcast")]
                hwa::MotionDelta::new(),
            ),
            state: StepPlannerExecutorState::Idle,
            queue: [None; TIMER_QUEUE_SIZE],
            head: 0,
            tick_count: 0,
            waker: WakerRegistration::new(),
            num_queued: 0,
            tail: 0,
            actuator: None,
            current_stepper_enable_flags: hwa::CoordSel::empty(),
            current_stepper_dir_fwd_flags: hwa::CoordSel::empty(),
            #[cfg(any(feature = "assert-motion", test))]
            pulses: hwa::math::TVector::new(),
            tool_outputs: None,
            applied_tools: hwa::controllers::ToolActions::NONE,
            watch_triggered: false,
            watch_steps: hwa::math::TVector::new(),
            laser_permille: 0,
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
    pub fn setup(&self, _pins: hwa::controllers::StepActuatorController) {
        critical_section::with(|cs| {
            let mut r = self.0.borrow_ref_mut(cs);
            r.actuator.replace(_pins);
        })
    }

    /// Sets up the tool outputs the executor switches (see [hwa::controllers::ToolActions])
    pub fn setup_tools(&self, outputs: hwa::controllers::ToolOutputs) {
        critical_section::with(|cs| {
            self.0.borrow_ref_mut(cs).tool_outputs.replace(outputs);
        })
    }

    /// Applies tool actions right away, out of the step queue: when nothing is moving (the
    /// ticker is paused), or for the emergency paths
    pub fn apply_tools_now(&self, actions: &hwa::controllers::ToolActions) {
        if actions.is_empty() {
            return;
        }
        critical_section::with(|cs| {
            self.0.borrow_ref_mut(cs).apply_tools(actions);
        })
    }

    /// The state of the tool outputs: every tool action applied so far
    pub fn applied_tools(&self) -> hwa::controllers::ToolActions {
        critical_section::with(|cs| self.0.borrow_ref(cs).applied_tools)
    }

    /// Queues tool actions behind the plans already queued (see [StepPlan::marker])
    pub fn push_tools(
        &self,
        order_num: u32,
        actions: hwa::controllers::ToolActions,
    ) -> impl Future<Output = ()> + '_ {
        self.push_plan(StepPlan::marker(order_num, actions))
    }

    /// Starts a move of the motion driver (homing, probing): clears the end-stop trigger and the
    /// step counters of the watched plans (see [StepPlan::watched]). The queue must be empty
    pub fn begin_watch(&self) {
        critical_section::with(|cs| {
            let mut r = self.0.borrow_ref_mut(cs);
            r.watch_triggered = false;
            r.watch_steps = hwa::math::TVector::zero();
        })
    }

    /// Whether an end-stop of the watched plans triggered
    pub fn watch_triggered(&self) -> bool {
        critical_section::with(|cs| self.0.borrow_ref(cs).watch_triggered)
    }

    /// Ends a move of the motion driver, once flushed: whether an end-stop triggered, and the
    /// steps emitted per motor
    pub fn end_watch(&self) -> (bool, hwa::math::TVector<u32>) {
        critical_section::with(|cs| {
            let r = self.0.borrow_ref(cs);
            (r.watch_triggered, r.watch_steps)
        })
    }

    /// Pushes a plan onto the internal queue, waiting for a free slot
    pub fn push_plan(&self, plan: StepPlan) -> impl Future<Output = ()> + '_ {
        poll_fn(move |cx| self.poll_push_plan(cx, &plan))
    }

    /// Polls a [`push_plan`](Self::push_plan): queues the plan when there is a free slot, or
    /// registers the waker and returns `Poll::Pending` while the queue is full.
    fn poll_push_plan(&self, cx: &mut Context<'_>, plan: &StepPlan) -> Poll<()> {
        critical_section::with(|cs| {
            let mut r = self.0.borrow_ref_mut(cs);
            if r.num_queued as usize >= r.queue.len() {
                r.waker.register(cx.waker());
                Poll::Pending
            } else {
                r.num_queued += 1;
                let current_tail = r.tail;
                hwa::trace!("u-segment queued at {}", current_tail);
                r.queue[current_tail as usize] = Some(*plan);
                r.tail = if (r.tail + 1) as usize == r.queue.len() {
                    0
                } else {
                    r.tail + 1
                };
                Poll::Ready(())
            }
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
    /// - `speed_ratio`: The speed of the window relative to the programmed one, per mille (laser
    ///   dynamic power, see [StepPlan::speed_ratio]).
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
        _order_num: u32,
        #[cfg(feature = "with-motion-broadcast")] delta: hwa::MotionDelta,
        multi_timer: MultiTimer,
        stepper_enable_flags: hwa::CoordSel,
        stepper_dir_fwd_flags: hwa::CoordSel,
        speed_ratio: Option<u16>,
    ) -> impl Future<Output = ()> + '_ {
        // Built once, not on every poll while the queue is full
        let mut plan = StepPlan::from(
            _order_num,
            #[cfg(feature = "with-motion-broadcast")]
            delta,
            multi_timer,
            stepper_enable_flags,
            stepper_dir_fwd_flags,
        );
        plan.speed_ratio = speed_ratio;
        self.push_plan(plan)
    }

    /// Polls a [`push`](Self::push): queues the plan when there is a free slot, or registers
    /// the waker and returns `Poll::Pending` while the queue is full.
    #[cfg(test)]
    fn poll_push(
        &self,
        cx: &mut Context<'_>,
        _order_num: u32,
        #[cfg(feature = "with-motion-broadcast")] delta: hwa::MotionDelta,
        multi_timer: MultiTimer,
        stepper_enable_flags: hwa::CoordSel,
        stepper_dir_fwd_flags: hwa::CoordSel,
    ) -> Poll<()> {
        self.poll_push_plan(
            cx,
            &StepPlan::from(
                _order_num,
                #[cfg(feature = "with-motion-broadcast")]
                delta,
                multi_timer,
                stepper_enable_flags,
                stepper_dir_fwd_flags,
            ),
        )
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

    /// Forgets the stepper enable and direction state applied so far, so that the next plan
    /// applies both again.
    ///
    /// It must be called whenever the drivers are enabled, disabled or redirected outside the
    /// executor (park, `M84`, emergency stop...). Otherwise a plan with the same flags as the
    /// previous one would skip enabling its drivers and step disabled ones.
    ///
    /// # Example
    ///
    /// ```
    /// let timer = SoftTimer::new();
    /// timer.reset();
    /// ```
    pub fn reset(&self) {
        critical_section::with(|cs| {
            let mut r = self.0.borrow_ref_mut(cs);
            r.current_stepper_enable_flags = hwa::CoordSel::UNSET;
            r.current_stepper_dir_fwd_flags = hwa::CoordSel::UNSET;
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
            if r.num_queued > 0 || r.state != StepPlannerExecutorState::Idle {
                r.waker.register(cx.waker());
                Poll::Pending
            } else {
                cfg_if::cfg_if! {
                    if #[cfg(any(feature = "assert-motion", test))] {
                        hwa::trace!("Pulses: {:?}", r.pulses);
                        let pulses = r.pulses;
                        r.pulses.set_coord(hwa::CoordSel::all_axis(), Some(0));
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

/// Whether the step ticker runs (see [resume_ticker] and [pause_ticker])
static TICKER_RUNNING: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// Resumes the step ticker (the ISR feeding [STEP_DRIVER])
pub fn resume_ticker() {
    TICKER_RUNNING.store(true, core::sync::atomic::Ordering::Release);
    <hwa::Contract as hwa::HwiContract>::resume_ticker();
}

/// Pauses the step ticker. Only once [STEP_DRIVER] is flushed: the queued plans would wait
pub fn pause_ticker() {
    TICKER_RUNNING.store(false, core::sync::atomic::Ordering::Release);
    <hwa::Contract as hwa::HwiContract>::pause_ticker();
}

/// Whether the step ticker runs, as left by [resume_ticker] and [pause_ticker]
pub fn ticker_running() -> bool {
    TICKER_RUNNING.load(core::sync::atomic::Ordering::Acquire)
}

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
/// - If the `debug-signals` feature is enabled, this function will **change
///   the purpose** of X and Y direction pins to measure stepping timings latency.
///   Convention is:
///     * X_DIR is HIGH at ISR START.
///     * X_DIR is LOW at ISR COMPLETION.
///     * Y_DIR is HIGH at ISR activity START
///     * Y_DIR is LOW at ISR activity COMPLETION
///   Expectation is that:
///     * X_DIR shall always be constant at [hwa::HwiContract::STEP_PLANNER_CLOCK_FREQUENCY] Hz with no delay
///     * When there is step activity:
///       * Y_DIR shall be HIGH with no relevant delay with previous X_DIR HIGH.
///       * Y_DIR shall become LOW the sooner, the better.
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
#[unsafe(no_mangle)]
pub extern "Rust" fn do_tick() {
    let _t0 = super::motion_metrics::now_us();
    critical_section::with(|cs| {
        let mut step_driver = STEP_DRIVER.0.borrow_ref_mut(cs);
        cfg_if::cfg_if! {
            if #[cfg(feature = "debug-signals")] {
                match &step_driver.actuator {
                    None => {}
                    Some(pins) => {
                        pins.set_forward_direction(hwa::CoordSel::X, hwa::CoordSel::X)
                    }
                }
            }
        }
        let _idle = step_driver.on_interrupt();
        super::motion_metrics::isr_tick(cs, _t0, _idle);
        cfg_if::cfg_if! {
            if #[cfg(feature = "debug-signals")] {
                match &step_driver.actuator {
                    None => {}
                    Some(pins) => {
                        pins.set_forward_direction(hwa::CoordSel::empty(), hwa::CoordSel::X)
                    }
                }
            }
        }
    });
}

#[cfg(test)]
mod executor_tests;
