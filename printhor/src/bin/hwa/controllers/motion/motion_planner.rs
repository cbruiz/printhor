use crate::{control, hwa};
use hwa::controllers::ExecPlan::Homing;
use hwa::math;
use hwa::Contract;
use hwa::HwiContract;

use embassy_sync::mutex::MutexGuard;
use hwa::controllers::motion::motion_ring_buffer::RingBuffer;
use hwa::controllers::{motion, MovType, PlanEntry, ScheduledMove};
use hwa::{EventFlags, EventStatus, PersistentState};
use math::Real;

use math::TVector;

/// The execution plan action dequeued from the buffer
pub enum ExecPlan {
    Segment(motion::Segment, hwa::CommChannel, u32),
    Dwell(Option<u32>, hwa::CommChannel, u32),
    Homing(hwa::CommChannel, u32),
}

/// The `MotionPlanner` struct is responsible for handling the motion planning logic
/// within the system. It encompasses the mechanisms for buffering motion plans,
/// signaling motion states, and interacting with the associated motion driver.
///
/// # Fields
///
/// * `defer_channel` - A reference to the channel used for sending deferred events.
/// * `ring_buffer` - A `Mutex` guarding the ring buffer that stores motion plans.
/// * `move_planned` - Configuration state indicating whether a move is planned.
/// * `available` - Configuration state indicating availability for new motion plans.
/// * `motion_config` - Reference to the configuration settings for the motion control.
/// * `motion_st` - A `Mutex` guarding the motion status data.
/// * `motion_driver` - Reference to the driver responsible for executing motion commands.
pub struct MotionPlanner {
    //pub event_bus: EventBusRef,
    // The channel to send deferred events
    pub defer_channel: hwa::types::DeferChannel,

    ring_buffer: hwa::types::MotionRingBuffer,
    move_planned: &'static PersistentState<hwa::types::MotionSignalMutexType, bool>,
    available: &'static PersistentState<hwa::types::MotionSignalMutexType, bool>,
    motion_config: hwa::controllers::MotionConfig,
    motion_status: hwa::controllers::MotionStatus,
    motion_driver: hwa::types::MotionDriver,
}

// TODO: Refactor in progress
#[allow(unused)]
impl MotionPlanner {
    /// This constructor sets up the `MotionPlanner` with provided references to the
    /// necessary components such as the `defer_channel`, `motion_config`, and `motion_driver`.
    /// It also initializes internal fields such as the motion plan ring buffer, motion status,
    /// and various configuration states.
    ///
    /// # Warning
    /// A motion planner can only be instantiated once
    ///
    /// # Arguments
    ///
    /// * `defer_channel` - A reference to the channel for sending deferred events.
    /// * `motion_config` - Reference to the configuration settings for motion control.
    /// * `motion_driver` - Reference to the driver responsible for executing motion commands.
    ///
    /// # Returns
    ///
    /// A new `MotionPlanner` instance with the specified configuration.
    ///
    /// # Explanation
    ///
    /// The `new` method is essential for creating a well-initialized `MotionPlanner` object that
    /// can handle motion planning tasks effectively. By setting up the `defer_channel`, `motion_config`,
    /// and `motion_driver`, the `MotionPlanner` is equipped with the necessary tools to manage motion commands
    /// and states. The initialization of the ring buffer and motion status ensures that the planner starts
    /// with a clean slate and is ready to process new motion plans.
    ///
    /// # Examples
    ///
    /// ```
    /// use crate::hwa::{DeferChannelRef, MotionConfigRef, MotionDriverRef, EventBusRef};
    /// use crate::MotionPlanner;
    ///
    /// // Assume defer_channel, motion_config, motion_driver, and event_bus are already defined
    /// let defer_channel: DeferChannelRef = /* initialize defer channel */;
    /// let motion_config: MotionConfigRef = /* initialize motion config */;
    /// let motion_driver: MotionDriverRef = /* initialize motion driver */;
    /// let event_bus: EventBusRef = /* initialize event bus */;
    ///
    /// let motion_planner = MotionPlanner::new(defer_channel, motion_config, motion_driver);
    ///
    /// // To make the motion_planner ready, call the `start` method
    /// async {
    ///     motion_planner.start(&event_bus).await;
    /// };
    ///
    /// // Now the motion_planner is ready to be used
    /// ```
    pub fn new(
        defer_channel: hwa::types::DeferChannel,
        motion_config: hwa::controllers::MotionConfig,
        motion_status: hwa::controllers::MotionStatus,
        motion_driver: hwa::types::MotionDriver,
    ) -> Self {
        type PersistentStateType<M> = hwa::PersistentState<M, bool>;

        Self {
            defer_channel,
            motion_config,
            ring_buffer: hwa::make_static_async_controller!(
                "MotionRingBuffer",
                hwa::types::MotionRingBufferMutexStrategy,
                RingBuffer::new()
            ),
            move_planned: hwa::make_static_ref!(
                "MovePlannedSignal",
                PersistentStateType<hwa::types::MotionSignalMutexType>,
                PersistentState::new()
            ),
            available: hwa::make_static_ref!(
                "MoveAvailableSignal",
                PersistentStateType<hwa::types::MotionSignalMutexType>,
                PersistentState::new()
            ),
            motion_status,
            motion_driver,
        }
    }

    /// Starts the motion planner and publishes an event indicating that the motion queue is empty.
    ///
    /// This method resets the `move_planned` configuration, signals the `available` configuration,
    /// and publishes an event to the provided `event_bus` indicating that the movement queue is empty.
    ///
    /// # Arguments
    ///
    /// * `event_bus` - A reference to the event bus for publishing events.
    ///
    /// # Explanation
    ///
    /// This method is essential for initializing the motion planner's state and
    /// ensuring it starts from a clean slate. By resetting the `move_planned` configuration,
    /// the method ensures that any previous move plans are discarded,
    /// preventing unintended motions. The `available` configuration is set to `true` to
    /// signal that the motion planner is ready to accept new motion plans.
    ///
    /// Publishing an event indicating that the motion queue is empty is crucial for
    /// other components of the system to recognize that the motion planner is in an idle state
    /// and ready for new instructions. This helps in maintaining synchronization across
    /// different parts of the system and ensuring smooth operation.
    ///
    /// It is important to note that the motion planner will remain idle and not execute any movements until this `start` method is called.
    /// This ensures that no motion-related activities commence before the system is fully set up and ready, preventing any accidental or unplanned actions.
    pub async fn start(&self, event_bus: &hwa::types::EventBus) {
        self.move_planned.reset();
        self.available.signal(true);
        event_bus
            .publish_event(
                hwa::EventStatus::containing(EventFlags::MOV_QUEUE_EMPTY)
                    .and_not_containing(EventFlags::MOV_QUEUE_FULL),
            )
            .await;
    }

    // Dequeues actual velocity plan

    /// Dequeues the current velocity plan segment.
    ///
    /// This method is used to retrieve the current segment of the motion plan and its associated communication channel.
    /// It continuously checks the ring buffer for an available plan entry and processes it accordingly.
    /// If a `PlannedMove` is found, it returns the planned data and the communication channel.
    ///
    /// # Arguments
    ///
    /// * `event_bus` - A reference to the event bus for publishing events.
    ///
    /// # Returns
    ///
    /// An `Option` containing a tuple with the segment data and the communication channel.
    /// If no segment data is available, it returns `None`.
    ///
    /// # Explanation
    ///
    /// This method plays a crucial role in the motion planning process as it dequeues the current segment of
    /// the velocity plan, which is essential for executing motion commands in a sequential manner.
    /// It ensures that the motion planner continually processes and executes planned movements, dwell commands,
    /// and homing commands by checking the ring buffer and handling different types of `PlanEntry`.
    /// If a `PlannedMove` entry is found, it marks it as `Executing` and returns the planned data and communication channel.
    ///
    /// In the case of a `Homing` entry, it publishes an event indicating that a homing operation is in progress.
    /// For dwell commands, it sets a flag to indicate that dwelling needs to be performed.
    ///
    /// The method ensures that the motion planner's state is correctly updated and synchronized with the event
    /// bus, which is important for maintaining the overall system's consistency and reliability.
    pub async fn next_plan(&self, event_bus: &hwa::types::EventBus) -> ExecPlan {
        loop {
            let _ = self.move_planned.wait().await;
            {
                let mut rb = self.ring_buffer.lock().await;
                let head = rb.head as usize;
                match rb.data[head] {
                    PlanEntry::Empty => {
                        self.move_planned.reset();
                    }
                    PlanEntry::Dwell(channel, sleep, _deferred, order_num) => {
                        rb.data[head] =
                            PlanEntry::Executing(MovType::Dwell(channel), true, order_num);
                        return ExecPlan::Dwell(sleep, channel, order_num);
                    }
                    PlanEntry::PlannedMove(planned_data, action, channel, deferred, order_num) => {
                        hwa::debug!(
                            "PlannedMove starting: {} / {} h={}",
                            rb.used,
                            Contract::SEGMENT_QUEUE_SIZE,
                            head
                        );
                        rb.data[head] = PlanEntry::Executing(
                            MovType::Move(action, channel),
                            deferred,
                            order_num,
                        );
                        return ExecPlan::Segment(planned_data, channel, order_num);
                    }
                    PlanEntry::Homing(channel, _deferred, order_num) => {
                        event_bus
                            .publish_event(hwa::EventStatus::containing(hwa::EventFlags::HOMING))
                            .await;
                        rb.data[head] =
                            PlanEntry::Executing(MovType::Homing(channel), true, order_num);
                        return Homing(channel, order_num);
                    }
                    PlanEntry::Executing(_, _, _) => {
                        self.move_planned.reset();
                        hwa::error!("Unexpected error: RingBuffer Overrun");
                    }
                }
            }
        }
    }

    pub async fn num_queued(&self) -> u8 {
        let mut rb = self.ring_buffer.lock().await;
        rb.used
    }

    /// Consumes the current segment data from the ring buffer.
    ///
    /// # Description
    ///
    /// This method is responsible for processing the current segment data that has been executed in the motion planner.
    /// It retrieves the current entry from the ring buffer and performs the necessary actions based on the type of movement.
    /// The method ensures the proper handling of homing, dwell, and move actions by publishing appropriate events
    /// and updating the system's state.
    ///
    /// # Arguments
    ///
    /// * `event_bus` - A reference to the event bus for publishing events.
    ///
    /// # Returns
    ///
    /// A `u8` value indicating the remaining used slots in the ring buffer after consuming the current segment data.
    ///
    /// # Usage
    ///
    /// This method is essential for the motion planning system to maintain synchronization between the executed movements
    /// and the event bus. It ensures that events such as homing completion and dwell completion are correctly published,
    /// and it also handles the deferment of actions if necessary.
    ///
    /// This function is typically called after the execution of a movement has been completed to update the state
    /// of the ring buffer and signal the availability of slots for new commands. It is an integral part of the system
    /// to ensure that the motion planner operates smoothly and consistently by managing the state of planned and executed movements.
    pub async fn consume_current_plan(&self, event_bus: &hwa::types::EventBus) -> u8 {
        let mut rb = self.ring_buffer.lock().await;
        let head = rb.head;
        hwa::debug!("Movement completed @rq[{}] (ongoing={})", head, rb.used - 1);
        match &rb.data[head as usize] {
            PlanEntry::Executing(MovType::Homing(channel), _, order_num) => {
                event_bus
                    .publish_event(hwa::EventStatus::not_containing(hwa::EventFlags::HOMING))
                    .await;
                self.defer_channel
                    .send(hwa::DeferEvent::Completed(
                        hwa::DeferAction::Homing,
                        *channel,
                        *order_num,
                    ))
                    .await;
            }
            PlanEntry::Executing(MovType::Dwell(channel), _, order_num) => {
                self.defer_channel
                    .send(hwa::DeferEvent::Completed(
                        hwa::DeferAction::Dwell,
                        *channel,
                        *order_num,
                    ))
                    .await;
            }
            PlanEntry::Executing(MovType::Move(action, channel), deferred, order_num) => {
                if *deferred {
                    self.defer_channel
                        .send(hwa::DeferEvent::Completed(*action, *channel, *order_num))
                        .await;
                }
            }
            _ => {
                panic!("cound not happen")
            }
        }
        rb.data[head as usize] = PlanEntry::Empty;
        rb.head = match head + 1 < Contract::SEGMENT_QUEUE_SIZE.into() {
            true => head + 1,
            false => 0u8,
        };
        rb.used -= 1;
        hwa::debug!("- used={}, h={} ", rb.used, head);
        let mut queue_status_event = EventStatus::not_containing(EventFlags::MOV_QUEUE_FULL);
        event_bus
            .publish_event(match rb.used == 0 {
                true => queue_status_event.and_containing(EventFlags::MOV_QUEUE_EMPTY),
                false => queue_status_event,
            })
            .await;
        self.available.signal(true);
        rb.used
    }

    /// Schedules a raw movement operation for the motion planner.
    ///
    /// # Description
    ///
    /// This method is responsible for scheduling a raw movement action. It processes the given movement type and enqueues it in the
    /// motion planner's ring buffer. This ensures the motion planner operates sequentially, managing the movement commands
    /// based on their arrival and execution status.
    ///
    /// Scheduling a raw move involves:
    /// - Checking availability in the ring buffer to enqueue new commands.
    /// - Locking the ring buffer to ensure thread safety during enqueue operations.
    /// - Possibly deferring the action if the ring buffer is full.
    /// - Publishing events to notify other subsystems about the execution status and results.
    ///
    /// # Usage
    ///
    /// This method is commonly used in scenarios where precise control over the motion actions is required. The operations are:
    /// - Scheduling homing actions for system calibration.
    /// - Adding delay (dwell) operations between movements.
    /// - Enqueuing move actions to control hardware actuators like motors and servos.
    ///
    /// The method is executed asynchronously to ensure non-blocking operations, allowing the motion planner to handle other tasks
    /// concurrently. After enqueuing the new command, it signals availability in the ring buffer, coordinating subsequent operations
    /// and maintaining smooth motion control.
    ///
    /// # Arguments
    ///
    /// * `channel` - The communication channel through which the motion planner receives commands.
    /// * `action` - The deferred action (if any) that specifies the type of move.
    /// * `move_type` - The type of movement to be scheduled (e.g., raw move, homing, dwell).
    /// * `blocking` - A boolean indicating whether the operation should block until completion.
    /// * `event_bus` - A reference to the event bus for publishing events.
    ///
    /// # Returns
    ///
    /// A `Result` indicating the success or failure of the code execution. On success, it provides `CodeExecutionSuccess`; on failure,
    /// it returns `CodeExecutionFailure`.
    ///
    /// # Example
    ///
    /// ```rust
    /// let result = planner.schedule_raw_move(channel, action, move_type, blocking, event_bus).await;
    /// match result {
    ///     Ok(success) => println!("Move scheduled successfully: {:?}", success),
    ///     Err(failure) => eprintln!("Failed to schedule move: {:?}", failure),
    /// }
    /// ```
    ///
    async fn schedule_raw_move(
        &self,
        _mnemonic: &'static str,
        channel: hwa::CommChannel,
        action: hwa::DeferAction,
        move_type: ScheduledMove,
        blocking: bool,
        event_bus: &hwa::types::EventBus,
        num_order: u32,
        line_tag: Option<u32>,
    ) -> Result<control::CodeExecutionSuccess, control::CodeExecutionFailure> {
        hwa::debug!("schedule_raw_move() BEGIN");
        loop {
            self.available.wait().await;
            {
                let mut rb = self.ring_buffer.lock().await;
                let mut is_defer = rb.used == Contract::SEGMENT_QUEUE_SIZE - 1;

                if rb.used < Contract::SEGMENT_QUEUE_SIZE {
                    #[cfg(feature = "cornering")]
                    let mut do_adjust = false;

                    let curr_insert_index = rb.index_from_tail(0).unwrap();

                    let (curr_planned_entry, event) = match move_type {
                        ScheduledMove::Move(curr_segment_in, order_num) => {
                            let mut curr_segment = curr_segment_in;
                            // TODO:
                            // 1) Find a better way to approximate
                            // 2) Move outside ringbuffer lock
                            // solving sqrt(((abs(x-v_0))/jmax)) > q_1  for x
                            // x < v_0 - jmax q_1^2
                            // x > jmax * q_1^2 + v_0

                            let mut curr_vmax = curr_segment.speed_target_su_s;
                            #[cfg(feature = "cornering")]
                            {
                                let q_1 = curr_segment.displacement_su;
                                let v_0 = curr_segment.speed_enter_su_s;
                                let t_jmax =
                                    curr_segment.constraints.a_max / curr_segment.constraints.j_max;

                                loop {
                                    let t_jstar = Real::vmin(
                                        (((curr_vmax - v_0).abs())
                                            / curr_segment.constraints.j_max)
                                            .sqrt(),
                                        Some(t_jmax),
                                    )
                                    .unwrap_or(math::ZERO);

                                    let q_lim = if t_jstar < t_jmax {
                                        t_jstar * (v_0 + curr_vmax)
                                    } else {
                                        ((v_0 + curr_vmax) / math::TWO)
                                            * (t_jstar
                                                + ((curr_vmax - v_0).abs()
                                                    / curr_segment.constraints.a_max))
                                    };

                                    if q_1 >= q_lim {
                                        break;
                                    } else {
                                        curr_vmax *= Real::from_f32(0.5);
                                    }
                                }
                            }

                            let curr_boundary = curr_vmax.min(curr_segment.constraints.v_max);
                            let v_marginal_gain = curr_boundary - curr_segment.speed_exit_su_s;
                            hwa::debug!(
                                "\t- constraint = {:?} bound = {:?}",
                                curr_vmax,
                                curr_boundary
                            );
                            hwa::debug!("\t= v_margin: {:?}", v_marginal_gain);
                            curr_segment.speed_enter_constrained_su_s = v_marginal_gain;
                            curr_segment.speed_exit_constrained_su_s = v_marginal_gain;

                            hwa::debug!(
                                "s: vi = [{:?} < {:?}] - vtarget = {:?} - vo = [{:?} < {:?}] / d = {:?}",
                                curr_segment.speed_enter_su_s,
                                curr_segment.speed_enter_constrained_su_s,
                                curr_segment.speed_target_su_s,
                                curr_segment.speed_exit_constrained_su_s,
                                curr_segment.speed_exit_su_s,
                                curr_segment.displacement_su
                            );

                            hwa::debug!("\t= max_gain: {:?}", v_marginal_gain);

                            if let Ok(prev_index) = rb.index_from_tail(1) {
                                match &mut rb.data[prev_index as usize] {
                                    PlanEntry::PlannedMove(prev_segment, _, _, _, _) => {
                                        let proj: Real = prev_segment
                                            .unit_vector_dir
                                            .orthogonal_projection(curr_segment.unit_vector_dir);
                                        if proj.is_defined_positive() {
                                            hwa::debug!("RingBuffer [{:?}, {:?}] chained: ({:?}) proj ({:?}) = ({:?})", prev_index, curr_insert_index,
                                                prev_segment.unit_vector_dir, curr_segment.unit_vector_dir, proj
                                            );
                                            hwa::debug!("\ts : vi = [{:?} < {:?}] - vtarget = {:?} - vo = [{:?} < {:?}]:",
                                                prev_segment.speed_enter_su_s, prev_segment.speed_enter_constrained_su_s,
                                                prev_segment.speed_target_su_s,
                                                prev_segment.speed_exit_su_s, prev_segment.speed_exit_constrained_su_s,
                                            );
                                            hwa::debug!("\t\tproj = {:?}", proj);
                                            cfg_if::cfg_if! {
                                                if #[cfg(feature = "cornering")] {
                                                    do_adjust = true;
                                                }
                                            }
                                            prev_segment.proj_next = proj;
                                            curr_segment.proj_prev = proj;
                                        }
                                    }
                                    _ => {}
                                }
                            }
                            #[cfg(feature = "debug-motion")]
                            hwa::info!(
                                "[MotionPlanner] order_num:{:?} L:{:?} MotionPlan queued. src: [{:?}], dest: [{:?}], vmax: [ {:?}, [{:?}] ]",
                                order_num, line_tag.unwrap_or(0),
                                curr_segment.src_pos, curr_segment.dest_pos,
                                curr_segment.speed_target_su_s,
                                curr_segment.unit_vector_dir.abs() * curr_segment.speed_target_su_s
                            );
                            {
                                let pos = hwa::controllers::Position::new_from(
                                    curr_segment.dest_world_pos,
                                    curr_segment.dest_pos,
                                );
                                self.motion_status
                                    .update_last_planned_position(order_num, &pos);
                            }
                            (
                                PlanEntry::PlannedMove(
                                    curr_segment,
                                    action,
                                    channel,
                                    is_defer,
                                    order_num,
                                ),
                                hwa::EventStatus::containing(EventFlags::NOTHING),
                            )
                        }
                        ScheduledMove::Homing(order_num) => {
                            is_defer = true;
                            (
                                PlanEntry::Homing(channel, is_defer, order_num),
                                hwa::EventStatus::not_containing(hwa::EventFlags::HOMING),
                            )
                        }
                        ScheduledMove::Dwell(sleep, order_num) => {
                            is_defer = true;
                            (
                                PlanEntry::Dwell(channel, sleep, is_defer, order_num),
                                hwa::EventStatus::containing(hwa::EventFlags::MOV_QUEUE_EMPTY),
                            )
                        }
                    };

                    hwa::debug!(
                        "Mov queued @{} ({} / {})",
                        curr_insert_index,
                        rb.used + 1,
                        Contract::SEGMENT_QUEUE_SIZE
                    );

                    rb.data[curr_insert_index as usize] = curr_planned_entry;
                    rb.used += 1;
                    #[cfg(feature = "cornering")]
                    if do_adjust {
                        let _ = perform_cornering(rb);
                    }

                    event_bus
                        .publish_event(hwa::EventStatus::not_containing(
                            hwa::EventFlags::MOV_QUEUE_EMPTY,
                        ))
                        .await;
                    self.move_planned.signal(true);
                    return if is_defer {
                        // Shall wait for one de-allocation in order to enqueue more
                        hwa::debug!("schedule_raw_move() - Finally deferred");
                        #[cfg(feature = "trace-commands")]
                        hwa::info!(
                            "[trace-commands] Promised {} #order: {} line: {:?}",
                            _mnemonic,
                            num_order,
                            line_tag
                        );
                        self.defer_channel
                            .send(hwa::DeferEvent::AwaitRequested(action, channel, num_order))
                            .await;
                        Ok(control::CodeExecutionSuccess::DEFERRED(event))
                    } else {
                        #[cfg(feature = "trace-commands")]
                        hwa::info!(
                            "[trace-commands] Commited #order: {} #line: {:?}",
                            num_order,
                            line_tag
                        );
                        hwa::debug!("schedule_raw_move() END - Finally queued");
                        Ok(control::CodeExecutionSuccess::QUEUED)
                    };
                } else {
                    self.available.reset();
                    event_bus
                        .publish_event(EventStatus::containing(EventFlags::MOV_QUEUE_FULL))
                        .await;
                    if !blocking {
                        hwa::warn!(
                            "Mov rejected: {} / {} h={}",
                            rb.used,
                            Contract::SEGMENT_QUEUE_SIZE,
                            rb.head
                        );
                        return Err(control::CodeExecutionFailure::BUSY);
                    } else {
                        hwa::trace!("schedule_raw_move() Looping again");
                    }
                }
            }
        }
    }

    pub fn motion_config(&self) -> &hwa::controllers::MotionConfig {
        &self.motion_config
    }

    pub fn motion_status(&self) -> &hwa::controllers::MotionStatus {
        &self.motion_status
    }

    pub fn motion_driver(&self) -> &hwa::types::MotionDriver {
        &self.motion_driver
    }

    /// Plans and schedules a series of motion commands based on the given GCode.
    ///
    /// Depending on the provided GCode, this method performs different motion planning
    /// and scheduling actions. It supports G0 (rapid move), G1 (linear move), G4 (dwell),
    /// G28 (homing), G29 (leveling), G29_1, and G29_2 commands. Any unsupported GCodes
    /// will yield an error.
    ///
    /// # Parameters
    /// - `channel`: The communication channel to be used for issuing commands.
    /// - `gc`: The GCode command that needs to be processed.
    /// - `blocking`: A boolean indicating if the command should block until completion.
    /// - `event_bus`: A reference to the event bus for publishing events.
    ///
    /// # Returns
    /// A `Result` which is:
    /// - `Ok` if the command was successfully planned and scheduled.
    /// - `Err` if the command was not yet implemented or other execution failures occurred.
    ///
    /// # Usage
    /// This method is integral to motion control systems where precise planning and
    /// execution of motion commands are required. It ensures that various types of
    /// motion commands from GCode are handled correctly and appropriately scheduled
    /// to execute in an asynchronous manner. This helps in maintaining smooth and
    /// coordinated movements for CNC machines or 3D printers.
    pub async fn plan(
        &self,
        channel: hwa::CommChannel,
        gc: &control::GCodeCmd,
        blocking: bool,
        event_bus: &hwa::types::EventBus,
    ) -> Result<control::CodeExecutionSuccess, control::CodeExecutionFailure> {
        match &gc.value {
            control::GCodeValue::G0(t) => Ok(self
                .schedule_move(
                    "G0",
                    channel,
                    hwa::DeferAction::RapidMove,
                    t.as_vector(),
                    t.f,
                    blocking,
                    event_bus,
                    gc.order_num,
                    gc.line_tag,
                )
                .await?),
            control::GCodeValue::G1(t) => Ok(self
                .schedule_move(
                    "G1",
                    channel,
                    hwa::DeferAction::LinearMove,
                    t.as_vector(),
                    t.f,
                    blocking,
                    event_bus,
                    gc.order_num,
                    gc.line_tag,
                )
                .await?),
            control::GCodeValue::G4(t) => Ok(self
                .schedule_raw_move(
                    "G4",
                    channel,
                    hwa::DeferAction::Dwell,
                    ScheduledMove::Dwell(t.s.map(|v| v.int() as u32), gc.order_num),
                    blocking,
                    event_bus,
                    gc.order_num,
                    gc.line_tag,
                )
                .await?),
            #[cfg(feature = "with-motion")]
            control::GCodeValue::G28(_x) => {
                event_bus
                    .publish_event(hwa::EventStatus::containing(hwa::EventFlags::HOMING))
                    .await;
                Ok(self
                    .schedule_raw_move(
                        "G28",
                        channel,
                        hwa::DeferAction::Homing,
                        ScheduledMove::Homing(gc.order_num),
                        blocking,
                        event_bus,
                        gc.order_num,
                        gc.line_tag,
                    )
                    .await?)
            }
            control::GCodeValue::G29 => Ok(control::CodeExecutionSuccess::OK),
            control::GCodeValue::G29_1 => Ok(control::CodeExecutionSuccess::OK),
            control::GCodeValue::G29_2 => Ok(control::CodeExecutionSuccess::OK),
            _ => Err(control::CodeExecutionFailure::NotYetImplemented),
        }
    }

    async fn schedule_move(
        &self,
        mnemonic: &'static str,
        channel: hwa::CommChannel,
        action: hwa::DeferAction,
        p1_wu: TVector<Real>,
        requested_motion_speed: Option<Real>,
        blocking: bool,
        event_bus: &hwa::types::EventBus,
        num: u32,
        line: Option<u32>,
    ) -> Result<control::CodeExecutionSuccess, control::CodeExecutionFailure> {
        let p0_pos = self.motion_status.get_current_position();
        let relevant_world_coords = p1_wu.not_nan_coords();

        if !p0_pos.is_set {
            return Err(control::CodeExecutionFailure::HomingRequired);
        }

        #[cfg(feature = "debug-motion")]
        hwa::info!(
            "[MotionPlanner] order_num:{:?} L:{:?} Received motion. world: src [{:?}] {} dest: [{:?}] {}, absolute: {}, speed: {:?} {}/s",
            num, line.unwrap_or(0),
            p0_pos.world_pos.selecting(relevant_world_coords), Contract::WORLD_UNIT_MAGNITUDE,
            p1_wu, Contract::WORLD_UNIT_MAGNITUDE,
            self.motion_status.is_absolute_positioning(), requested_motion_speed,
            Contract::WORLD_UNIT_MAGNITUDE
        );

        let p1_pos = {
            if self.motion_status.is_absolute_positioning() {
                p0_pos.complete_and_project(&p1_wu)
            } else {
                p0_pos.complete_and_project(&(p0_pos.world_pos + p1_wu))
            }
        };

        //----
        let dts = self.motion_config.get_default_travel_speed();
        let flow_rate = self.motion_config.get_flow_rate_as_real();
        let speed_rate = self.motion_config.get_speed_rate_as_real();
        let max_speed = self.motion_config.get_max_speed();
        let max_accel = self.motion_config.get_max_accel();
        let max_jerk = self.motion_config.get_max_jerk();
        //----

        // Compute distance and decompose as unit vector and module.
        // When dist is zero, value is map to None (NaN).
        // In case o E dimension, flow rate factor is applied
        let ds = p1_pos.space_pos - p0_pos.space_pos;
        let relevant_space_coords = ds.not_negligible_coords();

        hwa::debug!(
            "[MotionPlanner] order_num:{:?} L:{:?} Relevant space coords: [{:?}]",
            num,
            line.unwrap_or(0),
            relevant_space_coords
        );

        #[cfg(feature = "debug-motion")]
        hwa::info!(
            "[MotionPlanner] order_num:{:?} L:{:?} Transformed to space [ src: [{:?}] dest: [{:?}] ] {}, speed: {:?} {}/s",
            num, line.unwrap_or(0), p0_pos.space_pos.selecting(relevant_space_coords),
            p1_pos.space_pos.selecting(relevant_space_coords),
            Contract::SPACE_UNIT_MAGNITUDE,
            requested_motion_speed,
            Contract::SPACE_UNIT_MAGNITUDE,
        );

        let (unit_vector_dir, module_target_distance) = ds
            .map_values(|coord, coord_value| match coord {
                #[cfg(feature = "with-e-axis")]
                hwa::CoordSel::E => match coord_value.is_zero() {
                    true => None,
                    false => Some(coord_value * flow_rate),
                },
                _ => match coord_value.is_zero() {
                    true => None,
                    false => Some(coord_value),
                },
            })
            .decompose_normal();

        // Compute the speed module applying speed_rate factor
        let speed_module = requested_motion_speed.unwrap_or(dts);
        // Compute per-axis distance
        let disp_vector: TVector<Real> = unit_vector_dir.abs() * speed_module;
        // Clamp per-axis target speed to the physical restrictions
        let clamped_speed = disp_vector.clamp_lower_than(max_speed);

        // Finally, per-axis relative speed
        let speed_vector = clamped_speed * speed_rate;

        let module_target_speed = speed_vector.norm2().unwrap_or(math::ZERO);
        let module_target_accel = (unit_vector_dir.abs() * max_accel)
            .norm2()
            .unwrap_or(math::ZERO);
        let module_target_jerk = (unit_vector_dir.abs() * max_jerk)
            .norm2()
            .unwrap_or(math::ZERO);

        let move_result = if module_target_distance.is_negligible() {
            #[cfg(feature = "debug-motion")]
            hwa::info!(
                "[MotionPlanner] order_num:{:?} L:{:?} Discarding plan. src: [{:?}], dest: [{:?}], vmax: [ {:?}, [{:?}] ]",
                num, line.unwrap_or(0), p0_pos.space_pos.selecting(relevant_space_coords),
                p1_pos.space_pos.selecting(relevant_space_coords), module_target_speed, speed_vector
            );
            Ok(control::CodeExecutionSuccess::OK)
        } else if !module_target_speed.is_zero()
            && !module_target_accel.is_zero()
            && !module_target_jerk.is_zero()
        {
            let segment_data = motion::Segment {
                speed_enter_su_s: Real::zero(),
                speed_exit_su_s: Real::zero(),
                speed_target_su_s: module_target_speed,
                displacement_su: module_target_distance,
                speed_enter_constrained_su_s: Real::zero(),
                speed_exit_constrained_su_s: Real::zero(),
                proj_prev: Real::zero(),
                unit_vector_dir,
                src_pos: p0_pos.space_pos.selecting(relevant_space_coords),
                dest_pos: p1_pos.space_pos.selecting(relevant_space_coords),
                dest_world_pos: p1_pos.world_pos.selecting(relevant_world_coords),
                tool_power: Real::zero(),
                constraints: control::motion::Constraints {
                    v_max: module_target_speed,
                    a_max: module_target_accel,
                    j_max: module_target_jerk,
                },
                proj_next: Real::zero(),
            };

            let r = self
                .schedule_raw_move(
                    mnemonic,
                    channel,
                    action,
                    ScheduledMove::Move(segment_data, num),
                    blocking,
                    event_bus,
                    num,
                    line,
                )
                .await?;

            hwa::debug!(
                "speed: {:?} -> {:?} ",
                requested_motion_speed.unwrap_or(Real::zero()).rdp(4),
                module_target_speed.rdp(4)
            );
            hwa::debug!("speed_vector: {:?}", speed_vector.rdp(4));
            hwa::debug!("clamped_speed: {:?}", clamped_speed.rdp(4));
            hwa::debug!("speed_rates: {:?}", speed_rate.rdp(4));
            hwa::debug!("speed_module: {:?}", module_target_speed.rdp(4));
            return Ok(r);
        } else {
            hwa::warn!(
                "[MotionPlanner] order_num:{:?} L:{:?} Bad plan. src: [{:?}], dest: [{:?}], vmax: [ {:?}, [{:?}] ]",
                num, line.unwrap_or(0), p0_pos.space_pos.selecting(relevant_space_coords), p1_pos.space_pos.selecting(relevant_space_coords), module_target_speed, speed_vector
            );
            Ok(control::CodeExecutionSuccess::OK)
        };
        match move_result {
            Ok(resp) => Ok(resp),
            Err(err) => match err {
                control::CodeExecutionFailure::BUSY => {
                    todo!("Delegate to deferrals!!!")
                }
                _r => {
                    todo!("bad response: {:?}", _r);
                    //Err(err)
                }
            },
        }
    }

    pub async fn do_homing(
        &self,
        order_num: u32,
        event_bus: &hwa::types::EventBus,
    ) -> Result<(), ()> {
        match self
            .motion_driver
            .lock()
            .await
            .homing_action(&self.motion_config)
            .await
        {
            Ok(_pos) => {
                self.motion_status().update_last_planned_position(
                    order_num,
                    &hwa::controllers::Position::new_with_world_projection(&_pos),
                );
            }
            Err(_pos) => {
                self.motion_status().update_last_planned_position(
                    order_num,
                    &hwa::controllers::Position::new_with_world_projection(&_pos),
                );
                // hwa::error!("Unable to complete homming. [Not yet] Raising SYS_ALARM");
                // self.event_bus.publish_event(EventStatus::containing(EventFlags::SYS_ALARM)).await;
                // return Err(())
            }
        };
        event_bus
            .publish_event(hwa::EventStatus::not_containing(hwa::EventFlags::HOMING))
            .await;
        Ok(())
    }

    #[cfg(all(feature = "native", feature = "plot-timings"))]
    pub async fn start_segment(
        &self,
        ref_time: embassy_time::Instant,
        real_time: embassy_time::Instant,
    ) {
        self.motion_driver
            .lock()
            .await
            .start_segment(ref_time, real_time)
    }

    #[cfg(all(feature = "native", feature = "plot-timings"))]
    pub async fn end_segment(&self) {
        self.motion_driver.lock().await.end_segment()
    }

    #[cfg(all(feature = "native", feature = "plot-timings"))]
    pub async fn mark_microsegment(&self) {
        self.motion_driver.lock().await.mark_microsegment();
    }
}

impl Clone for MotionPlanner {
    fn clone(&self) -> Self {
        Self {
            defer_channel: self.defer_channel.clone(),
            ring_buffer: self.ring_buffer.clone(),
            move_planned: self.move_planned,
            available: self.available,
            motion_config: self.motion_config.clone(),
            motion_status: self.motion_status.clone(),
            motion_driver: self.motion_driver.clone(),
        }
    }
}

/// This method performs the cornering optimization algorithm on the given `RingBuffer`.
///
/// # Arguments
///
/// * `rb` - A mutable guard that locks the `hwa::ControllerMutexType` and provides access to the `RingBuffer`.
///
/// # Returns
///
/// * `Result<(), ()>` - Returns `Ok(())` if the cornering operation is successful, otherwise returns `Err(())`.
///
/// # Usage
///
/// This method is used to optimize the movement segments that are stored in the `RingBuffer`,
/// ensuring smooth transitions and efficient processing of corners or turns in the motion path.
/// It does this by adjusting the `speed_enter_mms` and `speed_exit_mms` fields of the segments
/// based on their projected speeds and constraints.
///
/// The optimization runs a flood fill algorithm to determine the best possible speed values
/// for entering and exiting each segment, aiming for minimal speed variations and smooth transitions.
///
/// The method is typically invoked during the planning phase of motion control, where multiple
/// segments are queued, and the goal is to optimize the entire motion path by smoothing out the
/// corners, hence enhancing overall motion performance.
#[cfg(feature = "cornering")]
fn perform_cornering(
    mut rb: MutexGuard<<Contract as HwiContract>::MotionRingBufferMutexType, RingBuffer>,
) -> Result<(), ()> {
    let mut left_offset = 2;
    let mut left_watermark = math::ZERO;
    let mut right_watermark = math::ZERO;
    // Assuming queued items are computed left to right:
    // First, locate the top left segment. That one with null projection
    // Also, set
    for index in 2..rb.used {
        match rb.planned_segment_from_tail(index) {
            Ok(prev_segment_candidate) => {
                if prev_segment_candidate.proj_next.is_defined_positive() {
                    left_offset = index;
                    left_watermark = prev_segment_candidate.speed_enter_su_s;
                } else {
                    break;
                }
            }
            Err(_) => break,
        }
    }

    #[allow(unused)]
    let from_offset = left_offset;
    #[allow(unused)]
    let to_offset = 1;

    hwa::debug!("Cornering algorithm START");
    let mut right_offset = 1;

    // Perform cornering optimization in a single pass with a flood fill algorithm
    loop {
        if right_offset > left_offset {
            break;
        } else if left_offset == right_offset {
            let mid_segment = rb.mut_planned_segment_from_tail(left_offset)?;

            mid_segment.speed_enter_su_s = left_watermark;
            mid_segment.speed_exit_su_s = right_watermark;
            break;
        } else {
            let (left_segment, right_segment) =
                match rb.entries_from_tail(left_offset, right_offset) {
                    (
                        Some(PlanEntry::PlannedMove(_s, _, _, _, _)),
                        Some(PlanEntry::PlannedMove(_t, _, _, _, _)),
                    ) => (_s, _t),
                    _ => panic!(""),
                };

            hwa::trace!("\tleft [{}] right[{}]", left_offset, right_offset);

            let left_max_inc = (left_segment.proj_next * left_segment.speed_target_su_s)
                .min(left_segment.speed_exit_constrained_su_s);

            let right_max_inc = (right_segment.proj_prev * right_segment.speed_target_su_s)
                .min(right_segment.speed_enter_constrained_su_s);

            let water_left = (left_watermark + left_max_inc).min(left_segment.speed_target_su_s);
            let water_right =
                (right_watermark + right_max_inc).min(right_segment.speed_target_su_s);

            if water_left <= water_right {
                // Flood at right
                hwa::trace!("flood to right  [{:?} {:?}]", water_left, water_right);
                left_segment.speed_enter_su_s = left_watermark;
                left_segment.speed_exit_su_s = water_left;
                right_segment.speed_enter_su_s = water_left;
                right_segment.speed_exit_su_s = right_watermark;
                left_watermark = water_left;
                left_offset -= 1;
            } else {
                // Flood at left
                hwa::trace!("flood to left [{:?} {:?}]", water_left, water_right);
                left_segment.speed_enter_su_s = left_watermark;
                left_segment.speed_exit_su_s = water_right;
                right_segment.speed_enter_su_s = water_right;
                right_segment.speed_exit_su_s = right_watermark;
                right_watermark = water_right;
                right_offset += 1;
            }
        }
    }
    #[cfg(feature = "native")]
    display_content(&rb, from_offset, to_offset)?;

    hwa::debug!("Cornering algorithm END");
    Ok(())
}

#[cfg(feature = "native")]
#[allow(unused)]
pub fn display_content(
    rb: &MutexGuard<
        <hwa::types::MotionRingBufferMutexStrategy as hwa::AsyncMutexStrategy>::AsyncMutexType,
        RingBuffer,
    >,
    left_offset: u8,
    right_offset: u8,
) -> Result<(), ()> {
    let mut stb = Vec::new();
    for i in 0..left_offset - right_offset + 1 {
        let s = rb.planned_segment_from_tail(left_offset - i).unwrap();
        stb.push(format!(
            "[{:?},{:?}]",
            s.speed_enter_su_s, s.speed_exit_su_s
        ))
    }

    hwa::debug!(": {:?}", stb.join(" "));
    Ok(())
}

#[cfg(test)]
pub mod planner_test {

    //#[cfg(feature = "wip-tests")]
    #[test]
    fn discrete_positioning_case_1() {
        // TODO: Recover
        /*
        use crate::hwa;
        use hwa::controllers::{LinearMicrosegmentStepInterpolator, StepPlanner};
        use printhor_hwa_common::{CommChannel, CoordSel, DeferAction, DeferEvent};

        use crate::control::motion::{Constraints, SCurveMotionProfile};
        use crate::hwa::controllers::motion::{Segment, SegmentIterator};
        use crate::math;
        use crate::math::Real;
        use crate::math::TVector;

        const STEPPER_PLANNER_MICROSEGMENT_FREQUENCY: u32 = 200;
        const STEPPER_PLANNER_CLOCK_FREQUENCY: u32 = 20_000;

        const STEPPER_PLANNER_MICROSEGMENT_PERIOD_US: u32 =
            1_000_000 / STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;
        const STEPPER_PLANNER_CLOCK_PERIOD_US: u32 = 1_000_000 / STEPPER_PLANNER_CLOCK_FREQUENCY;

        let segment = Segment {
            speed_enter_wu_s: Real::from_f32(200.0),
            speed_exit_wu_s: Real::from_f32(200.0),
            speed_target_wu: Real::from_f32(200.0),
            displacement_wu: Real::from_f32(0.505982578),
            speed_enter_constrained_wu_s: Real::from_f32(6.25),
            speed_exit_constrained_wu_s: Real::from_f32(6.25),
            proj_prev: Real::from_f32(0.999986052),
            proj_next: Real::from_f32(0.999938488),
            unit_vector_dir: TVector::new()
                .with_coord(CoordSel::X, Some(Real::from_f32(-0.901716948)))
                .with_coord(CoordSel::Y, Some(Real::from_f32(-0.432327151))),
            src_pos: TVector::zero(),
            dest_pos: TVector::new()
                .with_coord(CoordSel::X, Some(Real::from_f32(79.9687576)))
                .with_coord(CoordSel::Y, Some(Real::from_f32(100.387497))),
            tool_power: math::ZERO,
            constraints: Constraints {
                v_max: Real::from_f32(200.0),
                a_max: Real::from_f32(3000.0),
                j_max: Real::from_f32(6000.0),
            },
        };

        let neutral_element = segment.unit_vector_dir.map_val(&math::ZERO);
        let units_per_world_unit = TVector::from_coords(
            Some(Real::from_f32(10.0)),
            Some(Real::from_f32(10.0)),
            None,
            None,
        );
        let usteps = TVector::from_coords(
            Some(Real::from_f32(8.0)),
            Some(Real::from_f32(8.0)),
            None,
            None,
        );
        let micro_segment_period_secs =
            Real::from_lit(STEPPER_PLANNER_MICROSEGMENT_PERIOD_US.into(), 6);
        let sampling_time = Real::from_lit(STEPPER_PLANNER_CLOCK_PERIOD_US.into(), 6);

        let motion_profile = SCurveMotionProfile::compute(
            segment.displacement_wu,
            segment.speed_enter_wu_s,
            segment.speed_exit_wu_s,
            &segment.constraints,
            false,
        )
        .unwrap();

        let units_per_world_unit = neutral_element + units_per_world_unit;
        let steps_per_world_unit = units_per_world_unit * usteps;

        let mut micro_segment_real_time_rel = micro_segment_period_secs;
        let mut microsegment_iterator = SegmentIterator::new(&motion_profile, math::ZERO);

        let mut microsegment_interpolator = LinearMicrosegmentStepInterpolator::new(
            segment.unit_vector_dir.abs(),
            segment.displacement_wu,
            steps_per_world_unit,
        );

        let mut prev_time = math::ZERO;
        let mut p0 = math::ZERO;
        let mut real_advanced_steps: TVector<u32> = TVector::zero();

        loop {
            if let Some((estimated_position, _)) =
                microsegment_iterator.next(micro_segment_real_time_rel)
            {
                let tprev = micro_segment_real_time_rel - prev_time;
                let tmax = motion_profile.i7_end() - prev_time;

                let ds = estimated_position - p0;
                p0 = estimated_position;
                let current_period_width_0 = if tprev < tmax {
                    tprev
                } else {
                    if segment.segment_data.speed_exit_wu_s > math::ZERO {
                        (ds / segment.segment_data.speed_exit_wu_s).max(sampling_time)
                    } else {
                        tmax
                    }
                };

                let current_period_width = current_period_width_0;

                prev_time += current_period_width;
                micro_segment_real_time_rel += current_period_width;

                let w = (current_period_width * Real::from_f32(1000000.)).round();

                let has_more = microsegment_interpolator.advance_to(estimated_position, w);

                let mut step_planner = StepPlanner::from(
                    microsegment_interpolator.state().clone(),
                    CoordSel::empty(),
                    CoordSel::empty(),
                );

                let mut tick_count = 0;
                loop {
                    match step_planner.next(STEPPER_PLANNER_CLOCK_PERIOD_US) {
                        None => {
                            break;
                        }
                        Some(_t) => {
                            tick_count += STEPPER_PLANNER_CLOCK_PERIOD_US;
                            if !_t.is_empty() {
                                real_advanced_steps.increment(_t.into(), 1);
                            }
                            // std::println!("t = {} : {} w = {}", tick_count, real_advanced_steps, width);
                            if tick_count >= microsegment_interpolator.width() {
                                break;
                            }
                        }
                    }
                }
                if !has_more {
                    break;
                }
            } else {
                break;
            }
        }
        let expected_advanced_steps = microsegment_interpolator.advanced_steps();
        assert_eq!(
            expected_advanced_steps, real_advanced_steps,
            "Advanced steps matching. Expected: {:?} Got {:?}",
            expected_advanced_steps, real_advanced_steps
        )

         */
    }

    #[test]
    fn discrete_positioning_case_2() {
        // TODO: Recover
        /*
        use crate::hwa;
        use hwa::controllers::{LinearMicrosegmentStepInterpolator, StepPlanner};
        use printhor_hwa_common::{CommChannel, CoordSel, DeferAction, DeferEvent};
        use crate::control::motion::{Constraints, SCurveMotionProfile};
        use crate::hwa::controllers::motion::motion_segment::{
            Segment, SegmentIterator,
        };
        use crate::math;
        use crate::math::Real;
        use crate::math::TVector;

        const STEPPER_PLANNER_MICROSEGMENT_FREQUENCY: u32 = 500;
        const STEPPER_PLANNER_CLOCK_FREQUENCY: u32 = 50_000;

        const STEPPER_PLANNER_MICROSEGMENT_PERIOD_US: u32 =
            1_000_000 / STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;
        const STEPPER_PLANNER_CLOCK_PERIOD_US: u32 = 1_000_000 / STEPPER_PLANNER_CLOCK_FREQUENCY;

        let segment = Segment {
            speed_enter_wu_s: Real::from_f32(400.0),
            speed_exit_wu_s: Real::from_f32(400.0),
            speed_target_wu: Real::from_f32(400.0),
            displacement_wu: Real::from_f32(0.505982578),
            speed_enter_constrained_wu_s: Real::from_f32(6.25),
            speed_exit_constrained_wu_s: Real::from_f32(6.25),
            proj_prev: Real::from_f32(0.999986052),
            proj_next: Real::from_f32(0.999938488),
            unit_vector_dir: TVector::from_coords(
                Some(Real::from_f32(-0.901716948)),
                Some(Real::from_f32(-0.432327151)),
                None,
                None,
            ),
            src_pos: TVector::zero(),
            dest_pos: TVector::from_coords(
                Some(Real::from_f32(79.9687576)),
                Some(Real::from_f32(100.387497)),
                None,
                None,
            ),
            tool_power: math::ZERO,
            constraints: Constraints {
                v_max: Real::from_f32(400.0),
                a_max: Real::from_f32(3000.0),
                j_max: Real::from_f32(6000.0),
            },
        };

        let neutral_element = segment.unit_vector_dir.map_val(&math::ZERO);
        let units_per_mm = TVector::from_coords(
            Some(Real::from_f32(10.0)),
            Some(Real::from_f32(10.0)),
            None,
            None,
        );
        let usteps = TVector::from_coords(
            Some(Real::from_f32(8.0)),
            Some(Real::from_f32(8.0)),
            None,
            None,
        );
        let micro_segment_period_secs =
            Real::from_lit(STEPPER_PLANNER_MICROSEGMENT_PERIOD_US.into(), 6);
        let sampling_time = Real::from_lit(STEPPER_PLANNER_CLOCK_PERIOD_US.into(), 6);

        let motion_profile = SCurveMotionProfile::compute(
            segment.segment_data.displacement_wu,
            segment.segment_data.speed_enter_wu_s,
            segment.segment_data.speed_exit_wu_s,
            &segment.segment_data.constraints,
            false,
        )
        .unwrap();

        let units_per_mm = neutral_element + units_per_mm;
        let steps_per_mm = units_per_mm * usteps;

        let mut micro_segment_real_time_rel = micro_segment_period_secs;
        let mut microsegment_iterator = SegmentIterator::new(&motion_profile, math::ZERO);

        let mut microsegment_interpolator = LinearMicrosegmentStepInterpolator::new(
            segment.segment_data.unit_vector_dir.abs(),
            segment.segment_data.displacement_wu,
            steps_per_mm,
        );

        let mut prev_time = math::ZERO;
        let mut p0 = math::ZERO;
        let mut real_advanced_steps: TVector<u32> = TVector::zero();

        loop {
            if let Some((estimated_position, _)) =
                microsegment_iterator.next(micro_segment_real_time_rel)
            {
                let tprev = micro_segment_real_time_rel - prev_time;
                let tmax = motion_profile.i7_end() - prev_time;

                let ds = estimated_position - p0;
                p0 = estimated_position;
                let current_period_width_0 = if tprev < tmax {
                    tprev
                } else {
                    if segment.segment_data.speed_exit_wu_s > math::ZERO {
                        (ds / segment.segment_data.speed_exit_wu_s).max(sampling_time)
                    } else {
                        tmax
                    }
                };

                let current_period_width = current_period_width_0;

                prev_time += current_period_width;
                micro_segment_real_time_rel += current_period_width;

                let w = (current_period_width * Real::from_f32(1000000.)).round();

                let has_more = microsegment_interpolator.advance_to(estimated_position, w);

                let mut step_planner = StepPlanner::from(
                    microsegment_interpolator.state().clone(),
                    CoordSel::empty(),
                    CoordSel::empty(),
                );

                let mut tick_count = 0;
                loop {
                    match step_planner.next(STEPPER_PLANNER_CLOCK_PERIOD_US) {
                        None => {
                            break;
                        }
                        Some(_t) => {
                            tick_count += STEPPER_PLANNER_CLOCK_PERIOD_US;
                            if !_t.is_empty() {
                                real_advanced_steps.increment(_t.into(), 1);
                            }
                            // std::println!("t = {} : {} w = {}", tick_count, real_advanced_steps, width);
                            if tick_count >= microsegment_interpolator.width() {
                                break;
                            }
                        }
                    }
                }
                if !has_more {
                    break;
                }
            } else {
                break;
            }
        }
        let expected_advanced_steps = microsegment_interpolator.advanced_steps();
        assert_eq!(
            expected_advanced_steps, real_advanced_steps,
            "Advanced steps matching. Expected: {:?} Got {:?}",
            expected_advanced_steps, real_advanced_steps
        )
         */
    }

    #[test]
    fn discrete_positioning_case_3() {
        // TODO: Recover
        /*
        use crate::hwa;
        use hwa::controllers::{LinearMicrosegmentStepInterpolator, StepPlanner};
        use printhor_hwa_common::{CommChannel, CoordSel, DeferAction, DeferEvent};
        use crate::control::motion::{Constraints, SCurveMotionProfile};
        use crate::hwa::controllers::motion::{Segment, SegmentData, SegmentIterator};
        use crate::math;
        use crate::math::Real;
        use crate::math::TVector;
        use printhor_hwa_common::StepperChannel;

        const STEPPER_PLANNER_MICROSEGMENT_FREQUENCY: u32 = 500;
        const STEPPER_PLANNER_CLOCK_FREQUENCY: u32 = 200_000;

        const STEPPER_PLANNER_MICROSEGMENT_PERIOD_US: u32 =
            1_000_000 / STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;
        const STEPPER_PLANNER_CLOCK_PERIOD_US: u32 = 1_000_000 / STEPPER_PLANNER_CLOCK_FREQUENCY;

        let segment = Segment::new(SegmentData {
            speed_enter_wu_s: Real::from_f32(400.0),
            speed_exit_wu_s: Real::from_f32(400.0),
            speed_target_wu: Real::from_f32(400.0),
            displacement_wu: Real::from_f32(0.505982578),
            speed_enter_constrained_wu_s: Real::from_f32(6.25),
            speed_exit_constrained_wu_s: Real::from_f32(6.25),
            proj_prev: Real::from_f32(0.999986052),
            proj_next: Real::from_f32(0.999938488),
            unit_vector_dir: TVector::from_coords(
                Some(Real::from_f32(-0.901716948)),
                Some(Real::from_f32(-0.432327151)),
                None,
                None,
            ),
            dest_pos: TVector::from_coords(
                Some(Real::from_f32(79.9687576)),
                Some(Real::from_f32(100.387497)),
                None,
                None,
            ),
            tool_power: math::ZERO,
            constraints: Constraints {
                v_max: Real::from_f32(400.0),
                a_max: Real::from_f32(3000.0),
                j_max: Real::from_f32(6000.0),
            },
        });

        let neutral_element = segment.segment_data.unit_vector_dir.map_val(&math::ZERO);
        let units_per_mm = TVector::from_coords(
            Some(Real::from_f32(10.0)),
            Some(Real::from_f32(10.0)),
            None,
            None,
        );
        let usteps = TVector::from_coords(
            Some(Real::from_f32(8.0)),
            Some(Real::from_f32(8.0)),
            None,
            None,
        );
        let micro_segment_period_secs =
            Real::from_lit(STEPPER_PLANNER_MICROSEGMENT_PERIOD_US.into(), 6);
        let sampling_time = Real::from_lit(STEPPER_PLANNER_CLOCK_PERIOD_US.into(), 6);

        let motion_profile = SCurveMotionProfile::compute(
            segment.segment_data.displacement_wu,
            segment.segment_data.speed_enter_wu_s,
            segment.segment_data.speed_exit_wu_s,
            &segment.segment_data.constraints,
            false,
        )
        .unwrap();

        let units_per_mm = neutral_element + units_per_mm;
        let steps_per_mm = units_per_mm * usteps;

        let mut micro_segment_real_time_rel = micro_segment_period_secs;
        let mut microsegment_iterator = SegmentIterator::new(&motion_profile, math::ZERO);

        let mut microsegment_interpolator = LinearMicrosegmentStepInterpolator::new(
            segment.segment_data.unit_vector_dir.abs(),
            segment.segment_data.displacement_wu,
            steps_per_mm,
        );

        let mut prev_time = math::ZERO;
        let mut p0 = math::ZERO;
        let mut real_advanced_steps: TVector<u32> = TVector::zero();

        loop {
            if let Some((estimated_position, _)) =
                microsegment_iterator.next(micro_segment_real_time_rel)
            {
                let tprev = micro_segment_real_time_rel - prev_time;
                let tmax = motion_profile.i7_end() - prev_time;

                let ds = estimated_position - p0;
                p0 = estimated_position;
                let current_period_width_0 = if tprev < tmax {
                    tprev
                } else {
                    if segment.segment_data.speed_exit_wu_s > math::ZERO {
                        (ds / segment.segment_data.speed_exit_wu_s).max(sampling_time)
                    } else {
                        tmax
                    }
                };

                let current_period_width = current_period_width_0;

                prev_time += current_period_width;
                micro_segment_real_time_rel += current_period_width;

                let w = (current_period_width * Real::from_f32(1000000.)).round();

                let has_more = microsegment_interpolator.advance_to(estimated_position, w);

                let mut step_planner = StepPlanner::from(
                    microsegment_interpolator.state().clone(),
                    StepperChannel::empty(),
                    StepperChannel::empty(),
                );

                let mut tick_count = 0;
                loop {
                    match step_planner.next(STEPPER_PLANNER_CLOCK_PERIOD_US) {
                        None => {
                            break;
                        }
                        Some(_t) => {
                            tick_count += STEPPER_PLANNER_CLOCK_PERIOD_US;
                            if !_t.is_empty() {
                                real_advanced_steps.increment(_t.into(), 1);
                            }
                            // std::println!("t = {} : {} w = {}", tick_count, real_advanced_steps, width);
                            if tick_count >= microsegment_interpolator.width() {
                                break;
                            }
                        }
                    }
                }
                if !has_more {
                    break;
                }
            } else {
                break;
            }
        }
        let expected_advanced_steps = microsegment_interpolator.advanced_steps();
        assert_eq!(
            expected_advanced_steps, real_advanced_steps,
            "Advanced steps matching. Expected: {:?} Got {:?}",
            expected_advanced_steps, real_advanced_steps
        )

         */
    }

    /*
    #[futures_test::test]
    async fn boundary_conditions_tests() {
    use crate::hwa;
        use hwa::controllers::{LinearMicrosegmentStepInterpolator, StepPlanner};
        use printhor_hwa_common::{CommChannel, CoordSel, DeferAction, DeferEvent};
        // Test with zero values
        let segment_zero = Segment::new(0.0, 0.0, 0.0, 0.0, 0.0, Constraints::new(0.0, 0.0, 0.0));
        assert_eq!(segment_zero.compute(), Ok(()));

        // Test with very large positive values
        let segment_large = Segment::new(
            f32::MAX,
            f32::MAX,
            f32::MAX,
            f32::MAX,
            f32::MAX,
            Constraints::new(f32::MAX, f32::MAX, f32::MAX),
        );
        assert_eq!(segment_large.compute(), Ok(()));

        // Test with very small positive values (close to zero but not zero)
        let segment_small = Segment::new(
            f32::EPSILON,
            f32::EPSILON,
            f32::EPSILON,
            f32::EPSILON,
            f32::EPSILON,
            Constraints::new(f32::EPSILON, f32::EPSILON, f32::EPSILON),
        );
        assert_eq!(segment_small.compute(), Ok(()));

        // Test with negative values
        let segment_negative = Segment::new(
            -100.0,
            -100.0,
            -100.0,
            -100.0,
            -100.0,
            Constraints::new(-100.0, -100.0, -100.0),
        );
        assert_eq!(segment_negative.compute(), Ok(()));

        // Test with normal values
        let segment_normal = Segment::new(
            100.0,
            150.0,
            200.0,
            250.0,
            300.0,
            Constraints::new(400.0, 500.0, 600.0),
        );
        assert_eq!(segment_normal.compute(), Ok(()));
    }
    */

    #[futures_test::test]
    async fn receiver_receives_given_try_send_async() {
        use crate::hwa;
        type MutexType = hwa::AsyncCsMutexType;
        type ResourceType = hwa::EventBusChannelController<MutexType>;
        type EventBusMutexStrategyType = hwa::AsyncStandardStrategy<MutexType, ResourceType>;

        let event_bus = hwa::GenericEventBus::new(hwa::make_static_async_controller!(
            "EventBusChannelController",
            EventBusMutexStrategyType,
            hwa::EventBusChannelController::new(hwa::make_static_ref!(
                "EventBusChannel",
                hwa::EventBusPubSubType<MutexType>,
                hwa::EventBusPubSubType::new()
            ))
        ));

        let defer_channel: hwa::GenericDeferChannel<MutexType> =
            hwa::GenericDeferChannel::new(hwa::make_static_ref!(
                "DeferChannel",
                hwa::DeferChannelChannelType<MutexType>,
                hwa::DeferChannelChannelType::new()
            ));

        let _motion_config = hwa::make_static_sync_controller!(
            "MotionConfig",
            hwa::SyncStandardStrategy<hwa::SyncCsMutexType, hwa::controllers::MotionConfigContent>,
            hwa::controllers::MotionConfigContent::new()
        );

        let _st = event_bus.get_status().await;

        defer_channel
            .send(hwa::DeferEvent::Completed(
                hwa::DeferAction::Homing,
                hwa::CommChannel::Internal,
                1,
            ))
            .await;
        /*
        Work in progress
        */
    }
}
