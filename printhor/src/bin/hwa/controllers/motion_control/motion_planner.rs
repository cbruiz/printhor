//! Motion Planner
use crate::{hwa, motion, processing};
use hwa::Contract;
use hwa::HwiContract;
use hwa::controllers::ExecPlan::Homing;
use hwa::math;

use hwa::controllers::motion_control::motion_ring_buffer::RingBuffer;
use hwa::controllers::{MovType, PlanEntry, ScheduledMove, motion_control};
use hwa::{EventFlags, EventStatus, PersistentState};
use math::Real;

use math::TVector;

/// The execution plan action dequeued from the buffer
pub enum ExecPlan {
    Segment(motion_control::Segment, hwa::CommChannel, u32),
    Dwell(Option<u32>, hwa::CommChannel, u32),
    SetPosition(motion_control::Position, hwa::CommChannel, u32),
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

/// Max length of each linear segment approximating an arc (G2/G3) or a spline (G5), in world
/// units. Shorter segments are used where the chordal tolerance requires it
#[cfg(any(feature = "with-arcs", feature = "with-beziers"))]
const ARC_SEGMENT_LENGTH_WU: Real = math::ONE;
/// Upper bound of linear segments per arc or spline
#[cfg(any(feature = "with-arcs", feature = "with-beziers"))]
const ARC_MAX_SEGMENTS: i64 = 2048;
/// Max distance between a curve (G2/G3/G5) and the segments approximating it, as a fraction of
/// the smallest XY step. The step lattice itself adds up to ~0.71 steps, so 0.2 keeps the path
/// within one step of the ideal curve
#[cfg(any(feature = "with-arcs", feature = "with-beziers"))]
const CHORD_TOLERANCE_STEPS: Real = hwa::make_real!(0.2);

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
                    PlanEntry::SetPosition(channel, position, _deferred, order_num) => {
                        rb.data[head] = PlanEntry::Executing(
                            MovType::SetPosition(channel),
                            _deferred,
                            order_num,
                        );
                        return ExecPlan::SetPosition(position, channel, order_num);
                    }
                    PlanEntry::PlannedMove(planned_data, action, channel, _deferred, order_num) => {
                        hwa::debug!(
                            "PlannedMove starting: {} / {} h={}",
                            rb.used,
                            Contract::SEGMENT_QUEUE_SIZE,
                            head
                        );
                        rb.data[head] = PlanEntry::Executing(
                            MovType::Move(action, channel),
                            _deferred,
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
            PlanEntry::Executing(MovType::Homing(channel), _is_defer, order_num) => {
                event_bus
                    .publish_event(hwa::EventStatus::not_containing(hwa::EventFlags::HOMING))
                    .await;
                if *_is_defer {
                    self.defer_channel
                        .send(hwa::DeferEvent::Completed(
                            hwa::DeferAction::Homing,
                            *channel,
                            *order_num,
                        ))
                        .await;
                }
            }
            PlanEntry::Executing(MovType::Dwell(channel), _is_defer, order_num) => {
                if *_is_defer {
                    self.defer_channel
                        .send(hwa::DeferEvent::Completed(
                            hwa::DeferAction::Dwell,
                            *channel,
                            *order_num,
                        ))
                        .await;
                }
            }
            PlanEntry::Executing(MovType::SetPosition(channel), _is_defer, order_num) => {
                if *_is_defer {
                    self.defer_channel
                        .send(hwa::DeferEvent::Completed(
                            hwa::DeferAction::SetPosition,
                            *channel,
                            *order_num,
                        ))
                        .await;
                }
            }
            PlanEntry::Executing(MovType::Move(action, channel), _is_defer, order_num) => {
                if *_is_defer {
                    self.defer_channel
                        .send(hwa::DeferEvent::Completed(*action, *channel, *order_num))
                        .await;
                }
            }
            _ => {
                hwa::error!("consume_current_plan: unexpected PlanEntry at head={}", head);
            }
        }
        rb.data[head as usize] = PlanEntry::Empty;
        rb.head = match head + 1 < Contract::SEGMENT_QUEUE_SIZE as u8 {
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
    ) -> Result<processing::CodeExecutionSuccess, processing::CodeExecutionFailure> {
        hwa::debug!("schedule_raw_move() BEGIN");
        loop {
            self.available.wait().await;
            {
                let mut rb = self.ring_buffer.lock().await;
                let mut is_defer = rb.used == Contract::SEGMENT_QUEUE_SIZE - 1;

                if rb.used < Contract::SEGMENT_QUEUE_SIZE {
                    #[cfg(feature = "with-motion-chaining")]
                    let mut do_adjust = false;

                    let curr_insert_index = rb.index_from_tail(0).unwrap();

                    let (curr_planned_entry, event) = match move_type {
                        ScheduledMove::Move(curr_segment_in, _order_num) => {
                            let mut curr_segment = curr_segment_in;
                            #[cfg(feature = "with-motion-chaining")]
                            motion::compute_maximum_enter_and_exit_speed(true, _order_num, &mut curr_segment);

                            if let Ok(prev_index) = rb.index_from_tail(1) {
                                match &mut rb.data[prev_index as usize] {
                                    PlanEntry::PlannedMove(prev_segment, _, _, _, _) => {
                                        #[cfg(feature = "with-motion-chaining")]
                                        {
                                            let proj: Real =
                                                prev_segment.unit_vector_dir.scalar_projection(
                                                    curr_segment.unit_vector_dir,
                                                );
                                            if proj.is_defined_positive() {
                                                hwa::debug!(
                                                    "RingBuffer [{:?}, {:?}] chained: ({:?}) proj ({:?}) = ({:?})",
                                                    prev_index,
                                                    curr_insert_index,
                                                    prev_segment.unit_vector_dir,
                                                    curr_segment.unit_vector_dir,
                                                    proj
                                                );
                                                #[cfg(feature = "with-motion-chaining")]
                                                hwa::debug!(
                                                    "\ts : vi = [{:?} < {:?}] - vtarget = {:?} - vo = [{:?} < {:?}]:",
                                                    prev_segment.speed_enter_su_s,
                                                    prev_segment.speed_enter_constrained_su_s,
                                                    prev_segment.speed_target_su_s,
                                                    prev_segment.speed_exit_su_s,
                                                    prev_segment.speed_exit_constrained_su_s,
                                                );
                                                hwa::debug!("\t\tproj = {:?}", proj);
                                                do_adjust = true;
                                                prev_segment.proj_next = proj;
                                                curr_segment.proj_prev = proj;
                                                // This segment is no longer last; let it exit
                                                // up to its target speed so the backward pass
                                                // can find the optimal junction from v_max down.
                                                prev_segment.speed_exit_su_s = prev_segment.speed_target_su_s;
                                            }
                                        }
                                    }
                                    _ => {}
                                }
                            }

                            #[cfg(feature = "debug-motion")]
                            hwa::info!(
                                "[MotionPlanner] order_num:{:?} L:{:?} MotionPlan queued. space src: [{:#?}], dest: [{:#?}], vmax: [ {:?}, [{:?}] ]",
                                _order_num,
                                line_tag.unwrap_or(0),
                                curr_segment.src_pos,
                                curr_segment.dest_pos,
                                curr_segment.speed_target_su_s,
                                curr_segment.unit_vector_dir.abs() * curr_segment.speed_target_su_s
                            );
                            {
                                let pos = hwa::controllers::Position::new_from(
                                    curr_segment.dest_world_pos,
                                    curr_segment.dest_pos,
                                );
                                self.motion_status
                                    .update_last_planned_position(_order_num, &pos);
                            }
                            (
                                PlanEntry::PlannedMove(
                                    curr_segment,
                                    action,
                                    channel,
                                    is_defer,
                                    _order_num,
                                ),
                                hwa::EventStatus::containing(EventFlags::NOTHING),
                            )
                        }
                        ScheduledMove::Homing(order_num) => {
                            is_defer = true;
                            let pos = hwa::controllers::Position::new_with_world_projection(
                                &self.motion_config.get_logical_homing_point(),
                            );
                            self.motion_status
                                .update_last_planned_position(order_num, &pos);
                            (
                                PlanEntry::Homing(channel, is_defer, order_num),
                                hwa::EventStatus::not_containing(hwa::EventFlags::HOMING),
                            )
                        }
                        ScheduledMove::Dwell(sleep, order_num) => {
                            is_defer = true;
                            (
                                PlanEntry::Dwell(channel, sleep, is_defer, order_num),
                                hwa::EventStatus::containing(EventFlags::MOV_QUEUE_EMPTY),
                            )
                        }
                        ScheduledMove::SetPosition(position, _order_num) => {
                            is_defer = false;
                            self.motion_status
                                .update_last_planned_position(_order_num, &position);
                            (
                                PlanEntry::SetPosition(channel, position, is_defer, _order_num),
                                hwa::EventStatus::containing(EventFlags::NOTHING),
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
                    #[cfg(feature = "with-motion-chaining")]
                    if do_adjust {
                        // The stepper corrects the position deviation (a rounding residual of
                        // whole steps, at most ~½ step per axis) by re-deriving each displacement
                        // before executing it. One full step per axis bounds that change with a
                        // 2x margin; the planner keeps every blend feasible within it.
                        let displacement_tolerance = self
                            .motion_config
                            .get_steps_per_space_unit_as_vector()
                            .map_values(|_, steps| match steps > math::ZERO {
                                true => Some(steps.recip()),
                                false => None,
                            })
                            .norm2()
                            .unwrap_or(math::ZERO);
                        let _ = motion::perform_motion_chaining(&mut rb, displacement_tolerance);
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
                        Ok(processing::CodeExecutionSuccess::DEFERRED(event))
                    } else {
                        #[cfg(feature = "trace-commands")]
                        hwa::info!(
                            "[trace-commands] Commited #order: {} #line: {:?}",
                            num_order,
                            line_tag
                        );
                        hwa::debug!("schedule_raw_move() END - Finally queued");
                        Ok(processing::CodeExecutionSuccess::QUEUED)
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
                        return Err(processing::CodeExecutionFailure::BUSY);
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
        gc: &processing::GCodeCmd,
        blocking: bool,
        event_bus: &hwa::types::EventBus,
    ) -> Result<processing::CodeExecutionSuccess, processing::CodeExecutionFailure> {
        match &gc.value {
            processing::GCodeValue::G0(t) => Ok(self
                .schedule_move(
                    "G0",
                    channel,
                    hwa::DeferAction::RapidMove,
                    t.into(),
                    t.f,
                    blocking,
                    event_bus,
                    gc.order_num,
                    gc.line_tag,
                )
                .await?),
            processing::GCodeValue::G1(t) => Ok(self
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
            #[cfg(feature = "with-beziers")]
            processing::GCodeValue::G5(bz) => Ok(self
                .schedule_bezier(channel, bz, event_bus, gc.order_num, gc.line_tag)
                .await?),
            #[cfg(feature = "with-arcs")]
            processing::GCodeValue::G2(arc) => Ok(self
                .schedule_arc("G2", channel, arc, true, event_bus, gc.order_num, gc.line_tag)
                .await?),
            #[cfg(feature = "with-arcs")]
            processing::GCodeValue::G3(arc) => Ok(self
                .schedule_arc("G3", channel, arc, false, event_bus, gc.order_num, gc.line_tag)
                .await?),
            processing::GCodeValue::G4(t) => {
                // G4 S<seconds> / G4 P<milliseconds>
                let millis = match (t.p, t.s) {
                    (Some(p), _) => Some(p.int().max(0) as u32),
                    (None, Some(s)) => Some((s * Real::from_lit(1000, 0)).int().max(0) as u32),
                    (None, None) => None,
                };
                Ok(self
                    .schedule_raw_move(
                        "G4",
                        channel,
                        hwa::DeferAction::Dwell,
                        ScheduledMove::Dwell(millis, gc.order_num),
                        blocking,
                        event_bus,
                        gc.order_num,
                        gc.line_tag,
                    )
                    .await?)
            }
            #[cfg(feature = "with-e-axis")]
            processing::GCodeValue::G10 => Ok(self
                .schedule_retraction(channel, true, blocking, event_bus, gc.order_num, gc.line_tag)
                .await?),
            #[cfg(feature = "with-e-axis")]
            processing::GCodeValue::G11 => Ok(self
                .schedule_retraction(channel, false, blocking, event_bus, gc.order_num, gc.line_tag)
                .await?),
            processing::GCodeValue::M400 => {
                // A zero-length dwell is always deferred and only completes when every previously
                // queued move has been executed, which is exactly the M400 semantics.
                Ok(self
                    .schedule_raw_move(
                        "M400",
                        channel,
                        hwa::DeferAction::Dwell,
                        ScheduledMove::Dwell(Some(0), gc.order_num),
                        blocking,
                        event_bus,
                        gc.order_num,
                        gc.line_tag,
                    )
                    .await?)
            }
            #[cfg(feature = "with-motion")]
            processing::GCodeValue::G28(_x) => {
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
            processing::GCodeValue::G92(t) => {
                let new_coords: TVector<Real> = TVector::from(t.into());
                #[cfg(feature = "with-e-axis")]
                if let Some(e) = new_coords.get_coord(hwa::CoordSel::E) {
                    // The logical extruder position is a planning-time state
                    self.motion_status.set_logical_e(e);
                }
                let mut position = self.motion_status.get_last_planned_position();
                position.update_from_world_coordinates(&new_coords);
                Ok(self
                    .schedule_raw_move(
                        "G92",
                        channel,
                        hwa::DeferAction::SetPosition,
                        ScheduledMove::SetPosition(position, gc.order_num),
                        blocking,
                        event_bus,
                        gc.order_num,
                        gc.line_tag,
                    )
                    .await?)
            }
            processing::GCodeValue::G29 => Ok(processing::CodeExecutionSuccess::OK),
            processing::GCodeValue::G29_1 => Ok(processing::CodeExecutionSuccess::OK),
            processing::GCodeValue::G29_2 => Ok(processing::CodeExecutionSuccess::OK),
            _ => Err(processing::CodeExecutionFailure::NotYetImplemented),
        }
    }

    /// Converts a `F` word (world units per minute, as in standard GCode) to world units per
    /// second and updates the modal feed rate. Returns the feed rate to apply to the move.
    fn update_feed_rate(&self, f_wu_min: Option<Real>) -> Option<Real> {
        if let Some(f) = f_wu_min {
            if f.is_defined_positive() {
                self.motion_status
                    .set_feed_rate(f / Real::from_lit(60, 0));
            }
        }
        self.motion_status.get_feed_rate()
    }

    /// Plans a linear move (G0/G1) to `p1_wu`, interpreted according to the positioning modes
    /// (G90/G91 and M82/M83). `f_wu_min` is the (optional) `F` word in world units per minute.
    async fn schedule_move(
        &self,
        mnemonic: &'static str,
        channel: hwa::CommChannel,
        action: hwa::DeferAction,
        p1_wu: TVector<Real>,
        f_wu_min: Option<Real>,
        blocking: bool,
        event_bus: &hwa::types::EventBus,
        order_num: u32,
        line: Option<u32>,
    ) -> Result<processing::CodeExecutionSuccess, processing::CodeExecutionFailure> {
        let p0_pos = self.motion_status.get_last_planned_position();
        if !p0_pos.is_set {
            return Err(processing::CodeExecutionFailure::HomingRequired);
        }
        let feed_rate = self.update_feed_rate(f_wu_min);

        // Motion axis (E is handled separately, as it's not a kinematic coordinate)
        let p1_axis = p1_wu.selecting(hwa::CoordSel::motion_relevant_axis());
        let target_wu = if self.motion_status.is_absolute_positioning() {
            p1_axis
        } else {
            (p0_pos.world_pos + p1_axis).selecting(p1_axis.not_nan_coords())
        };

        // Extruder: E words are converted to a displacement from the logical position
        #[cfg(feature = "with-e-axis")]
        let (e_delta, e_target) = match p1_wu.get_coord(hwa::CoordSel::E) {
            Some(e) => {
                let logical_e = self.motion_status.get_logical_e();
                let target = match self.motion_status.is_e_relative() {
                    true => logical_e + e,
                    false => e,
                };
                (target - logical_e, Some(target))
            }
            None => (math::ZERO, None),
        };
        #[cfg(not(feature = "with-e-axis"))]
        let e_delta = math::ZERO;

        let result = self
            .schedule_displacement(
                mnemonic, channel, action, &p0_pos, &target_wu, e_delta, feed_rate, blocking,
                event_bus, order_num, line,
            )
            .await;
        #[cfg(feature = "with-e-axis")]
        if let (Ok(_), Some(e)) = (&result, e_target) {
            self.motion_status.set_logical_e(e);
        }
        result
    }

    /// Plans a firmware retraction (G10) or recover (G11) according to M207/M208 settings.
    ///
    /// Retraction does not alter the logical extruder position.
    #[cfg(feature = "with-e-axis")]
    async fn schedule_retraction(
        &self,
        channel: hwa::CommChannel,
        retract: bool,
        blocking: bool,
        event_bus: &hwa::types::EventBus,
        order_num: u32,
        line: Option<u32>,
    ) -> Result<processing::CodeExecutionSuccess, processing::CodeExecutionFailure> {
        if self.motion_status.is_retracted() == retract {
            // Already in the requested state
            return Ok(processing::CodeExecutionSuccess::OK);
        }
        let p0_pos = self.motion_status.get_last_planned_position();
        if !p0_pos.is_set {
            return Err(processing::CodeExecutionFailure::HomingRequired);
        }
        let cfg = self.motion_config.get_retraction();
        let (e_delta, feed_rate, z_hop) = match retract {
            true => (-cfg.length, cfg.feed_rate, cfg.z_hop),
            false => (cfg.length + cfg.recover_extra, cfg.recover_feed_rate, -cfg.z_hop),
        };
        #[allow(unused_mut)]
        let mut target_wu = TVector::new();
        #[cfg(feature = "with-z-axis")]
        if !z_hop.is_zero() {
            target_wu.set_coord(
                hwa::CoordSel::Z,
                p0_pos
                    .world_pos
                    .get_coord(hwa::CoordSel::Z)
                    .map(|z| z + z_hop),
            );
        }
        let _ = z_hop;
        let result = self
            .schedule_displacement(
                if retract { "G10" } else { "G11" },
                channel,
                hwa::DeferAction::LinearMove,
                &p0_pos,
                &target_wu,
                e_delta,
                Some(feed_rate),
                blocking,
                event_bus,
                order_num,
                line,
            )
            .await;
        if result.is_ok() {
            self.motion_status.set_retracted(retract);
        }
        result
    }

    /// Max distance (world units) between a curve and the segments approximating it:
    /// [CHORD_TOLERANCE_STEPS] of the smallest XY step, according to the current steps per unit
    #[cfg(any(feature = "with-arcs", feature = "with-beziers"))]
    fn chord_tolerance(&self) -> Real {
        let spm = self.motion_config.get_steps_per_space_unit_as_vector();
        let finest = Real::vmax(
            spm.get_coord(hwa::CoordSel::X),
            spm.get_coord(hwa::CoordSel::Y),
        );
        match finest {
            Some(f) if f.is_defined_positive() => CHORD_TOLERANCE_STEPS / f,
            // Unknown resolution: a conservative value for typical machines
            _ => Real::from_lit(25, 4),
        }
    }

    /// Plans an arc (G2 clockwise / G3 counter-clockwise) in the XY plane, approximated by
    /// linear segments of at most [ARC_SEGMENT_LENGTH_WU] world units, shortened as needed so that
    /// the path stays within the chordal tolerance ([MotionPlanner::chord_tolerance]).
    ///
    /// Intermediate segments are scheduled as internal blocking moves, so that the issuer only
    /// gets a single response, corresponding to the last segment.
    #[cfg(feature = "with-arcs")]
    async fn schedule_arc(
        &self,
        mnemonic: &'static str,
        channel: hwa::CommChannel,
        arc: &processing::ARC,
        clockwise: bool,
        event_bus: &hwa::types::EventBus,
        order_num: u32,
        line: Option<u32>,
    ) -> Result<processing::CodeExecutionSuccess, processing::CodeExecutionFailure> {
        use hwa::CoordSel;
        let p0_pos = self.motion_status.get_last_planned_position();
        if !p0_pos.is_set {
            return Err(processing::CodeExecutionFailure::HomingRequired);
        }
        let feed_rate = self.update_feed_rate(arc.coords.f);
        let absolute = self.motion_status.is_absolute_positioning();

        let start = p0_pos.world_pos;
        let x0 = start.get_coord(CoordSel::X).unwrap_or(math::ZERO);
        let y0 = start.get_coord(CoordSel::Y).unwrap_or(math::ZERO);
        let resolve = |value: Option<Real>, origin: Real| -> Real {
            match value {
                Some(v) if absolute => v,
                Some(v) => origin + v,
                None => origin,
            }
        };
        let x1 = resolve(arc.coords.x, x0);
        let y1 = resolve(arc.coords.y, y0);
        #[cfg(feature = "with-z-axis")]
        let z0 = start.get_coord(CoordSel::Z).unwrap_or(math::ZERO);
        #[cfg(feature = "with-z-axis")]
        let z1 = resolve(arc.coords.z, z0);

        // Center of the arc
        let (cx, cy) = match (arc.i, arc.j, arc.r) {
            (None, None, Some(r)) => {
                // Radius form. Positive R: shortest arc (< 180º). Negative R: longest arc.
                let dx = x1 - x0;
                let dy = y1 - y0;
                let d2 = dx * dx + dy * dy;
                if d2.is_negligible() {
                    hwa::error!("[MotionPlanner] {}: R form requires distinct end point", mnemonic);
                    return Err(processing::CodeExecutionFailure::ERR);
                }
                let h2 = (r * r) / d2 - Real::from_lit(25, 2);
                let mut h = match h2.sqrt() {
                    Some(h) if !h2.is_negligible() && h2.is_positive() => h,
                    _ => math::ZERO,
                };
                if clockwise {
                    h = -h;
                }
                if r.is_positive() == false {
                    h = -h;
                }
                (
                    x0 + dx * math::HALF - h * dy,
                    y0 + dy * math::HALF + h * dx,
                )
            }
            (i, j, _) if i.is_some() || j.is_some() => {
                (x0 + i.unwrap_or(math::ZERO), y0 + j.unwrap_or(math::ZERO))
            }
            _ => {
                hwa::error!("[MotionPlanner] {}: missing I/J or R", mnemonic);
                return Err(processing::CodeExecutionFailure::ERR);
            }
        };

        let r0x = x0 - cx;
        let r0y = y0 - cy;
        let r1x = x1 - cx;
        let r1y = y1 - cy;
        let radius = match (r0x * r0x + r0y * r0y).sqrt() {
            Some(r) if !r.is_negligible() => r,
            _ => {
                hwa::error!("[MotionPlanner] {}: null radius", mnemonic);
                return Err(processing::CodeExecutionFailure::ERR);
            }
        };
        let two_pi = math::PI * math::TWO;
        // Angular travel (always positive, in the direction of the arc)
        let a0 = r0y.atan2(r0x);
        let a1 = r1y.atan2(r1x);
        let mut angular_travel = if clockwise { a0 - a1 } else { a1 - a0 };
        if !angular_travel.is_defined_positive() {
            angular_travel += two_pi;
        }
        // Same start and end point means a full circle
        if (x1 - x0).is_negligible()
            && (y1 - y0).is_negligible()
            && (angular_travel - two_pi).abs() < Real::from_lit(1, 3)
        {
            angular_travel = two_pi;
        }

        let arc_length = angular_travel * radius;
        // Chordal tolerance: a chord of length c has a sagitta R - √(R² - c²/4) <= tol for
        // c <= 2·√(2·R·tol - tol²). Taking c as the arc length of each segment is conservative
        // (the arc is longer than its chord)
        let tol = self.chord_tolerance();
        let max_len = if tol >= radius {
            ARC_SEGMENT_LENGTH_WU
        } else {
            let c = math::TWO
                * (math::TWO * radius * tol - tol * tol)
                    .sqrt()
                    .unwrap_or(ARC_SEGMENT_LENGTH_WU);
            if c < ARC_SEGMENT_LENGTH_WU { c } else { ARC_SEGMENT_LENGTH_WU }
        };
        let num_segments = (arc_length / max_len)
            .ceil()
            .int()
            .clamp(1, ARC_MAX_SEGMENTS);

        #[cfg(feature = "with-e-axis")]
        let (e_delta_total, e_target) = match arc.coords.e {
            Some(e) => {
                let logical_e = self.motion_status.get_logical_e();
                let target = match self.motion_status.is_e_relative() {
                    true => logical_e + e,
                    false => e,
                };
                (target - logical_e, Some(target))
            }
            None => (math::ZERO, None),
        };
        #[cfg(not(feature = "with-e-axis"))]
        let e_delta_total = math::ZERO;

        let n = Real::from_lit(num_segments, 0);
        let mut result = Ok(processing::CodeExecutionSuccess::OK);
        for segment in 1..=num_segments {
            let last = segment == num_segments;
            let fraction = Real::from_lit(segment, 0) / n;
            let (x, y) = if last {
                (x1, y1)
            } else {
                let angle = if clockwise {
                    a0 - angular_travel * fraction
                } else {
                    a0 + angular_travel * fraction
                };
                (cx + radius * angle.cos(), cy + radius * angle.sin())
            };
            #[allow(unused_mut)]
            let mut target_wu = TVector::new()
                .with_coord(CoordSel::X, Some(x))
                .with_coord(CoordSel::Y, Some(y));
            #[cfg(feature = "with-z-axis")]
            if arc.coords.z.is_some() {
                target_wu.set_coord(CoordSel::Z, Some(z0 + (z1 - z0) * fraction));
            }
            let p_from = self.motion_status.get_last_planned_position();
            result = self
                .schedule_displacement(
                    mnemonic,
                    if last { channel } else { hwa::CommChannel::Internal },
                    hwa::DeferAction::LinearMove,
                    &p_from,
                    &target_wu,
                    e_delta_total / n,
                    feed_rate,
                    // Intermediate segments must wait for room in the queue
                    true,
                    event_bus,
                    order_num,
                    line,
                )
                .await;
            if result.is_err() {
                break;
            }
        }
        #[cfg(feature = "with-e-axis")]
        if let (Ok(_), Some(e)) = (&result, e_target) {
            self.motion_status.set_logical_e(e);
        }
        result
    }

    /// Plans a cubic Bézier spline (G5) in the XY plane, from the current position to `X`/`Y`,
    /// with control points `start + (I, J)` and `end + (P, Q)`. Z and E (when given) are
    /// interpolated linearly with the curve parameter.
    ///
    /// The curve is approximated by linear segments of at most ~[ARC_SEGMENT_LENGTH_WU] world
    /// units (the length is bounded by the control polygon), shortened as needed so that the
    /// path stays within the chordal tolerance ([MotionPlanner::chord_tolerance]). As for arcs, intermediate
    /// segments are internal blocking moves: the issuer only gets the response of the last one.
    #[cfg(feature = "with-beziers")]
    async fn schedule_bezier(
        &self,
        channel: hwa::CommChannel,
        bz: &processing::BEZIER,
        event_bus: &hwa::types::EventBus,
        order_num: u32,
        line: Option<u32>,
    ) -> Result<processing::CodeExecutionSuccess, processing::CodeExecutionFailure> {
        use hwa::CoordSel;
        const MNEMONIC: &str = "G5";
        let p0_pos = self.motion_status.get_last_planned_position();
        if !p0_pos.is_set {
            return Err(processing::CodeExecutionFailure::HomingRequired);
        }
        if bz.i.is_none() && bz.j.is_none() || bz.p.is_none() && bz.q.is_none() {
            hwa::error!("[MotionPlanner] G5: missing I/J or P/Q");
            return Err(processing::CodeExecutionFailure::ERR);
        }
        let feed_rate = self.update_feed_rate(bz.coords.f);
        let absolute = self.motion_status.is_absolute_positioning();

        let start = p0_pos.world_pos;
        let x0 = start.get_coord(CoordSel::X).unwrap_or(math::ZERO);
        let y0 = start.get_coord(CoordSel::Y).unwrap_or(math::ZERO);
        let resolve = |value: Option<Real>, origin: Real| -> Real {
            match value {
                Some(v) if absolute => v,
                Some(v) => origin + v,
                None => origin,
            }
        };
        let x3 = resolve(bz.coords.x, x0);
        let y3 = resolve(bz.coords.y, y0);
        let x1 = x0 + bz.i.unwrap_or(math::ZERO);
        let y1 = y0 + bz.j.unwrap_or(math::ZERO);
        let x2 = x3 + bz.p.unwrap_or(math::ZERO);
        let y2 = y3 + bz.q.unwrap_or(math::ZERO);
        #[cfg(feature = "with-z-axis")]
        let z0 = start.get_coord(CoordSel::Z).unwrap_or(math::ZERO);
        #[cfg(feature = "with-z-axis")]
        let z1 = resolve(bz.coords.z, z0);

        // The control polygon length is an upper bound of the curve length
        let dist = |ax: Real, ay: Real, bx: Real, by: Real| {
            let (dx, dy) = (bx - ax, by - ay);
            (dx * dx + dy * dy).sqrt().unwrap_or(math::ZERO)
        };
        let polygon = dist(x0, y0, x1, y1) + dist(x1, y1, x2, y2) + dist(x2, y2, x3, y3);
        // Chordal tolerance. The polyline through n uniform parameter steps deviates at most
        // max|B''| / (8·n²) from the curve, and |B''| <= 6·max(|P0-2P1+P2|, |P1-2P2+P3|) = 6·M,
        // so n >= √(3·M / (4·tol)) keeps the deviation within tol
        let tol = self.chord_tolerance();
        let m = dist(math::ZERO, math::ZERO, x0 - math::TWO * x1 + x2, y0 - math::TWO * y1 + y2)
            .max(dist(math::ZERO, math::ZERO, x1 - math::TWO * x2 + x3, y1 - math::TWO * y2 + y3));
        let by_tolerance = (math::THREE * m / (math::FOUR * tol))
            .sqrt()
            .unwrap_or(math::ZERO)
            .ceil()
            .int();
        let by_length = (polygon / ARC_SEGMENT_LENGTH_WU).ceil().int();
        let num_segments = by_tolerance.max(by_length).clamp(1, ARC_MAX_SEGMENTS);

        #[cfg(feature = "with-e-axis")]
        let (e_delta_total, e_target) = match bz.coords.e {
            Some(e) => {
                let logical_e = self.motion_status.get_logical_e();
                let target = match self.motion_status.is_e_relative() {
                    true => logical_e + e,
                    false => e,
                };
                (target - logical_e, Some(target))
            }
            None => (math::ZERO, None),
        };
        #[cfg(not(feature = "with-e-axis"))]
        let e_delta_total = math::ZERO;

        let n = Real::from_lit(num_segments, 0);
        let mut result = Ok(processing::CodeExecutionSuccess::OK);
        for segment in 1..=num_segments {
            let last = segment == num_segments;
            let t = Real::from_lit(segment, 0) / n;
            let (x, y) = if last {
                (x3, y3)
            } else {
                // B(t) = (1-t)³·P0 + 3(1-t)²t·P1 + 3(1-t)t²·P2 + t³·P3
                let u = math::ONE - t;
                let b0 = u * u * u;
                let b1 = math::THREE * u * u * t;
                let b2 = math::THREE * u * t * t;
                let b3 = t * t * t;
                (
                    b0 * x0 + b1 * x1 + b2 * x2 + b3 * x3,
                    b0 * y0 + b1 * y1 + b2 * y2 + b3 * y3,
                )
            };
            #[allow(unused_mut)]
            let mut target_wu = TVector::new()
                .with_coord(CoordSel::X, Some(x))
                .with_coord(CoordSel::Y, Some(y));
            #[cfg(feature = "with-z-axis")]
            if bz.coords.z.is_some() {
                target_wu.set_coord(CoordSel::Z, Some(z0 + (z1 - z0) * t));
            }
            let p_from = self.motion_status.get_last_planned_position();
            result = self
                .schedule_displacement(
                    MNEMONIC,
                    if last {
                        channel
                    } else {
                        hwa::CommChannel::Internal
                    },
                    hwa::DeferAction::LinearMove,
                    &p_from,
                    &target_wu,
                    e_delta_total / n,
                    feed_rate,
                    // Intermediate segments must wait for room in the queue
                    true,
                    event_bus,
                    order_num,
                    line,
                )
                .await;
            if result.is_err() {
                break;
            }
        }
        #[cfg(feature = "with-e-axis")]
        if let (Ok(_), Some(e)) = (&result, e_target) {
            self.motion_status.set_logical_e(e);
        }
        result
    }

    /// Schedules the displacement from `p0_pos` to `target_wu` (only the set coordinates are
    /// considered) plus an extruder displacement of `e_delta` world units.
    ///
    /// `feed_rate_wu_s` applies to the path length of the motion axis, or to the extruder when
    /// it's an extruder-only move (standard GCode semantics). When `None`, the default travel
    /// speed is used.
    async fn schedule_displacement(
        &self,
        mnemonic: &'static str,
        channel: hwa::CommChannel,
        action: hwa::DeferAction,
        p0_pos: &hwa::controllers::Position,
        target_wu: &TVector<Real>,
        e_delta: Real,
        feed_rate_wu_s: Option<Real>,
        blocking: bool,
        event_bus: &hwa::types::EventBus,
        _order_num: u32,
        line: Option<u32>,
    ) -> Result<processing::CodeExecutionSuccess, processing::CodeExecutionFailure> {
        #[allow(unused_mut)]
        let mut p0_pos = *p0_pos;
        #[allow(unused_mut)]
        let mut p1_pos = p0_pos.complete_and_project(target_wu);
        #[allow(unused_mut)]
        let mut relevant_world_coords = target_wu.not_nan_coords();
        // The extruder position is always tracked as a displacement from zero
        #[cfg(feature = "with-e-axis")]
        {
            p0_pos.world_pos.set_coord(hwa::CoordSel::E, Some(math::ZERO));
            p0_pos.space_pos.set_coord(hwa::CoordSel::E, Some(math::ZERO));
            p1_pos.world_pos.set_coord(hwa::CoordSel::E, Some(e_delta));
            p1_pos.space_pos.set_coord(hwa::CoordSel::E, Some(e_delta));
            if !e_delta.is_zero() {
                relevant_world_coords = relevant_world_coords.union(hwa::CoordSel::E);
            }
        }
        #[cfg(not(feature = "with-e-axis"))]
        let _ = e_delta;
        let requested_motion_speed = feed_rate_wu_s;

        #[cfg(feature = "debug-motion")]
        hwa::info!(
            "[MotionPlanner] order_num:{:?} L:{:?} Received motion. relevant world coords: [{:?}] world: src [{:?}] {} dest: [{:?}] {}, absolute: {}, speed: {:?} {}/s",
            _order_num,
            line.unwrap_or(0),
            relevant_world_coords,
            p0_pos.world_pos.selecting(relevant_world_coords),
            Contract::WORLD_UNIT_MAGNITUDE,
            p1_pos.world_pos.selecting(relevant_world_coords),
            Contract::WORLD_UNIT_MAGNITUDE,
            self.motion_status.is_absolute_positioning(),
            requested_motion_speed,
            Contract::WORLD_UNIT_MAGNITUDE
        );

        // Boundaries
        let dts = self.motion_config.get_default_travel_speed();
        let min_speed = self.motion_config.get_min_speed();
        let max_speed = self.motion_config.get_max_speed();
        let max_feed_rate = self.motion_config.get_max_feed_rate();
        let max_accel = self.motion_config.get_max_accel();
        let max_jerk = self.motion_config.get_max_jerk();

        // Multipliers
        let flow_rate = self.motion_config.get_flow_rate_as_real();
        let speed_rate = self.motion_config.get_speed_rate_as_real();

        // Compute distance and decompose as unit vector and module.
        // When dist is zero, value is map to None (NaN).
        // In case o E dimension, flow rate factor is applied
        let ds = p1_pos.space_pos - p0_pos.space_pos;
        let relevant_space_coords = ds.not_negligible_coords();

        #[cfg(feature = "debug-motion")]
        hwa::info!(
            "[MotionPlanner] order_num:{:?} L:{:?} Relevant space coords: [{:#?}]",
            _order_num,
            line.unwrap_or(0),
            relevant_space_coords
        );

        #[cfg(feature = "debug-motion")]
        hwa::info!(
            "[MotionPlanner] order_num:{:?} L:{:?} Motion transformed to space [ src: [{:#?}] dest: [{:#?}] ] {}, speed: {:?} {}/s",
            _order_num,
            line.unwrap_or(0),
            p0_pos.space_pos.selecting(relevant_space_coords),
            p1_pos.space_pos.selecting(relevant_space_coords),
            Contract::SPACE_UNIT_MAGNITUDE,
            requested_motion_speed,
            Contract::SPACE_UNIT_MAGNITUDE,
        );

        let displacement = ds.map_values(|coord, coord_value| match coord {
            #[cfg(feature = "with-e-axis")]
            hwa::CoordSel::E => match coord_value.is_zero() {
                true => None,
                false => Some(coord_value * flow_rate),
            },
            _ => match coord_value.is_negligible() {
                true => None,
                false => Some(coord_value),
            },
        });
        let (unit_vector_dir, module_target_distance) = displacement.decompose_normal();

        // The feed rate refers to the path of the motion axis (extruder excluded), unless it's an
        // extruder-only move. Translate it to the speed along the full (axis + extruder) vector.
        let feed_rate = requested_motion_speed.unwrap_or(dts);
        let speed_module = match displacement
            .selecting(hwa::CoordSel::motion_relevant_axis())
            .norm2()
        {
            Some(axis_distance) if !axis_distance.is_negligible() => {
                feed_rate * module_target_distance / axis_distance
            }
            _ => feed_rate,
        };
        // Compute per-axis distance
        let speed_vector: TVector<Real> = unit_vector_dir.abs() * speed_module;
        // The rest (minimum) speed along the path: the minimum speeds of the axes that move, in the
        // direction of the move (the norm of every axis minimum would overestimate it, e.g. 14.3
        // instead of 10 mm/s for an X move with X/Y/Z minimums of 10/10/2 mm/s)
        let min_speed_module = (unit_vector_dir.abs() * min_speed)
            .norm2()
            .unwrap_or(math::ZERO);
        // Clamp per-axis target speed to the physical restrictions
        let speed_vector = (speed_vector * speed_rate)
            .clamp_higher_than(min_speed)
            .clamp_lower_than(max_speed)
            .clamp_lower_than(max_feed_rate);

        let module_target_speed = speed_vector.norm2().unwrap_or(math::ZERO);
        let module_target_accel = (unit_vector_dir.abs() * max_accel)
            .norm2()
            .unwrap_or(math::ZERO);
        let module_target_jerk = (unit_vector_dir.abs() * max_jerk)
            .norm2()
            .unwrap_or(math::ZERO);

        // Per-axis safety caps. The path limit L must satisfy |u_i|·L ≤ L_i for every moving axis,
        // i.e. L ≤ min_i(L_i / |u_i|). The norm-based values above are exact when all the axes share
        // the same limit, but over-estimate it when they differ (e.g. a slow Z axis in a diagonal
        // XZ move), making that axis exceed its limits (lost steps).
        let u_abs = unit_vector_dir.abs();
        let axis_cap = |limit: Real, per_axis: TVector<Real>| -> Real {
            match (per_axis / u_abs).vmin() {
                Some(cap) if cap > math::ZERO => limit.min(cap),
                _ => limit,
            }
        };
        let module_target_speed = axis_cap(axis_cap(module_target_speed, max_speed), max_feed_rate);
        let module_target_accel = axis_cap(module_target_accel, max_accel);
        let module_target_jerk = axis_cap(module_target_jerk, max_jerk);

        // Path acceleration limit (M204): print acceleration for extruding moves, travel otherwise
        #[cfg(feature = "with-e-axis")]
        let is_extruding = !e_delta.is_zero();
        #[cfg(not(feature = "with-e-axis"))]
        let is_extruding = false;
        let path_accel_limit = match is_extruding {
            true => self.motion_config.get_print_accel(),
            false => self.motion_config.get_travel_accel(),
        };
        let module_target_accel = match path_accel_limit {
            Some(limit) if limit.is_defined_positive() => module_target_accel.min(limit),
            _ => module_target_accel,
        };

        let move_result = if module_target_distance.is_negligible() {
            #[cfg(feature = "debug-motion")]
            hwa::info!(
                "[MotionPlanner] order_num:{:?} L:{:?} Discarding plan. src: [{:?}], dest: [{:?}], vmax: [ {:?}, [{:?}] ]",
                _order_num,
                line.unwrap_or(0),
                p0_pos.space_pos.selecting(relevant_space_coords),
                p1_pos.space_pos.selecting(relevant_space_coords),
                module_target_speed.rdp(3),
                speed_vector
            );
            Ok(processing::CodeExecutionSuccess::OK)
        } else if !module_target_speed.is_zero()
            && !module_target_accel.is_zero()
            && !module_target_jerk.is_zero()
        {
            // A segment starts and ends at rest, which can never be faster than its own target:
            // otherwise its (pinned) entry could exceed what it can brake from in its displacement
            let rest_speed = min_speed_module.min(module_target_speed);
            let segment_data = motion_control::Segment {
                speed_enter_su_s: rest_speed,
                speed_exit_su_s: rest_speed,
                #[cfg(feature = "with-motion-chaining")]
                accel_enter_su_s: math::ZERO,
                #[cfg(feature = "with-motion-chaining")]
                accel_exit_su_s: math::ZERO,
                #[cfg(feature = "with-motion-chaining")]
                t_1: math::ZERO,
                #[cfg(feature = "with-motion-chaining")]
                t_2: math::ZERO,
                #[cfg(feature = "with-motion-chaining")]
                t_3: math::ZERO,
                speed_target_su_s: module_target_speed,
                displacement_su: module_target_distance,
                speed_enter_constrained_su_s: module_target_speed,
                speed_exit_constrained_su_s: module_target_speed,
                #[cfg(feature = "with-motion-chaining")]
                proj_prev: Real::one(),
                unit_vector_dir,
                src_pos: p0_pos.space_pos.selecting(relevant_space_coords),
                dest_pos: p1_pos.space_pos.selecting(relevant_space_coords),
                dest_world_pos: p1_pos.world_pos.selecting(relevant_world_coords),
                tool_power: Real::zero(),
                constraints: motion::Constraints {
                    v_max: module_target_speed,
                    a_max: module_target_accel,
                    j_max: module_target_jerk,
                },
                #[cfg(feature = "with-motion-chaining")]
                proj_next: Real::one(),
            };

            let r = self
                .schedule_raw_move(
                    mnemonic,
                    channel,
                    action,
                    ScheduledMove::Move(segment_data, _order_num),
                    blocking,
                    event_bus,
                    _order_num,
                    line,
                )
                .await?;

            hwa::debug!(
                "speed: {:?} -> {:?} ",
                requested_motion_speed.unwrap_or(Real::zero()).rdp(4),
                module_target_speed.rdp(4)
            );
            hwa::debug!("speed_vector: {:?}", speed_vector.rdp(4));
            hwa::debug!("speed_rates: {:?}", speed_rate.rdp(4));
            hwa::debug!("speed_module: {:?}", module_target_speed.rdp(4));
            return Ok(r);
        } else {
            hwa::warn!(
                "[MotionPlanner] order_num:{:?} L:{:?} Bad plan. src: [{:?}], dest: [{:?}], vmax: [ {:?}, [{:?}] ]",
                _order_num,
                line.unwrap_or(0),
                p0_pos.space_pos.selecting(relevant_space_coords),
                p1_pos.space_pos.selecting(relevant_space_coords),
                module_target_speed,
                speed_vector
            );
            Err(processing::CodeExecutionFailure::ERR)
        };
        move_result
    }

    pub async fn do_homing(
        &self,
        order_num: u32,
        event_bus: &hwa::types::EventBus,
    ) -> Result<(), ()> {
        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] Locking for homming");

        let result = self
            .motion_driver
            .lock()
            .await
            .homing_action(&self.motion_config)
            .await;
        event_bus
            .publish_event(hwa::EventStatus::not_containing(hwa::EventFlags::HOMING))
            .await;
        result.map(|_| ()).map_err(|_| ())
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

#[cfg(test)]
pub mod motion_planner_test {

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
