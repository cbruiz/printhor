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
                panic!("cound not happen")
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
                                let v_1 = curr_segment.speed_exit_su_s;
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
                                        if curr_vmax < v_0 || curr_vmax < v_1 {
                                            //TODO: Study this case
                                            break;
                                        }
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
                                            hwa::debug!(
                                                "RingBuffer [{:?}, {:?}] chained: ({:?}) proj ({:?}) = ({:?})",
                                                prev_index,
                                                curr_insert_index,
                                                prev_segment.unit_vector_dir,
                                                curr_segment.unit_vector_dir,
                                                proj
                                            );
                                            hwa::debug!(
                                                "\ts : vi = [{:?} < {:?}] - vtarget = {:?} - vo = [{:?} < {:?}]:",
                                                prev_segment.speed_enter_su_s,
                                                prev_segment.speed_enter_constrained_su_s,
                                                prev_segment.speed_target_su_s,
                                                prev_segment.speed_exit_su_s,
                                                prev_segment.speed_exit_constrained_su_s,
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
                                "[MotionPlanner] order_num:{:?} L:{:?} MotionPlan queued. space src: [{:#?}], dest: [{:#?}], vmax: [ {:?}, [{:?}] ]",
                                order_num,
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
                            let pos = hwa::controllers::Position::new_from(
                                Contract::DEFAULT_WORLD_HOMING_POINT_WU,
                                Contract
                                    .project_to_space(&Contract::DEFAULT_WORLD_HOMING_POINT_WU)
                                    .unwrap(),
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
                                hwa::EventStatus::containing(hwa::EventFlags::MOV_QUEUE_EMPTY),
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
                    #[cfg(feature = "cornering")]
                    if do_adjust {
                        let _ = crate::motion::perform_motion_chaining(rb);
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
            processing::GCodeValue::G4(t) => Ok(self
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
                let mut position = self.motion_status.get_last_planned_position();
                position.update_from_world_coordinates(&hwa::math::TVector::from(t.into()));
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

    async fn schedule_move(
        &self,
        mnemonic: &'static str,
        channel: hwa::CommChannel,
        action: hwa::DeferAction,
        p1_wu: TVector<Real>,
        requested_motion_speed: Option<Real>,
        blocking: bool,
        event_bus: &hwa::types::EventBus,
        _order_num: u32,
        line: Option<u32>,
    ) -> Result<processing::CodeExecutionSuccess, processing::CodeExecutionFailure> {
        let p0_pos = self.motion_status.get_last_planned_position();
        let relevant_world_coords = p1_wu.not_nan_coords();

        if !p0_pos.is_set {
            return Err(processing::CodeExecutionFailure::HomingRequired);
        }

        #[cfg(feature = "debug-motion")]
        hwa::info!(
            "[MotionPlanner] order_num:{:?} L:{:?} Received motion. relevant world coords: [{:?}] world: src [{:?}] {} dest: [{:?}] {}, absolute: {}, speed: {:?} {}/s",
            _order_num,
            line.unwrap_or(0),
            relevant_world_coords,
            p0_pos.world_pos.selecting(relevant_world_coords),
            Contract::WORLD_UNIT_MAGNITUDE,
            p1_wu,
            Contract::WORLD_UNIT_MAGNITUDE,
            self.motion_status.is_absolute_positioning(),
            requested_motion_speed,
            Contract::WORLD_UNIT_MAGNITUDE
        );

        let p1_pos = {
            if self.motion_status.is_absolute_positioning() {
                p0_pos.complete_and_project(&p1_wu)
            } else {
                p0_pos.complete_and_project(&(p0_pos.world_pos + p1_wu))
            }
        };

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

        let (unit_vector_dir, module_target_distance) = ds
            .map_values(|coord, coord_value| match coord {
                #[cfg(feature = "with-e-axis")]
                hwa::CoordSel::E => match coord_value.is_zero() {
                    true => None,
                    false => Some(coord_value * flow_rate),
                },
                _ => match coord_value.is_negligible() {
                    true => None,
                    false => Some(coord_value),
                },
            })
            .decompose_normal();

        // Compute the speed module applying speed_rate factor
        let speed_module = requested_motion_speed.unwrap_or(dts);
        // Compute per-axis distance
        let speed_vector: TVector<Real> = unit_vector_dir.abs() * speed_module;
        let min_speed_module = min_speed.norm2().unwrap_or(math::ZERO);
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
            let segment_data = motion_control::Segment {
                speed_enter_su_s: min_speed_module,
                speed_exit_su_s: min_speed_module,
                speed_target_su_s: module_target_speed,
                displacement_su: module_target_distance,
                speed_enter_constrained_su_s: min_speed_module,
                speed_exit_constrained_su_s: min_speed_module,
                proj_prev: Real::zero(),
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
                proj_next: Real::zero(),
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
        match move_result {
            Ok(resp) => Ok(resp),
            Err(err) => match err {
                processing::CodeExecutionFailure::BUSY => {
                    todo!("Delegate to deferrals!!!")
                }
                _r => {
                    unreachable!("bad response: {:?}", _r);
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
        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] Locking for homming");

        match self
            .motion_driver
            .lock()
            .await
            .homing_action(&self.motion_config)
            .await
        {
            Ok(_pos) => {}
            Err(_pos) => {
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
