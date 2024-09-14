//! This feature is being stabilized
use crate::control;
use crate::hwa;
use crate::math;
use crate::math::Real;
use crate::sync::config::Config;
use crate::tgeo::CoordSel;
use crate::tgeo::TVector;
use embassy_sync::mutex::{Mutex, MutexGuard};

use crate::hwa::controllers::motion;
use crate::hwa::drivers::motion_driver::MotionDriverRef;

pub enum ScheduledMove {
    Move(motion::SegmentData),
    Homing,
    Dwell,
}

/////

pub struct MotionPlanner {
    //pub event_bus: EventBusRef,
    // The channel to send deferred events
    pub defer_channel: hwa::DeferChannelRef,

    ringbuffer: Mutex<hwa::ControllerMutexType, RingBuffer>,
    move_planned: Config<hwa::ControllerMutexType, bool>,
    available: Config<hwa::ControllerMutexType, bool>,
    motion_config: motion::MotionConfigRef,
    motion_st: Mutex<hwa::ControllerMutexType, motion::MotionStatus>,
    pub motion_driver: MotionDriverRef,
}

// TODO: Refactor in progress
#[allow(unused)]
impl MotionPlanner {
    pub const fn new(
        defer_channel: hwa::DeferChannelRef,
        motion_config: motion::MotionConfigRef,
        motion_driver: MotionDriverRef,
    ) -> Self {
        Self {
            //event_bus,
            defer_channel,
            motion_config,
            ringbuffer: Mutex::new(RingBuffer::new()),
            move_planned: Config::new(),
            available: Config::new(),
            motion_st: Mutex::new(motion::MotionStatus::new()),
            motion_driver,
        }
    }

    pub async fn start(&self, event_bus: &hwa::EventBusRef) {
        self.move_planned.reset();
        self.available.signal(true);
        event_bus
            .publish_event(hwa::EventStatus::containing(
                hwa::EventFlags::MOV_QUEUE_EMPTY,
            ))
            .await;
    }

    // Dequeues actual velocity plan
    pub async fn get_current_segment_data(
        &self,
        event_bus: &hwa::EventBusRef,
    ) -> Option<(motion::Segment, hwa::CommChannel)> {
        loop {
            // FIXME: wrong with timeout
            let _ = self.move_planned.wait().await;
            let mut do_dwell = false;
            {
                let mut rb = self.ringbuffer.lock().await;
                let head = rb.head as usize;
                match rb.data[head] {
                    PlanEntry::Empty => {
                        self.move_planned.reset();
                    }
                    PlanEntry::Dwell(channel, _deferred) => {
                        rb.data[head] = PlanEntry::Executing(MovType::Dwell(channel), true);
                        do_dwell = true;
                    }
                    PlanEntry::PlannedMove(planned_data, action, channel, deferred) => {
                        hwa::debug!(
                            "Exec starting: {} / {} h={}",
                            rb.used,
                            hwa::SEGMENT_QUEUE_SIZE,
                            head
                        );
                        rb.data[head] =
                            PlanEntry::Executing(MovType::Move(action, channel), deferred);
                        return Some((planned_data, channel));
                    }
                    PlanEntry::Homing(channel, _deferred) => {
                        event_bus
                            .publish_event(hwa::EventStatus::containing(hwa::EventFlags::HOMMING))
                            .await;
                        rb.data[head] = PlanEntry::Executing(MovType::Homing(channel), true);
                        return None;
                    }
                    PlanEntry::Executing(_, _) => {
                        self.move_planned.reset();
                        hwa::error!("Unexpected error: RingBuffer Overrun");
                    }
                }
            }
            if do_dwell {
                unreachable!("suspecting not OK");
                //self.consume_current_segment_data(&event_bus).await;
            }
        }
    }

    pub async fn consume_current_segment_data(&self, event_bus: &hwa::EventBusRef) -> u8 {
        let mut rb = self.ringbuffer.lock().await;
        let head = rb.head;
        hwa::debug!("Movement completed @rq[{}] (ongoing={})", head, rb.used - 1);
        match &rb.data[head as usize] {
            PlanEntry::Executing(MovType::Homing(channel), _) => {
                event_bus
                    .publish_event(hwa::EventStatus::not_containing(hwa::EventFlags::HOMMING))
                    .await;
                self.defer_channel
                    .send(hwa::DeferEvent::Completed(
                        hwa::DeferAction::Homing,
                        *channel,
                    ))
                    .await;
            }
            PlanEntry::Executing(MovType::Dwell(channel), _) => {
                self.defer_channel
                    .send(hwa::DeferEvent::Completed(
                        hwa::DeferAction::Homing,
                        *channel,
                    ))
                    .await;
            }
            PlanEntry::Executing(MovType::Move(action, channel), deferred) => {
                if *deferred {
                    self.defer_channel
                        .send(hwa::DeferEvent::Completed(*action, *channel))
                        .await;
                }
            }
            _ => {
                panic!("cound not happen")
            }
        }
        rb.data[head as usize] = PlanEntry::Empty;
        rb.head = match head + 1 < hwa::SEGMENT_QUEUE_SIZE {
            true => head + 1,
            false => 0u8,
        };
        rb.used -= 1;
        hwa::debug!("- used={}, h={} ", rb.used, head);
        if rb.used == 0 {
            event_bus
                .publish_event(hwa::EventStatus::containing(
                    hwa::EventFlags::MOV_QUEUE_EMPTY,
                ))
                .await;
        }
        self.available.signal(true);
        rb.used
    }

    pub async fn schedule_raw_move(
        &self,
        channel: hwa::CommChannel,
        action: hwa::DeferAction,
        move_type: ScheduledMove,
        blocking: bool,
        event_bus: &hwa::EventBusRef,
    ) -> Result<control::CodeExecutionSuccess, control::CodeExecutionFailure> {
        hwa::debug!("schedule_raw_move() BEGIN");
        loop {
            self.available.wait().await;
            {
                let mut rb = self.ringbuffer.lock().await;
                let mut is_defer = rb.used == hwa::SEGMENT_QUEUE_SIZE - 1;

                if rb.used < hwa::SEGMENT_QUEUE_SIZE {
                    #[cfg(feature = "cornering")]
                    let mut do_adjust = false;

                    let curr_insert_index = rb.index_from_tail(0).unwrap();

                    let (curr_planned_entry, event) = match move_type {
                        ScheduledMove::Move(curr_segment_data) => {
                            let mut curr_segment = motion::Segment::new(curr_segment_data);
                            // TODO:
                            // 1) Find a better way to approximate
                            // 2) Move outside ringbuffer lock
                            // solving sqrt(((abs(x-v_0))/jmax)) > q_1  for x
                            // x < v_0 - jmax q_1^2
                            // x > jmax * q_1^2 + v_0

                            let mut curr_vmax = curr_segment.segment_data.speed_target_mms;
                            #[cfg(feature = "cornering")]
                            {
                                let q_1 = curr_segment.segment_data.displacement_mm;
                                let v_0 = curr_segment.segment_data.speed_enter_mms;
                                let t_jmax = curr_segment.segment_data.constraints.a_max
                                    / curr_segment.segment_data.constraints.j_max;

                                loop {
                                    let t_jstar = Real::vmin(
                                        (((curr_vmax - v_0).abs())
                                            / curr_segment.segment_data.constraints.j_max)
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
                                                    / curr_segment.segment_data.constraints.a_max))
                                    };

                                    if q_1 >= q_lim {
                                        break;
                                    } else {
                                        curr_vmax *= Real::from_f32(0.5);
                                    }
                                }
                            }

                            let curr_boundary =
                                curr_vmax.min(curr_segment.segment_data.constraints.v_max);
                            let v_marginal_gain =
                                curr_boundary - curr_segment.segment_data.speed_exit_mms;
                            hwa::debug!("\t- constraint = {} bound = {}", curr_vmax, curr_boundary);
                            hwa::debug!("\t= v_margin: {}", v_marginal_gain);
                            curr_segment.segment_data.speed_enter_constrained_mms = v_marginal_gain;
                            curr_segment.segment_data.speed_exit_constrained_mms = v_marginal_gain;

                            hwa::debug!(
                                "s: vi = [{} < {}] - vtarget = {} - vo = [{} < {}] / d = {}",
                                curr_segment.segment_data.speed_enter_mms,
                                curr_segment.segment_data.speed_enter_constrained_mms,
                                curr_segment.segment_data.speed_target_mms,
                                curr_segment.segment_data.speed_exit_constrained_mms,
                                curr_segment.segment_data.speed_exit_mms,
                                curr_segment.segment_data.displacement_mm
                            );

                            hwa::debug!("\t= max_gain: {}", v_marginal_gain);

                            if let Ok(prev_index) = rb.index_from_tail(1) {
                                match &mut rb.data[prev_index as usize] {
                                    PlanEntry::PlannedMove(prev_segment, _, _, _) => {
                                        let proj: Real = prev_segment
                                            .segment_data
                                            .vdir
                                            .orthogonal_projection(curr_segment.segment_data.vdir);
                                        if proj.is_defined_positive() {
                                            hwa::debug!("RingBuffer [{}, {}] chained: ({}) proj ({}) = ({})", prev_index, curr_insert_index,
                                                prev_segment.segment_data.vdir, curr_segment.segment_data.vdir, proj
                                            );
                                            hwa::debug!("\ts : vi = [{} < {}] - vtarget = {} - vo = [{} < {}]:",
                                                prev_segment.segment_data.speed_enter_mms, prev_segment.segment_data.speed_enter_constrained_mms,
                                                prev_segment.segment_data.speed_target_mms,
                                                prev_segment.segment_data.speed_exit_mms, prev_segment.segment_data.speed_exit_constrained_mms,
                                            );
                                            hwa::debug!("\t\tproj = {}", proj);
                                            cfg_if::cfg_if! {
                                                if #[cfg(feature = "cornering")] {
                                                    do_adjust = true;
                                                }
                                            }
                                            prev_segment.segment_data.proj_next = proj;
                                            curr_segment.segment_data.proj_prev = proj;
                                        }
                                    }
                                    _ => {}
                                }
                            }

                            self.update_last_planned_pos(&curr_segment.segment_data.dest_pos)
                                .await;
                            (
                                PlanEntry::PlannedMove(curr_segment, action, channel, is_defer),
                                hwa::EventStatus::new(),
                            )
                        }
                        ScheduledMove::Homing => {
                            is_defer = true;
                            (
                                PlanEntry::Homing(channel, is_defer),
                                hwa::EventStatus::not_containing(hwa::EventFlags::HOMMING),
                            )
                        }
                        ScheduledMove::Dwell => {
                            is_defer = true;
                            (
                                PlanEntry::Dwell(channel, is_defer),
                                hwa::EventStatus::containing(hwa::EventFlags::MOV_QUEUE_EMPTY),
                            )
                        }
                    };

                    hwa::debug!(
                        "Mov queued @{} ({} / {})",
                        curr_insert_index,
                        rb.used + 1,
                        hwa::SEGMENT_QUEUE_SIZE
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
                        self.defer_channel
                            .send(hwa::DeferEvent::AwaitRequested(action, channel))
                            .await;
                        Ok(control::CodeExecutionSuccess::DEFERRED(event))
                    } else {
                        hwa::debug!("schedule_raw_move() END - Finally queued");
                        Ok(control::CodeExecutionSuccess::QUEUED)
                    }
                } else {
                    self.available.reset();
                    if !blocking {
                        hwa::warn!(
                            "Mov rejected: {} / {} h={}",
                            rb.used,
                            hwa::SEGMENT_QUEUE_SIZE,
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

    pub fn motion_cfg(&self) -> motion::MotionConfigRef {
        self.motion_config.clone()
    }

    pub fn motion_driver(&self) -> MotionDriverRef {
        self.motion_driver.clone()
    }

    pub async fn set_absolute_positioning(&self, absolute_is_set: bool) {
        let mut st = self.motion_st.lock().await;
        st.absolute_positioning = absolute_is_set;
    }

    pub async fn is_absolute_positioning(&self) -> bool {
        self.motion_st.lock().await.absolute_positioning
    }

    /// Positioning

    pub async fn get_last_planned_pos(&self) -> Option<TVector<Real>> {
        self.motion_st.lock().await.last_planned_pos.clone()
    }
    pub async fn set_last_planned_pos(&self, pos: &TVector<Real>) {
        let mut mg = self.motion_st.lock().await;
        let p = mg.last_planned_pos.unwrap_or(TVector::zero());
        mg.last_planned_pos.replace(p.apply(pos));
    }

    pub async fn get_last_planned_real_pos(&self) -> Option<TVector<Real>> {
        self.motion_st.lock().await.last_real_pos.clone()
    }
    pub async fn set_last_planned_real_pos(&self, pos: &TVector<Real>) {
        self.motion_st
            .lock()
            .await
            .last_real_pos
            .replace(pos.map_nan(&crate::math::ZERO));
    }

    /***
    Update last planned position. E is always ignored (So far, only relative E moves are implemented)
     */
    pub async fn update_last_planned_pos(&self, updated_position_coords: &TVector<Real>) {
        let mut stg = self.motion_st.lock().await;
        if let Some(last_position) = &mut stg.last_planned_pos {
            last_position.assign_if_set(CoordSel::XYZ, updated_position_coords);
        }
    }

    pub async fn set_flow_rate(&self, rate: u8) {
        self.motion_config.lock().await.flow_rate = rate;
    }

    pub async fn get_flow_rate_as_real(&self) -> Real {
        Real::new(self.motion_config.lock().await.flow_rate as i64, 0) / math::ONE_HUNDRED
    }

    pub async fn set_speed_rate(&self, rate: u8) {
        self.motion_config.lock().await.speed_rate = rate;
    }

    pub async fn get_speed_rate_as_real(&self) -> Real {
        Real::new(self.motion_config.lock().await.speed_rate as i64, 0) / math::ONE_HUNDRED
    }

    pub async fn get_default_travel_speed(&self) -> u16 {
        self.motion_config.lock().await.default_travel_speed
    }
    pub async fn set_default_travel_speed(&self, speed: u16) {
        self.motion_config.lock().await.default_travel_speed = speed;
    }

    pub async fn set_steps_per_mm(&self, x_spm: Real, y_spm: Real, z_spm: Real, e_spm: Real) {
        self.motion_config.lock().await.units_per_mm =
            TVector::from_coords(Some(x_spm), Some(y_spm), Some(z_spm), Some(e_spm));
    }

    pub async fn get_steps_per_mm_as_vector(&self) -> TVector<Real> {
        self.motion_config.lock().await.units_per_mm
    }

    pub async fn set_usteps(&self, x_usteps: u8, y_usteps: u8, z_usteps: u8, e_usteps: u8) {
        self.motion_config.lock().await.usteps = [
            x_usteps.into(),
            y_usteps.into(),
            z_usteps.into(),
            e_usteps.into(),
        ];
    }

    pub async fn set_machine_bounds(&self, x: u16, y: u16, z: u16) {
        self.motion_config.lock().await.machine_bounds = TVector::from_coords(
            Some(Real::new(x.into(), 0)),
            Some(Real::new(y.into(), 0)),
            Some(Real::new(z.into(), 0)),
            None,
        );
    }

    pub async fn get_usteps_as_vector(&self) -> TVector<Real> {
        self.motion_config.lock().await.get_usteps_as_vector()
    }

    pub async fn get_default_travel_speed_as_real(&self) -> Real {
        Real::new(
            self.motion_config.lock().await.default_travel_speed as i64,
            0,
        )
    }

    pub async fn get_max_accel(&self) -> TVector<u32> {
        self.motion_config.lock().await.max_accel
    }

    pub async fn get_max_speed(&self) -> TVector<u32> {
        self.motion_config.lock().await.max_speed
    }
    pub async fn set_max_speed(&self, speed: TVector<u32>) {
        self.motion_config
            .lock()
            .await
            .max_speed
            .assign(CoordSel::all(), &speed);
    }

    pub fn mc_set_max_speed(
        &self,
        mutex_guard: &mut MutexGuard<motion::MotionConfigMutexType, motion::MotionConfig>,
        speed: TVector<u32>,
    ) {
        mutex_guard.max_speed.assign(CoordSel::all(), &speed);
    }

    pub async fn get_max_speed_as_vreal(&self) -> TVector<Real> {
        self.motion_config
            .lock()
            .await
            .max_speed
            .map_coords(|c| Some(Real::new(c as i64, 0)))
    }

    pub async fn set_max_accel(&self, accel: TVector<u32>) {
        self.motion_config
            .lock()
            .await
            .max_accel
            .assign(CoordSel::all(), &accel);
    }

    pub async fn get_max_accel_as_vreal(&self) -> TVector<Real> {
        self.motion_config
            .lock()
            .await
            .max_accel
            .map_coords(|c| Some(Real::new(c as i64, 0)))
    }

    pub async fn set_max_jerk(&self, jerk: TVector<u32>) {
        self.motion_config
            .lock()
            .await
            .max_jerk
            .assign(CoordSel::all(), &jerk);
    }

    pub async fn plan(
        &self,
        channel: hwa::CommChannel,
        gc: &control::GCode,
        blocking: bool,
        event_bus: &hwa::EventBusRef,
    ) -> Result<control::CodeExecutionSuccess, control::CodeExecutionFailure> {
        match gc {
            control::GCode::G0(t) => Ok(self
                .schedule_move(
                    channel,
                    hwa::DeferAction::RapidMove,
                    TVector {
                        x: t.x,
                        y: t.y,
                        z: t.z,
                        e: None,
                    },
                    t.f,
                    blocking,
                    event_bus,
                )
                .await?),
            control::GCode::G1(t) => Ok(self
                .schedule_move(
                    channel,
                    hwa::DeferAction::LinearMove,
                    TVector {
                        x: t.x,
                        y: t.y,
                        z: t.z,
                        e: t.e,
                    },
                    t.f,
                    blocking,
                    event_bus,
                )
                .await?),
            control::GCode::G4 => Ok(self
                .schedule_raw_move(
                    channel,
                    hwa::DeferAction::Dwell,
                    ScheduledMove::Dwell,
                    blocking,
                    event_bus,
                )
                .await?),
            control::GCode::G28(_x) => {
                event_bus
                    .publish_event(hwa::EventStatus::containing(hwa::EventFlags::HOMMING))
                    .await;
                Ok(self
                    .schedule_raw_move(
                        channel,
                        hwa::DeferAction::Homing,
                        ScheduledMove::Homing,
                        blocking,
                        event_bus,
                    )
                    .await?)
            }
            control::GCode::G29 => Ok(control::CodeExecutionSuccess::OK),
            control::GCode::G29_1 => Ok(control::CodeExecutionSuccess::OK),
            control::GCode::G29_2 => Ok(control::CodeExecutionSuccess::OK),
            _ => Err(control::CodeExecutionFailure::NotYetImplemented),
        }
    }

    async fn schedule_move(
        &self,
        channel: hwa::CommChannel,
        action: hwa::DeferAction,
        p1_t: TVector<Real>,
        requested_motion_speed: Option<Real>,
        blocking: bool,
        event_bus: &hwa::EventBusRef,
    ) -> Result<control::CodeExecutionSuccess, control::CodeExecutionFailure> {
        let p0 = self
            .get_last_planned_pos()
            .await
            .ok_or(control::CodeExecutionFailure::HomingRequired)?;
        let _pdest = if self.is_absolute_positioning().await {
            p1_t
        } else {
            p0 + p1_t
        };

        let steps_per_unit =
            self.get_steps_per_mm_as_vector().await * self.get_usteps_as_vector().await;
        let rounded_pos: TVector<Real> = ((_pdest - p0) * steps_per_unit).ceil() / steps_per_unit;

        let p1 = p0 + rounded_pos;
        hwa::debug!("p1 [{}] -> [{}]", p1_t, p1);
        hwa::debug!("P_POS: {} d: {}", p1, p1 - p0);

        let cfg = self.motion_cfg();
        let cfg_g = cfg.lock().await;
        //----
        let dts = Real::from_lit(cfg_g.default_travel_speed as i64, 0);
        let flow_rate = Real::from_lit(cfg_g.flow_rate as i64, 0) / math::ONE_HUNDRED;
        let speed_rate = Real::from_lit(cfg_g.speed_rate as i64, 0) / math::ONE_HUNDRED;
        let max_speed = cfg_g
            .max_speed
            .map_coords(|c| Some(Real::from_lit(c as i64, 0)));
        let max_accel = cfg_g
            .max_accel
            .map_coords(|c| Some(Real::from_lit(c as i64, 0)));
        let max_jerk = cfg_g
            .max_jerk
            .map_coords(|c| Some(Real::from_lit(c as i64, 0)));
        //----
        drop(cfg_g);

        // Compute distance and decompose as unit vector and module.
        // When dist is zero, value is map to None (NaN).
        // In case o E dimension, flow rate factor is applied
        let ds = p1 - p0;
        let (vdir, module_target_distance) = ds
            .map_coord(CoordSel::all(), |coord_value, coord_idx| match coord_idx {
                CoordSel::X | CoordSel::Y | CoordSel::Z => match coord_value.is_zero() {
                    true => None,
                    false => Some(coord_value),
                },
                CoordSel::E => match coord_value.is_zero() {
                    true => None,
                    false => Some(coord_value * flow_rate),
                },
                _ => None,
            })
            .decompose_normal();

        // Compute the speed module applying speed_rate factor
        let speed_module = requested_motion_speed.unwrap_or(dts);
        // Compute per-axis distance
        let disp_vector: TVector<Real> = vdir.abs() * speed_module;
        // Clamp per-axis target speed to the physical restrictions
        let clamped_speed = disp_vector.clamp(max_speed);

        // Finally, per-axis relative speed
        let speed_vector = clamped_speed * speed_rate;

        let module_target_speed = speed_vector.norm2().unwrap_or(math::ZERO);
        let module_target_accel = (vdir.abs() * max_accel).norm2().unwrap_or(math::ZERO);
        let module_target_jerk = (vdir.abs() * max_jerk).norm2().unwrap_or(math::ZERO);

        let move_result = if module_target_distance.is_negligible() {
            Ok(control::CodeExecutionSuccess::OK)
        } else if !module_target_speed.is_zero()
            && !module_target_accel.is_zero()
            && !module_target_jerk.is_zero()
        {
            let segment_data = motion::SegmentData {
                speed_enter_mms: Real::zero(),
                speed_exit_mms: Real::zero(),
                speed_target_mms: module_target_speed,
                displacement_mm: module_target_distance,
                speed_enter_constrained_mms: Real::zero(),
                speed_exit_constrained_mms: Real::zero(),
                proj_prev: Real::zero(),
                //speed_max_gain_mms: Real::zero(),
                vdir,
                dest_pos: p1,
                tool_power: Real::zero(),
                constraints: control::motion_planning::Constraints {
                    v_max: module_target_speed,
                    a_max: module_target_accel,
                    j_max: module_target_jerk,
                },
                proj_next: Real::zero(),
            };

            let r = self
                .schedule_raw_move(
                    channel,
                    action,
                    ScheduledMove::Move(segment_data),
                    blocking,
                    event_bus,
                )
                .await?;

            hwa::debug!(
                "speed: {} -> {} ",
                requested_motion_speed.unwrap_or(Real::zero()).rdp(4),
                module_target_speed.rdp(4)
            );
            hwa::debug!("speed_vector: {}", speed_vector.rdp(4));
            hwa::debug!("clamped_speed: {}", clamped_speed.rdp(4));
            hwa::debug!("speed_rates: {}", speed_rate.rdp(4));
            hwa::debug!("speed_module: {}", module_target_speed.rdp(4));

            return Ok(r);
        } else {
            hwa::warn!("Incomplete move");
            Ok(control::CodeExecutionSuccess::OK)
        };
        match move_result {
            Ok(resp) => Ok(resp),
            Err(err) => match err {
                control::CodeExecutionFailure::BUSY => {
                    todo!("Delegate to deferrals!!!")
                }
                _ => Err(err),
            },
        }
    }

    pub async fn do_homing(&self, event_bus: &hwa::EventBusRef) -> Result<(), ()> {
        match self
            .motion_driver
            .lock()
            .await
            .homing_action(&self.motion_config)
            .await
        {
            Ok(_pos) => {
                self.set_last_planned_pos(&_pos).await;
            }
            Err(_pos) => {
                hwa::error!("Unable to complete homming. [Not yet] Raising SYS_ALARM");
                self.set_last_planned_pos(&_pos).await;
                //self.event_bus.publish_event(EventStatus::containing(EventFlags::SYS_ALARM)).await;
                // return Err(())
            }
        };
        event_bus
            .publish_event(hwa::EventStatus::not_containing(hwa::EventFlags::HOMMING))
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

#[cfg(feature = "cornering")]
fn perform_cornering(mut rb: MutexGuard<hwa::ControllerMutexType, RingBuffer>) -> Result<(), ()> {
    let mut left_offset = 2;
    let mut left_watermark = math::ZERO;
    let mut right_watermark = math::ZERO;
    // Assuming queued items are computed left to right:
    // First, locate the top left segment. That one with null projection
    // Also, set
    for index in 2..rb.used {
        match rb.planned_segment_from_tail(index) {
            Ok(prev_segment_candidate) => {
                if prev_segment_candidate
                    .segment_data
                    .proj_next
                    .is_defined_positive()
                {
                    left_offset = index;
                    left_watermark = prev_segment_candidate.segment_data.speed_enter_mms;
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

            mid_segment.segment_data.speed_enter_mms = left_watermark;
            mid_segment.segment_data.speed_exit_mms = right_watermark;
            break;
        } else {
            let (left_segment, right_segment) =
                match rb.entries_from_tail(left_offset, right_offset) {
                    (
                        Some(PlanEntry::PlannedMove(_s, _, _, _)),
                        Some(PlanEntry::PlannedMove(_t, _, _, _)),
                    ) => (_s, _t),
                    _ => panic!(""),
                };

            hwa::trace!("\tleft [{}] right[{}]", left_offset, right_offset);

            let left_max_inc = (left_segment.segment_data.proj_next
                * left_segment.segment_data.speed_target_mms)
                .min(left_segment.segment_data.speed_exit_constrained_mms);

            let right_max_inc = (right_segment.segment_data.proj_prev
                * right_segment.segment_data.speed_target_mms)
                .min(right_segment.segment_data.speed_enter_constrained_mms);

            let water_left =
                (left_watermark + left_max_inc).min(left_segment.segment_data.speed_target_mms);
            let water_right =
                (right_watermark + right_max_inc).min(right_segment.segment_data.speed_target_mms);

            if water_left <= water_right {
                // Flood at right
                hwa::trace!("flood to right  [{} {}]", water_left, water_right);
                left_segment.segment_data.speed_enter_mms = left_watermark;
                left_segment.segment_data.speed_exit_mms = water_left;
                right_segment.segment_data.speed_enter_mms = water_left;
                right_segment.segment_data.speed_exit_mms = right_watermark;
                left_watermark = water_left;
                left_offset -= 1;
            } else {
                // Flood at left
                hwa::trace!("flood to left [{} {}]", water_left, water_right);
                left_segment.segment_data.speed_enter_mms = left_watermark;
                left_segment.segment_data.speed_exit_mms = water_right;
                right_segment.segment_data.speed_enter_mms = water_right;
                right_segment.segment_data.speed_exit_mms = right_watermark;
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
    rb: &MutexGuard<hwa::ControllerMutexType, RingBuffer>,
    left_offset: u8,
    right_offset: u8,
) -> Result<(), ()> {
    let mut stb = Vec::new();
    for i in 0..left_offset - right_offset + 1 {
        let s = rb.planned_segment_from_tail(left_offset - i).unwrap();
        stb.push(format!(
            "{}[{},{}]",
            s.id, s.segment_data.speed_enter_mms, s.segment_data.speed_exit_mms
        ))
    }

    hwa::debug!(": {}", stb.join(" "));
    Ok(())
}

#[allow(unused)]
pub struct RingBuffer {
    pub(self) data: [PlanEntry; hwa::SEGMENT_QUEUE_SIZE as usize],
    pub(self) head: u8,
    pub(self) used: u8,
}

impl RingBuffer {
    pub const fn new() -> Self {
        Self {
            data: [PlanEntry::Empty; hwa::SEGMENT_QUEUE_SIZE as usize],
            head: 0,
            used: 0,
        }
    }

    /// A proper helper to get the index of relative offset starting from tail in reverse order.
    /// Example:
    /// * index_from_tail(0) returns the index of tail position
    /// * index_from_tail(1) returns the index of last inserted element
    pub fn index_from_tail(&self, offset: u8) -> Result<u8, ()> {
        if offset > self.used {
            Err(())
        } else {
            let absolute_offset = self.head as u16 + self.used as u16 - offset as u16;
            let len = self.data.len() as u16;
            match absolute_offset < len {
                true => Ok(absolute_offset as u8),
                false => Ok((absolute_offset - len) as u8),
            }
        }
    }

    #[allow(unused)]
    pub fn entry_from_tail(&self, offset: u8) -> Option<&PlanEntry> {
        let absolute_offset = self.head as u16 + self.used as u16 - offset as u16;
        let len = self.data.len() as u16;

        let index = match absolute_offset < len {
            true => absolute_offset as u8,
            false => (absolute_offset - len) as u8,
        };
        self.data.get(index as usize)
    }
    #[allow(unused)]
    pub fn mut_entry_from_tail(&mut self, offset: u8) -> Option<&mut PlanEntry> {
        let absolute_offset = self.head as u16 + self.used as u16 - offset as u16;
        let len = self.data.len() as u16;

        let index = match absolute_offset < len {
            true => absolute_offset as u8,
            false => (absolute_offset - len) as u8,
        };
        self.data.get_mut(index as usize)
    }
    #[allow(unused)]
    pub fn mut_planned_segment_from_tail(
        &mut self,
        offset: u8,
    ) -> Result<&mut motion::Segment, ()> {
        match self.mut_entry_from_tail(offset) {
            Some(PlanEntry::PlannedMove(_s, _, _, _)) => Ok(_s),
            _ => Err(()),
        }
    }

    #[allow(unused)]
    pub fn planned_segment_from_tail(&self, offset: u8) -> Result<&motion::Segment, ()> {
        match self.entry_from_tail(offset) {
            Some(PlanEntry::PlannedMove(_s, _, _, _)) => Ok(_s),
            _ => Err(()),
        }
    }

    #[allow(unused)]
    pub fn entries_from_tail(
        &mut self,
        offset1: u8,
        offset2: u8,
    ) -> (Option<&mut PlanEntry>, Option<&mut PlanEntry>) {
        let len = self.data.len();
        let absolute_offset1 = self.head as usize + self.used as usize - offset1 as usize;
        let index1 = match absolute_offset1 < len {
            true => absolute_offset1,
            false => (absolute_offset1 - len),
        };
        let absolute_offset2 = self.head as usize + self.used as usize - offset2 as usize;
        let index2 = match absolute_offset2 < len {
            true => absolute_offset2,
            false => (absolute_offset2 - len),
        };
        if index1 == index2 {
            (self.data.get_mut(index1), None)
        } else if index1 < index2 {
            let (l, r) = self.data.split_at_mut(index2);
            (l.get_mut(index1), r.get_mut(0))
        } else {
            let (l, r) = self.data.split_at_mut(index1);
            (r.get_mut(0), l.get_mut(index2))
        }
    }
}

#[derive(Clone, Copy)]
pub enum MovType {
    Move(hwa::DeferAction, hwa::CommChannel),
    Homing(hwa::CommChannel),
    Dwell(hwa::CommChannel),
}

#[derive(Clone, Copy)]
pub enum PlanEntry {
    Empty,
    /// A planned move tuple
    /// {_1: Segment} The motion segment
    /// {_2: CommChannel} The input channel requesting the move
    /// {_3: bool} Indicates if motion is deferred or not
    PlannedMove(motion::Segment, hwa::DeferAction, hwa::CommChannel, bool),
    /// A homing action request
    /// {_1: CommChannel} The input channel requesting the move
    /// {_2: bool} Indicates if motion is deferred or not
    Homing(hwa::CommChannel, bool),
    /// A Dwell action request
    /// {_1: CommChannel} The input channel requesting the move
    /// {_2: bool} Indicates if motion is deferred or not
    Dwell(hwa::CommChannel, bool),
    Executing(MovType, bool),
}

impl Default for PlanEntry {
    fn default() -> Self {
        PlanEntry::Empty
    }
}

#[derive(Clone)]
pub struct MotionPlannerRef {
    inner: &'static MotionPlanner,
}

impl MotionPlannerRef {
    pub const fn new(inner: &'static MotionPlanner) -> Self {
        Self { inner }
    }
}

impl core::ops::Deref for MotionPlannerRef {
    type Target = MotionPlanner;

    fn deref(&self) -> &Self::Target {
        self.inner
    }
}

#[cfg(test)]
pub mod test {
    #[allow(unused)]
    use cortex_m::register::control;

    //#[cfg(feature = "wip-tests")]
    #[test]
    fn discrete_positioning_case_1() {
        use crate::control::motion_planning::{Constraints, SCurveMotionProfile};
        use crate::control::motion_timing::StepPlanner;
        use crate::control::task_stepper::LinearMicrosegmentStepInterpolator;
        use crate::hwa::controllers::motion::{Segment, SegmentData, SegmentIterator};
        use crate::math;
        use crate::math::Real;
        use crate::tgeo::TVector;
        use printhor_hwa_common::StepperChannel;

        const STEPPER_PLANNER_MICROSEGMENT_FREQUENCY: u32 = 200;
        const STEPPER_PLANNER_CLOCK_FREQUENCY: u32 = 20_000;

        const STEPPER_PLANNER_MICROSEGMENT_PERIOD_US: u32 =
            1_000_000 / STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;
        const STEPPER_PLANNER_CLOCK_PERIOD_US: u32 = 1_000_000 / STEPPER_PLANNER_CLOCK_FREQUENCY;

        let segment = Segment::new(SegmentData {
            speed_enter_mms: Real::from_f32(200.0),
            speed_exit_mms: Real::from_f32(200.0),
            speed_target_mms: Real::from_f32(200.0),
            displacement_mm: Real::from_f32(0.505982578),
            speed_enter_constrained_mms: Real::from_f32(6.25),
            speed_exit_constrained_mms: Real::from_f32(6.25),
            proj_prev: Real::from_f32(0.999986052),
            proj_next: Real::from_f32(0.999938488),
            vdir: TVector::from_coords(
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
                v_max: Real::from_f32(200.0),
                a_max: Real::from_f32(3000.0),
                j_max: Real::from_f32(6000.0),
            },
        });

        let neutral_element = segment.segment_data.vdir.map_val(&math::ZERO);
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
            segment.segment_data.displacement_mm,
            segment.segment_data.speed_enter_mms,
            segment.segment_data.speed_exit_mms,
            &segment.segment_data.constraints,
            false,
        )
        .unwrap();

        let units_per_mm = neutral_element + units_per_mm;
        let steps_per_mm = units_per_mm * usteps;

        let mut micro_segment_real_time_rel = micro_segment_period_secs;
        let mut microsegment_iterator = SegmentIterator::new(&motion_profile, math::ZERO);

        let mut microsegment_interpolator = LinearMicrosegmentStepInterpolator::new(
            segment.segment_data.vdir.abs(),
            segment.segment_data.displacement_mm,
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
                    if segment.segment_data.speed_exit_mms > math::ZERO {
                        (ds / segment.segment_data.speed_exit_mms).max(sampling_time)
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
        assert!(
            expected_advanced_steps == real_advanced_steps,
            "Advanced steps matching. Expected: {} Got {}",
            expected_advanced_steps,
            real_advanced_steps
        )
    }

    #[test]
    fn discrete_positioning_case_2() {
        use crate::control::motion_planning::{Constraints, SCurveMotionProfile};
        use crate::control::motion_timing::StepPlanner;
        use crate::control::task_stepper::LinearMicrosegmentStepInterpolator;
        use crate::hwa::controllers::motion::motion_segment::{
            Segment, SegmentData, SegmentIterator,
        };
        use crate::math;
        use crate::math::Real;
        use crate::tgeo::TVector;
        use printhor_hwa_common::StepperChannel;

        const STEPPER_PLANNER_MICROSEGMENT_FREQUENCY: u32 = 500;
        const STEPPER_PLANNER_CLOCK_FREQUENCY: u32 = 50_000;

        const STEPPER_PLANNER_MICROSEGMENT_PERIOD_US: u32 =
            1_000_000 / STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;
        const STEPPER_PLANNER_CLOCK_PERIOD_US: u32 = 1_000_000 / STEPPER_PLANNER_CLOCK_FREQUENCY;

        let segment = Segment::new(SegmentData {
            speed_enter_mms: Real::from_f32(400.0),
            speed_exit_mms: Real::from_f32(400.0),
            speed_target_mms: Real::from_f32(400.0),
            displacement_mm: Real::from_f32(0.505982578),
            speed_enter_constrained_mms: Real::from_f32(6.25),
            speed_exit_constrained_mms: Real::from_f32(6.25),
            proj_prev: Real::from_f32(0.999986052),
            proj_next: Real::from_f32(0.999938488),
            vdir: TVector::from_coords(
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

        let neutral_element = segment.segment_data.vdir.map_val(&math::ZERO);
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
            segment.segment_data.displacement_mm,
            segment.segment_data.speed_enter_mms,
            segment.segment_data.speed_exit_mms,
            &segment.segment_data.constraints,
            false,
        )
        .unwrap();

        let units_per_mm = neutral_element + units_per_mm;
        let steps_per_mm = units_per_mm * usteps;

        let mut micro_segment_real_time_rel = micro_segment_period_secs;
        let mut microsegment_iterator = SegmentIterator::new(&motion_profile, math::ZERO);

        let mut microsegment_interpolator = LinearMicrosegmentStepInterpolator::new(
            segment.segment_data.vdir.abs(),
            segment.segment_data.displacement_mm,
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
                    if segment.segment_data.speed_exit_mms > math::ZERO {
                        (ds / segment.segment_data.speed_exit_mms).max(sampling_time)
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
        assert!(
            expected_advanced_steps == real_advanced_steps,
            "Advanced steps matching. Expected: {} Got {}",
            expected_advanced_steps,
            real_advanced_steps
        )
    }

    #[test]
    fn discrete_positioning_case_3() {
        use crate::control::motion_planning::{Constraints, SCurveMotionProfile};
        use crate::control::motion_timing::StepPlanner;
        use crate::control::task_stepper::LinearMicrosegmentStepInterpolator;
        use crate::hwa::controllers::motion::{Segment, SegmentData, SegmentIterator};
        use crate::math;
        use crate::math::Real;
        use crate::tgeo::TVector;
        use printhor_hwa_common::StepperChannel;

        const STEPPER_PLANNER_MICROSEGMENT_FREQUENCY: u32 = 500;
        const STEPPER_PLANNER_CLOCK_FREQUENCY: u32 = 200_000;

        const STEPPER_PLANNER_MICROSEGMENT_PERIOD_US: u32 =
            1_000_000 / STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;
        const STEPPER_PLANNER_CLOCK_PERIOD_US: u32 = 1_000_000 / STEPPER_PLANNER_CLOCK_FREQUENCY;

        let segment = Segment::new(SegmentData {
            speed_enter_mms: Real::from_f32(400.0),
            speed_exit_mms: Real::from_f32(400.0),
            speed_target_mms: Real::from_f32(400.0),
            displacement_mm: Real::from_f32(0.505982578),
            speed_enter_constrained_mms: Real::from_f32(6.25),
            speed_exit_constrained_mms: Real::from_f32(6.25),
            proj_prev: Real::from_f32(0.999986052),
            proj_next: Real::from_f32(0.999938488),
            vdir: TVector::from_coords(
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

        let neutral_element = segment.segment_data.vdir.map_val(&math::ZERO);
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
            segment.segment_data.displacement_mm,
            segment.segment_data.speed_enter_mms,
            segment.segment_data.speed_exit_mms,
            &segment.segment_data.constraints,
            false,
        )
        .unwrap();

        let units_per_mm = neutral_element + units_per_mm;
        let steps_per_mm = units_per_mm * usteps;

        let mut micro_segment_real_time_rel = micro_segment_period_secs;
        let mut microsegment_iterator = SegmentIterator::new(&motion_profile, math::ZERO);

        let mut microsegment_interpolator = LinearMicrosegmentStepInterpolator::new(
            segment.segment_data.vdir.abs(),
            segment.segment_data.displacement_mm,
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
                    if segment.segment_data.speed_exit_mms > math::ZERO {
                        (ds / segment.segment_data.speed_exit_mms).max(sampling_time)
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
        assert!(
            expected_advanced_steps == real_advanced_steps,
            "Advanced steps matching. Expected: {} Got {}",
            expected_advanced_steps,
            real_advanced_steps
        )
    }
}
