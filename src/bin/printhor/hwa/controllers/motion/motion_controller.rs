//! This feature is being stabilized
use crate::hwa;
use embassy_sync::mutex::Mutex;
use printhor_hwa_common::{CommChannel, ControllerRef, DeferAction, DeferChannelRef, EventBusRef, EventFlags, EventStatus};
use printhor_hwa_common::DeferEvent;
use crate::control::GCode;
use crate::control::{CodeExecutionSuccess, CodeExecutionFailure};
use crate::control::motion_planning::{Constraints, SCurveMotionProfile};
use crate::math::{ONE_HUNDRED, Real, ZERO};
use crate::sync::config::Config;
use crate::tgeo::TVector;
use crate::tgeo::CoordSel;

use crate::hwa::controllers::motion::motion_segment::{Segment, SegmentData};

pub enum ScheduledMove {
    Move(SegmentData, SCurveMotionProfile),
    Homing,
    Dwell,
}

/////
#[allow(unused)]
pub struct MotionConfig {
    pub max_accel: TVector<u16>,
    pub max_speed: TVector<u16>,
    pub max_jerk: TVector<u32>,
    pub default_travel_speed: u16,
    pub mm_per_unit: TVector<Real>,
    pub machine_bounds: TVector<Real>,
    pub usteps: [u16; 4],
    flow_rate: u8,
    speed_rate: u8,
}

impl MotionConfig {
    pub const fn new() -> Self {
        Self {
            max_accel: TVector::new(),
            max_speed: TVector::new(),
            max_jerk: TVector::new(),
            mm_per_unit: TVector::new(),
            machine_bounds: TVector::new(),
            usteps: [0; 4],
            default_travel_speed: 1,
            flow_rate: 100,
            speed_rate: 100,
        }
    }
}

pub type MotionConfigRef = ControllerRef<MotionConfig>;

pub struct MotionStatus {
    #[allow(unused)]
    pub(crate) current_pos_steps: Option<TVector<u32>>,
    pub(crate) last_planned_pos: Option<TVector<Real>>,
    pub(crate) absolute_positioning: bool,
    #[cfg(feature="with-laser")]
    #[allow(unused)]
    pub(crate) laser: bool,
}

impl MotionStatus {
    pub const fn new() -> Self {
        Self {
            current_pos_steps: None,
            last_planned_pos: None,
            absolute_positioning: true,
            #[cfg(feature="with-laser")]
            laser: false,
        }
    }
}


#[allow(unused)]
pub struct MotionPlanner {
    pub event_bus: EventBusRef,
    // The channel to send deferred events
    pub defer_channel: printhor_hwa_common::DeferChannelRef,

    pub(self) ringbuffer: Mutex<hwa::ControllerMutexType, RingBuffer>,
    pub(self) move_planned: Config<hwa::ControllerMutexType, bool>,
    pub(self) available: Config<hwa::ControllerMutexType, bool>,
    pub(self) motion_config: MotionConfigRef,
    motion_st: Mutex<hwa::ControllerMutexType, MotionStatus>,
    pub motion_driver: Mutex<hwa::ControllerMutexType, hwa::drivers::MotionDriver>,
}

#[allow(unused)]
impl MotionPlanner {
    pub const fn new(event_bus: EventBusRef,
                     defer_channel: DeferChannelRef,
                     motion_config: MotionConfigRef,
                     motion_driver: hwa::drivers::MotionDriver,
    ) -> Self {
        Self {
            event_bus,
            defer_channel,
            motion_config,
            ringbuffer: Mutex::new(RingBuffer::new()),
            move_planned: Config::new(),
            available: Config::new(),
            motion_st: Mutex::new(MotionStatus::new()),
            motion_driver: Mutex::new(motion_driver),
        }
    }

    pub async fn start(&self) {
        self.move_planned.reset();
        self.available.signal(true);
        self.event_bus.publish_event(EventStatus::containing(EventFlags::MOV_QUEUE_EMPTY)).await;
    }

    pub async fn get_current_segment_data(&self) -> Option<(Segment, CommChannel)> {
        loop {
            let _ = self.move_planned.wait().await;
            let mut do_dwell = false;
            {
                let mut rb = self.ringbuffer.lock().await;
                let head = rb.head as usize;
                match rb.data[head] {
                    PlanEntry::Empty => {
                        self.move_planned.reset();
                    },
                    PlanEntry::Dwell(channel, _) => {
                        rb.data[head] = PlanEntry::Executing(MovType::Dwell(channel), true);
                        do_dwell = true;
                    },
                    PlanEntry::PlannedMove(planned_data, action, channel, deferred) => {
                        hwa::debug!("Exec starting: {} / {} h={}", rb.used, hwa::SEGMENT_QUEUE_SIZE, head);
                        rb.data[head] = PlanEntry::Executing(MovType::Move(action, channel), deferred);
                        return Some((planned_data, channel));
                    },
                    PlanEntry::Homing(channel, _) => {
                        self.event_bus.publish_event(EventStatus::containing(EventFlags::HOMMING)).await;
                        rb.data[head] = PlanEntry::Executing(MovType::Homing(channel), true);
                        return None;
                    },
                    PlanEntry::Executing(_, _) => {
                        self.move_planned.reset();
                        hwa::error!("Unexpected error: RingBuffer Overrun");
                    },
                }
            }
            if do_dwell {
                self.consume_current_segment_data().await;
            }
        }
    }

    pub async fn consume_current_segment_data(&self) -> u8 {
        let mut rb = self.ringbuffer.lock().await;
        let head = rb.head;
        hwa::debug!("Movement completed @rq[{}] (ongoing={})", head, rb.used - 1);
        match &rb.data[head as usize] {
            PlanEntry::Executing(MovType::Homing(channel), _) => {
                self.event_bus.publish_event(EventStatus::not_containing(EventFlags::HOMMING)).await;
                self.defer_channel.send(DeferEvent::Completed(DeferAction::Homing, *channel)).await;
            }
            PlanEntry::Executing(MovType::Dwell(channel), _) => {
                self.defer_channel.send(DeferEvent::Completed(DeferAction::Homing, *channel)).await;
            }
            PlanEntry::Executing(MovType::Move(action, channel), deferred) => {
                if *deferred {
                    self.defer_channel.send(DeferEvent::Completed(*action, *channel)).await;
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
            self.event_bus.publish_event(EventStatus::containing(EventFlags::MOV_QUEUE_EMPTY)).await;
        }
        self.available.signal(true);
        rb.used
    }

    pub async fn schedule_raw_move(&self, channel: CommChannel, action: DeferAction, move_type: ScheduledMove, blocking: bool) -> Result<CodeExecutionSuccess, CodeExecutionFailure> {
        // FIXME: Better to set subscription to defer channel from here to avoid msg loss
        hwa::debug!("schedule_raw_move() BEGIN");
        loop {
            self.available.wait().await;
            {
                let mut rb = self.ringbuffer.lock().await;
                let mut is_defer = (rb.used == hwa::SEGMENT_QUEUE_SIZE - 1);

                if rb.used < (hwa::SEGMENT_QUEUE_SIZE as u8) {
                    let used = rb.used;
                    let head = rb.head;
                    let mut could_replan = false;

                    let (next_planned_entry, event) = match &move_type {
                        ScheduledMove::Move(segment_data, motion_profile) => {
                            could_replan = true;
                            self.update_last_planned_pos(&segment_data.dest_pos).await;
                            (PlanEntry::PlannedMove(Segment::new(*segment_data, *motion_profile), action, channel, is_defer), EventStatus::new())
                        }
                        ScheduledMove::Homing => {
                            is_defer = true;
                            (PlanEntry::Homing(channel, is_defer), EventStatus::not_containing(EventFlags::HOMMING))
                        }
                        ScheduledMove::Dwell => {
                            is_defer = true;
                            (PlanEntry::Dwell(channel, is_defer), EventStatus::containing(EventFlags::MOV_QUEUE_EMPTY))
                        }
                    };

                    let next_insert_index = rb.index_from_tail(0);

                    hwa::debug!("Mov queued @{} ({} / {})", next_insert_index, rb.used + 1, hwa::SEGMENT_QUEUE_SIZE);
                    if could_replan && used > 0 {
                        let last_inserted_index = rb.index_from_tail(1);
                        hwa::debug!(" - check_replan {} -> {}", last_inserted_index, next_insert_index);
                        match &rb.data[last_inserted_index as usize] {
                            PlanEntry::Executing(_, _) | PlanEntry::Homing(_, _) | PlanEntry::Empty => {
                                hwa::debug!(" -- not chained");
                                rb.data[next_insert_index as usize] = next_planned_entry;
                            }
                            PlanEntry::PlannedMove(last_planned_segment, last_planned_segment_action, last_planned_segment_channel, last_planned_segment_deferred) => {
                                match next_planned_entry {
                                    PlanEntry::PlannedMove(mut next_planned_segment, next_planned_segment_action, next_planned_segment_channel, next_planned_segment_deferred) => {
                                        let vd1 = last_planned_segment.segment_data.vdir;
                                        let vm1 = last_planned_segment.motion_profile.v_lim;
                                        let vd2 = next_planned_segment.segment_data.vdir;
                                        let vm2 = next_planned_segment.motion_profile.v_lim;
                                        let proj = vd1.orthogonal_projection(vd2);
                                        let x = proj.is_positive();

                                        if proj.is_defined_positive() {
                                            hwa::debug!("RingBuffer [{}, {}] chained: ({}) proj ({}) = ({}) ", last_inserted_index, next_insert_index, vd1, vd2, proj);
                                            let v_exit_module = last_planned_segment.motion_profile.v_lim.min(next_planned_segment.motion_profile.v_lim).min(last_planned_segment.motion_profile.constraints.v_max).min(next_planned_segment.motion_profile.constraints.v_max);
                                            let v = (last_planned_segment.segment_data.vdir * v_exit_module).norm2().unwrap_or(Real::zero());
                                            rb.data[last_inserted_index as usize] = PlanEntry::PlannedMove(
                                                last_planned_segment.recalculate(last_planned_segment.segment_data.speed_enter_mms, v, true),
                                                *last_planned_segment_action, *last_planned_segment_channel, *last_planned_segment_deferred,
                                            );
                                            rb.data[next_insert_index as usize] = PlanEntry::PlannedMove(
                                                next_planned_segment.recalculate(v, Real::zero(), true),
                                                next_planned_segment_action, next_planned_segment_channel, next_planned_segment_deferred,
                                            );
                                        }
                                        else {
                                            hwa::debug!("RingBuffer [{}, {}] not chained: ({}) proj ({}) = ({})",  last_inserted_index, next_insert_index, vd1, vd2, proj);
                                            rb.data[next_insert_index as usize] = next_planned_entry;
                                        }

                                    }
                                    _ => {
                                        rb.data[next_insert_index as usize] = next_planned_entry;
                                    }
                                }

                            }
                            _ => {
                                unreachable!("Could not happen");
                            }
                        }

                    }
                    else {
                        rb.data[next_insert_index as usize] = next_planned_entry;
                    }

                    rb.used += 1;
                    self.event_bus.publish_event(EventStatus::not_containing(EventFlags::MOV_QUEUE_EMPTY)).await;
                    self.move_planned.signal(true);
                    if is_defer {
                        // Shall wait for one de-allocation in order to enqueue more
                        hwa::debug!("schedule_raw_move() - Finally deferred");
                        self.defer_channel.send(DeferEvent::AwaitRequested(action, channel)).await;
                        return Ok(CodeExecutionSuccess::DEFERRED(event))
                    }
                    else {
                        hwa::debug!("schedule_raw_move() END - Finally queued");
                        return Ok(CodeExecutionSuccess::QUEUED)
                    }

                } else {
                    self.available.reset();
                    if !blocking {
                        hwa::warn!("Mov rejected: {} / {} h={}", rb.used, hwa::SEGMENT_QUEUE_SIZE, rb.head);
                        return Err(CodeExecutionFailure::BUSY)
                    }
                    else {
                        hwa::trace!("schedule_raw_move() Looping again");
                    }
                }
            }
        }
        hwa::debug!("schedule_raw_move() END");
    }

    pub fn motion_cfg(&self) -> MotionConfigRef {
        self.motion_config.clone()
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
        self.motion_st.lock().await.last_planned_pos.replace(pos.map_nan(Real::zero()));
    }

    pub async fn get_last_planned_step_pos(&self) -> Option<TVector<u32>> {
        self.motion_st.lock().await.current_pos_steps.clone()
    }
    pub async fn set_last_planned_step_pos(&self, pos: &TVector<u32>) {
        self.motion_st.lock().await.current_pos_steps.replace(pos.map_nan(0));
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
        Real::new(self.motion_config.lock().await.flow_rate as i64, 0) / ONE_HUNDRED
    }

    pub async fn set_speed_rate(&self, rate: u8) {
        self.motion_config.lock().await.speed_rate = rate;
    }

    pub async fn get_speed_rate_as_real(&self) -> Real {
        Real::new(self.motion_config.lock().await.speed_rate as i64, 0) / ONE_HUNDRED
    }

    pub async fn get_default_travel_speed(&self) -> u16 {
        self.motion_config.lock().await.default_travel_speed
    }
    pub async fn set_default_travel_speed(&self, speed: u16) {
        self.motion_config.lock().await.default_travel_speed = speed;
    }

    pub async fn set_steps_per_mm(&self, x_spm: Real, y_spm: Real, z_spm: Real, e_spm: Real) {
        self.motion_config.lock().await.mm_per_unit = TVector::from_coords(
            Some(x_spm),
            Some(y_spm),
            Some(z_spm),
            Some(e_spm),
        );
    }

    pub async fn get_steps_per_mm_as_vector(&self) -> TVector<Real> {
        self.motion_config.lock().await.mm_per_unit
    }

    pub async fn set_usteps(&self, x_usteps: u8, y_usteps: u8, z_usteps: u8, e_usteps: u8) {
        self.motion_config.lock().await.usteps = [
            x_usteps.into(), y_usteps.into(), z_usteps.into(), e_usteps.into()
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
        let mcfg = self.motion_config.lock().await;
        TVector::from_coords(
            Some(Real::new(mcfg.usteps[0].into(), 0)),
            Some(Real::new(mcfg.usteps[1].into(), 0)),
            Some(Real::new(mcfg.usteps[2].into(), 0)),
            Some(Real::new(mcfg.usteps[3].into(), 0)),
        )
    }

    pub async fn get_default_travel_speed_as_real(&self) -> Real {
        Real::new(self.motion_config.lock().await.default_travel_speed as i64, 0)
    }

    pub async fn get_max_accel(&self) -> TVector<u16> {
        self.motion_config.lock().await.max_accel
    }

    pub async fn get_max_speed(&self) -> TVector<u16> {
        self.motion_config.lock().await.max_speed
    }
    pub async fn set_max_speed(&self, speed: TVector<u16>) {
        self.motion_config.lock().await.max_speed.assign(CoordSel::all(), &speed);
    }
    pub async fn get_max_speed_as_vreal(&self) -> TVector<Real> {
        self.motion_config.lock().await.max_speed.map_coords(|c| Some(Real::new(c as i64, 0)))
    }

    pub async fn set_max_accel(&self, accel: TVector<u16>) {
        self.motion_config.lock().await.max_accel.assign(CoordSel::all(), &accel);
    }

    pub async fn get_max_accel_as_vreal(&self) -> TVector<Real> {
        self.motion_config.lock().await.max_accel.map_coords(|c| Some(Real::new(c as i64, 0)))
    }

    pub async fn set_max_jerk(&self, jerk: TVector<u32>) {
        self.motion_config.lock().await.max_jerk.assign(CoordSel::all(), &jerk);
    }

    pub async fn plan(&self, channel: CommChannel, gc: &GCode, blocking: bool) -> Result<CodeExecutionSuccess, CodeExecutionFailure>{
        match gc {
            GCode::G0(t) => {
                Ok(self.schedule_move(channel, DeferAction::RapidMove, TVector{
                    x: t.x, y: t.y, z: t.z, e: None,
                }, t.f, blocking).await?)
            }
            GCode::G1(t) => {
                Ok(self.schedule_move(channel, DeferAction::LinearMove, TVector{
                    x: t.x, y: t.y, z: t.z, e: t.e
                }, t.f, blocking).await?)
            }
            GCode::G4 => {
                Ok(self.schedule_raw_move(channel, DeferAction::Dwell, ScheduledMove::Dwell, blocking).await?)
            }
            GCode::G28(_x) => {
                self.event_bus.publish_event(EventStatus::containing(EventFlags::HOMMING)).await;
                Ok(self.schedule_raw_move(channel, DeferAction::Homing, ScheduledMove::Homing, blocking).await?)
            }
            GCode::G29 => {
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::G29_1 => {
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::G29_2 => {
                Ok(CodeExecutionSuccess::OK)
            }
            _ => {
                Err(CodeExecutionFailure::NotYetImplemented)
            }
        }
    }

    async fn schedule_move(&self, channel: CommChannel, action: DeferAction, p1: TVector<Real>, requested_motion_speed: Option<Real>, blocking: bool) -> Result<CodeExecutionSuccess, CodeExecutionFailure> {

        let t0 = embassy_time::Instant::now();

        let p0 = self.get_last_planned_pos().await.ok_or(CodeExecutionFailure::HomingRequired)?;
        let p1 = if self.is_absolute_positioning().await { p1 } else { p0 + p1 };

        let cfg = self.motion_cfg();
        let cfg_g = cfg.lock().await;
        //----
        let dts = Real::from_lit(cfg_g.default_travel_speed as i64, 0);
        let flow_rate = Real::from_lit(cfg_g.flow_rate as i64, 0) / ONE_HUNDRED;
        let speed_rate = Real::from_lit(cfg_g.speed_rate as i64, 0) / ONE_HUNDRED;
        let max_speed = cfg_g.max_speed.map_coords(|c| Some(Real::from_lit(c as i64, 0)));
        let max_accel = cfg_g.max_accel.map_coords(|c| Some(Real::from_lit(c as i64, 0)));
        let max_jerk = cfg_g.max_jerk.map_coords(|c| Some(Real::from_lit(c as i64, 0)));
        //----
        drop(cfg_g);

        let t1 = embassy_time::Instant::now();

        // Compute distance and decompose as unit vector and module.
        // When dist is zero, value is map to None (NaN).
        // In case o E dimension, flow rate factor is applied
        let (vdir, module_target_distance) = (p1 - p0)
            .map_coord(CoordSel::all(), |coord_value, coord_idx| {
                match coord_idx {
                    CoordSel::X | CoordSel::Y | CoordSel::Z => {
                        match coord_value.is_zero() {
                            true => None,
                            false => Some(coord_value),
                        }
                    },
                    CoordSel::E => {
                        match coord_value.is_zero() {
                            true => None,
                            false => Some(coord_value * flow_rate),
                        }
                    },
                    _ => None,
                }
            }).decompose_normal();

        // Compute the speed module applying speed_rate factor
        let speed_module = requested_motion_speed.unwrap_or(dts);
        // Compute per-axis distance
        let disp_vector: TVector<Real> = vdir.abs() * speed_module;
        // Clamp per-axis target speed to the physical restrictions
        let clamped_speed = (vdir.map_val(Real::one()) * speed_module).clamp(max_speed);
        // Compute max time
        let max_time = (disp_vector / clamped_speed).max().unwrap_or(ZERO);
        // Finally, per-axis relative speed
        let speed_vector = (disp_vector / max_time) * speed_rate;

        let module_target_speed = speed_vector.norm2().unwrap_or(ZERO);
        let module_target_accel = (vdir.abs() * max_accel).norm2().unwrap_or(ZERO);
        let module_target_jerk = (vdir.abs() * max_jerk).norm2().unwrap_or(ZERO);

        let t2 = embassy_time::Instant::now();

        let move_result = if module_target_distance.is_zero() {
            // TODO: set current speed!
            Ok(CodeExecutionSuccess::OK)
        }
        else {
            let profile = match !module_target_speed.is_zero() && !module_target_accel.is_zero() && !module_target_jerk.is_zero() {
                true => {
                    let _constraints = Constraints {
                        v_max: module_target_speed,
                        a_max: module_target_accel,
                        j_max: module_target_jerk,
                    };
                    let profile = SCurveMotionProfile::compute(module_target_distance.clone(), ZERO, ZERO, &_constraints, true)?;
                    Some(profile)
                },
                false => {
                    hwa::error!("p0: {}", p0.rdp(4));
                    hwa::error!("p1: {}", p1.rdp(4));
                    hwa::error!("dist: {} mm", module_target_distance.rdp(4));
                    hwa::error!("vdir: {} mm/s", vdir.rdp(4));
                    hwa::error!("speed_vector: {} mm/s", speed_vector.rdp(4));
                    hwa::error!("clamped_speed: {} mm/s", clamped_speed.rdp(4));

                    None
                }
            };
            //
            let t3 = embassy_time::Instant::now();
            hwa::debug!("P0 ({}) mm", p0);
            hwa::debug!("P1 ({}) mm", p1);
            hwa::debug!("MOTION PLAN: dist: {} mm, speed_max: {} mm/s, accel_max: {} mm/s^2, jerk_max: {} mm/s^3",
                module_target_distance.rdp(4),
                module_target_speed.rdp(4),
                module_target_accel.rdp(4),
                module_target_jerk.rdp(4)
            );
            match profile {
                Some(profile) => {
                    let segment_data = SegmentData {
                        speed_enter_mms: Real::zero(),
                        speed_exit_mms: Real::zero(),
                        displacement_mm: module_target_distance,
                        vdir,
                        dest_pos: p1,
                        tool_power: Real::zero(),
                    };
                    let r = self.schedule_raw_move(
                        channel,
                        action,
                        ScheduledMove::Move(segment_data, profile),
                        blocking
                    ).await?;

                    hwa::debug!("speed: {} -> {} ", requested_motion_speed.unwrap_or(Real::zero()).rdp(4), module_target_speed.rdp(4));
                    hwa::debug!("speed_vector: {}", speed_vector.rdp(4));
                    hwa::debug!("clamped_speed: {}", clamped_speed.rdp(4));
                    hwa::debug!("speed_rates: {}", speed_rate.rdp(4));

                    #[cfg(feature = "std")]
                    hwa::debug!("profile: {} {} {} {} {}", profile.t_j1.rdp(4), profile.t_a.rdp(4), profile.t_v.rdp(4), profile.t_d.rdp(4), profile.t_j2.rdp(4));

                    hwa::debug!("Conf retr in: {} us", (t1-t0).as_micros());
                    hwa::debug!("Move prepared in: {} us", (t2-t1).as_micros());
                    hwa::debug!("Move profiled in: {} us", (t3-t2).as_micros());
                    return Ok(r);
                }
                None => {
                    hwa::warn!("Incomplete move");
                }
            }

            hwa::debug!("----");

            Ok(CodeExecutionSuccess::OK)
        };
        match move_result {
            Ok(resp) => Ok(resp),
            Err(err) => match err {
                CodeExecutionFailure::BUSY => {
                    todo!("Delegate to deferrals!!!")
                }
                _ => {
                    Err(err)
                }
            }
        }
    }

    pub async fn do_homing(&self) -> Result<(), ()>{
        let r = self.motion_driver.lock().await.homing_action(&self.motion_config).await;
        if r.is_err() {
            hwa::error!("Unable to complete homming. [Not yet] Raising SYS_ALARM");
            //self.event_bus.publish_event(EventStatus::containing(EventFlags::SYS_ALARM)).await;
        }
        self.set_last_planned_pos(&TVector::zero()).await;
        self.event_bus.publish_event(EventStatus::not_containing(EventFlags::HOMMING)).await;
        r
    }

    #[cfg(all(feature = "native", feature = "plot-timings"))]
    pub async fn start_segment(&self, ref_time: embassy_time::Instant, real_time: embassy_time::Instant) {
        self.motion_driver.lock().await.start_segment(ref_time, real_time)
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
    pub fn index_from_tail(&self, offset: u8) -> u8 {
        let absolute_offset = self.head as u16 + self.used as u16 - offset as u16;
        let len =  self.data.len() as u16;
        match absolute_offset < len {
            true => absolute_offset as u8,
            false => (absolute_offset - len) as u8,
        }
    }
}

#[derive(Clone, Copy)]
pub enum MovType {
    Move(DeferAction, CommChannel, ),
    Homing(CommChannel),
    Dwell(CommChannel),
}

#[derive(Clone, Copy)]
pub enum PlanEntry {
    Empty,
    /// A planned move tuple
    /// {_1: Segment} The motion segment
    /// {_2: CommChannel} The input channel requesting the move
    /// {_3: bool} Indicates if motion is deferred or not
    PlannedMove(Segment, DeferAction, CommChannel, bool),
    /// A homing action request
    /// {_1: CommChannel} The input channel requesting the move
    /// {_2: bool} Indicates if motion is deferred or not
    Homing(CommChannel, bool),
    /// A Dwell action request
    /// {_1: CommChannel} The input channel requesting the move
    /// {_2: bool} Indicates if motion is deferred or not
    Dwell(CommChannel, bool),
    Executing(MovType, bool),
}

impl Default for PlanEntry {
    fn default() -> Self {
        PlanEntry::Empty
    }
}

#[derive(Clone)]
pub struct MotionPlannerRef {
    inner: &'static MotionPlanner
}

impl MotionPlannerRef {
    pub const fn new(inner: &'static MotionPlanner) -> Self {
        Self {
            inner
        }
    }
}

//#[cfg(feature = "native")]
unsafe impl Send for MotionPlannerRef {}

impl core::ops::Deref for MotionPlannerRef {
    type Target = MotionPlanner;

    fn deref(&self) -> &Self::Target {
        self.inner
    }
}