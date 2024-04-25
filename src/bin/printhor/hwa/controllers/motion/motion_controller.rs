//! This feature is being stabilized
use crate::{hwa, math};
use embassy_sync::mutex::{Mutex, MutexGuard};
use printhor_hwa_common::{CommChannel, ControllerMutexType, ControllerRef, DeferAction, DeferChannelRef, EventBusRef, EventFlags, EventStatus};
use printhor_hwa_common::DeferEvent;
use crate::control::GCode;
use crate::control::{CodeExecutionSuccess, CodeExecutionFailure};
use crate::control::motion_planning::{Constraints};
use crate::math::*;
use crate::sync::config::Config;
use crate::tgeo::TVector;
use crate::tgeo::CoordSel;

use crate::hwa::controllers::motion::motion_segment::{Segment, SegmentData};

pub enum ScheduledMove {
    Move(SegmentData),
    Homing,
    Dwell,
}

/////
#[allow(unused)]
pub struct MotionConfig {
    pub max_accel: TVector<u32>,
    pub max_speed: TVector<u32>,
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

    pub fn get_usteps_as_vector(&self) -> TVector<Real> {
        TVector::from_coords(
            Some(Real::new(self.usteps[0].into(), 0)),
            Some(Real::new(self.usteps[1].into(), 0)),
            Some(Real::new(self.usteps[2].into(), 0)),
            Some(Real::new(self.usteps[3].into(), 0)),
        )
    }
}

pub type MotionConfigRef = ControllerRef<MotionConfig>;

pub struct MotionStatus {
    pub(crate) last_real_pos: Option<TVector<Real>>,
    pub(crate) last_planned_pos: Option<TVector<Real>>,
    pub(crate) absolute_positioning: bool,
    #[cfg(feature="with-laser")]
    #[allow(unused)]
    pub(crate) laser: bool,
}

impl MotionStatus {
    pub const fn new() -> Self {
        Self {
            last_real_pos: None,
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
        hwa::debug!("schedule_raw_move() BEGIN");
        loop {
            self.available.wait().await;
            {
                let mut rb = self.ringbuffer.lock().await;
                let mut is_defer = (rb.used == hwa::SEGMENT_QUEUE_SIZE - 1);

                if rb.used < (hwa::SEGMENT_QUEUE_SIZE as u8) {
                    let used = rb.used;
                    let head = rb.head;

                    #[cfg(feature = "cornering")]
                    let mut do_adjust = false;

                    let curr_insert_index = rb.index_from_tail(0).unwrap();

                    let (mut curr_planned_entry, event) = match move_type {
                        ScheduledMove::Move(curr_segment_data) => {

                            let mut curr_segment = Segment::new(curr_segment_data);
                            // TODO:
                            // 1) Find a better way to approximate
                            // 2) Move outside ringbuffer lock
                            // solving sqrt(((abs(x-v_0))/jmax)) > q_1  for x
                            // x < v_0 - jmax q_1^2
                            // x > jmax * q_1^2 + v_0

                            let mut curr_vmax = math::ZERO;

                            let mut curr_vmax = curr_segment.segment_data.speed_target_mms;
                            #[cfg(feature = "cornering")]
                            {
                                let q_1 = curr_segment.segment_data.displacement_mm;
                                let v_0 = curr_segment.segment_data.speed_enter_mms;
                                let t_jmax = curr_segment.segment_data.constraints.a_max / curr_segment.segment_data.constraints.j_max;

                                loop {
                                    let t_jstar = Real::vmin(
                                        (((curr_vmax - v_0).abs()) / curr_segment.segment_data.constraints.j_max).sqrt(),
                                        Some(t_jmax),
                                    ).unwrap_or(ZERO);

                                    let q_lim = if t_jstar < t_jmax {
                                        t_jstar * (v_0 + curr_vmax)
                                    }
                                    else {
                                        ((v_0 + curr_vmax) / TWO) * (t_jstar + ((curr_vmax - v_0).abs() / curr_segment.segment_data.constraints.a_max))
                                    };

                                    if q_1 >= q_lim {
                                        break;
                                    }
                                    else {
                                        curr_vmax *= Real::from_f32(0.5);
                                    }
                                }
                            }

                            let curr_boundary = curr_vmax.min(curr_segment.segment_data.constraints.v_max);
                            let mut v_marginal_gain = curr_boundary -  curr_segment.segment_data.speed_exit_mms;
                            let mut v_slope_gain = math::ZERO;
                            hwa::debug!("\t- constraint = {} bound = {}", curr_vmax, curr_boundary);
                            hwa::debug!("\t= v_margin: {}", v_marginal_gain);
                            curr_segment.segment_data.speed_enter_constrained_mms = v_marginal_gain;
                            curr_segment.segment_data.speed_exit_constrained_mms = v_marginal_gain;

                            hwa::debug!("s: vi = [{} < {}] - vtarget = {} - vo = [{} < {}] / d = {}",
                                curr_segment.segment_data.speed_enter_mms,
                                curr_segment.segment_data.speed_enter_constrained_mms,
                                curr_segment.segment_data.speed_target_mms,
                                curr_segment.segment_data.speed_exit_constrained_mms,
                                curr_segment.segment_data.speed_exit_mms, curr_segment.segment_data.displacement_mm);

                            let mut num_segments_before = 0;
                            let mut max_gain = v_marginal_gain;
                            hwa::debug!("\t= max_gain: {}", max_gain);

                            if let Ok(prev_index) = rb.index_from_tail(1) {
                                match &mut rb.data[prev_index as usize] {
                                    PlanEntry::PlannedMove(prev_segment, _, _, _) => {
                                        let proj: Real = prev_segment.segment_data.vdir.orthogonal_projection(curr_segment.segment_data.vdir);
                                        if proj.is_defined_positive() {
                                            hwa::debug!("RingBuffer [{}, {}] chained: ({}) proj ({}) = ({})", prev_index, curr_insert_index,
                                                prev_segment.segment_data.vdir, curr_segment.segment_data.vdir, proj
                                            );
                                            let v_exit_module = prev_segment.segment_data.speed_target_mms.min(curr_segment.segment_data.speed_target_mms) * proj;

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

                            self.update_last_planned_pos(&curr_segment.segment_data.dest_pos).await;
                            (PlanEntry::PlannedMove(curr_segment, action, channel, is_defer), EventStatus::new())
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

                    hwa::debug!("Mov queued @{} ({} / {})", curr_insert_index, rb.used + 1, hwa::SEGMENT_QUEUE_SIZE);

                    rb.data[curr_insert_index as usize] = curr_planned_entry;
                    rb.used += 1;
                    #[cfg(feature = "cornering")]
                    if do_adjust {
                        let r = perform_cornering(rb);
                    }

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
        let mut mg = self.motion_st.lock().await;
        let p = mg.last_planned_pos.unwrap_or(TVector::zero());
        mg.last_planned_pos.replace(p.apply(pos));
    }

    pub async fn get_last_planned_real_pos(&self) -> Option<TVector<Real>> {
        self.motion_st.lock().await.last_real_pos.clone()
    }
    pub async fn set_last_planned_real_pos(&self, pos: &TVector<Real>) {
        self.motion_st.lock().await.last_real_pos.replace(pos.map_nan(&crate::math::ZERO));
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
        self.motion_config.lock().await.get_usteps_as_vector()
    }

    pub async fn get_default_travel_speed_as_real(&self) -> Real {
        Real::new(self.motion_config.lock().await.default_travel_speed as i64, 0)
    }

    pub async fn get_max_accel(&self) -> TVector<u32> {
        self.motion_config.lock().await.max_accel
    }

    pub async fn get_max_speed(&self) -> TVector<u32> {
        self.motion_config.lock().await.max_speed
    }
    pub async fn set_max_speed(&self, speed: TVector<u32>) {
        self.motion_config.lock().await.max_speed.assign(CoordSel::all(), &speed);
    }
    pub async fn get_max_speed_as_vreal(&self) -> TVector<Real> {
        self.motion_config.lock().await.max_speed.map_coords(|c| Some(Real::new(c as i64, 0)))
    }

    pub async fn set_max_accel(&self, accel: TVector<u32>) {
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

    async fn schedule_move(&self, channel: CommChannel, action: DeferAction, p1_t: TVector<Real>, requested_motion_speed: Option<Real>, blocking: bool) -> Result<CodeExecutionSuccess, CodeExecutionFailure> {

        let t0 = embassy_time::Instant::now();

        let p0 = self.get_last_planned_pos().await.ok_or(CodeExecutionFailure::HomingRequired)?;
        let _pdest = if self.is_absolute_positioning().await { p1_t } else { p0 + p1_t };

        let steps_per_unit = self.get_steps_per_mm_as_vector().await * self.get_usteps_as_vector().await;
        let rounded_pos: TVector<Real> = ((_pdest - p0) * steps_per_unit).ceil() / steps_per_unit;

        let p1 = p0 + rounded_pos;
        hwa::debug!("p1 [{}] -> [{}]", p1_t, p1);

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

        // Compute distance and decompose as unit vector and module.
        // When dist is zero, value is map to None (NaN).
        // In case o E dimension, flow rate factor is applied
        let ds = (p1 - p0);
        let (vdir, module_target_distance) = ds
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

        // Finally, per-axis relative speed
        let speed_vector = clamped_speed * speed_rate;

        let module_target_speed = speed_vector.norm2().unwrap_or(ZERO);
        let module_target_accel = (vdir.abs() * max_accel).norm2().unwrap_or(ZERO);
        let module_target_jerk = (vdir.abs() * max_jerk).norm2().unwrap_or(ZERO);

        let move_result = if module_target_distance.is_negligible() {
            Ok(CodeExecutionSuccess::OK)
        }
        else if !module_target_speed.is_zero() && !module_target_accel.is_zero() && !module_target_jerk.is_zero() {
            let segment_data = SegmentData {
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
                constraints: Constraints {
                    v_max: module_target_speed,
                    a_max: module_target_accel,
                    j_max: module_target_jerk,
                },
                proj_next: Real::zero(),
            };

            let r = self.schedule_raw_move(
                channel,
                action,
                ScheduledMove::Move(segment_data),
                blocking
            ).await?;

            hwa::debug!("speed: {} -> {} ", requested_motion_speed.unwrap_or(Real::zero()).rdp(4), module_target_speed.rdp(4));
            hwa::debug!("speed_vector: {}", speed_vector.rdp(4));
            hwa::debug!("clamped_speed: {}", clamped_speed.rdp(4));
            hwa::debug!("speed_rates: {}", speed_rate.rdp(4));
            hwa::debug!("speed_module: {}", module_target_speed.rdp(4));

            #[cfg(feature = "std")]
            hwa::debug!("profile: {} {} {} {} {}", profile.t_j1.rdp(4), profile.t_a.rdp(4), profile.t_v.rdp(4), profile.t_d.rdp(4), profile.t_j2.rdp(4));

            return Ok(r);
        }
        else {
            hwa::warn!("Incomplete move");
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
        match self.motion_driver.lock().await.homing_action(&self.motion_config).await {
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
        self.event_bus.publish_event(EventStatus::not_containing(EventFlags::HOMMING)).await;
        Ok(())
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

#[cfg(feature = "cornering")]
fn perform_cornering(mut rb: MutexGuard<ControllerMutexType, RingBuffer>) -> Result<(),()>{
    let mut left_offset = 2;
    let mut left_watermark = math::ZERO;
    let mut right_watermark = math::ZERO;
    // Assuming queued items are computed left to right:
    // First, locate the top left segment. That one with null projection
    // Also, set
    for index in 2..rb.used {
        match rb.planned_segment_from_tail(index) {
            Ok(prev_segment_candidate) => {
                if prev_segment_candidate.segment_data.proj_next.is_defined_positive() {
                    left_offset = index;
                    left_watermark = prev_segment_candidate.segment_data.speed_enter_mms;
                }
                else {
                    break;
                }
            }
            Err(_) => break
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
            let (left_segment, right_segment) = match rb.entries_from_tail(left_offset, right_offset) {
                (Some(PlanEntry::PlannedMove(_s, _, _, _)), Some(PlanEntry::PlannedMove(_t, _, _, _))) => { (_s, _t) }
                _ => panic!("")
            };

            hwa::trace!("\tleft [{}] right[{}]", left_offset, right_offset);

            let left_max_inc = (left_segment.segment_data.proj_next * left_segment.segment_data.speed_target_mms)
                .min(left_segment.segment_data.speed_exit_constrained_mms);

            let right_max_inc = (right_segment.segment_data.proj_prev * right_segment.segment_data.speed_target_mms)
                .min(right_segment.segment_data.speed_enter_constrained_mms);

            let water_left = (left_watermark + left_max_inc).min(left_segment.segment_data.speed_target_mms);
            let water_right = (right_watermark + right_max_inc).min(right_segment.segment_data.speed_target_mms);

            if water_left <= water_right { // Flood at right
                hwa::trace!("flood to right  [{} {}]", water_left, water_right);
                left_segment.segment_data.speed_enter_mms = left_watermark;
                left_segment.segment_data.speed_exit_mms = water_left;
                right_segment.segment_data.speed_enter_mms = water_left;
                right_segment.segment_data.speed_exit_mms = right_watermark;
                left_watermark = water_left;
                left_offset -= 1;
            }
            else { // Flood at left
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
pub fn display_content(rb: &MutexGuard<ControllerMutexType, RingBuffer>, left_offset: u8, right_offset: u8) -> Result<(),()>{
    let mut stb = Vec::new();
    for i in 0 .. left_offset - right_offset + 1 {
        let s = rb.planned_segment_from_tail(left_offset - i).unwrap();
        stb.push(format!("{}[{},{}]", s.id, s.segment_data.speed_enter_mms, s.segment_data.speed_exit_mms))
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
    pub fn index_from_tail(&self, offset: u8) -> Result<u8,()> {
        if offset > self.used {
            Err(())
        }
        else {
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
        let len =  self.data.len() as u16;

        let index = match absolute_offset < len {
            true => absolute_offset as u8,
            false => (absolute_offset - len) as u8,
        };
        self.data.get(index as usize)
    }
    #[allow(unused)]
    pub fn mut_entry_from_tail(&mut self, offset: u8) -> Option<&mut PlanEntry> {
        let absolute_offset = self.head as u16 + self.used as u16 - offset as u16;
        let len =  self.data.len() as u16;

        let index = match absolute_offset < len {
            true => absolute_offset as u8,
            false => (absolute_offset - len) as u8,
        };
        self.data.get_mut(index as usize)
    }
    #[allow(unused)]
    pub fn mut_planned_segment_from_tail(&mut self, offset: u8) -> Result<&mut Segment, ()> {
        match self.mut_entry_from_tail(offset) {
            Some(PlanEntry::PlannedMove(_s, _, _, _)) => { Ok(_s) }
            _ => Err(())
        }
    }

    #[allow(unused)]
    pub fn planned_segment_from_tail(&self, offset: u8) -> Result<&Segment, ()> {
        match self.entry_from_tail(offset) {
            Some(PlanEntry::PlannedMove(_s, _, _, _)) => { Ok(_s) }
            _ => Err(())
        }
    }

    #[allow(unused)]
    pub fn entries_from_tail(&mut self, offset1: u8, offset2: u8) -> (Option<&mut PlanEntry>, Option<&mut PlanEntry>) {
        let len =  self.data.len();
        let absolute_offset1 = self.head as usize + self.used as usize - offset1 as usize;
        let index1 = match absolute_offset1 < len {
            true => absolute_offset1 as usize,
            false => (absolute_offset1 - len) as usize,
        };
        let absolute_offset2 = self.head as usize + self.used as usize - offset2 as usize;
        let index2 = match absolute_offset2 < len {
            true => absolute_offset2 as usize,
            false => (absolute_offset2 - len) as usize,
        };
        if index1 == index2 {
            (self.data.get_mut(index1), None)
        }
        else if index1 < index2 {
            let (l, r) = self.data.split_at_mut(index2);
            (l.get_mut(index1), r.get_mut(0))
        }
        else {
            let (l, r) = self.data.split_at_mut(index1);
            (r.get_mut(0), l.get_mut(index2))
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