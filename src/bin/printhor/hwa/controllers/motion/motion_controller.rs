//! TODO: This feature is still very experimental
use crate::hwa;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use printhor_hwa_common::{EventBusRef, EventFlags, EventStatus};
use crate::control::GCode;
use crate::planner::{Constraints, SCurveMotionProfile};
use crate::math::{ONE_HUNDRED, Real, ZERO};
use crate::sync::config::Config;
use crate::tgeo::TVector;
use crate::tgeo::CoordSel;

use crate::ctrl::*;
use crate::hwa::controllers::motion::motion_segment::{Segment, SegmentData};

/// The maximum number of movements that can be queued. Warning! each one takes too memory as of now
const SEGMENT_QUEUE_SIZE: u8 = 4;
pub enum DeferType {
    AwaitRequested,
    Completed,
}

///! These are the Events that can be deferred
#[allow(unused)]
pub enum DeferEvent {
    Homing(DeferType),
    RapidMove(DeferType),
    LinearMove(DeferType),
    Dwell(DeferType),
    HotendTemperature(DeferType),
    HotbedTemperature(DeferType),
}

pub enum ScheduledMove {
    Move(SegmentData, SCurveMotionProfile),
    Homing,
    Dwell,
}

/////
#[allow(unused)]
pub struct MotionConfig {
    pub(crate) microsteps: TVector<u8>,
    pub(crate) max_accel: TVector<u16>,
    pub(crate) max_speed: TVector<u16>,
    pub(crate) max_jerk: TVector<u16>,
    pub(crate) default_travel_speed: u16,
    pub(crate) flow_rate: u8,
    pub(crate) speed_rate: u8,
}

impl MotionConfig {
    pub(crate) const fn new() -> Self {
        Self {
            microsteps: TVector::new(),
            max_accel: TVector::new(),
            max_speed: TVector::new(),
            max_jerk: TVector::new(),
            default_travel_speed: 1,
            flow_rate: 100,
            speed_rate: 100,
        }
    }
}

#[allow(unused)]
pub struct MotionStatus {
    pub(crate) last_planned_pos: Option<TVector<Real>>,
}

impl MotionStatus {
    pub const fn new() -> Self {
        Self {
            last_planned_pos: None,
        }
    }
}


#[allow(unused)]
pub struct MotionPlanner {
    pub event_bus: EventBusRef,
    pub defer_channel: Channel<CriticalSectionRawMutex, DeferEvent, 4>,
    pub(self) ringbuffer: Mutex<CriticalSectionRawMutex, RingBuffer>,
    pub(self) move_planned: Config<CriticalSectionRawMutex, bool>,
    pub(self) available: Config<CriticalSectionRawMutex, bool>,
    pub(self) motion_cfg: Mutex<CriticalSectionRawMutex, MotionConfig>,
    pub(self) motion_st: Mutex<CriticalSectionRawMutex, MotionStatus>,
    pub motion_driver: Mutex<CriticalSectionRawMutex, hwa::drivers::MotionDriver>,

}

#[allow(unused)]
impl MotionPlanner {
    pub const fn new(event_bus: EventBusRef, motion_driver: hwa::drivers::MotionDriver) -> Self {
        Self {
            defer_channel: Channel::new(),
            event_bus,
            ringbuffer: Mutex::new(RingBuffer::new()),
            move_planned: Config::new(),
            available: Config::new(),
            motion_cfg: Mutex::new(MotionConfig::new()),
            motion_st: Mutex::new(MotionStatus::new()),
            motion_driver: Mutex::new(motion_driver),
        }
    }

    pub async fn start(&self) {
        self.move_planned.reset();
        self.available.signal(true);
        self.event_bus.publish_event(EventStatus::containing(EventFlags::MOV_QUEUE_EMPTY)).await;
    }

    pub async fn get_current_segment_data(&self) -> Option<Segment> {
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
                    PlanEntry::Dwell => {
                        rb.data[head] = PlanEntry::Executing(MovType::Dwell);
                        do_dwell = true;
                    },
                    PlanEntry::PlannedMove(planned_data) => {
                        hwa::debug!("Exec starting: {} / {} h={}", rb.used, SEGMENT_QUEUE_SIZE, head);
                        rb.data[head] = PlanEntry::Executing(MovType::Move);
                        return Some(planned_data);
                    },
                    PlanEntry::Homing => {
                        self.event_bus.publish_event(EventStatus::containing(EventFlags::HOMMING)).await;
                        rb.data[head] = PlanEntry::Executing(MovType::Homing);
                        return None;
                    },
                    PlanEntry::Executing(_) => {
                        self.move_planned.reset();
                        hwa::error!("Unexpected error");
                    },
                }
            }
            if do_dwell {
                self.consume_current_segment_data().await;
            }
        }
    }

    pub async fn consume_current_segment_data(&self) {

        let mut rb = self.ringbuffer.lock().await;
        let head = rb.head;
        hwa::debug!("Movement completed @rq[{}] (ongoing={})", head, rb.used - 1);
        match & rb.data[head as usize] {
            PlanEntry::Executing(MovType::Homing) => {
                self.event_bus.publish_event(EventStatus::not_containing(EventFlags::HOMMING)).await;
                self.defer_channel.send(DeferEvent::Homing(DeferType::Completed)).await;
            }
            PlanEntry::Executing(MovType::Dwell) => {
                self.defer_channel.send(DeferEvent::Dwell(DeferType::Completed)).await;
            }
            PlanEntry::Executing(MovType::Move) => {
            }
            _ => {
                panic!("cound not happen")
            }
        }
        rb.data[head as usize] = PlanEntry::Empty;
        rb.head = match head + 1 < SEGMENT_QUEUE_SIZE {
            true => head + 1,
            false => 0u8,
        };
        rb.used -= 1;
        hwa::debug!("- used={}, h={} ", rb.used, head);
        if rb.used == 0 {
            self.event_bus.publish_event(EventStatus::containing(EventFlags::MOV_QUEUE_EMPTY)).await;
        }
        self.available.signal(true);
    }

    pub async fn schedule_raw_move(&self, move_type: ScheduledMove, blocking: bool) -> Result<CodeExecutionSuccess, CodeExecutionFailure> {

        loop {
            self.available.wait().await;
            {
                let mut rb = self.ringbuffer.lock().await;

                let mut must_defer = true;

                if rb.used < (SEGMENT_QUEUE_SIZE as u8) {
                    let used = rb.used;
                    let head = rb.head;
                    let mut could_replan = false;

                    let (entry, event) = match move_type {
                        ScheduledMove::Move(segment_data, motion_profile) => {
                            could_replan = true;
                            must_defer = false;
                            self.update_last_planned_pos(&segment_data.dest_pos).await;
                            (PlanEntry::PlannedMove(Segment::new(segment_data, motion_profile)), EventStatus::new())
                        }
                        ScheduledMove::Homing => {
                            self.set_last_planned_pos(&TVector::zero()).await;
                            (PlanEntry::Homing, EventStatus::not_containing(EventFlags::HOMMING))
                        }
                        ScheduledMove::Dwell => {
                            (PlanEntry::Dwell, EventStatus::containing(EventFlags::MOV_QUEUE_EMPTY))
                        }
                    };

                    let index = head as u16 + used as u16;

                    let index = match index < SEGMENT_QUEUE_SIZE as u16 {
                        true => index,
                        false => index - SEGMENT_QUEUE_SIZE as u16,
                    } as u8;

                    rb.data[index as usize] = entry;

                    hwa::debug!("Mov queued @{} ({} / {})", index, rb.used + 1, SEGMENT_QUEUE_SIZE);
                    if could_replan && used > 0 {
                        let last_inserted_idx = rb.head as u16 + rb.used as u16 - 1;
                        let last_inserted_idx = match last_inserted_idx < (SEGMENT_QUEUE_SIZE as u16) {
                            true => last_inserted_idx,
                            false => 0,
                        };
                        hwa::debug!(" - check_replan {}", last_inserted_idx);
                        let current_slot = &mut rb.data[last_inserted_idx as usize];
                        match current_slot {
                            PlanEntry::Executing(_) | PlanEntry::Homing | PlanEntry::Empty => {
                                hwa::debug!(" -- not chained")
                            }
                            PlanEntry::PlannedMove(old_data) => {
                                old_data.motion_profile.recalculate();
                                hwa::debug!(" -- chained")
                            }
                            _ => {
                                unreachable!("Could not happen");
                            }
                        }
                    }
                    rb.used += 1;
                    self.event_bus.publish_event(EventStatus::not_containing(EventFlags::MOV_QUEUE_EMPTY)).await;
                    self.move_planned.signal(true);
                    if must_defer || (rb.used == (SEGMENT_QUEUE_SIZE as u8)) {
                        // Shall wait for one deallocation in order to enqueue more
                        return Ok(CodeExecutionSuccess::DEFERRED(event))
                    }
                    else {
                        return Ok(CodeExecutionSuccess::QUEUED)
                    }

                } else {
                    self.available.reset();
                    if !blocking {
                        hwa::warn!("Mov rejected: {} / {} h={}", rb.used, SEGMENT_QUEUE_SIZE, rb.head);
                        return Err(CodeExecutionFailure::BUSY)
                    }
                }
            }
        }
    }

    pub fn motion_cfg(&self) -> &'_ Mutex<CriticalSectionRawMutex,MotionConfig> {
        &self.motion_cfg
    }

    pub async fn get_last_planned_pos(&self) -> Option<TVector<Real>> {
        self.motion_st.lock().await.last_planned_pos.clone()
    }
    pub async fn set_last_planned_pos(&self, pos: &TVector<Real>) {
        self.motion_st.lock().await.last_planned_pos.replace(pos.map_nan(Real::zero()));
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

    pub async fn get_flow_rate(&self) -> u8 {
        self.motion_cfg.lock().await.flow_rate
    }

    pub async fn set_flow_rate(&self, rate: u8) {
        self.motion_cfg.lock().await.flow_rate = rate;
    }

    pub async fn get_flow_rate_as_real(&self) -> Real {
        Real::new(self.motion_cfg.lock().await.flow_rate as i64, 0)
    }

    pub async fn get_speed_rate(&self) -> u8 {
        self.motion_cfg.lock().await.speed_rate
    }

    pub async fn set_speed_rate(&self, rate: u8) {
        self.motion_cfg.lock().await.speed_rate = rate;
    }

    pub async fn get_speed_rate_as_real(&self) -> Real {
        Real::new(self.motion_cfg.lock().await.speed_rate as i64, 0)
    }

    pub async fn get_default_travel_speed(&self) -> u16 {
        self.motion_cfg.lock().await.default_travel_speed
    }
    pub async fn set_default_travel_speed(&self, speed: u16) {
        self.motion_cfg.lock().await.default_travel_speed = speed;
    }

    pub async fn get_default_travel_speed_as_real(&self) -> Real {
        Real::new(self.motion_cfg.lock().await.default_travel_speed as i64, 0)
    }

    pub async fn get_max_accel(&self) -> TVector<u16> {
        self.motion_cfg.lock().await.max_accel
    }

    pub async fn get_max_speed(&self) -> TVector<u16> {
        self.motion_cfg.lock().await.max_speed
    }
    pub async fn set_max_speed(&self, speed: TVector<u16>) {
        self.motion_cfg.lock().await.max_speed.assign(CoordSel::all(), &speed);
    }
    pub async fn get_max_speed_as_vreal(&self) -> TVector<Real> {
        self.motion_cfg.lock().await.max_speed.map_coords(|c| Some(Real::new(c as i64, 0)))
    }

    pub async fn set_max_accel(&self, accel: TVector<u16>) {
        self.motion_cfg.lock().await.max_accel.assign(CoordSel::all(), &accel);
    }

    pub async fn get_max_accel_as_vreal(&self) -> TVector<Real> {
        self.motion_cfg.lock().await.max_accel.map_coords(|c| Some(Real::new(c as i64, 0)))
    }

    pub async fn set_max_jerk(&self, jerk: TVector<u16>) {
        self.motion_cfg.lock().await.max_jerk.assign(CoordSel::all(), &jerk);
    }

    pub async fn plan(&self, gc: &GCode, blocking: bool) -> Result<CodeExecutionSuccess, CodeExecutionFailure>{
        match gc {
            GCode::G0(t) => {
                Ok(self.schedule_move(TVector{
                    x: t.x, y: t.y, z: t.z, e: None,
                }, t.f, blocking).await?)
            }
            GCode::G1(t) => {
                Ok(self.schedule_move(TVector{
                    x: t.x, y: t.y, z: t.z, e: t.e
                }, t.f, blocking).await?)
            }
            GCode::G4 => {
                Ok(self.schedule_raw_move(ScheduledMove::Dwell, blocking).await?)
            }
            GCode::G28(_x) => {
                // FIXME Remove when complete
                self.defer_channel.send(DeferEvent::Homing(DeferType::AwaitRequested)).await;
                self.event_bus.publish_event(EventStatus::containing(EventFlags::HOMMING)).await;
                Ok(self.schedule_raw_move(ScheduledMove::Homing, blocking).await?)
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

    async fn schedule_move(&self, p1: TVector<Real>, requested_motion_speed: Option<Real>, blocking: bool) -> Result<CodeExecutionSuccess, CodeExecutionFailure> {

        let t0 = embassy_time::Instant::now();

        let p0 = self.get_last_planned_pos().await.ok_or(CodeExecutionFailure::HomingRequired)?;

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
        let speed_module = requested_motion_speed.unwrap_or(dts) * speed_rate;
        // Compute per-axis target speed
        let speed_vector: TVector<Real> = vdir.abs() * speed_module;
        // Clamp per-axis target speed to the physical restrictions
        let clamped_speed = speed_vector.clamp(max_speed);
        // Finally, per-axis relative speed
        let speed_rate = clamped_speed / speed_vector;

        let module_target_speed = clamped_speed.min().unwrap_or(ZERO);
        let module_target_accel = (max_accel * speed_rate).min().unwrap_or(ZERO);
        let module_target_jerk = (max_jerk * speed_rate).min().unwrap_or(ZERO);

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
                    let profile = SCurveMotionProfile::compute(module_target_distance.clone(), ZERO, ZERO, &_constraints)?;
                    Some(profile)
                },
                false => {
                    hwa::error!("p0: {}", p0.rdp(4));
                    hwa::error!("p1: {}", p1.rdp(4));
                    hwa::error!("dist: {} mm", module_target_distance.rdp(4));
                    hwa::error!("vdir: {} mm/s", vdir.rdp(4));
                    hwa::error!("speed_vector: {} mm/s", speed_vector.rdp(4));
                    hwa::error!("clamped_speed: {} mm/s", clamped_speed.rdp(4));
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
                        speed_enter_mms: 0,
                        speed_exit_mms: 0,
                        displacement_u: (module_target_distance * Real::from_lit(1000, 0)) .to_i32().unwrap_or(0) as u32,
                        vdir,
                        dest_pos: Default::default(),
                    };
                    let r = self.schedule_raw_move(
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

    #[inline(always)]
    pub async fn do_homing(&self) -> Result<(), ()>{
        let r = self.motion_driver.lock().await.homing_action().await;
        if r.is_err() {
            self.event_bus.publish_event(EventStatus::containing(EventFlags::SYS_ALARM));
        }
        else {
            self.event_bus.publish_event(EventStatus::not_containing(EventFlags::HOMMING));
        }
        r
    }
}


#[allow(unused)]
pub struct RingBuffer {
    pub(self) data: [PlanEntry; SEGMENT_QUEUE_SIZE as usize],
    pub(self) head: u8,
    pub(self) used: u8,
}

impl RingBuffer {
    pub const fn new() -> Self {
        Self {
            data: [PlanEntry::Empty; SEGMENT_QUEUE_SIZE as usize],
            head: 0,
            used: 0,
        }
    }
}

#[derive(Clone, Copy)]
pub enum MovType {
    Move,
    Homing,
    Dwell,
}

#[allow(unused)]
#[derive(Clone, Copy)]
pub enum PlanEntry {
    Empty,
    PlannedMove(Segment),
    Homing,
    Dwell,
    Executing(MovType),
}

impl Default for PlanEntry {
    fn default() -> Self {
        PlanEntry::Empty
    }
}



#[derive(Clone)]
pub struct MotionPlannerRef {
    pub(crate) inner: &'static MotionPlanner
}

//#[cfg(feature = "native")]
unsafe impl Send for MotionPlannerRef {}

impl core::ops::Deref for MotionPlannerRef {
    type Target = MotionPlanner;

    fn deref(&self) -> &Self::Target {
        self.inner
    }
}