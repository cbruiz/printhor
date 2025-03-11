//! A module to hold and plot discrete signal changes
use crate::{hwa, motion};

pub(crate) struct DataPointsDimension {
    time_offset: f64,
    last_time: f64,
    point_offset: f64,
    last_point: f64,
    /// Position datapoint time in secs
    pub times: Vec<f64>,
    /// Position datapoint displacement in units
    pub points: Vec<f64>,
}

impl DataPointsDimension {
    pub fn new() -> Self {
        Self {
            time_offset: 0.0,
            last_time: 0.0,
            point_offset: 0.0,
            last_point: 0.0,
            times: vec![0.0],
            points: vec![0.0],
        }
    }

    pub fn displace(&mut self) {
        self.displace_time();
        self.point_offset += self.last_point;
    }

    pub fn displace_time(&mut self) {
        self.time_offset += self.last_time;
    }

    pub fn push_relative(&mut self, time: f64, point: f64) {
        self.last_time = time;
        self.last_point = point;
        self.times.push(self.time_offset + self.last_time);
        self.points.push(self.point_offset + self.last_point);
    }

    pub fn push_time_relative(&mut self, time: f64, point: f64) {
        self.last_time = time;
        self.last_point = point;
        self.times.push(self.time_offset + self.last_time);
        self.points.push(self.last_point);
    }
}

pub(crate) struct DataPoints {
    pub current_segment_id: usize,
    pub current_micro_segment_id: usize,
    pub total_displacement: hwa::math::Real,

    pub segment_position_marks: DataPointsDimension,
    pub sampled_positions: DataPointsDimension,

    pub segment_velocity_marks: DataPointsDimension,
    pub sampled_velocities: DataPointsDimension,
}

impl DataPoints {
    pub fn new() -> Self {
        Self {
            current_segment_id: 0,
            current_micro_segment_id: 0,
            total_displacement: hwa::math::ZERO,

            segment_position_marks: DataPointsDimension::new(),
            sampled_positions: DataPointsDimension::new(),

            segment_velocity_marks: DataPointsDimension::new(),
            sampled_velocities: DataPointsDimension::new(),
        }
    }

    pub fn segment_starts(&mut self, trajectory: &motion::SCurveMotionProfile) {
        self.current_segment_id += 1;
        self.current_micro_segment_id = 0;
        self.segment_position_marks.push_relative(
            self.sampled_positions.last_time,
            self.sampled_positions.last_point,
        );
        self.segment_velocity_marks.push_time_relative(
            self.sampled_positions.last_time,
            trajectory.v_0.to_f64(),
        )
    }
    pub fn segment_ends(&mut self) {
        self.segment_position_marks.displace();
        self.sampled_positions.displace();
        self.segment_velocity_marks.displace_time();
        self.sampled_velocities.displace_time();
    }

    pub fn num_segments(&self) -> usize {
        self.current_segment_id
    }

    pub fn current_segment_id(&self) -> usize {
        self.current_segment_id
    }

    pub fn micro_segment_ends(&mut self) {}

    pub fn current_micro_segment_id(&self) -> usize {
        self.current_micro_segment_id
    }

    pub fn total_displacement(&self) -> hwa::math::Real {
        self.total_displacement
    }

    pub fn sampling_tick(&mut self, sampler: &motion::SegmentSampler<motion::SCurveMotionProfile>) {
        self.sampled_positions.push_relative(
            sampler.current_time().to_f64(),
            sampler.current_position().to_f64(),
        );
        self.sampled_velocities.push_relative(
            sampler.current_time().to_f64(),
            (sampler.ds() / sampler.dt()).to_f64(),
        )
    }
}
