//! A module to hold and plot discrete signal changes

use crate::{hwa, motion};

pub(crate) struct DataPointsDimension {
    time_offset: f64,
    last_time: f64,
    point_offset: f64,
    last_point: f64,
    min: Option<f64>,
    max: Option<f64>,
    /// Position datapoint time in secs
    pub times: Vec<f64>,
    /// Position datapoint displacement in units
    pub points: Vec<f64>,
}

impl DataPointsDimension {
    pub const fn new() -> Self {
        Self {
            time_offset: 0.0,
            last_time: 0.0,
            point_offset: 0.0,
            last_point: 0.0,
            min: None,
            max: None,
            times: vec![],
            points: vec![],
        }
    }

    pub fn accumulate_offset(&mut self) {
        // last_point already contains the absolute position (point_offset + segment_displacement).
        // Set the new offset to this absolute value so the next segment starts from here.
        self.point_offset = self.last_point;
    }

    pub fn displace_time(&mut self) {
        self.time_offset += self.last_time;
    }
    
    fn update_min_max(&mut self, value: f64) {
        if let Some(current_min) = self.min {
            self.min = Some(current_min.min(value));
        }
        else {
            self.min = Some(value);
        }
        if let Some(current_max) = self.max {
            self.max = Some(current_max.max(value));
        }
        else {
            self.max = Some(value);
        }
    }

    pub fn push(&mut self, time: f64, point: f64) {
        self.last_time = time;
        self.last_point = point;
        self.times.push(time);
        self.points.push(point);
        self.update_min_max(point);
    }
}

pub(crate) struct DataPoints {
    pub reference_time: f64,
    pub current_segment_id: usize,
    pub current_micro_segment_id: usize,
    pub current_interval: usize,
    pub total_displacement: f64,
    
    pub segment_marks: Vec<f64>,
    pub interval_marks: Vec<f64>,
    
    pub sampled_positions: DataPointsDimension,

    pub sampled_velocities: DataPointsDimension,
    pub derived_velocities: DataPointsDimension,

    pub sampled_accelerations: DataPointsDimension,
    pub derived_accelerations: DataPointsDimension,
}

impl DataPoints {
    pub fn new() -> Self {
        Self {
            reference_time: 0.0,
            current_segment_id: 0,
            current_micro_segment_id: 0,
            current_interval: 0,
            total_displacement: 0.0,

            segment_marks: Vec::new(),
            interval_marks: Vec::new(),
            
            sampled_positions: DataPointsDimension::new(),

            sampled_velocities: DataPointsDimension::new(),
            derived_velocities: DataPointsDimension::new(),

            sampled_accelerations: DataPointsDimension::new(),
            derived_accelerations: DataPointsDimension::new(),
        }
    }

    pub fn segment_starts(&mut self, _trajectory: &motion::HybridMotionProfile) {
        self.current_segment_id += 1;
        self.current_micro_segment_id = 0;
        self.current_interval = 0;
        hwa::debug!("[s-plot] segment starts at {:.4}", self.reference_time);
        self.segment_marks.push(self.reference_time);
    }
    
    pub fn segment_ends(&mut self) {
        // last_point is absolute (point_offset + segment_displacement); subtract current
        // offset to get only this segment's contribution.
        self.total_displacement += self.sampled_positions.last_point - self.sampled_positions.point_offset;
        self.segment_marks.push(self.reference_time);
        self.sampled_positions.accumulate_offset();
        self.sampled_positions.displace_time();
        self.sampled_velocities.displace_time();
        self.derived_velocities.displace_time();
        self.sampled_accelerations.displace_time();
        self.derived_accelerations.displace_time();
        hwa::debug!("[s-plot] segment ends at {:.4}", self.reference_time);
    }

    pub fn num_segments(&self) -> usize {
        self.current_segment_id
    }

    #[allow(unused)]
    pub fn current_segment_id(&self) -> usize {
        self.current_segment_id
    }

    pub fn micro_segment_ends(&mut self) {}

    #[allow(unused)]
    pub fn current_micro_segment_id(&self) -> usize {
        self.current_micro_segment_id
    }

    pub fn total_displacement(&self) -> hwa::math::Real {
        hwa::math::Real::from_f32(self.total_displacement as f32)
    }

    pub fn sampling_tick(&mut self, sampler: &motion::SegmentSampler<motion::HybridMotionProfile>) {
        // Actual width of this window (the last one of a segment may differ from the period)
        self.reference_time += sampler.dt().to_f64();
        hwa::debug!("[s-plot] segment sampled at {:.4}", self.reference_time);
        
        let interval = sampler.current_interval();
        if interval != self.current_interval {
            self.current_interval = interval;
            self.interval_marks.push(self.reference_time);
        }
        
        self.sampled_positions.push(
            self.reference_time,
            self.sampled_positions.point_offset + sampler.current_position().to_f64(),
        );
        self.sampled_velocities.push(
            self.reference_time,
            sampler.current_speed().to_f64(),
        );
        self.derived_velocities.push(
            self.reference_time,
            sampler.instant_velocity().to_f64(),
        );
        self.sampled_accelerations.push(
            self.reference_time,
            sampler.current_acceleration().to_f64(),
        );
        self.derived_accelerations.push(
            self.reference_time,
            sampler.instant_acceleration().to_f64(),
        );
    }
    
    pub fn min_position(&self) -> Option<f64> {
        self.sampled_positions.min
    }

    pub fn min_velocity(&self) -> Option<f64> {
        self.sampled_velocities.min
    }

    pub fn min_acceleration(&self) -> Option<f64> {
        self.sampled_accelerations.min
    }

    pub fn max_position(&self) -> Option<f64> {
        self.sampled_positions.max
    }

    pub fn max_velocity(&self) -> Option<f64> {
        self.sampled_velocities.max
    }

    pub fn max_acceleration(&self) -> Option<f64> {
        self.sampled_accelerations.max
    }

    pub fn write_csv(&self, path: &str) -> std::io::Result<()> {
        use std::io::Write;
        let mut file = std::fs::File::create(path)?;
        writeln!(file, "time,position,velocity,acceleration")?;
        for i in 0..self.sampled_positions.times.len() {
            writeln!(
                file,
                "{:.6},{:.6},{:.6},{:.6}",
                self.sampled_positions.times[i],
                self.sampled_positions.points[i],
                self.sampled_velocities.points[i],
                self.sampled_accelerations.points[i]
            )?;
        }
        Ok(())
    }
}
