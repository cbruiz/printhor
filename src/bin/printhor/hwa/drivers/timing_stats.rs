use embassy_time::{Duration, Instant};
use crate::hwa;

pub struct Timings {
    // Segment reference time
    t_segment: Instant,
    // Micro-segment reference time
    t_u_segment: Instant,
    // Time to prepare a segment
    t_segment_preparation: Duration,
    t_segment_execution: Duration,
    // Aggregated computation for every micro-segment
    t_u_segment_computation_agg: Duration,
    // Aggregated stepping time for every micro-segment
    t_u_segment_stepping_agg: Duration,
    // Aggregated remaining time for every micro-segment
    t_u_segment_remaining_agg: Duration,
    // Min computation for every micro-segment
    t_u_segment_computation_min: Duration,
    // Min stepping time for every micro-segment
    t_u_segment_stepping_min: Duration,
    // Min remaining time for every micro-segment
    t_u_segment_remaining_min: Duration,
    // Max computation for every micro-segment
    t_u_segment_computation_max: Duration,
    // Max stepping time for every micro-segment
    t_u_segment_stepping_max: Duration,
    // Max remaining time for every micro-segment
    t_u_segment_remaining_max: Duration,
    t_segment_remaining: Duration,
}

impl Timings {
    pub fn new() -> Self {
        Self {
            t_segment: Instant::now(),
            t_u_segment: Instant::now(),
            t_segment_preparation: Duration::from_ticks(0),
            t_segment_execution: Duration::from_ticks(0),
            t_u_segment_computation_agg: Duration::from_ticks(0),
            t_u_segment_stepping_agg: Duration::from_ticks(0),
            t_u_segment_remaining_agg: Duration::from_ticks(0),
            t_u_segment_computation_min: Duration::from_ticks(999999999),
            t_u_segment_stepping_min: Duration::from_ticks(999999999),
            t_u_segment_remaining_min: Duration::from_ticks(999999999),
            t_u_segment_computation_max: Duration::from_ticks(0),
            t_u_segment_stepping_max: Duration::from_ticks(0),
            t_u_segment_remaining_max: Duration::from_ticks(0),
            t_segment_remaining: Duration::from_ticks(0),
        }
    }
    pub fn u_reset(&mut self) {
        self.t_u_segment = Instant::now();
    }

    pub fn set_prep(&mut self) {
        self.t_segment_preparation = self.t_segment.elapsed();
    }

    pub fn add_u_comp(&mut self) {
        let elapsed = self.t_u_segment.elapsed();
        self.t_u_segment_computation_min = self.t_u_segment_computation_min.min(elapsed);
        self.t_u_segment_computation_max = self.t_u_segment_computation_max.max(elapsed);
        self.t_u_segment_computation_agg += elapsed;
        self.u_reset();
    }

    pub fn add_u_stepping(&mut self) {
        let elapsed = self.t_u_segment.elapsed();
        self.t_u_segment_stepping_min = self.t_u_segment_stepping_min.min(elapsed);
        self.t_u_segment_stepping_max = self.t_u_segment_stepping_max.max(elapsed);
        self.t_u_segment_stepping_agg += elapsed;
        self.u_reset();
    }

    pub fn add_u_remaining(&mut self) {
        let elapsed = self.t_u_segment.elapsed();
        self.t_u_segment_remaining_min = self.t_u_segment_remaining_min.min(elapsed);
        self.t_u_segment_remaining_max = self.t_u_segment_remaining_max.max(elapsed);
        self.t_u_segment_remaining_agg += elapsed;
        self.u_reset();
    }

    pub fn set_execution(&mut self) {
        self.t_segment_execution = self.t_segment.elapsed();
    }

    pub fn set_remaining(&mut self) {
        self.t_segment_remaining = self.t_segment.elapsed() - self.t_segment_execution;
    }

    pub fn report(&self) {
        hwa::info!("Segment timings: {} + {} + {} = {}",
            self.t_segment_preparation.as_micros(),
            self.t_segment_execution.as_micros(),
            self.t_segment_remaining.as_micros(),
            (self.t_segment_preparation + self.t_segment_execution + self.t_segment_remaining).as_micros()
        );
        hwa::info!("\tMicro-segment timings: {} + {} + {} = {}",
            self.t_u_segment_computation_agg.as_micros(),
            self.t_u_segment_stepping_agg.as_micros(),
            self.t_u_segment_remaining_agg.as_micros(),
            (self.t_u_segment_computation_agg + self.t_u_segment_stepping_agg + self.t_u_segment_remaining_agg).as_micros()
        );
        hwa::info!("\tMicro-segment ranges: [{} .. {}] + [{} .. {}] + [{} .. {}] = [{} .. {}]",
            self.t_u_segment_computation_min.as_micros(),
            self.t_u_segment_computation_max.as_micros(),
            self.t_u_segment_stepping_min.as_micros(),
            self.t_u_segment_stepping_max.as_micros(),
            self.t_u_segment_remaining_min.as_micros(),
            self.t_u_segment_remaining_max.as_micros(),
            (self.t_u_segment_computation_min + self.t_u_segment_stepping_min + self.t_u_segment_remaining_min).as_micros(),
            (self.t_u_segment_computation_max + self.t_u_segment_stepping_max + self.t_u_segment_remaining_max).as_micros(),
        );
    }
}