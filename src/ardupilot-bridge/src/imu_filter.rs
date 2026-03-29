/// Coning and sculling IMU pre-integrator.
///
/// Accumulates N high-rate (1500 Hz) IMU samples and outputs one
/// corrected sample at ~400 Hz for ArduPilot's SCHED_LOOP_RATE.
///
/// **Coning correction** captures the non-commutativity of rotations:
/// when a body oscillates in a conical pattern within the accumulation
/// window, naive averaging of angular rate loses the net rotation.
/// The cross-product correction term recovers it.
///
/// **Sculling correction** accounts for the coupling between rotation
/// and acceleration within the window: if the body rotates while
/// accelerating, the average specific force in the body frame differs
/// from the naive average of the samples.
///
/// References:
///   - Savage, "Strapdown Inertial Navigation Integration Algorithm Design"
///   - VectorNav Technical Note: "Coning and Sculling Integrals"

const SAMPLES_PER_OUTPUT: u32 = 4;

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn add3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale3(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

pub struct FilteredImu {
    pub gyro: [f64; 3],
    pub accel: [f64; 3],
    pub dt: f64,
}

pub struct ConingIntegrator {
    accum_delta_angle: [f64; 3],
    accum_delta_vel: [f64; 3],
    prev_delta_angle: [f64; 3],
    coning_integral: [f64; 3],
    accum_dt: f64,
    sample_count: u32,
}

impl ConingIntegrator {
    pub fn new() -> Self {
        Self {
            accum_delta_angle: [0.0; 3],
            accum_delta_vel: [0.0; 3],
            prev_delta_angle: [0.0; 3],
            coning_integral: [0.0; 3],
            accum_dt: 0.0,
            sample_count: 0,
        }
    }

    /// Feed one raw IMU sample (already in FRD frame).
    /// Returns `Some(FilteredImu)` every SAMPLES_PER_OUTPUT samples.
    pub fn push(
        &mut self,
        gyro_rads: [f64; 3],
        accel_ms2: [f64; 3],
        dt: f64,
    ) -> Option<FilteredImu> {
        if dt <= 0.0 || dt > 0.1 {
            return None;
        }

        let delta_angle = scale3(gyro_rads, dt);
        let delta_vel = scale3(accel_ms2, dt);

        // Coning correction: accumulate (2/3) * cross(prev_delta_angle, delta_angle)
        if self.sample_count > 0 {
            let coning_term = scale3(cross(self.prev_delta_angle, delta_angle), 2.0 / 3.0);
            self.coning_integral = add3(self.coning_integral, coning_term);
        }

        // Sculling correction: delta_vel += (1/2) * cross(accum_delta_angle, delta_vel_i)
        let sculling_term = scale3(cross(self.accum_delta_angle, delta_vel), 0.5);
        let corrected_delta_vel = add3(delta_vel, sculling_term);

        self.accum_delta_angle = add3(self.accum_delta_angle, delta_angle);
        self.accum_delta_vel = add3(self.accum_delta_vel, corrected_delta_vel);
        self.prev_delta_angle = delta_angle;
        self.accum_dt += dt;
        self.sample_count += 1;

        if self.sample_count >= SAMPLES_PER_OUTPUT {
            let total_dt = self.accum_dt;
            let corrected_angle = add3(self.accum_delta_angle, self.coning_integral);

            let output = FilteredImu {
                gyro: scale3(corrected_angle, 1.0 / total_dt),
                accel: scale3(self.accum_delta_vel, 1.0 / total_dt),
                dt: total_dt,
            };

            self.accum_delta_angle = [0.0; 3];
            self.accum_delta_vel = [0.0; 3];
            self.prev_delta_angle = [0.0; 3];
            self.coning_integral = [0.0; 3];
            self.accum_dt = 0.0;
            self.sample_count = 0;

            Some(output)
        } else {
            None
        }
    }
}
