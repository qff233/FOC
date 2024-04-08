#[allow(dead_code)]
pub struct Pid {
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub time_interval: f32,
    pub output_ramp: f32,
    pub output_limit: f32,
    pub integral_limit: f32,
    pub last_output: f32,
    pub integral: f32,
    pub last_error: f32,
}

impl Pid {
    #[allow(dead_code)]
    pub fn new(
        p: f32,
        i: f32,
        d: f32,
        time_interval: f32,
        output_ramp: f32,
        output_limit: f32,
        integral_limit: f32,
    ) -> Self {
        Self {
            p,
            i,
            d,
            time_interval,
            output_limit,
            output_ramp,
            integral_limit,
            last_error: 0.,
            integral: 0.,
            last_output: 0.,
        }
    }

    #[allow(dead_code)]
    pub fn update(&mut self, error: f32) -> f32 {
        let dt = self.time_interval;
        self.integral += self.i * 0.5 * dt * (error + self.last_error);
        self.integral = self
            .integral
            .min(self.integral_limit)
            .max(-self.integral_limit);

        // debug!("{}", self.integral);

        let d_term = self.d * (error - self.last_error) / dt;
        let mut output = self.p * error + self.integral + d_term;

        if self.output_ramp > 0. {
            let output_rate = (output - self.last_error) / dt;
            if output_rate > self.output_ramp {
                output = self.last_output + self.output_ramp * dt;
            } else if output_rate < -self.output_ramp {
                output = self.last_output - self.output_ramp * dt;
            }
        }
        output = output.max(-self.output_limit).min(self.output_limit);

        self.last_output = output;
        self.last_error = error;

        output
    }
}
