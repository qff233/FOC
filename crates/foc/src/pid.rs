#[allow(dead_code)]
pub struct PID {
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub time_interval: f32,
    pub output_ramp: f32,
    pub output_limit: f32,
    pub integral_limit: f32,
    pub last_output: f32,
    pub last_error: f32,
    pub last_integral: f32,
}

impl PID {
    #[allow(dead_code)]
    fn new(
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
            last_output: 0.,
            last_integral: 0.,
        }
    }

    #[allow(dead_code)]
    pub fn update(&mut self, error: f32) -> f32 {
        let dt = self.time_interval;

        let p_term = self.p * error;
        let i_term = self.last_integral + self.i * 0.5 * dt * (error + self.last_error);

        let d_term = self.d * (error - self.last_error) / dt;
        let output = p_term + i_term + d_term;

        if self.output_ramp > 0. {
            let output_rate = (output - self.last_error) / dt;

        }
        self.last_integral = i_term;

        output
    }
}
