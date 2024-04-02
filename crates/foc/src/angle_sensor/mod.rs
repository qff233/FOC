use core::f32::consts::PI;

pub trait AngleSensor {
    fn get_angle(&mut self) -> f32;
}

pub struct TestAngleSensor {
    current_angle: f32,
    angle_step: f32,
    current_loop_dt: f32,
}

impl TestAngleSensor {
    pub fn new(angle_step: f32, current_loop_dt: f32) -> Self {
        Self {
            current_angle: 0.0,
            angle_step,
            current_loop_dt,
        }
    }
}
impl AngleSensor for TestAngleSensor {
    fn get_angle(&mut self) -> f32 {
        self.current_angle += self.angle_step * self.current_loop_dt;
        if self.current_angle > 2. * PI {
            self.current_angle -= 2. * PI
        } else if self.current_angle < 0. {
            self.current_angle += 2. * PI
        }
        // debug!(
        //     "{}, {}, {}",
        //     self.current_angle, self.angle_step, self.current_loop_dt
        // );
        self.current_angle
    }
}
