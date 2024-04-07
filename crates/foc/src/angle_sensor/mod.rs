pub mod as5048;
pub mod mt6818;

use core::f32::consts::PI;

pub trait AngleSensor {
    type Error;

    fn set_direction(&mut self, dir: Direction);

    /// return [0, 2PI)
    fn get_angle(&mut self) -> Result<f32, Self::Error>;
}

pub struct TestAngleSensor {
    current_angle: f32,
    angle_step: f32,
    current_loop_dt: f32,
}

pub enum Direction {
    CW,
    CCW,
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
    type Error = ();

    fn set_direction(&mut self, _dir: Direction) {}

    fn get_angle(&mut self) -> Result<f32, Self::Error> {
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
        Ok(self.current_angle)
    }
}
