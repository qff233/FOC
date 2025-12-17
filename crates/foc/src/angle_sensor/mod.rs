pub mod as5048;
pub mod mt6818;

use core::f32::consts::PI;

pub enum AngleSensorError {
    NoMagWarning,
    DataError,
    NoData,
}

pub trait AngleSensor {
    fn set_direction(&mut self, dir: Direction);

    /// return [0, 2PI)
    fn get_lap_angle(&mut self) -> Result<f32, AngleSensorError>;
    fn get_angle(&mut self) -> Result<(i32, f32), AngleSensorError>;
}

pub struct TestAngleSensor {
    current_angle: f32,
    angle_step: f32,
    current_loop_dt: f32,

    count_lap: i32,
    last_angle: f32,
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
            count_lap: 0,
            last_angle: 0.,
        }
    }
}

impl AngleSensor for TestAngleSensor {
    fn set_direction(&mut self, _dir: Direction) {}

    fn get_lap_angle(&mut self) -> Result<f32, AngleSensorError> {
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

    fn get_angle(&mut self) -> Result<(i32, f32), AngleSensorError> {
        let current_angle = self.get_lap_angle()?;
        let delta_angle = current_angle - self.last_angle;
        if delta_angle.abs() > PI {
            if delta_angle.is_sign_positive() {
                self.count_lap -= 1;
            } else {
                self.count_lap += 1;
            }
        }
        self.last_angle = current_angle;
        Ok((self.count_lap, current_angle))
    }
}
