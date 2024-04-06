#![no_std]

mod pid;
mod utils;

pub mod angle_sensor;
pub mod current_sensor;

pub mod driver;

pub use pid::PID;

use angle_sensor::AngleSensor;
use core::f32::consts::PI;
use current_sensor::CurrentSensor;
use driver::interface::{Adcs, Pwms};
use micromath::F32Ext;
use utils::{inv_park, park, svpwm, AngleSinCos};

pub enum SetError {
    NoRequireSetCurrent,
    NoRequireSetVelocity,
    NoRequireSetPosition,
}

pub enum CurrentLoopError {
    MisCurrentSensor,
    MisCurrentAdcs,
    MisAngleSensor,
    NoAngleData,
}

pub enum VelocityLoopError {
    NoRequireVelocityLoop,
}

pub enum PositionLoopError {
    NoRequirePositionLoop,
}

#[allow(dead_code)]
pub enum LoopMode {
    None,
    OpenVelocity {
        voltage: f32,
        expect_velocity: f32,
    },
    Calibration,
    PositionWithSensor {
        current_pid: PID,
        velocity_pid: PID,
        position_pid: PID,
        expect_position: f32,
    },
    VelocityWithSensor {
        current_pid: PID,
        velocity_pid: PID,
        expect_velocity: f32,
    },
    TorqueWithSensor {
        current_pid: PID,
        expect_current: f32,
    },
    Velocity {
        //TODO 无感不会写
    },
}

#[allow(dead_code)]
pub struct MotorParams {
    pub pole_num: u32,
    pub resistance: Option<f32>,
    pub inductance: Option<f32>,
    pub encoder_offset: Option<f32>,
}

pub struct CurrentState {
    pub i_uvw: (f32, f32, f32), // u v w
    pub i_dq: (f32, f32),       // id, iq
    pub velocity: f32,
    pub position: f32,
}

#[allow(dead_code)]
pub struct FOC {
    motor_params: MotorParams,
    loop_mode: LoopMode,
    current_sensor: Option<CurrentSensor>,

    current_state: CurrentState,

    current_loop_dt: f32,
    velocity_loop_dt: f32,
    position_loop_dt: f32,
}

impl FOC {
    #[allow(dead_code)]
    pub fn new(
        motor_params: MotorParams,
        loop_mode: LoopMode,
        current_sensor: Option<CurrentSensor>,
        // angle_sensor: Option<AngleSensor>,
        current_loop_dt: f32,
        velocity_loop_dt: f32,
        position_loop_dt: f32,
    ) -> Self {
        Self {
            motor_params,
            loop_mode,
            current_sensor,
            current_state: CurrentState {
                i_uvw: (0., 0., 0.),
                i_dq: (0., 0.),
                velocity: 0.,
                position: 0.,
            },
            current_loop_dt,
            velocity_loop_dt,
            position_loop_dt,
        }
    }

    fn update_i_velocity_position(
        pole_num: u32,
        current_loop_dt: f32,
        mut angle_sensor: Option<&mut impl AngleSensor>,
        mut current_sensor_adcs: (Option<&mut CurrentSensor>, Option<&mut dyn Adcs>),
        current_state: &mut CurrentState,
    ) -> Result<AngleSinCos, CurrentLoopError> {
        let current_sensor = current_sensor_adcs
            .0
            .as_mut()
            .ok_or(CurrentLoopError::MisCurrentSensor)?;
        let uvw_adcs = current_sensor_adcs
            .1
            .as_mut()
            .ok_or(CurrentLoopError::MisCurrentAdcs)?;
        let angle_sensor = angle_sensor
            .as_mut()
            .ok_or(CurrentLoopError::MisAngleSensor)?;

        let last_position = current_state.position;
        let angle = angle_sensor
            .get_angle()
            .map_err(|_| CurrentLoopError::NoAngleData)?;
        current_state.position = angle * pole_num as f32 % (2. * PI);

        let mut delta_position = current_state.position - last_position;
        if delta_position.abs() > 2. * PI / 3. {
            if delta_position > 0. {
                // From negitive to positive deg.
                delta_position = -(2. * PI - last_position.abs() + current_state.position);
            } else {
                // From positive to negitive deg.
                delta_position = 2. * PI - last_position + current_state.position.abs();
            }
        }
        current_state.velocity = delta_position / current_loop_dt;
        let (a, b, c) = current_sensor.get_currnet(*uvw_adcs);
        current_state.i_uvw = (a, b, c);

        let angle_sin_cos = AngleSinCos::new(current_state.position);
        current_state.i_dq = park(a, b, c, &angle_sin_cos);
        Ok(angle_sin_cos)
    }

    #[allow(dead_code)]
    pub fn current_tick(
        &mut self,
        _bus_voltage: f32,
        angle_sensor: Option<&mut impl AngleSensor>,
        uvw_adcs: Option<&mut dyn Adcs>,
        pwm: &mut dyn Pwms,
    ) -> Result<(), CurrentLoopError> {
        let (ud, uq, angle_sin_cos) = match &mut self.loop_mode {
            LoopMode::None => (0., 0., AngleSinCos::new(0.)),
            LoopMode::OpenVelocity {
                voltage,
                expect_velocity: _,
            } => (0., *voltage, AngleSinCos::new(self.current_state.position)),
            LoopMode::Calibration => (0., 0., AngleSinCos::new(0.)),
            LoopMode::TorqueWithSensor {
                current_pid: _,
                expect_current,
            } => {
                let angle_sin_cos = Self::update_i_velocity_position(
                    self.motor_params.pole_num,
                    self.current_loop_dt,
                    angle_sensor,
                    (self.current_sensor.as_mut(), uvw_adcs),
                    &mut self.current_state,
                )?;
                // let iq = current_pid.update(*expect_current - self.current_i.1);
                // (0., iq, self.current_position)
                (0., *expect_current, angle_sin_cos)
            }
            LoopMode::VelocityWithSensor {
                current_pid,
                velocity_pid,
                expect_velocity: _,
            } => {
                let angle_sin_cos = Self::update_i_velocity_position(
                    self.motor_params.pole_num,
                    self.current_loop_dt,
                    angle_sensor,
                    (self.current_sensor.as_mut(), uvw_adcs),
                    &mut self.current_state,
                )?;
                let iq = current_pid.update(velocity_pid.last_output - self.current_state.i_dq.1);
                (0., iq, angle_sin_cos)
            }
            LoopMode::PositionWithSensor {
                current_pid,
                velocity_pid,
                position_pid: _,
                expect_position: _,
            } => {
                let angle_sin_cos = Self::update_i_velocity_position(
                    self.motor_params.pole_num,
                    self.current_loop_dt,
                    angle_sensor,
                    (self.current_sensor.as_mut(), uvw_adcs),
                    &mut self.current_state,
                )?;
                let iq = current_pid.update(velocity_pid.last_output - self.current_state.i_dq.1);
                (0., iq, angle_sin_cos)
            }
            LoopMode::Velocity {} => (0., 0., AngleSinCos::new(0.)),
        };

        let (a, b, c) = inv_park(
            ud * 3_f32.sqrt() / 2.,
            uq * 3_f32.sqrt() / 2.,
            &angle_sin_cos,
        );
        let (a, b, c) = svpwm(a, b, c);
        pwm.set_duty(a, b, c);

        Ok(())
    }

    #[allow(dead_code)]
    pub fn velocity_tick(&mut self) -> Result<(), VelocityLoopError> {
        match &mut self.loop_mode {
            LoopMode::None => (),
            LoopMode::OpenVelocity {
                voltage: _,
                expect_velocity,
            } => {
                self.current_state.position +=
                    *expect_velocity * self.velocity_loop_dt * self.motor_params.pole_num as f32;
                if self.current_state.position.abs() >= PI {
                    self.current_state.position = if *expect_velocity > 0. { -PI } else { PI }
                };
            }
            LoopMode::Calibration => (),
            LoopMode::VelocityWithSensor {
                current_pid: _,
                velocity_pid,
                expect_velocity,
            } => {
                velocity_pid.update(*expect_velocity - self.current_state.velocity);
            }
            LoopMode::PositionWithSensor {
                current_pid: _,
                velocity_pid,
                position_pid,
                expect_position: _,
            } => {
                velocity_pid.update(position_pid.last_output - self.current_state.velocity);
            }
            LoopMode::TorqueWithSensor {
                current_pid: _,
                expect_current: _,
            } => {
                return Err(VelocityLoopError::NoRequireVelocityLoop);
            }
            LoopMode::Velocity {} => {
                //TODO 等大佬写无感
            }
        }
        Ok(())
    }

    #[allow(dead_code)]
    pub fn position_tick(&mut self) -> Result<(), PositionLoopError> {
        match &mut self.loop_mode {
            LoopMode::PositionWithSensor {
                current_pid: _,
                velocity_pid: _,
                position_pid,
                expect_position,
            } => {
                position_pid.update(*expect_position - self.current_state.position);
            }
            _ => {
                return Err(PositionLoopError::NoRequirePositionLoop);
            }
        }
        Ok(())
    }

    #[allow(dead_code)]
    pub fn set_expect_current(&mut self, current: f32) -> Result<(), SetError> {
        match &mut self.loop_mode {
            LoopMode::TorqueWithSensor {
                current_pid: _,
                expect_current,
            } => {
                *expect_current = current;
                Ok(())
            }
            _ => Err(SetError::NoRequireSetCurrent),
        }
    }

    #[allow(dead_code)]
    pub fn set_expect_velocity(&mut self, velocity: f32) -> Result<(), SetError> {
        match &mut self.loop_mode {
            LoopMode::OpenVelocity {
                voltage: _,
                expect_velocity,
            } => {
                *expect_velocity = velocity;
                Ok(())
            }
            LoopMode::VelocityWithSensor {
                current_pid: _,
                velocity_pid: _,
                expect_velocity,
            } => {
                *expect_velocity = velocity;
                Ok(())
            }
            LoopMode::Velocity {} => Ok(()),
            _ => Err(SetError::NoRequireSetVelocity),
        }
    }

    #[allow(dead_code)]
    pub fn set_expect_position(&mut self, position: f32) -> Result<(), SetError> {
        match &mut self.loop_mode {
            LoopMode::PositionWithSensor {
                current_pid: _,
                velocity_pid: _,
                position_pid: _,
                expect_position,
            } => {
                *expect_position = position;
                Ok(())
            }
            _ => Err(SetError::NoRequireSetPosition),
        }
    }

    #[allow(dead_code)]
    pub fn get_position(&self) -> f32 {
        self.current_state.position / self.motor_params.pole_num as f32
    }

    #[allow(dead_code)]
    pub fn get_velocity(&self) -> f32 {
        self.current_state.velocity / self.motor_params.pole_num as f32
    }

    #[allow(dead_code)]
    pub fn get_i_dq(&self) -> (f32, f32) {
        self.current_state.i_dq
    }

    #[allow(dead_code)]
    pub fn get_i_uvw(&self) -> (f32, f32, f32) {
        self.current_state.i_uvw
    }
}
