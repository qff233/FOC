#![no_std]

mod lowpass;
mod pid;
mod utils;

pub mod angle_sensor;
pub mod current_sensor;

pub mod driver;

use defmt::info;
pub use lowpass::Pll2;
pub use pid::Pid;

use angle_sensor::AngleSensor;
use core::f32::consts::PI;
use current_sensor::ISensor;
use driver::interface::{Adcs, Pwms};
use micromath::F32Ext;
use utils::{park, svpwm, AngleSinCos};

pub enum SetError {
    NoRequireSetCurrent,
    NoRequireSetVelocity,
    NoRequireSetPosition,
}

pub enum CheckError {
    MisCurrentSensor,
    MisCurrentAdcs,
    MisAngleSensor,
}

pub enum CurrentLoopError {
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
    Calibration {
        has_encoder_offset: bool,
    },
    PositionWithSensor {
        current_pid: (Pid, Pid),
        position_pid: Pid,
        expect_position: (i32, f32),
    },
    PositionVelocityWithSensor {
        current_pid: (Pid, Pid),
        velocity_pid: Pid,
        position_pid: Pid,
        pll: Pll2,
        expect_position: (i32, f32),
    },
    VelocityWithSensor {
        current_pid: (Pid, Pid),
        velocity_pid: Pid,
        pll: Pll2,
        expect_velocity: f32,
    },
    TorqueWithSensor {
        current_pid: (Pid, Pid),
        expect_current: (f32, f32),
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
    pub u_dq: (f32, f32),       // ud, uq
    pub i_dq: (f32, f32),       // id, iq
    pub velocity: f32,
    pub position: (i32, f32),
}

#[allow(dead_code)]
pub struct Foc {
    motor_params: MotorParams,
    loop_mode: LoopMode,
    i_sensor: ISensor,

    current_state: CurrentState,

    current_loop_dt: f32,
    velocity_loop_dt: f32,
    position_loop_dt: f32,
}

impl Foc {
    #[allow(dead_code)]
    pub fn new(
        motor_params: MotorParams,
        i_sensor: ISensor,
        current_loop_dt: f32,
        velocity_loop_dt: f32,
        position_loop_dt: f32,
    ) -> Self {
        Self {
            motor_params,
            loop_mode: LoopMode::None,
            i_sensor,
            current_state: CurrentState {
                i_uvw: (0., 0., 0.),
                u_dq: (0., 0.),
                i_dq: (0., 0.),
                velocity: 0.,
                position: (0, 0.),
            },
            current_loop_dt,
            velocity_loop_dt,
            position_loop_dt,
        }
    }

    pub fn set_mode(mut self, loop_mode: LoopMode) -> Self {
        self.loop_mode = loop_mode;
        self
    }

    pub fn init(&mut self, angle_sensor: Option<&mut impl AngleSensor>) {
        let mut position = (0, 0.);
        if angle_sensor.is_some() {
            position = angle_sensor.unwrap().get_angle().unwrap_or((0, 0.));
        }
        self.current_state.position = position;
    }

    fn update_i_velocity_position(
        motor_params: &MotorParams,
        mut angle_sensor: Option<&mut impl AngleSensor>,
        current_sensor_adcs: (&mut ISensor, &mut dyn Adcs),
        current_state: &mut CurrentState,
    ) -> Result<AngleSinCos, CurrentLoopError> {
        let current_sensor = current_sensor_adcs.0;
        let uvw_adcs = current_sensor_adcs.1;
        let angle_sensor = angle_sensor.as_mut().unwrap();

        current_state.position = angle_sensor
            .get_angle()
            .map_err(|_| CurrentLoopError::NoAngleData)?;

        let (a, b, c) = current_sensor.get_currnet(uvw_adcs);
        current_state.i_uvw = (a, b, c);

        let encoder_offset = motor_params.encoder_offset.unwrap_or(0.);
        let mut elec_angle =
            (current_state.position.1 * motor_params.pole_num as f32 + encoder_offset) % (2. * PI);
        if elec_angle < 0. {
            elec_angle += 2. * PI;
        }
        // debug!(
        //     "angle:{}, elec_angle: {}",
        //     current_state.position, elec_angle
        // );
        let angle_sin_cos = AngleSinCos::new(elec_angle);
        current_state.i_dq = park(a, b, c, &angle_sin_cos);

        Ok(angle_sin_cos)
    }

    fn set_u_dq_angle(ud: f32, uq: f32, angle_sin_cos: &AngleSinCos, pwm: &mut dyn Pwms) {
        let (a, b, c) = svpwm(ud, uq, angle_sin_cos);
        pwm.set_duty(a, b, c);
    }

    #[allow(dead_code)]
    pub fn current_tick(
        &mut self,
        _bus_voltage: f32,
        mut angle_sensor: Option<&mut impl AngleSensor>,
        uvw_adcs: &mut dyn Adcs,
        pwm: &mut dyn Pwms,
        mut delay: impl embedded_hal::delay::DelayNs,
    ) -> Result<(), CurrentLoopError> {
        match &mut self.loop_mode {
            LoopMode::None => pwm.disable(),
            LoopMode::OpenVelocity {
                voltage,
                expect_velocity: _,
            } => {
                Self::set_u_dq_angle(
                    0.,
                    *voltage,
                    &AngleSinCos::new(self.current_state.position.1),
                    pwm,
                );
            }
            LoopMode::Calibration { has_encoder_offset } => {
                if !*has_encoder_offset {
                    Self::set_u_dq_angle(0.5, 0., &AngleSinCos::new(0.0), pwm);
                    delay.delay_ms(1000);

                    let mut encoder_offset_count = 0.;
                    for _ in 0..1000 {
                        let angle = angle_sensor
                            .as_mut()
                            .unwrap()
                            .get_lap_angle()
                            .map_err(|_| CurrentLoopError::NoAngleData)?;
                        let elec_angle = (angle * self.motor_params.pole_num as f32) % (2.0 * PI);
                        // debug!("angle:{}, ele_angle: {}", angle, elec_angle);
                        encoder_offset_count += elec_angle;
                    }
                    let encoder_offset = -encoder_offset_count / 1000.;
                    Self::set_u_dq_angle(0., 0., &AngleSinCos::new(0.0), pwm);

                    self.motor_params.encoder_offset = Some(encoder_offset);
                    info!("encoder_offset: {}", encoder_offset);

                    *has_encoder_offset = true;
                }
                self.loop_mode = LoopMode::None;
            }
            LoopMode::TorqueWithSensor {
                current_pid,
                expect_current,
            } => {
                let angle_sin_cos = Self::update_i_velocity_position(
                    &self.motor_params,
                    angle_sensor,
                    (&mut self.i_sensor, uvw_adcs),
                    &mut self.current_state,
                )?;
                self.current_state.u_dq.0 = current_pid
                    .0
                    .update(expect_current.0 - self.current_state.i_dq.0);
                self.current_state.u_dq.1 = current_pid
                    .1
                    .update(expect_current.1 - self.current_state.i_dq.1);
                Self::set_u_dq_angle(
                    self.current_state.u_dq.0,
                    self.current_state.u_dq.1,
                    &angle_sin_cos,
                    pwm,
                );
            }
            LoopMode::PositionWithSensor {
                current_pid,
                position_pid,
                expect_position: _,
            } => {
                let angle_sin_cos = Self::update_i_velocity_position(
                    &self.motor_params,
                    angle_sensor,
                    (&mut self.i_sensor, uvw_adcs),
                    &mut self.current_state,
                )?;
                self.current_state.u_dq.0 = current_pid.1.update(0.0 - self.current_state.i_dq.0);
                self.current_state.u_dq.1 = current_pid
                    .0
                    .update(position_pid.last_output - self.current_state.i_dq.1);
                Self::set_u_dq_angle(
                    self.current_state.u_dq.0,
                    self.current_state.u_dq.1,
                    &angle_sin_cos,
                    pwm,
                );
            }
            LoopMode::VelocityWithSensor {
                current_pid,
                velocity_pid,
                pll: _,
                expect_velocity: _,
            } => {
                let angle_sin_cos = Self::update_i_velocity_position(
                    &self.motor_params,
                    angle_sensor,
                    (&mut self.i_sensor, uvw_adcs),
                    &mut self.current_state,
                )?;
                self.current_state.u_dq.0 = current_pid.1.update(0.0 - self.current_state.i_dq.0);
                self.current_state.u_dq.1 = current_pid
                    .0
                    .update(velocity_pid.last_output - self.current_state.i_dq.1);
                Self::set_u_dq_angle(
                    self.current_state.u_dq.0,
                    self.current_state.u_dq.1,
                    &angle_sin_cos,
                    pwm,
                );
            }
            LoopMode::PositionVelocityWithSensor {
                current_pid,
                velocity_pid,
                position_pid: _,
                pll: _,
                expect_position: _,
            } => {
                let angle_sin_cos = Self::update_i_velocity_position(
                    &self.motor_params,
                    angle_sensor,
                    (&mut self.i_sensor, uvw_adcs),
                    &mut self.current_state,
                )?;
                self.current_state.u_dq.0 = current_pid.1.update(0.0 - self.current_state.i_dq.0);
                self.current_state.u_dq.1 = current_pid
                    .0
                    .update(velocity_pid.last_output - self.current_state.i_dq.1);
                Self::set_u_dq_angle(
                    self.current_state.u_dq.0,
                    self.current_state.u_dq.1,
                    &angle_sin_cos,
                    pwm,
                );
            }
            LoopMode::Velocity {} => {
                // TODO 无感
                Self::set_u_dq_angle(0., 0., &AngleSinCos::new(0.), pwm);
            }
        };

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
                self.current_state.position.1 +=
                    *expect_velocity * self.velocity_loop_dt * self.motor_params.pole_num as f32;
                if self.current_state.position.1.abs() >= PI {
                    self.current_state.position.1 = if *expect_velocity > 0. { -PI } else { PI }
                };
            }
            LoopMode::VelocityWithSensor {
                current_pid: _,
                velocity_pid,
                pll,
                expect_velocity,
            } => {
                let state = &mut self.current_state;
                state.velocity = pll.update(state.position.1);
                velocity_pid.update(*expect_velocity - state.velocity);
            }
            LoopMode::PositionVelocityWithSensor {
                current_pid: _,
                velocity_pid,
                position_pid,
                pll,
                expect_position: _,
            } => {
                let state = &mut self.current_state;
                state.velocity = pll.update(state.position.1);
                velocity_pid.update(position_pid.last_output - self.current_state.velocity);
            }
            LoopMode::Velocity {} => {
                //TODO 等大佬写无感
            }
            _ => {
                return Err(VelocityLoopError::NoRequireVelocityLoop);
            }
        }
        Ok(())
    }

    #[allow(dead_code)]
    pub fn position_tick(&mut self) -> Result<(), PositionLoopError> {
        match &mut self.loop_mode {
            LoopMode::PositionWithSensor {
                current_pid: _,
                position_pid,
                expect_position,
            } => {
                let lap_err = expect_position.0 - self.current_state.position.0;
                let angle_err = expect_position.1 - self.current_state.position.1;
                let err = lap_err as f32 * 2. * PI + angle_err;
                position_pid.update(err);
            }
            LoopMode::PositionVelocityWithSensor {
                current_pid: _,
                velocity_pid: _,
                position_pid,
                pll: _,
                expect_position,
            } => {
                let lap_err = expect_position.0 - self.current_state.position.0;
                let angle_err = expect_position.1 - self.current_state.position.1;
                let err = lap_err as f32 * 2. * PI + angle_err;
                position_pid.update(err);
            }
            _ => {
                return Err(PositionLoopError::NoRequirePositionLoop);
            }
        }
        Ok(())
    }

    #[allow(dead_code)]
    pub fn set_expect_current(&mut self, current: (f32, f32)) -> Result<(), SetError> {
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
                pll: _,
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
    pub fn set_expect_position(&mut self, position: (i32, f32)) -> Result<(), SetError> {
        match &mut self.loop_mode {
            LoopMode::PositionWithSensor {
                current_pid: _,
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
    pub fn get_state(&self) -> &CurrentState {
        &self.current_state
    }

    #[allow(dead_code)]
    pub fn get_motor_param(&self) -> &MotorParams {
        &self.motor_params
    }
}
