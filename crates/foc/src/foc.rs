use crate::angle_sensor::AngleSensor;
use crate::current_sensor::CurrentSensor;
use crate::driver::interface::{Adcs, Pwms};
use crate::pid::PID;
use crate::utils::{inv_park, park, svpwm};

pub enum Error {
    MisCurrentSensor,
    MisAngleSensor,

    NoRequireVelocityLoop,
    NoRequirePositionLoop,

    NoRequireSetCurrent,
    NoRequireSetVelocity,
    NoRequireSetPosition,
}

#[allow(dead_code)]
pub enum LoopMode {
    OpenVelocity {
        voltage: f32,
        expect_velocity: f32,
    },
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
}

#[allow(dead_code)]
pub struct FOC {
    pub motor_params: MotorParams,
    pub loop_mode: LoopMode,
    pub current_sensor: Option<CurrentSensor>,

    pub current_i: (f32, f32),
    pub current_velocity: f32,
    pub current_position: f32,

    pub current_loop_dt: f32,
    pub velocity_loop_dt: f32,
    pub position_loop_dt: f32,
}

impl FOC {
    #[allow(dead_code)]
    pub fn new(
        motor_params: MotorParams,
        loop_mode: LoopMode,
        current_sensor: Option<CurrentSensor>,
        current_loop_dt: f32,
        velocity_loop_dt: f32,
        position_loop_dt: f32,
    ) -> Self {
        Self {
            motor_params,
            loop_mode,
            current_sensor,
            current_i: (0., 0.), // id iq
            current_velocity: 0.,
            current_position: 0.,
            current_loop_dt,
            velocity_loop_dt,
            position_loop_dt,
        }
    }

    fn update_i_velocity_position(
        pole_num: u32,
        current_loop_dt: f32,
        current_position: &mut f32,
        current_velocity: &mut f32,
        current_i: &mut (f32, f32),
        current_sensor: &Option<CurrentSensor>,
        angle_sensor: &impl AngleSensor,
        voltage_adc: &impl Adcs,
    ) -> Result<(), Error> {
        let sensor = current_sensor.as_ref().ok_or(Error::MisCurrentSensor)?;

        let last_position = *current_position;
        *current_position = angle_sensor.get_angle() * pole_num as f32;
        *current_velocity = (*current_position - last_position) / current_loop_dt;
        let (a, b, c) = sensor.get_currnet(voltage_adc);
        //TODO current velocity
        *current_i = park(a, b, c, *current_position);
        Ok(())
    }

    #[allow(dead_code)]
    pub fn current_tick(
        &mut self,
        bus_voltage: f32,
        angle_sensor: &impl AngleSensor,
        voltage_adc: &impl Adcs,
        pwm: &mut impl Pwms,
    ) -> Result<(), Error> {
        let (ud, uq, angle) = match &mut self.loop_mode {
            LoopMode::OpenVelocity {
                voltage,
                expect_velocity: _,
            } => (0., *voltage, self.current_position),
            LoopMode::TorqueWithSensor {
                current_pid,
                expect_current,
            } => {
                Self::update_i_velocity_position(
                    self.motor_params.pole_num,
                    self.current_loop_dt,
                    &mut self.current_position,
                    &mut self.current_velocity,
                    &mut self.current_i,
                    &mut self.current_sensor,
                    angle_sensor,
                    voltage_adc,
                )?;
                let iq = current_pid.update(*expect_current - self.current_i.1);
                (0., iq, self.current_position)
            }
            LoopMode::VelocityWithSensor {
                current_pid,
                velocity_pid,
                expect_velocity: _,
            } => {
                Self::update_i_velocity_position(
                    self.motor_params.pole_num,
                    self.current_loop_dt,
                    &mut self.current_position,
                    &mut self.current_velocity,
                    &mut self.current_i,
                    &mut self.current_sensor,
                    angle_sensor,
                    voltage_adc,
                )?;
                let iq = current_pid.update(velocity_pid.last_output - self.current_i.1);
                (0., iq, self.current_position)
            }
            LoopMode::PositionWithSensor {
                current_pid,
                velocity_pid,
                position_pid: _,
                expect_position: _,
            } => {
                Self::update_i_velocity_position(
                    self.motor_params.pole_num,
                    self.current_loop_dt,
                    &mut self.current_position,
                    &mut self.current_velocity,
                    &mut self.current_i,
                    &mut self.current_sensor,
                    angle_sensor,
                    voltage_adc,
                )?;
                let iq = current_pid.update(velocity_pid.last_output - self.current_i.1);
                (0., iq, self.current_position)
            }
            LoopMode::Velocity {} => (0., 0., 0.),
        };

        let (a, b, c) = inv_park(ud * bus_voltage, uq * bus_voltage, angle);
        let (a, b, c) = svpwm(a, b, c);
        pwm.set_duty(a, b, c);

        Ok(())
    }

    #[allow(dead_code)]
    fn velocity_tick(&mut self) -> Result<(), Error> {
        match &mut self.loop_mode {
            LoopMode::OpenVelocity {
                voltage: _,
                expect_velocity,
            } => {
                self.current_position += *expect_velocity * self.velocity_loop_dt;
            }
            LoopMode::VelocityWithSensor {
                current_pid: _,
                velocity_pid,
                expect_velocity,
            } => {
                velocity_pid.update(*expect_velocity - self.current_velocity);
            }
            LoopMode::PositionWithSensor {
                current_pid: _,
                velocity_pid,
                position_pid,
                expect_position: _,
            } => {
                velocity_pid.update(position_pid.last_output - self.current_velocity);
            }
            LoopMode::TorqueWithSensor {
                current_pid: _,
                expect_current: _,
            } => {
                return Err(Error::NoRequireVelocityLoop);
            }
            LoopMode::Velocity {} => {
                //TODO 等大佬写无感
            }
        }
        Ok(())
    }

    #[allow(dead_code)]
    fn position_tick(&mut self) -> Result<(), Error> {
        match &mut self.loop_mode {
            LoopMode::PositionWithSensor {
                current_pid: _,
                velocity_pid: _,
                position_pid,
                expect_position,
            } => {
                position_pid.update(*expect_position - self.current_position);
            }
            _ => {
                return Err(Error::NoRequireVelocityLoop);
            }
        }
        Ok(())
    }

    #[allow(dead_code)]
    fn set_expect_current(&mut self, current: f32) -> Result<(), Error> {
        match &mut self.loop_mode {
            LoopMode::TorqueWithSensor {
                current_pid: _,
                expect_current,
            } => {
                *expect_current = current;
                Ok(())
            }
            _ => Err(Error::NoRequireSetCurrent),
        }
    }

    #[allow(dead_code)]
    fn set_expect_velocity(&mut self, velocity: f32) -> Result<(), Error> {
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
            _ => Err(Error::NoRequireSetVelocity),
        }
    }

    #[allow(dead_code)]
    fn set_expect_position(&mut self, position: f32) -> Result<(), Error> {
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
            _ => Err(Error::NoRequireSetPosition),
        }
    }

    #[allow(dead_code)]
    fn set_loop_mode(&mut self, loop_mode: LoopMode) {
        self.loop_mode = loop_mode;
    }
}
