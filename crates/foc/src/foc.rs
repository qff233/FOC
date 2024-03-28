use crate::angle_sensor::AngleSensor;
use crate::current_sensor::CurrentSensor;
use crate::driver::interface::{Adcs, Pwms};
use crate::pid::PID;
use crate::utils::{self, inv_park, svpwm};

pub enum Error {
    MisCurrentSensor,
    MisAngleSensor,
}

#[allow(dead_code)]
pub enum LoopMode {
    OpenVelocity {
        voltage: f32,
        angle: f32,
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
    Torque {
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
}

impl FOC {
    #[allow(dead_code)]
    fn new(
        motor_params: MotorParams,
        loop_mode: LoopMode,
        current_sensor: Option<CurrentSensor>,
    ) -> Self {
        Self {
            motor_params,
            loop_mode,
            current_sensor,
            current_i: (0., 0.),
            current_velocity: 0.,
            current_position: 0.,
        }
    }

    #[allow(dead_code)]
    fn current_tick(
        &mut self,
        bus_voltage: f32,
        angle_sensor: &impl AngleSensor,
        voltage_adc: &impl Adcs,
        pwm: &mut impl Pwms,
    ) -> Result<(), Error> {
        let (ud, uq, angle) = match &mut self.loop_mode {
            LoopMode::OpenVelocity {
                voltage,
                angle,
                expect_velocity: _,
            } => (0., *voltage, *angle),
            LoopMode::PositionWithSensor {
                current_pid,
                velocity_pid,
                position_pid: _,
                expect_position: _,
            } => {
                let sensor = self
                    .current_sensor
                    .as_ref()
                    .ok_or(Error::MisCurrentSensor)?;

                let angle = angle_sensor.get_angle() * self.motor_params.pole_num as f32;
                let (a, b, c) = sensor.get_currnet(voltage_adc);

                let (d, q) = utils::park(a, b, c, angle);
                self.current_i = (d, q);

                let iq = current_pid.update(velocity_pid.last_output - q);
                (0., iq, angle)
            }
            LoopMode::VelocityWithSensor {
                current_pid,
                velocity_pid,
                expect_velocity: _,
            } => {
                let sensor = self
                    .current_sensor
                    .as_ref()
                    .ok_or(Error::MisCurrentSensor)?;

                let angle = angle_sensor.get_angle() * self.motor_params.pole_num as f32;
                let (a, b, c) = sensor.get_currnet(voltage_adc);

                let (d, q) = utils::park(a, b, c, angle);
                self.current_i = (d, q);

                let iq = current_pid.update(velocity_pid.last_output - q);
                (0., iq, angle)
            }
            LoopMode::Torque {
                current_pid,
                expect_current,
            } => {
                let sensor = self
                    .current_sensor
                    .as_ref()
                    .ok_or(Error::MisCurrentSensor)?;

                let angle = angle_sensor.get_angle() * self.motor_params.pole_num as f32;
                let (a, b, c) = sensor.get_currnet(voltage_adc);

                let (d, q) = utils::park(a, b, c, angle);
                self.current_i = (d, q);

                let iq = current_pid.update(*expect_current - q);
                (0., iq, angle)
            }
            LoopMode::Velocity {} => (0., 0., 0.),
        };

        let (a, b, c) = inv_park(ud * bus_voltage, uq * bus_voltage, angle);
        let (a, b, c) = svpwm(a, b, c);
        pwm.set_duty(a, b, c);

        Ok(())
    }

    #[allow(dead_code)]
    fn velocity_tick(&mut self) {}
}
