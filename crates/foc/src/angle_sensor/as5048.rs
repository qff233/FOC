use core::f32::consts::PI;

use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;
use micromath::F32Ext;

use super::{AngleSensor, AngleSensorError, Direction};

#[allow(dead_code)]
pub struct As5048<T: SpiBus, P: OutputPin> {
    spi: T,
    cs_pin: P,
    direction: Direction,
    count_lap: i32,
    last_angle: f32,
}

impl<T: SpiBus, P: OutputPin> As5048<T, P> {
    #[allow(dead_code)]
    pub fn new(spi: T, cs_pin: P, direction: Direction) -> Self {
        Self {
            spi,
            cs_pin,
            direction,
            count_lap: 0,
            last_angle: 0.0,
        }
    }
}

impl<T: SpiBus, P: OutputPin> AngleSensor for As5048<T, P> {
    fn set_direction(&mut self, dir: Direction) {
        self.direction = dir;
    }
    fn get_lap_angle(&mut self) -> Result<f32, AngleSensorError> {
        let send_data: [u8; 2] = [0xFF, 0xFF];
        let mut recv_data = [0; 2];

        self.cs_pin.set_low().unwrap();
        self.spi.transfer(&mut recv_data, &send_data).unwrap();
        self.spi.flush().unwrap();
        self.cs_pin.set_high().unwrap();

        let recv_data = (recv_data[0] as u16) << 8 | recv_data[1] as u16;
        // debug!("recv_data: {:#018b}", recv_data);

        let mut count = 0;
        for i in 0..16 {
            if (recv_data >> i) & 0x01 != 0 {
                count += 1;
            }
        }
        if count & 0x0001 != 0 {
            return Err(AngleSensorError::DataError);
        }

        // if recv_data & (1 << 14) as u16 != 0 {
        //     return Err(As5048Error::NoMagWarning);
        // }

        let data = recv_data & 0x3FFF;
        // debug!("angle: {}", data as f32 * 2. * PI / 16383.);
        if let Direction::CW = self.direction {
            Ok(data as f32 * 2. * PI / 16383.)
        } else {
            Ok(2. * PI - (data as f32) * 2. * PI / 16383.)
        }
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
