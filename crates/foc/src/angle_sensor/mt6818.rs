use core::f32::consts::PI;

use embedded_hal::{digital::OutputPin, spi::SpiBus};

use crate::angle_sensor::AngleSensor;

pub enum Mt6818Error {
    NoMagWarning,
    NoData,
}

#[allow(dead_code)]
pub struct Mt6818<T: SpiBus, P: OutputPin> {
    spi: T,
    cs_pin: P,
}

impl<T: SpiBus, P: OutputPin> Mt6818<T, P> {
    #[allow(dead_code)]
    pub fn new(spi: T, cs_pin: P) -> Self {
        Self { spi, cs_pin }
    }
}

impl<T: SpiBus, P: OutputPin> AngleSensor for Mt6818<T, P> {
    type Error = Mt6818Error;
    fn get_angle(&mut self) -> Result<f32, Self::Error> {
        let send_data: [u8; 4] = [0x80 | 0x03, 0x00, 0x80 | 0x04, 0x00];
        let mut recv_data = [0; 4];

        let mut data: Option<u16> = None;
        for _ in 0..3 {
            self.cs_pin.set_low().unwrap();
            self.spi.transfer(&mut recv_data, &send_data).unwrap();
            self.spi.flush().unwrap();
            self.cs_pin.set_high().unwrap();

            let recv_data = (recv_data[1] as u16) << 8 | recv_data[3] as u16;
            let mut count = 0;
            (0..16).for_each(|pos| {
                if recv_data & (0x0001 << pos) != 0 {
                    count += 1;
                }
            });

            if count & 0x01 == 0 {
                data = Some(recv_data);
                break;
            }
        }

        match data {
            Some(data) => {
                if data & (0x0001 << 1) > 0 {
                    return Err(Mt6818Error::NoMagWarning);
                }
                let result = data >> 2;
                Ok(result as f32 * 2. * PI / 16384.)
            }
            None => Err(Mt6818Error::NoData),
        }
    }
}
