use core::f32::consts::PI;

use defmt::debug;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;

use super::AngleSensor;

#[allow(dead_code)]
pub enum As5048Error {
    NoMagWarning,
    NoData,
}

#[allow(dead_code)]
pub struct As5048<T: SpiBus, P: OutputPin> {
    spi: T,
    cs_pin: P,
}

impl<T: SpiBus, P: OutputPin> As5048<T, P> {
    #[allow(dead_code)]
    pub fn new(spi: T, cs_pin: P) -> Self {
        Self { spi, cs_pin }
    }
}

impl<T: SpiBus, P: OutputPin> AngleSensor for As5048<T, P> {
    type Error = As5048Error;
    fn get_angle(&mut self) -> Result<f32, Self::Error> {
        let send_data: [u8; 2] = [0xFF, 0xFF];
        let mut recv_data = [0; 4];

        self.cs_pin.set_low().unwrap();
        self.spi.transfer(&mut recv_data, &send_data).unwrap();
        self.spi.flush().unwrap();
        self.cs_pin.set_high().unwrap();

        debug!("recv_data: {}", recv_data);

        let recv_data = (recv_data[0] as u16) << 8 | recv_data[1] as u16;

        let mut count = 0;
        for i in 0..16 {
            if (recv_data >> i) & 0x01 != 0 {
                count += 1;
            }
        }

        debug!("count: {}", count);

        // if recv_data & (1 << 14) as u16 != 0 {
        //     return Err(As5048Error::NoMagWarning);
        // }

        let data = recv_data & 0x3FFF;
        Ok(data as f32 * 2. * PI / 16383.)
    }
}
