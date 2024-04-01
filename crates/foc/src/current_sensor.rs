use embedded_hal::delay;

use crate::driver::interface::{Adcs, Pwms};

#[allow(dead_code)]
pub struct CurrentSensor {
    sampling_resistor: f32,
    voltage_offset: (f32, f32, f32),
}

impl CurrentSensor {
    #[allow(dead_code)]
    pub fn new(
        sampling_resistor: f32,
        pwms: &mut dyn Pwms,
        delay: &mut dyn delay::DelayNs,
        uvw_adcs: &mut dyn Adcs,
    ) -> Self {
        pwms.disable();
        let voltage_offset = {
            let (mut uc, mut vc, mut wc) = (0., 0., 0.);
            for _ in 0..500 {
                let (u, v, w) = uvw_adcs.get_voltage();
                uc += u;
                vc += v;
                wc += w;
                delay.delay_ms(1)
            }
            (uc / 500., vc / 500., wc / 500.)
        };
        pwms.enable();
        Self {
            sampling_resistor,
            voltage_offset,
        }
    }

    #[allow(dead_code)]
    pub fn get_currnet(&mut self, uvw_adcs: &mut dyn Adcs) -> (f32, f32, f32) {
        let (u, v, w) = uvw_adcs.get_voltage();
        let (u_offset, v_offset, w_offset) = self.voltage_offset;
        let u_current = (u - u_offset) / self.sampling_resistor;
        let v_current = (v - v_offset) / self.sampling_resistor;
        let w_current = (w - w_offset) / self.sampling_resistor;
        (u_current, v_current, w_current)
    }

    #[allow(dead_code)]
    pub fn set_voltage_offset(&mut self, u: f32, v: f32, w: f32) {
        self.voltage_offset = (u, v, w);
    }
}
