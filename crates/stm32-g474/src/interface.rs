use embassy_stm32::adc::{Adc, AdcPin};
use embassy_stm32::peripherals;
use embassy_stm32::timer::complementary_pwm::ComplementaryPwm;
use embassy_stm32::timer::Channel;
use foc::driver::interface;

pub struct Adcs<A, B, C>
where
    A: AdcPin<peripherals::ADC1> + embassy_stm32::gpio::Pin,
    B: AdcPin<peripherals::ADC1> + embassy_stm32::gpio::Pin,
    C: AdcPin<peripherals::ADC1> + embassy_stm32::gpio::Pin,
{
    adc: Adc<'static, peripherals::ADC1>,
    pin0: A,
    pin1: B,
    pin2: C,
}

impl<A, B, C> Adcs<A, B, C>
where
    A: AdcPin<peripherals::ADC1> + embassy_stm32::gpio::Pin,
    B: AdcPin<peripherals::ADC1> + embassy_stm32::gpio::Pin,
    C: AdcPin<peripherals::ADC1> + embassy_stm32::gpio::Pin,
{
    pub fn new(adc: Adc<'static, peripherals::ADC1>, pin0: A, pin1: B, pin2: C) -> Self {
        Self {
            adc,
            pin0,
            pin1,
            pin2,
        }
    }
}

impl<A, B, C> interface::Adcs for Adcs<A, B, C>
where
    A: AdcPin<peripherals::ADC1> + embassy_stm32::gpio::Pin,
    B: AdcPin<peripherals::ADC1> + embassy_stm32::gpio::Pin,
    C: AdcPin<peripherals::ADC1> + embassy_stm32::gpio::Pin,
{
    fn get_voltage(&mut self) -> (f32, f32, f32) {
        let a = self.adc.read(&mut self.pin0) as f32 / 4096.;
        let b = self.adc.read(&mut self.pin1) as f32 / 4096.;
        let c = self.adc.read(&mut self.pin2) as f32 / 4096.;
        (a, b, c)
    }
}

pub struct Pwms {
    pwm: ComplementaryPwm<'static, peripherals::TIM1>,
}

impl Pwms {
    pub fn new(pwm: ComplementaryPwm<'static, peripherals::TIM1>) -> Self {
        Self { pwm }
    }
}

impl interface::Pwms for Pwms {
    fn set_duty(&mut self, u: f32, v: f32, w: f32) {
        let max_duty = self.pwm.get_max_duty() as f32;
        let u = (u * max_duty).max(0.0).min(max_duty) as u16;
        let v = (v * max_duty).max(0.0).min(max_duty) as u16;
        let w = (w * max_duty).max(0.0).min(max_duty) as u16;
        self.pwm.set_duty(Channel::Ch1, u);
        self.pwm.set_duty(Channel::Ch2, v);
        self.pwm.set_duty(Channel::Ch3, w);
    }
    fn enable(&mut self) {
        self.pwm.enable(Channel::Ch1);
        self.pwm.enable(Channel::Ch2);
        self.pwm.enable(Channel::Ch3);
    }
    fn disable(&mut self) {
        self.pwm.disable(Channel::Ch1);
        self.pwm.disable(Channel::Ch2);
        self.pwm.disable(Channel::Ch3);
    }
}
