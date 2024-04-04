use cortex_m::prelude::_embedded_hal_blocking_delay_DelayUs;
use defmt::debug;
use embassy_stm32::adc::{Adc, AdcPin};
use embassy_stm32::timer::complementary_pwm::ComplementaryPwm;
use embassy_stm32::timer::{AdvancedInstance4Channel, Channel};
use foc::driver::interface;

pub struct VbusAdc<T, A>
where
    T: embassy_stm32::adc::Instance,
    A: AdcPin<T> + embassy_stm32::gpio::Pin,
{
    adc: Adc<'static, T>,
    pin: A,
}

impl<T, A> VbusAdc<T, A>
where
    T: embassy_stm32::adc::Instance,
    A: AdcPin<T> + embassy_stm32::gpio::Pin,
{
    pub fn new(adc: Adc<'static, T>, pin: A) -> Self {
        Self { adc, pin }
    }

    pub fn get_voltage(&mut self) -> f32 {
        self.adc.read(&mut self.pin) as f32 * 3.3 / 4096. * 16.0
    }
}

pub struct Adcs {}

impl Adcs {
    pub fn new() -> Self {
        // set GPIO as analog
        embassy_stm32::pac::GPIOA.moder().modify(|w| {
            w.set_moder(0, embassy_stm32::pac::gpio::vals::Moder::ANALOG);
            w.set_moder(1, embassy_stm32::pac::gpio::vals::Moder::ANALOG);
            w.set_moder(2, embassy_stm32::pac::gpio::vals::Moder::ANALOG);
        });
        // embassy_stm32::pac::GPIOA
        //     .moder()
        //     .modify(|w| w.set_moder(1, embassy_stm32::pac::gpio::vals::Moder::ANALOG));
        // embassy_stm32::pac::GPIOA
        //     .moder()
        //     .modify(|w| w.set_moder(2, embassy_stm32::pac::gpio::vals::Moder::ANALOG));

        // embassy_stm32::pac::ADC1.cfgr().modify(|w| {
        // });
        // embassy_stm32::pac::ADC1.cfgr().modify(|w| {
        // });

        use embassy_stm32::pac::adc::vals;
        use embassy_stm32::pac::ADC1;
        ADC1.cfgr().modify(|w| {
            w.set_res(vals::Res::BITS16); // 分辨率
                                          // w.set_autdly(false); // low power auto wait

            w.set_discen(false);
            w.set_dmngt(vals::Dmngt::DR); // DMA transfer disable. Only save value in DR
            w.set_ovrmod(vals::Ovrmod::PRESERVE);
            w.set_cont(false); // Countinuous mode
            w.set_exten(vals::Exten::DISABLED);

            w.set_jdiscen(false);
            w.set_jqdis(false);
            w.set_jauto(false);
            w.set_jqm(vals::Jqm::MODE0);
        });
        ADC1.sqr1().modify(|w| {
            w.set_l(0); // set sequencer length
        });
        ADC1.jsqr().modify(|w| {
            w.set_jextsel(1);
            w.set_jl(2);
            w.set_jexten(vals::Jexten::RISINGEDGE); // 触发边缘
        });

        ADC1.cr().modify(|w| {
            w.set_deeppwd(false); // disable deep power down
            w.set_advregen(true); // enable internal regulator
        });
        embassy_time::Delay.delay_us(10u8); //等待上电稳定

        // 配置注入通道
        embassy_stm32::pac::ADC1.jsqr().modify(|w| {
            w.set_jsq1(0, 1);
            w.set_jsq1(1, 2);
            w.set_jsq1(2, 3);
        });
        // 配置采样周期
        embassy_stm32::pac::ADC1.smpr(0).modify(|w| {
            w.set_smp(0, vals::SampleTime::CYCLES2_5);
            w.set_smp(1, vals::SampleTime::CYCLES2_5);
            w.set_smp(2, vals::SampleTime::CYCLES2_5);
        });
        // 配置单端输入
        ADC1.difsel().modify(|w| {
            w.set_difsel(0, vals::Difsel::SINGLEENDED);
            w.set_difsel(1, vals::Difsel::SINGLEENDED);
            w.set_difsel(2, vals::Difsel::SINGLEENDED);
        });

        // 开始校准ADC
        debug!("Calibration ADC");
        ADC1.cr().modify(|w| {
            w.set_adcaldif(vals::Adcaldif::SINGLEENDED);
            w.set_adcallin(true);
        });
        ADC1.cr().modify(|w| w.set_adcal(true));
        while ADC1.cr().read().adcal() {}
        embassy_time::Delay.delay_us(1u8);

        // 使能ADC
        debug!("Enable ADC");
        ADC1.isr().write(|w| w.set_adrdy(true));
        ADC1.cr().modify(|w| w.set_aden(true));
        while ADC1.isr().read().adrdy() {}
        ADC1.isr().write(|w| w.set_adrdy(true));

        // 设置完成注入转换后触发中断
        ADC1.ier().write(|w| w.set_jeosie(true));

        // 开启注入ADC转换
        ADC1.cr().modify(|w| {
            w.set_jadstart(true);
        });

        // embassy_stm32::pac::ADC_COMMON.ccr().modify(|w| {
        //     w.set_ckmode(embassy_stm32::pac::adccommon::vals::Ckmode::SYNCDIV4);
        //     w.set_dual(embassy_stm32::pac::adccommon::vals::Dual::INDEPENDENT);
        // });

        Self {}
    }
}

impl interface::Adcs for Adcs {
    fn get_voltage(&mut self) -> (f32, f32, f32) {
        let adc = embassy_stm32::pac::ADC1;
        let a = adc.jdr(0).read().0 as f32 * 3.3 / 4096.;
        let b = adc.jdr(1).read().0 as f32 * 3.3 / 4096.;
        let c = adc.jdr(2).read().0 as f32 * 3.3 / 4096.;

        // debug!("{}, {}, {}", a, b, c);
        (a as f32, b as f32, c as f32)
    }
}

pub struct Pwms<T: AdvancedInstance4Channel> {
    pwm: ComplementaryPwm<'static, T>,
}

impl<T: AdvancedInstance4Channel> Pwms<T> {
    pub fn new(pwm: ComplementaryPwm<'static, T>) -> Self {
        Self { pwm }
    }
}

impl<T: AdvancedInstance4Channel> interface::Pwms for Pwms<T> {
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
