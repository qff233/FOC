use embassy_stm32::{adc::Adc, peripherals, timer::complementary_pwm::ComplementaryPwm};

// this task will be poll in PWM interrupt
#[embassy_executor::task]
pub async fn current_loop(
    _pwm: ComplementaryPwm<'static, peripherals::TIM1>,
    _adc: Adc<'static, peripherals::ADC1>,
) {
    loop {}
}
