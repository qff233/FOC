use embassy_stm32::peripherals;
use embassy_time::Timer;

use crate::{
    interface::{Adcs, Pwms},
    FocMutex,
};

// this task will be poll in PWM interrupt
#[embassy_executor::task]
pub async fn current_loop(
    _foc: &'static FocMutex,
    _pwm: Pwms,
    _adcs: Adcs<peripherals::PA0, peripherals::PA1, peripherals::PA2>,
) {
    // _adc: Adcs<peripherals.PA0, peripherals.PA1, peripherals.PA2>) {
    loop {
        Timer::after_micros(50).await
    }
}

#[embassy_executor::task]
pub async fn velocity_loop(_foc: &'static FocMutex) {
    loop {
        Timer::after_micros(125).await
    }
}

#[embassy_executor::task]
pub async fn position_loop(_foc: &'static FocMutex) {
    loop {
        Timer::after_micros(1000).await
    }
}
