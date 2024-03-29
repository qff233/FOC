use embassy_stm32::adc::Adc;
use embassy_stm32::peripherals;
use embassy_time::Timer;

use crate::{
    interface::{Adcs, Pwms, VbusAdc},
    FocMutex,
};

// this task will be poll in PWM interrupt
#[embassy_executor::task]
pub async fn current_loop(
    foc: &'static FocMutex,
    vbus_adc: VbusAdc<peripherals::ADC2, peripherals::PC5>,
    uvw_adcs: Adcs<peripherals::ADC1, peripherals::PA0, peripherals::PA1, peripherals::PA2>,
    pwm: Pwms,
) {
    // _adc: Adcs<peripherals.PA0, peripherals.PA1, peripherals.PA2>) {
    loop {
        let foc = foc.lock().await.as_mut().unwrap();
        // foc.current_tick(bus_voltage, angle_sensor, &mut adcs, &mut pwm);
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
