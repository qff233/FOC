use defmt::{info, warn};
use embassy_stm32::peripherals;
use embassy_time::Timer;
use foc::{angle_sensor::NoAngleSensor, CurrentLoopError, PositionLoopError, VelocityLoopError};

use crate::{
    interface::{Adcs, Pwms, VbusAdc},
    FocMutex,
};

// #[embassy_executor::task]
// pub async fn foc_loop(
//     foc: &'static FocMutex,
//     mut vbus_adc: VbusAdc<peripherals::ADC2, peripherals::PC5>,
//     mut uvw_adcs: Adcs<peripherals::ADC1, peripherals::PA0, peripherals::PA1, peripherals::PA2>,
//     mut pwm: Pwms,
// ) {
//     loop {
//         // info!("current_loop");
//         // current_loop(foc, &mut vbus_adc, &mut uvw_adcs, &mut pwm).await;
//         // info!("velocity_loop");
//         // velocity_loop(foc).await;
//         // info!("position_loop");
//         // position_loop(foc).await;
//         // Timer::after_micros(244444).await
//         join3(current_loop(foc, &mut vbus_adc, &mut uvw_adcs, &mut pwm), velocity_loop(foc), position_loop(foc)).await;
//         // Timer::after_micros(50).await;
//     }
// }

#[embassy_executor::task]
pub async fn current_loop(
    foc: &'static FocMutex,
    mut vbus_adc: VbusAdc<peripherals::ADC2, peripherals::PC5>,
    mut uvw_adcs: Adcs<peripherals::ADC1, peripherals::PA0, peripherals::PA1, peripherals::PA2>,
    mut pwm: Pwms,
) {
    // _adc: Adcs<peripherals.PA0, peripherals.PA1, peripherals.PA2>) {

    let mut angle_sensor = NoAngleSensor::new();
    loop {
        {
            let mut foc = foc.lock().await;
            let foc = foc.as_mut().unwrap();
            let bus_voltage = vbus_adc.get_voltage();
            // info!("{}", bus_voltage);

            if let Err(e) =
                foc.current_tick(bus_voltage, &mut angle_sensor, &mut uvw_adcs, &mut pwm)
            {
                match e {
                    CurrentLoopError::MisAngleSensor => {}
                    CurrentLoopError::MisCurrentSensor => {}
                }
            }
        }
        // info!("current_loop");
        Timer::after_micros(50).await
    }
}

#[embassy_executor::task]
pub async fn velocity_loop(foc: &'static FocMutex) {
    loop {
        {
            let mut foc = foc.lock().await;
            let foc = foc.as_mut().unwrap();
            if let Err(e) = foc.velocity_tick() {
                match e {
                    VelocityLoopError::NoRequireVelocityLoop => {
                        warn!("Current loop mode do not requre velocity loop")
                    }
                }
            };
        }

        // info!("velocity_loop");
        Timer::after_micros(125).await
    }
}

#[embassy_executor::task]
pub async fn position_loop(foc: &'static FocMutex) {
    loop {
        {
            let mut foc = foc.lock().await;
            let foc = foc.as_mut().unwrap();
            if let Err(e) = foc.position_tick() {
                match e {
                    PositionLoopError::NoRequirePositionLoop => {
                        warn!("Current loop mode do not requre position loop")
                    }
                }
            }
        }

        // info!("position_loop");
        Timer::after_micros(1000).await
    }
}
