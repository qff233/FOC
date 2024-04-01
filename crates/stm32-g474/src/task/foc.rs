use defmt::warn;
use embassy_stm32::peripherals;
use embassy_time::Timer;
use foc::{CurrentLoopError, PositionLoopError, VelocityLoopError};

use crate::{
    interface::{Adcs, Pwms, VbusAdc},
    FocMutex,
};

#[embassy_executor::task]
pub async fn current_loop(
    foc: &'static FocMutex,
    mut vbus_adc: VbusAdc<peripherals::ADC2, peripherals::PC5>,
    mut uvw_adcs: Adcs<peripherals::ADC1, peripherals::PA0, peripherals::PA1, peripherals::PA2>,
    mut pwm: Pwms,
) {
    loop {
        {
            let mut foc = foc.lock().await;
            let foc = foc.as_mut().unwrap();
            let bus_voltage = vbus_adc.get_voltage();
            // info!("{}", bus_voltage);

            if let Err(e) = foc.current_tick(bus_voltage, None, Some(&mut uvw_adcs), &mut pwm) {
                match e {
                    CurrentLoopError::MisAngleSensor => {}
                    CurrentLoopError::MisCurrentSensor => {}
                    CurrentLoopError::MisCurrentAdcs => {}
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
