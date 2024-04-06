use defmt::error;
use embassy_stm32::peripherals;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Sender};
use embassy_time::Timer;
use foc::{angle_sensor::TestAngleSensor, CurrentLoopError, PositionLoopError, VelocityLoopError};

use crate::{
    interface::{Adcs, Pwms, VbusAdc},
    FocMutex, SharedEvent,
};

#[embassy_executor::task]
pub async fn current_loop(
    foc: &'static FocMutex,
    comm_sender: Sender<'static, CriticalSectionRawMutex, SharedEvent, 64>,
    mut vbus_adc: VbusAdc<peripherals::ADC2, peripherals::PC5>,
    // mut uvw_adcs: Adcs<peripherals::ADC1, peripherals::PA0, peripherals::PA1, peripherals::PA2>,
    mut uvw_adcs: Adcs,
    mut pwm: Pwms<peripherals::TIM1>,
) {
    let mut angle_sensor = TestAngleSensor::new(180_f32.to_radians(), 0.00005);

    loop {
        //debug!("enter: {}", embassy_time::Instant::now());
        {
            // let begin = embassy_time::Instant::now();
            let mut foc = foc.lock().await;
            let foc = foc.as_mut().unwrap();
            let bus_voltage = vbus_adc.get_voltage();

            if let Err(e) = foc.current_tick(
                bus_voltage,
                Some(&mut angle_sensor),
                Some(&mut uvw_adcs),
                &mut pwm,
            ) {
                match e {
                    CurrentLoopError::MisAngleSensor => {
                        panic!("mis angle sensor")
                    }
                    CurrentLoopError::MisCurrentSensor => {
                        panic!("mis current sensor")
                    }
                    CurrentLoopError::MisCurrentAdcs => {
                        panic!("mis current adc")
                    }
                    CurrentLoopError::NoAngleData => {
                        error!("encoder is distracted!")
                    }
                }
            }

            // let end = embassy_time::Instant::now();
            // debug!("{}", (end - begin).as_micros());

            let (u, v, w) = foc.get_i_uvw();
            comm_sender.try_send(SharedEvent::Iuvw(u, v, w)).ok();

            // let (d, q) = foc.current_i_dq;
            // comm_sender.try_send(SharedEvent::Idq(d, q)).ok();

            // comm_sender
            //     .try_send(SharedEvent::Velocity {
            //         expect: 0.0,
            //         current: foc.get_velocity(),
            //     })
            //     .ok();
        }
        Timer::after_micros(5).await
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
                        // warn!("Current loop mode do not requre velocity loop")
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
                        // warn!("Current loop mode do not requre position loop")
                    }
                }
            }
        }

        // info!("position_loop");
        Timer::after_micros(1000).await
    }
}
