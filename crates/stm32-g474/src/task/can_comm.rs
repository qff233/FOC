// use embassy_stm32::{can::Fdcan, peripherals::FDCAN1};
// use embassy_time::Timer;

use embassy_stm32::{can, peripherals};
use embassy_time::Timer;

#[embassy_executor::task]
pub async fn can_comm(_can: can::Can<'static, peripherals::FDCAN1>) {
    loop {
        // info!("can");
        Timer::after_micros(200).await;
    }
}
