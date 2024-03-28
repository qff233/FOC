// use embassy_stm32::{can::Fdcan, peripherals::FDCAN1};
// use embassy_time::Timer;

use embassy_stm32::{can::Fdcan, peripherals};

#[embassy_executor::task]
pub async fn can_comm(_can: Fdcan<'static, peripherals::FDCAN1>) {
    loop {
        //         Timer::after_secs(10).await;
    }
}
