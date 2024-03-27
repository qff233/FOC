use embassy_stm32::{can::Fdcan, peripherals::FDCAN1};
use embassy_time::Timer;

pub async fn can_comm_task(_can: Fdcan<'static, FDCAN1>) {
    loop {
        Timer::after_secs(10).await;
    }
}
