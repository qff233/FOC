// use embassy_stm32::{can::Fdcan, peripherals::FDCAN1};
// use embassy_time::Timer;

use embassy_stm32::{can, peripherals};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Receiver};
use embassy_time::Timer;

use crate::SharedEvent;

#[embassy_executor::task]
pub async fn can_comm(
    _can: can::Can<'static, peripherals::FDCAN1>,
    _foc_recevier: Receiver<'static, CriticalSectionRawMutex, SharedEvent, 64>,
) {
    loop {
        // info!("can");
        Timer::after_micros(200).await;
    }
}
