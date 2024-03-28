// use defmt::info;
// use embassy_stm32::usb;
// use embassy_usb::class::cdc_acm::CdcAcmClass;
// use embassy_usb::driver::EndpointError;
// use heapless::Vec;

// use crate::GLOBALPERIPHERALS;

// enum State {
//     LoopSend,
//     Reply,
// }

use defmt::info;
use embassy_futures::join::join;
use embassy_stm32::{peripherals, usb::Driver};
use embassy_usb::{class::cdc_acm::CdcAcmClass, driver::EndpointError, UsbDevice};
use heapless::Vec;

enum State {
    Reply,
    LoopSend,
}

fn process_usb_data(recv_data: &[u8]) -> (Option<Vec<u8, 64>>, State) {
    if recv_data.len() == 3 {
        return (None, State::LoopSend);
    }
    (None, State::Reply)
}

async fn process_usb_connnect(mut class: CdcAcmClass<'static, Driver<'static, peripherals::USB>>) {
    loop {
        let mut current_state = State::Reply;
        let mut buf = [0; 64];
        loop {
            class.wait_connection().await;
            info!("USB Connected");
            loop {
                let data = match current_state {
                    State::Reply => {
                        let recv_data_len = match class.read_packet(&mut buf).await {
                            Ok(n) => n,
                            Err(e) => {
                                if let EndpointError::BufferOverflow = e {
                                    defmt::panic!("USB Buffer overflow")
                                }
                                break;
                            }
                        };

                        &buf[..recv_data_len]
                    }
                    State::LoopSend => &buf[..0],
                };

                let (send_data, next_state) = process_usb_data(data);
                current_state = next_state;
                if let Some(send_data) = send_data {
                    if let Err(e) = class.write_packet(&send_data).await {
                        if let EndpointError::BufferOverflow = e {
                            defmt::panic!("USB Buffer overflow");
                        }
                    };
                }
            }
            info!("USB Disconnected");
        }
    }
}

#[embassy_executor::task]
pub async fn usb_comm(
    mut usb: UsbDevice<'static, Driver<'static, peripherals::USB>>,
    class: CdcAcmClass<'static, Driver<'static, peripherals::USB>>,
) {
    join(usb.run(), process_usb_connnect(class)).await;
}
