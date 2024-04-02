use defmt::info;
use embassy_futures::join::join;
use embassy_stm32::{peripherals, usb::Driver};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex},
    channel::Receiver,
};
use embassy_usb::{class::cdc_acm::CdcAcmClass, driver::EndpointError, UsbDevice};
use heapless::Vec;

use crate::SharedEvent;

enum State {
    Reply,
    LoopSend,
}

async fn process_usb_data(
    foc_receiver: Receiver<'static, CriticalSectionRawMutex, SharedEvent, 64>,
    _recv_data: &[u8],
) -> (Option<Vec<u8, 32>>, State) {
    let data = match foc_receiver.receive().await {
        SharedEvent::UvwI(u, v, w) => {
            let mut data: Vec<u8, 32> = Vec::new();
            data.extend_from_slice(&u.to_le_bytes()).unwrap();
            data.extend_from_slice(&v.to_le_bytes()).unwrap();
            data.extend_from_slice(&w.to_le_bytes()).unwrap();
            data.extend_from_slice(&[0x00, 0x00, 0x80, 0x7F]).unwrap();
            data
        }
    };

    (Some(data), State::LoopSend)
}

async fn process_usb_connnect(
    class: CdcAcmClass<'static, Driver<'static, peripherals::USB>>,
    foc_receiver: Receiver<'static, CriticalSectionRawMutex, SharedEvent, 64>,
) {
    loop {
        let mut current_state = State::Reply;
        let (mut sender, mut recever) = class.split();
        let mut recv_buf = [0; 64];
        loop {
            sender.wait_connection().await;
            recever.wait_connection().await;
            info!("USB Connected");
            loop {
                let data = match current_state {
                    State::Reply => {
                        let recv_data_len = match recever.read_packet(&mut recv_buf).await {
                            Ok(n) => n,
                            Err(e) => {
                                if let EndpointError::BufferOverflow = e {
                                    defmt::panic!("USB Buffer overflow")
                                }
                                break;
                            }
                        };

                        &recv_buf[..recv_data_len]
                    }
                    State::LoopSend => &recv_buf[..0],
                };

                let (send_data, next_state) = process_usb_data(foc_receiver, data).await;
                current_state = next_state;
                if let Some(send_data) = send_data {
                    // debug!("{:#X}", *send_data);
                    if let Err(e) = sender.write_packet(&send_data[0..send_data.len()]).await {
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
    foc_receiver: Receiver<'static, CriticalSectionRawMutex, SharedEvent, 64>,
) {
    join(usb.run(), process_usb_connnect(class, foc_receiver)).await;
}
