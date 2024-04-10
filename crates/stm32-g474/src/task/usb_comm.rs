use defmt::info;
use embassy_futures::join::join;
use embassy_stm32::{peripherals, usb::Driver};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Receiver};
use embassy_usb::{class::cdc_acm::CdcAcmClass, driver::EndpointError, UsbDevice};
use heapless::Vec;

use crate::SharedEvent;

enum State {
    Reply,
    LoopSend,
}

macro_rules! add_justfloat_data {
    ($vec:ident, $($elem:expr),+) => {
        $($vec.extend_from_slice(&$elem.to_le_bytes()).unwrap());+;
    };
}

async fn process_usb_data(
    foc_receiver: Receiver<'static, CriticalSectionRawMutex, SharedEvent, 64>,
    _recv_data: &[u8],
) -> (Option<Vec<u8, 64>>, State) {
    let foc_receiver = foc_receiver.try_receive().ok();
    if foc_receiver.is_none() {
        return (None, State::LoopSend);
    }
    let mut data: Vec<u8, 64> = Vec::new();
    match foc_receiver.unwrap() {
        SharedEvent::Iuvw(u, v, w) => {
            add_justfloat_data!(data, u, v, w);
        }
        SharedEvent::Idq(d, q) => {
            add_justfloat_data!(data, d, q);
        }
        SharedEvent::Velocity(velociry) => {
            add_justfloat_data!(data, velociry);
        }
        SharedEvent::Position(position) => {
            add_justfloat_data!(data, position);
        }
        SharedEvent::State {
            i_uvw,
            u_dq,
            i_dq,
            position,
            velocity,
        } => {
            let (u, v, w) = i_uvw;
            let (ud, uq) = u_dq;
            let (id, iq) = i_dq;
            add_justfloat_data!(data, u, v, w, ud, uq, id, iq, velocity, position.0, position.1);
        }
    }
    data.extend_from_slice(&[0x00, 0x00, 0x80, 0x7F]).unwrap();

    (Some(data), State::LoopSend)
}

async fn process_usb_connnect(
    class: CdcAcmClass<'static, Driver<'static, peripherals::USB>>,
    foc_receiver: Receiver<'static, CriticalSectionRawMutex, SharedEvent, 64>,
) {
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
                if let Err(EndpointError::BufferOverflow) =
                    sender.write_packet(&send_data[0..send_data.len()]).await
                {
                    defmt::panic!("USB Buffer overflow");
                };
            }
        }
        info!("USB Disconnected");
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
