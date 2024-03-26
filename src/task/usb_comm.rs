use defmt::info;
use embassy_stm32::usb;
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::driver::EndpointError;
use heapless::Vec;

enum State {
    LoopSend,
    Reply,
}

pub async fn usb_comm_task<'d, T: usb::Instance + 'd>(
    class: &mut CdcAcmClass<'d, usb::Driver<'d, T>>,
) {
    let current_state = State::Reply;
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

            let command_result = process_data(data).await;
            if let Err(e) = class.write_packet(&command_result).await {
                if let EndpointError::BufferOverflow = e {
                    defmt::panic!("USB Buffer overflow");
                }
            };
        }
        info!("USB Disconnected");
    }
}

async fn process_data(_data: &[u8]) -> Vec<u8, 64> {
    Vec::new()
}
