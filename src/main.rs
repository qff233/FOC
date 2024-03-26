#![no_std]
#![no_main]

use defmt::*;
use embassy_futures::join::join;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::peripherals;
use embassy_stm32::time::mhz;
use embassy_stm32::usb;
use embassy_stm32::{bind_interrupts, Config};
use embassy_time::Delay;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::Builder;
use {defmt_rtt as _, panic_probe as _};

mod task;

bind_interrupts!(struct Irqs {
    USB_LP => usb::InterruptHandler<peripherals::USB>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSE(mhz(8)),
            prediv_m: PllM::DIV2,
            mul_n: PllN::MUL85,
            div_r: Some(PllR::DIV2),
            div_p: None,
            div_q: None,
        });
        // Main system clock at 170 MHz
        config.rcc.mux = ClockSrc::PLL;
        config.rcc.adc12_clock_source = AdcClockSource::SYS;
        config.rcc.adc345_clock_source = AdcClockSource::SYS;
    }
    let p = embassy_stm32::init(config);
    info!("System starting...");

    let driver = usb::Driver::new(p.USB, Irqs, p.PA12, p.PA11);
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-Serial Example");
    config.serial_number = Some("123456");

    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut [],
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    let mut usb = builder.build();

    let usb_fut = usb.run();

    let mut adc = Adc::new(p.ADC2, &mut Delay);
    adc.set_sample_time(SampleTime::Cycles32_5);

    join(usb_fut, task::usb_comm_task(&mut class)).await;
}
