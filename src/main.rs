#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::time::{khz, mhz};
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_stm32::{bind_interrupts, Config};
use embassy_stm32::{can, usb};
use embassy_stm32::{interrupt, peripherals};
use embassy_time::Delay;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::Builder;

use embassy_stm32::timer::complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin};

use {defmt_rtt as _, panic_probe as _};

mod task;
mod foc;
use task::can_comm_task;

bind_interrupts!(struct Irqs {
    USB_LP => usb::InterruptHandler<peripherals::USB>;
    FDCAN1_IT0 => can::IT0InterruptHandler<peripherals::FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<peripherals::FDCAN1>;
});

macro_rules! impl_pwm_pin {
    ($ch:ident,$postive_var:ident, $postive_pin:expr, $negetive_var:ident, $negetive_pin:expr) => {
        let $postive_var = PwmPin::$ch($postive_pin, embassy_stm32::gpio::OutputType::PushPull);
        let $negetive_var =
            ComplementaryPwmPin::$ch($negetive_pin, embassy_stm32::gpio::OutputType::PushPull);
    };
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: mhz(8),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV2,
            mul: PllMul::MUL85,
            divp: None,
            divq: Some(PllQDiv::DIV8),
            divr: Some(PllRDiv::DIV2),
        });
        // Main system clock at 170 MHz
        config.rcc.mux.fdcansel = mux::Fdcansel::PLL1_Q;
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.mux.adc12sel = mux::Adcsel::SYS;
    }
    let p = embassy_stm32::init(config);
    info!("System starting...");
    ////////////////////////////////////////////////////////////
    // Init USB Driver
    info!("Init USB Driver...");
    let driver = usb::Driver::new(p.USB, Irqs, p.PA12, p.PA11);
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("FOC Controller");
    config.serial_number = Some("666");
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
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);
    let mut usb = builder.build();
    let usb_fut = usb.run();

    ////////////////////////////////////////////////////////////
    // Init CAN
    info!("Init CAN Driver...");
    let mut can = can::FdcanConfigurator::new(p.FDCAN1, p.PB8, p.PB9, Irqs);
    can.set_extended_filter(
        can::filter::ExtendedFilterSlot::_0,
        can::filter::ExtendedFilter::accept_all_into_fifo1(),
    );
    can.set_bitrate(250_000);
    // can.set_fd_data_bitrate(1_000_000, false);  // use fdcan
    // let mut can = can.start(can::FdcanOperatingMode::InternalLoopbackMode);
    let can = can.start(can::FdcanOperatingMode::NormalOperationMode);

    ////////////////////////////////////////////////////////////
    // Init PWM
    info!("Init PWM Driver...");
    impl_pwm_pin!(new_ch1, u_h, p.PA8, u_l, p.PB13);
    impl_pwm_pin!(new_ch2, v_h, p.PA9, v_l, p.PB14);
    impl_pwm_pin!(new_ch3, w_h, p.PA10, w_l, p.PB15);
    // let u_h = PwmPin::new_ch1(p.PA8, embassy_stm32::gpio::OutputType::PushPull);
    let pwm = ComplementaryPwm::new(
        p.TIM1,
        Some(u_h),
        Some(u_l),
        Some(v_h),
        Some(v_l),
        Some(w_h),
        Some(w_l),
        None,
        None,
        khz(20),
        CountingMode::CenterAlignedUpInterrupts,
    );
    unsafe {
        interrupt::TIM1_UP_TIM16.enable();
    }
    interrupt::TIM1_UP_TIM16.set_priority(Priority::P6);

    ////////////////////////////////////////////////////////////
    // Init ADC
    info!("Init ADC Driver...");
    let mut adc = Adc::new(p.ADC1, &mut Delay);
    adc.set_sample_time(SampleTime::CYCLES2_5);
    adc.set_resolution(embassy_stm32::adc::Resolution::BITS12);
    // adc.read_internal(channel);
    join3(usb_fut, task::usb_comm_task(&mut class), can_comm_task(can)).await;
}

#[interrupt]
fn TIM1_UP_TIM16() {}
