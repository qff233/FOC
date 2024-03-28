#![no_std]
#![no_main]

use defmt::*;

use lazy_static::lazy_static;
use static_cell::StaticCell;

use embassy_executor::Spawner;
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

mod foc;
mod task;
// use task::can_comm_task;

bind_interrupts!(struct Irqs {
    USB_LP => usb::InterruptHandler<peripherals::USB>;
    FDCAN1_IT0 => can::IT0InterruptHandler<peripherals::FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<peripherals::FDCAN1>;
});

pub struct GlobalPeripherals {
    pub usb_class: CdcAcmClass<'static, usb::Driver<'static, peripherals::USB>>,
    pub can: can::Fdcan<'static, peripherals::FDCAN1>,
    pub pwm: ComplementaryPwm<'static, peripherals::TIM1>,
    pub adc: Adc<'static, peripherals::ADC1>,
}

lazy_static! {
    static ref GLOBALPERIPHERALS: StaticCell<GlobalPeripherals> = StaticCell::new();
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    let p = {
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
        embassy_stm32::init(config)
    };

    info!("System starting...");
    ////////////////////////////////////////////////////////////
    // Init USB Driver
    info!("Init USB Driver...");
    let (mut usb, usb_class) = {
        let driver = usb::Driver::new(p.USB, Irqs, p.PA12, p.PA11);
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("Embassy");
        config.product = Some("FOC Controller");
        config.serial_number = Some("666");
        config.device_class = 0xEF;
        config.device_sub_class = 0x02;
        config.device_protocol = 0x01;
        config.composite_with_iads = true;

        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
        static STATE: StaticCell<State> = StaticCell::new();
        let config_descriptor = CONFIG_DESCRIPTOR.init([0; 256]);
        let bos_descriptor = BOS_DESCRIPTOR.init([0; 256]);
        let control_buf = CONTROL_BUF.init([0; 64]);
        let state = STATE.init(State::new());

        let mut builder = Builder::new(
            driver,
            config,
            config_descriptor,
            bos_descriptor,
            &mut [], // no msos descriptors
            control_buf,
        );

        let usb_class: CdcAcmClass<'static, usb::Driver<'static, peripherals::USB>> =
            CdcAcmClass::new(&mut builder, state, 64);
        let usb = builder.build();
        (usb, usb_class)
    };

    ////////////////////////////////////////////////////////////
    // Init CAN
    info!("Init CAN Driver...");
    let can = {
        let mut can = can::FdcanConfigurator::new(p.FDCAN1, p.PB8, p.PB9, Irqs);
        can.set_extended_filter(
            can::filter::ExtendedFilterSlot::_0,
            can::filter::ExtendedFilter::accept_all_into_fifo1(),
        );
        can.set_bitrate(250_000);
        // can.set_fd_data_bitrate(1_000_000, false);  // use fdcan
        // let mut can = can.start(can::FdcanOperatingMode::InternalLoopbackMode);
        can.start(can::FdcanOperatingMode::NormalOperationMode)
    };

    ////////////////////////////////////////////////////////////
    // Init PWM
    info!("Init PWM Driver...");
    let pwm = {
        let u_h = PwmPin::new_ch1(p.PA8, embassy_stm32::gpio::OutputType::PushPull);
        let u_l = ComplementaryPwmPin::new_ch1(p.PB13, embassy_stm32::gpio::OutputType::PushPull);
        let v_h = PwmPin::new_ch2(p.PA9, embassy_stm32::gpio::OutputType::PushPull);
        let v_l = ComplementaryPwmPin::new_ch2(p.PB14, embassy_stm32::gpio::OutputType::PushPull);
        let w_h = PwmPin::new_ch3(p.PA10, embassy_stm32::gpio::OutputType::PushPull);
        let w_l = ComplementaryPwmPin::new_ch3(p.PB15, embassy_stm32::gpio::OutputType::PushPull);
        // let u_h = PwmPin::new_ch1(p.PA8, embassy_stm32::gpio::OutputType::PushPull);
        unsafe {
            interrupt::TIM1_UP_TIM16.enable();
        }
        interrupt::TIM1_UP_TIM16.set_priority(Priority::P6);
        ComplementaryPwm::new(
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
        )
    };

    ////////////////////////////////////////////////////////////
    // Init ADC
    info!("Init ADC Driver...");
    let adc = {
        let mut adc = Adc::new(p.ADC1, &mut Delay);
        adc.set_sample_time(SampleTime::CYCLES2_5);
        adc.set_resolution(embassy_stm32::adc::Resolution::BITS12);
        adc
    };

    ////////////////////////////////////////////////////////////
    GLOBALPERIPHERALS.init(GlobalPeripherals {
        usb_class,
        can,
        pwm,
        adc,
    });

    usb.run().await;
}

#[interrupt]
fn TIM1_UP_TIM16() {}
