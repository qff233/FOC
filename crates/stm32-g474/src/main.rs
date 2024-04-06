#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use embassy_executor::{Executor, InterruptExecutor};
use embassy_futures::block_on;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::gpio::Output;
use embassy_stm32::spi::{self, Spi, MODE_0, MODE_1};
use embassy_stm32::time::{khz, mhz};
use embassy_stm32::timer::complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin};
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_stm32::{bind_interrupts, gpio, timer, Config};
use embassy_stm32::{can, usb};
use embassy_stm32::{interrupt, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::Delay;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::Builder;
use foc::angle_sensor::as5048::As5048;
use foc::current_sensor::CurrentSensor;
use foc::{LoopMode, MotorParams};
use static_cell::StaticCell;

use crate::interface::{Adcs, Pwms, VbusAdc};

use {defmt_rtt as _, panic_probe as _};

mod interface;
mod task;

bind_interrupts!(struct Irqs {
    USB_LP => usb::InterruptHandler<peripherals::USB>;
    FDCAN1_IT0 => can::IT0InterruptHandler<peripherals::FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<peripherals::FDCAN1>;
});

static EXECUTOR_FOC_LOOP: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_COMM: StaticCell<Executor> = StaticCell::new();

#[allow(dead_code)]
enum SharedEvent {
    Iuvw(f32, f32, f32),
    Idq(f32, f32),
    Position(f32),
    Velocity(f32),
    State {
        i_uvw: (f32, f32, f32),
        i_dq: (f32, f32),
        position: f32,
        velocity: f32,
    },
}

type FocMutex = Mutex<CriticalSectionRawMutex, Option<foc::FOC>>;
static SHAREDCHANNEL: Channel<CriticalSectionRawMutex, SharedEvent, 64> = Channel::new();

#[entry]
fn main() -> ! {
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
    let (usb, usb_class) = {
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
    info!("Init CAN Driver...");
    let can = {
        let mut can = can::CanConfigurator::new(p.FDCAN1, p.PB8, p.PB9, Irqs);
        can.set_extended_filter(
            can::filter::ExtendedFilterSlot::_0,
            can::filter::ExtendedFilter::accept_all_into_fifo1(),
        );
        can.set_bitrate(250_000);
        // can.set_fd_data_bitrate(1_000_000, false);  // use fdcan
        // let mut can = can.start(can::FdcanOperatingMode::InternalLoopbackMode);
        can.start(can::OperatingMode::NormalOperationMode)
    };

    ////////////////////////////////////////////////////////////
    // Init PWM
    info!("Init PWM Driver...");
    let mut pwm = {
        let u_h = PwmPin::new_ch1(p.PA8, embassy_stm32::gpio::OutputType::PushPull);
        let u_l = ComplementaryPwmPin::new_ch1(p.PB13, gpio::OutputType::PushPull);
        let v_h = PwmPin::new_ch2(p.PA9, embassy_stm32::gpio::OutputType::PushPull);
        let v_l = ComplementaryPwmPin::new_ch2(p.PB14, gpio::OutputType::PushPull);
        let w_h = PwmPin::new_ch3(p.PA10, embassy_stm32::gpio::OutputType::PushPull);
        let w_l = ComplementaryPwmPin::new_ch3(p.PB15, gpio::OutputType::PushPull);
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
    pwm.set_duty(timer::Channel::Ch4, pwm.get_max_duty() - 50); // This is set for ADC toggle
    pwm.enable(timer::Channel::Ch4);

    ////////////////////////////////////////////////////////////
    info!("Init ADC Driver...");
    // let adc1 = {
    //     let mut adc = Adc::new(p.ADC1, &mut Delay);
    //     adc.set_sample_time(SampleTime::CYCLES2_5);
    //     adc.set_resolution(embassy_stm32::adc::Resolution::BITS16);
    //     adc
    // };
    let adc2 = {
        let mut adc = Adc::new(p.ADC2, &mut Delay);
        adc.set_sample_time(SampleTime::CYCLES2_5);
        adc.set_resolution(embassy_stm32::adc::Resolution::BITS16);
        adc
    };
    let mut uvw_adcs = Adcs::new();

    ////////////////////////////////////////////////////////////
    info!("Init FOC...");
    let mut pwms = Pwms::new(pwm);
    let vbus_adc = VbusAdc::new(adc2, p.PC5);
    // let mut uvw_adcs = Adcs::new(adc1, p.PA0, p.PA1, p.PA2);

    let foc = foc::FOC::new(
        MotorParams {
            pole_num: 7,
            resistance: None,
            inductance: None,
            encoder_offset: None,
        },
        // LoopMode::OpenVelocity {
        //     voltage: 0.15,
        //     expect_velocity: 90f32.to_radians(),
        // },
        LoopMode::TorqueWithSensor {
            current_pid: foc::PID::new(1.0, 0.0, 0.0, 0.000_5, 0.0, 1.0, 1.0),
            expect_current: 0.15,
        },
        Some(CurrentSensor::new(
            0.005,
            16.0,
            &mut pwms,
            &mut Delay,
            &mut uvw_adcs,
        )),
        1. / 20_000.,
        1. / 8_000.,
        1. / 1_000.,
    );
    static FOC: FocMutex = Mutex::new(None);
    block_on(async {
        *FOC.lock().await = Some(foc);
    });

    let mut spi_config = spi::Config::default();
    spi_config.mode = MODE_1;
    spi_config.frequency = mhz(1);
    let spi = Spi::new(
        p.SPI1, p.PB3, p.PB5, p.PB4, p.DMA1_CH1, p.DMA1_CH2, spi_config,
    );

    let as5048 = As5048::new(spi, Output::new(p.PB6, gpio::Level::High, gpio::Speed::Low));

    ////////////////////////////////////////////////////////////
    info!("Init PWM Interrupt...");

    let spawner = EXECUTOR_FOC_LOOP.start(interrupt::ADC1_2);
    spawner
        .spawn(task::current_loop(
            &FOC,
            SHAREDCHANNEL.sender(),
            vbus_adc,
            uvw_adcs,
            pwms,
            as5048,
        ))
        .unwrap();
    spawner.spawn(task::velocity_loop(&FOC)).unwrap();
    spawner.spawn(task::position_loop(&FOC)).unwrap();

    info!("Init exector");
    let executor = EXECUTOR_COMM.init(Executor::new());
    executor.run(|spawner| {
        spawner
            .spawn(task::usb_comm(usb, usb_class, SHAREDCHANNEL.receiver()))
            .unwrap();
        spawner
            .spawn(task::can_comm(can, SHAREDCHANNEL.receiver()))
            .unwrap();
    });
}

#[interrupt]
unsafe fn ADC1_2() {
    // debug!("{}", embassy_stm32::pac::ADC1.isr().read().0);
    if embassy_stm32::pac::ADC1.isr().read().jeos() {
        EXECUTOR_FOC_LOOP.on_interrupt();
        embassy_stm32::pac::ADC1.isr().write(|w| w.set_jeos(true));
    }
}
