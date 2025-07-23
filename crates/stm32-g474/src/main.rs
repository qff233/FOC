#![no_std]
#![no_main]
#![allow(mutable_transmutes)]

use core::mem;

use cortex_m_rt::entry;

use defmt::*;
use static_cell::StaticCell;

use embassy_executor::{Executor, InterruptExecutor};
use embassy_stm32::{
    adc::{Adc, SampleTime},
    bind_interrupts,
    can::{self, config::FdCanConfig},
    gpio::{self, Output},
    interrupt, peripherals,
    spi::{self, Spi, MODE_1},
    time::{khz, mhz},
    timer::{
        self,
        complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin},
        low_level::CountingMode,
        simple_pwm::PwmPin,
    },
    usb, Config,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State},
    Builder,
};

use foc::{
    angle_sensor::{as5048::As5048, Direction},
    current_sensor::ISensor,
    Foc, LoopMode, MotorParams, Pid, Pll2,
};

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
        u_dq: (f32, f32),
        i_dq: (f32, f32),
        position: (i32, f32),
        velocity: f32,
    },
}

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
        can.set_config(FdCanConfig::default());
        // can.set_bitrate(250_000);
        // can.set_fd_data_bitrate(1_000_000, false);  // use fdcan
        // let mut can = can.start(can::FdcanOperatingMode::InternalLoopbackMode);
        can.start(can::OperatingMode::NormalOperationMode)
    };

    ////////////////////////////////////////////////////////////
    // Init PWM
    info!("Init PWM Driver...");
    let mut pwm = {
        let u_h = PwmPin::new(p.PA8, gpio::OutputType::PushPull);
        let u_l = ComplementaryPwmPin::new(p.PB13, gpio::OutputType::PushPull);
        let v_h = PwmPin::new(p.PA9, embassy_stm32::gpio::OutputType::PushPull);
        let v_l = ComplementaryPwmPin::new(p.PB14, gpio::OutputType::PushPull);
        let w_h = PwmPin::new(p.PA10, embassy_stm32::gpio::OutputType::PushPull);
        let w_l = ComplementaryPwmPin::new(p.PB15, gpio::OutputType::PushPull);
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
        let mut adc = Adc::new(p.ADC2);
        adc.set_sample_time(SampleTime::CYCLES2_5);
        adc.set_resolution(embassy_stm32::adc::Resolution::BITS12);
        adc
    };
    let mut uvw_adcs = Adcs::new();

    ////////////////////////////////////////////////////////////
    info!("Init FOC...");
    let mut pwms = Pwms::new(pwm);
    let vbus_adc = VbusAdc::new(adc2, p.PC5);
    // let mut uvw_adcs = Adcs::new(adc1, p.PA0, p.PA1, p.PA2);

    let foc: foc::Foc = foc::Foc::new(
        MotorParams {
            pole_num: 7,
            resistance: None,
            inductance: None,
            encoder_offset: Some(-2.6825993),
        },
        ISensor::new(
            0.005,
            16.0,
            &mut pwms,
            &mut embassy_time::Delay,
            &mut uvw_adcs,
        ),
        1. / 20_000.,
        1. / 8_000.,
        1. / 1_000.,
    )
    .set_mode(
        LoopMode::PositionVelocityWithSensor {
            current_pid: (
                foc::Pid::new(0.3925, 277.78, 0.0, 0.000_05, 0.0, 0.5, 0.5), // id
                foc::Pid::new(0.3925, 277.78, 0.0, 0.000_05, 0.0, 0.5, 0.5), // iq
            ),
            velocity_pid: Pid::new(0.4, 0.001, 0.0, 1. / 8_000., 0.0, 3.0, 3.0),
            position_pid: Pid::new(10.0, 5.5, 0.0, 1. / 1_000., 0.0, 60., 1.5),
            pll: Pll2::new(0.005, 0.05, 1. / 8_000.),
            expect_position: (0, 0.0),
        },
        // LoopMode::VelocityWithSensor {
        //     current_pid: (
        //         foc::Pid::new(0.3925, 277.78, 0.0, 0.000_05, 0.0, 0.5, 0.5), // id
        //         foc::Pid::new(0.3925, 277.78, 0.0, 0.000_05, 0.0, 0.5, 0.5), // iq
        //     ),
        //     velocity_pid: Pid::new(0.4, 0.001, 0.0, 1. / 8_000., 0.0, 3.0, 3.0),
        //     pll: Pll2::new(0.005, 0.05, 1. / 8_000.),
        //     expect_velocity: 360_f32.to_radians(),
        // },
        // LoopMode::PositionWithSensor {
        //     current_pid: (
        //         foc::Pid::new(0.3925, 277.78, 0.0, 0.000_05, 0.0, 0.5, 0.5), // id
        //         foc::Pid::new(0.3925, 277.78, 0.0, 0.000_05, 0.0, 0.5, 0.5), // iq
        //     ),
        //     position_pid: Pid::new(0.00000000001, 0.0000, 0.8, 1. / 1_000., 0.7, 2.0, 2.0),
        //     expect_position: 0.0,
        // },
        // LoopMode::TorqueWithSensor {
        //     current_pid: (
        //         foc::Pid::new(0.3925, 277.78, 0.0, 0.000_05, 0.0, 0.5, 0.5), // id
        //         foc::Pid::new(0.3925, 277.78, 0.0, 0.000_05, 0.0, 0.5, 0.5), // iq
        //     ),
        //     expect_current: (0.0, 0.3),
        // },
        // LoopMode::Calibration {
        //     has_encoder_offset: false,
        // },
    );

    // block_on(async {
    //     *FOC.lock().await = Some(foc);
    // });

    let mut spi_config = spi::Config::default();
    spi_config.mode = MODE_1;
    spi_config.frequency = mhz(10);
    let spi = Spi::new_blocking(p.SPI1, p.PB3, p.PB5, p.PB4, spi_config);

    let as5048 = As5048::new(
        spi,
        Output::new(p.PD2, gpio::Level::High, gpio::Speed::Medium),
        Direction::CW,
    );

    ////////////////////////////////////////////////////////////
    info!("Init PWM Interrupt...");

    let spawner = EXECUTOR_FOC_LOOP.start(interrupt::ADC1_2);
    spawner
        .spawn(task::current_loop(
            unsafe { mem::transmute::<&Foc, &mut Foc>(&foc) },
            SHAREDCHANNEL.sender(),
            vbus_adc,
            uvw_adcs,
            pwms,
            as5048,
            CortexDelay::new(),
        ))
        .unwrap();
    spawner
        .spawn(task::velocity_loop(unsafe {
            mem::transmute::<&Foc, &mut Foc>(&foc)
        }))
        .unwrap();
    spawner
        .spawn(task::position_loop(unsafe {
            mem::transmute::<&Foc, &mut Foc>(&foc)
        }))
        .unwrap();

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

#[derive(Clone)]
struct CortexDelay {}
impl CortexDelay {
    fn new() -> Self {
        Self {}
    }
}
impl embedded_hal::delay::DelayNs for CortexDelay {
    fn delay_ns(&mut self, ns: u32) {
        cortex_m::asm::delay(ns);
    }
}
