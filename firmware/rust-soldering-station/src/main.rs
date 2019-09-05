#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate embedded_graphics;
extern crate embedded_hal;
extern crate numtoa;
extern crate panic_semihosting;
extern crate rtfm;
extern crate ssd1306;
extern crate stm32f1xx_hal;

use rt::{exception, ExceptionFrame};
use rtfm::{app, Instant};
use numtoa::NumToA;

use ssd1306::prelude::*;
use ssd1306::mode::GraphicsMode;
use ssd1306::Builder;

use embedded_graphics::fonts::Font12x16;
use embedded_graphics::fonts::Font6x8;
use embedded_graphics::prelude::*;

use stm32f1xx_hal::{
    prelude::*,
    pac,
    gpio::{gpioa::PA0, gpioc::PC13, gpiob::PB10, gpiob::PB11, Output, PushPull, OpenDrain, Alternate},
    device::I2C2,
    device::TIM2,
    device::ADC1,
    pwm::{C1, Pwm, Pins},
    i2c::{BlockingI2c, DutyCycle, Mode},
};

use embedded_hal::digital::v2::OutputPin;

struct MyChannels(PA0<Alternate<PushPull>>);

impl Pins<TIM2> for MyChannels
{
    const REMAP: u8 = 0b00;
    const C1: bool = true;
    const C2: bool = false;
    const C3: bool = false;
    const C4: bool = false;
    type Channels = Pwm<TIM2, C1>;
}

/////////////////////////////////////////
// Pins in HW revision 2:              //
// Heater gate:     PA0                //
// Tip temperature: PA1, ADC channel 1 //
// Vin sense:       PA3, ADC channel 3 //
// Potentiometer:   PB1, ADC channel 9 //
// Chip temperature: ADC channel 16    //
// SCL:             PB10               //
// SDA:             PB11               //
/////////////////////////////////////////

#[rtfm::app(device = stm32f1xx_hal::pac)]
const APP: () = {
    static mut display: ssd1306::mode::GraphicsMode<
        ssd1306::interface::I2cInterface<
            BlockingI2c<
                I2C2,(
                    PB10<Alternate<OpenDrain>>,
                    PB11<Alternate<OpenDrain>>
                )
            >
        >
    > = ();
    static mut led: PC13<Output<PushPull>> = ();
    static mut adc1: ADC1 = ();
    static mut pwm: Pwm<TIM2, C1> = ();

    static mut led_on: bool = true;
    static mut chip_temperature: i16 = 0;
    static mut tip_temperature: i16 = 0;
    static mut set_temperature: u16 = 0;
    static mut duty: u16 = 0;

    #[init(schedule = [led_blinker, regulator])]
    fn init(c: init::Context) -> init::LateResources {
        let mut peripherals: pac::Peripherals = c.device;

        let mut flash = peripherals.FLASH;
        flash.acr.modify(|_, w| unsafe { w.latency().bits(2u8) });
        let mut flash = flash.constrain();

        let mut rcc = peripherals.RCC;
        // Enable clock for ADC1
        rcc.apb2enr.modify(|_, w| w.adc1en().set_bit());
        rcc.cfgr.modify(|_, w| w.adcpre().div6());
        let mut rcc = rcc.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8_000_000.hz())
            .sysclk(72_000_000.hz())
            .pclk1(36_000_000.hz())
            .pclk2(72_000_000.hz())
            .hclk(72_000_000.hz())
            .freeze(&mut flash.acr);

        let mut afio = peripherals.AFIO.constrain(&mut rcc.apb2);

        let mut gpioa = peripherals.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = peripherals.GPIOB.split(&mut rcc.apb2);
        let mut gpioc = peripherals.GPIOC.split(&mut rcc.apb2);

        let heater_gate = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);

        let mut pwm_temp = peripherals.TIM2.pwm(
            MyChannels(heater_gate),
            &mut afio.mapr,
            20.khz(),
            clocks,
            &mut rcc.apb1
        );
        pwm_temp.set_duty(0);
        pwm_temp.enable();

        let scl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
        let sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);

        let i2c = BlockingI2c::i2c2(
            peripherals.I2C2,
            (scl, sda),
            Mode::Fast {
                frequency: 400_000,
                duty_cycle: DutyCycle::Ratio2to1,
            },
            clocks,
            &mut rcc.apb1,
            1000,
            255,
            1000,
            1000,
        );

        let mut adc1_temp = peripherals.ADC1;
        adc1_init(&mut adc1_temp);
        adc1_calibrate(&mut adc1_temp);

        let mut display_temp: GraphicsMode<_> = Builder::new().connect_i2c(i2c).into();
        display_temp.init().unwrap();
        display_temp.flush().unwrap();

        c.schedule.led_blinker(Instant::now() + 10_000_000.cycles()).unwrap();
        c.schedule
            .regulator(Instant::now() + 8_000_000.cycles())
            .unwrap();

        init::LateResources {
            led: gpioc.pc13.into_push_pull_output(&mut gpioc.crh),
            adc1: adc1_temp,
            pwm: pwm_temp,
            display: display_temp,
        }
    }

    #[idle(resources = [display, chip_temperature, set_temperature, tip_temperature, duty, adc1])]
    fn idle(mut c: idle::Context) -> ! {
        let mut buffer1 = [0u8; 20];
        let mut buffer2 = [0u8; 20];
        let mut buffer3 = [0u8; 20];

        loop {
            let chip_temperature_vsense = c.resources
                .adc1
                .lock(|adc1| adc1_measure_single_blocking(&mut *adc1, 16));
            // Temperature (in °C) = {(V25 - VSENSE) / Avg_Slope} + 25.
            // V25 = 1.43 V, Avg_Slope = 4.3 mV/°C
            let chip_temp = (8i16 * (1820i16 - chip_temperature_vsense as i16) / 43i16) + 200i16;
            c.resources
                .chip_temperature
                .lock(|chip_temperature| *chip_temperature = chip_temp);
            let chip_temp_str = chip_temp.numtoa_str(10, &mut buffer1);

            let set_temperature_vsense = c.resources
                .adc1
                .lock(|adc1| adc1_measure_single_blocking(&mut *adc1, 9));
            let set_temp = set_temperature_vsense as u16;
            c.resources
                .set_temperature
                .lock(|set_temperature| *set_temperature = set_temp);
            let set_temp_str = set_temp.numtoa_str(10, &mut buffer2);

            let tip_temp = c.resources
                .tip_temperature
                .lock(|tip_temperature| *tip_temperature);
            let tip_temp_str = tip_temp.numtoa_str(10, &mut buffer3);

            let duty_temp = c.resources
                .duty
                .lock(|duty| *duty);

            c.resources.display.clear();

            c.resources
                .display
                .draw(Font12x16::render_str("Chip:").into_iter());

            c.resources.display.draw(
                Font12x16::render_str(chip_temp_str)
                    .translate(Point::new(72, 0))
                    .into_iter(),
            );

            c.resources.display.draw(
                Font12x16::render_str("Tip:")
                    .translate(Point::new(0, 16))
                    .into_iter(),
            );

            c.resources.display.draw(
                Font12x16::render_str(tip_temp_str)
                    .translate(Point::new(72, 16))
                    .into_iter(),
            );

            c.resources.display.draw(
                Font12x16::render_str("Pot:")
                    .translate(Point::new(0, 32))
                    .into_iter(),
            );

            c.resources.display.draw(
                Font12x16::render_str(set_temp_str)
                    .translate(Point::new(72, 32))
                    .into_iter(),
            );

            // TODO: Port this:
            // c.resources.display.draw(
            //     embedded_graphics::primitives::Rectangle::new(Point::new(0, 62), Point::new(duty_temp as i32/512, 63)).stroke(Some(1u8.into())).into_iter(),
            // );

            // Send data to the display
            match c.resources.display.flush() {
                Ok(_) => (),
                Err(_) => (),
            };
        }
    }

    #[task(schedule = [regulator], resources = [pwm, set_temperature, tip_temperature, chip_temperature, duty, adc1], priority = 2)]
    fn regulator(mut c: regulator::Context) {
        c.resources.pwm.set_duty(0);
        cortex_m::asm::delay(10000);

        let tip_temperature_vsense = adc1_measure_single_blocking(&mut c.resources.adc1, 1);
        *c.resources.tip_temperature = tip_temperature_vsense as i16;

        //let max_duty = resources.pwm.get_max_duty();

        let error: i32 = (*c.resources.set_temperature as i32 - tip_temperature_vsense as i32)*50;
        let mut duty: u16 = 0;

        if error > 30000 {
            duty = 30000u16;
        } else if error > 0 {
            duty = error as u16;
        } else {
            duty = 0u16;
        }

        c.resources.pwm.set_duty(duty);
        *c.resources.duty = duty;

        c.schedule.regulator(c.scheduled + 1_000_000.cycles()).unwrap();
    }


    #[task(schedule = [led_blinker], resources = [led, led_on])]
    fn led_blinker(c: led_blinker::Context) {
        if *c.resources.led_on {
            c.resources.led.set_low();
            *c.resources.led_on = false; // No idea what is the reality
        } else {
            c.resources.led.set_high();
            *c.resources.led_on = true;
        }

        c.schedule.led_blinker(c.scheduled + 10_000_000.cycles()).unwrap();
    }

    extern "C" {
        fn RTCALARM();
        fn USART1();
    }
};

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

fn adc1_measure_single_blocking(adc1: &mut stm32f1xx_hal::device::ADC1, channel: u8) -> u16 {
    // Set number of conversions in sequence. 0b0000 means 1 conversion
    adc1.sqr1.modify(|_, w| unsafe { w.l().bits(0) });
    // Set the channel number for the first conversion
    adc1.sqr3.modify(|_, w| unsafe { w.sq1().bits(channel) });
    // Start conversion of regular channels
    adc1.cr2.modify(|_, w| w.swstart().set_bit());
    // Wait for the End Of Conversion
    while adc1.sr.read().eoc().bit_is_clear() {}
    adc1.dr.read().data().bits()
}

fn adc1_calibrate(adc1: &mut stm32f1xx_hal::device::ADC1) {
    // Initialize (reset) calibration registers
    adc1.cr2.modify(|_, w| w.rstcal().set_bit());
    // Wait for completion
    while adc1.cr2.read().rstcal().bit_is_set() {}
    // Start calibration
    adc1.cr2.modify(|_, w| w.cal().set_bit());
    // Wait for completion
    while adc1.cr2.read().cal().bit_is_set() {}
}

fn adc1_init(adc1: &mut stm32f1xx_hal::device::ADC1) {
    adc1.cr1.write(|w| unsafe { w.bits(0u32) });
    adc1.cr2.write(|w| unsafe { w.bits(0u32) });
    // Discontinuous mode
    adc1.cr1.modify(|_, w| w.discen().set_bit());
    // Number of discontinuous channels
    adc1.cr1.modify(|_, w| unsafe { w.discnum().bits(0) });
    // Enable vrefint and internal temperature sensor
    adc1.cr2
        .modify(|_, w| w.tsvrefe().set_bit().exttrig().set_bit());
    adc1.cr2.modify(|_, w| unsafe { w.extsel().bits(7) });
    // Setup sample times
    adc1.smpr1.modify(|_, w| unsafe {
        w.smp10()
            .bits(7)
            .smp11()
            .bits(7)
            .smp12()
            .bits(7)
            .smp13()
            .bits(7)
            .smp14()
            .bits(7)
            .smp15()
            .bits(7)
            .smp16()
            .bits(7)
            .smp17()
            .bits(7)
    });
    adc1.smpr2.modify(|_, w| unsafe {
        w.smp0()
            .bits(7)
            .smp1()
            .bits(7)
            .smp2()
            .bits(7)
            .smp3()
            .bits(7)
            .smp4()
            .bits(7)
            .smp5()
            .bits(7)
            .smp6()
            .bits(7)
            .smp7()
            .bits(7)
            .smp8()
            .bits(7)
            .smp9()
            .bits(7)
    });
    // Num of channels to convert: 1
    unsafe {
        adc1.sqr1.write(|w| w.bits(0u32));
        adc1.sqr2.write(|w| w.bits(0u32));
        adc1.sqr3.write(|w| w.bits(0u32));
    }
    // Turn on the ADC1
    adc1.cr2.modify(|_, w| w.adon().set_bit());
}
