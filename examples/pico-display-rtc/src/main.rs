#![no_std]
#![no_main]

use core::fmt::Write;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, FunctionSioOutput, Pin, PullDown};

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
// For in the graphics drawing utilities like the font
// and the drawing routines:
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
    text::{Baseline, Text},
};

use heapless::consts::*;
use heapless::String;

use u8g2_fonts::fonts::u8g2_font_wqy12_t_gb2312;
use u8g2_fonts::U8g2TextStyle;
// The display driver:
use ssd1306::{prelude::*, Ssd1306};

/// For ds1302 RTC
use ds1302_drv::{Calendar, Clock, DSClock, Hours, Mode, DS1302};

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure two pins as being I²C, not GPIO
    let sda: Pin<_, FunctionI2C, _> = pins.gpio2.reconfigure();
    let scl: Pin<_, FunctionI2C, _> = pins.gpio3.reconfigure();

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let mut i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda,
        scl, // Try `not_an_scl_pin` here
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // Create the I²C display interface:
    let interface = ssd1306::I2CDisplayInterface::new(i2c);

    // Create a driver instance and initialize:
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    // Create a text style for drawing the font:
    let character_style = U8g2TextStyle::new(u8g2_font_wqy12_t_gb2312, BinaryColor::On);

    // Configure io,clk,sclk pin for DS1302
    let io_ds1302: Pin<_, FunctionSioOutput, PullDown> = pins.gpio16.reconfigure();
    let sclk_ds1302: Pin<_, FunctionSioOutput, PullDown> = pins.gpio17.reconfigure();
    let ce_ds1302: Pin<_, FunctionSioOutput, PullDown> = pins.gpio18.reconfigure();

    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let timer_ds1302: DSClock<100> = DSClock::new(timer);
    // Configure GPIO25 as an output
    let mut led_pin = pins.gpio25.into_push_pull_output();

    let mut ds1302_ctl = DS1302::new(
        ce_ds1302,
        io_ds1302,
        sclk_ds1302,
        Mode::Hour24,
        timer_ds1302,
    )
    .unwrap();

    let mut data = String::<U32>::from(" ");

    let _ = ds1302_ctl.set_running(true);

    // If first run need to init the clock && calendar
    /*
    let h = Hours::Hour24(13);
    let clk = Clock {
        hours: h,
        minutes: 24,
        seconds: 0,
    };
    let cal = Calendar {
        day: 7,
        date: 10,
        month: 11,
        year: 2024,
    };

    let _ = ds1302_ctl.set_clock_calendar(clk, cal);
    */

    //let _ = ds1302_ctl.set_clock_mode(Mode::Hour12);
    // let _ = ds1302_ctl.set_clock_mode(Mode::Hour24);

    loop {
        data.clear();
        // timer.delay_ms(1000);
        let cl = ds1302_ctl.get_clock_calendar().unwrap();
        let (text, h) = match cl.0.hours {
            Hours::Hour24(h) => ("", h),
            Hours::Hour12am(h) => ("am", h),
            Hours::Hour12pm(h) => ("pm", h),
        };
        let _ = write!(
            data,
            "{} {}.{}.{} {:02}:{:02}:{:02} {}",
            cl.1.day, cl.1.date, cl.1.month, cl.1.year, h, cl.0.minutes, cl.0.seconds, text
        );

        let c2 = ds1302_ctl.get_year().unwrap();
        let c3 = ds1302_ctl.get_month().unwrap();
        let c4 = ds1302_ctl.get_date().unwrap();
        let c5 = ds1302_ctl.get_clock().unwrap();
        let c6 = ds1302_ctl.get_seconds().unwrap();
        let c7 = ds1302_ctl.get_minutes().unwrap();
        let c8 = ds1302_ctl.get_hours().unwrap();
        let c9 = ds1302_ctl.get_calendar().unwrap();

        let (text, h) = match c5.hours {
            Hours::Hour24(h) => ("", h),
            Hours::Hour12am(h) => ("am", h),
            Hours::Hour12pm(h) => ("pm", h),
        };

        Text::with_baseline(
            data.as_str(),
            Point::new(3, 2),
            character_style.clone(),
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        data.clear();
        let _ = write!(
            data,
            " {}-{}-{} {:02}:{:02}:{:02} {}",
            c2, c3, c4, h, c5.minutes, c5.seconds, text
        );

        Text::with_baseline(
            data.as_str(),
            Point::new(3, 25),
            character_style.clone(),
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        data.clear();
        let (text, h) = match c8 {
            Hours::Hour24(h) => ("", h),
            Hours::Hour12am(h) => ("am", h),
            Hours::Hour12pm(h) => ("pm", h),
        };
        let _ = write!(data, "{:02}:{:02}:{:02} {}", h, c7, c6, text);

        Text::with_baseline(
            data.as_str(),
            Point::new(3, 35),
            character_style.clone(),
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        data.clear();

        let _ = write!(data, "{}-{}-{} {}", c9.year, c9.month, c9.date, c9.day);

        Text::with_baseline(
            data.as_str(),
            Point::new(3, 45),
            character_style.clone(),
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        Line::new(Point::new(0, 0), Point::new(127, 0))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut display)
            .unwrap();

        Line::new(Point::new(0, 0), Point::new(0, 63))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut display)
            .unwrap();

        Line::new(Point::new(0, 63), Point::new(127, 63))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut display)
            .unwrap();

        Line::new(Point::new(127, 0), Point::new(127, 63))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut display)
            .unwrap();

        Line::new(Point::new(70, 0), Point::new(70, 16))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut display)
            .unwrap();

        Line::new(Point::new(0, 16), Point::new(127, 16))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut display)
            .unwrap();

        Line::new(Point::new(0, 15), Point::new(127, 15))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut display)
            .unwrap();

        let _ = led_pin.set_high();
        timer.delay_ms(250);

        let _ = led_pin.set_low();
        timer.delay_ms(250);

        display.flush().unwrap();
        let _ = display.clear(BinaryColor::Off);
        timer.delay_ms(1000);
    }
}
