//! DS1302 real time clock-calendar platform agnostic driver
//!
//! # About
//!
//!The DS1302 trickle-charge timekeeping chip contains a real-time clock/calendar and 31 bytes of static RAM. It
//!communicates with a microprocessor via a simple serial interface. The real-time clock/calendar provides seconds,
//!minutes, hours, day, date, month, and year information. The end of the month date is automatically adjusted for
//!months with fewer than 31 days, including corrections for leap year. The clock operates in either the 24-hour or
//!12-hour format with an AM/PM indicator. The chip driver is based on [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
//!
//!Datasheet: [DS1302](https://datasheets.maximintegrated.com/en/ds/DS1302.pdf)
//!
//! ## Driver features:
//! - Reading/setting clock/calendar data
//! - 12-hour (AM/PM) or 24-hour format
//! - Changing the time format while the chip is working
//!
//!
//! NEW (4.0.0 release):
//! - Programmable Trickle Charger configuration
//! - 31 x 8 Battery-Backed General-Purpose RAM operations
//!

#![no_std]
#![allow(non_camel_case_types)]

#[cfg(all(feature = "rp2040", feature = "rp2350"))]
compile_error!("You must not enable both the `rp2040` and `rp2350` Cargo features.");
#[cfg(not(any(feature = "rp2040", feature = "rp2350")))]
compile_error!("You must enable either the `rp2040` or the `rp2350` Cargo features.");

use core::convert::From;

mod registers;
use crate::registers::{Ds, RegisterRead, RegisterWrite, Rs, TrickleCharger};

//use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::digital::{InputPin, OutputPin};

use fugit::ExtU32;

#[cfg(feature = "rp2350")]
use rp235x_hal as hal;

#[cfg(feature = "rp2040")]
use rp2040_hal as hal;

use hal::gpio::{
    FunctionSio, FunctionSioOutput, Pin, PinId, PullDown, SioInput, SioOutput, ValidFunction,
};

/// CmdFlag definitions
pub enum CmdFlag {
    CLOCK_HALT,
    DISABLE_WRITE_PROTECT,
    WRITE_PROTECT,
    HOUR_12_BIT,
    HOUR_PM_BIT,
}

impl CmdFlag {
    pub fn addr(self) -> u8 {
        match self {
            Self::DISABLE_WRITE_PROTECT => 0x0 as u8,
            Self::HOUR_PM_BIT => 0x20 as u8,
            Self::CLOCK_HALT => 0x80 as u8,
            Self::WRITE_PROTECT => 0x80 as u8,
            Self::HOUR_12_BIT => 0x80 as u8,
        }
    }
}

/// DS1302 error
#[derive(Debug)]
pub enum Ds1302Error {
    Parameter,
    Unknown,
    ClockError,
    ReadError,
    ChipSelectError,
}

/// For timing `ds1302` uses [fugit](https://lib.rs/crates/fugit) crate which only provides `Duration` and `Instant` types.
/// It does not provide any clock or timer traits.
/// Therefore `ds1302` has its own `Delay` trait that provides all timing capabilities that are needed for the library.
/// User must implement this trait for the timer by itself.
pub trait Delay<const TIMER_HZ: u32> {
    /// An error that might happen during waiting
    type Error;

    /// Return current time `Instant`
    fn now(&mut self) -> fugit::TimerInstantU32<TIMER_HZ>;

    /// Start countdown with a `duration`
    fn start(&mut self, duration: fugit::TimerDurationU32<TIMER_HZ>) -> Result<(), Self::Error>;

    /// Wait until countdown `duration` has expired.
    /// Must return `nb::Error::WouldBlock` if countdown `duration` is not yet over.
    /// Must return `OK(())` as soon as countdown `duration` has expired.
    fn wait(&mut self) -> nb::Result<(), Self::Error>;
}

///Hour format: 12-hour (AM/PM) or 24-hour
#[derive(PartialEq, Copy, Clone)]
pub enum Mode {
    Hour24,
    Hour12,
}
///Hour information: 12-hour (AM/PM) or 24-hour
#[derive(Debug, Copy, Clone)]
pub enum Hours {
    Hour24(u8),
    Hour12am(u8),
    Hour12pm(u8),
}

impl Hours {
    fn convert(&self) -> Self {
        match *self {
            Hours::Hour24(h) => {
                if h >= 12 {
                    Hours::Hour12pm(h - 12)
                } else {
                    Hours::Hour12am(h)
                }
            }
            Hours::Hour12pm(h) => Hours::Hour24(h + 12),
            Hours::Hour12am(h) => Hours::Hour24(h),
        }
    }

    /// Get the hour.
    /// return.1: None => Hour24 mode; Some(false) => pm; Some(true) => am;
    pub fn hour(&self) -> (u8, Option<bool>) {
        match *self {
            Hours::Hour24(h) => (h, None),
            Hours::Hour12am(h) => (h, Some(false)),
            Hours::Hour12pm(h) => (h, Some(true)),
        }
    }
}

impl From<u8> for Hours {
    fn from(byte: u8) -> Self {
        if (byte & CmdFlag::HOUR_12_BIT.addr()) != 0 {
            //In case 12-hour format
            let hour = bcd_to_decimal(
                byte & (!(CmdFlag::HOUR_12_BIT.addr() | CmdFlag::HOUR_PM_BIT.addr())),
            );
            if (byte & CmdFlag::HOUR_PM_BIT.addr()) != 0 {
                // It's PM
                Hours::Hour12pm(hour)
            } else {
                // It's AM
                Hours::Hour12am(hour)
            }
        } else {
            let hour = bcd_to_decimal(byte);
            Hours::Hour24(hour)
        }
    }
}

impl From<Hours> for u8 {
    fn from(h: Hours) -> Self {
        match h {
            Hours::Hour24(hour) => decimal_to_bcd(hour),
            Hours::Hour12am(hour) => decimal_to_bcd(hour) | CmdFlag::HOUR_12_BIT.addr(),
            Hours::Hour12pm(hour) => {
                decimal_to_bcd(hour) | CmdFlag::HOUR_12_BIT.addr() | CmdFlag::HOUR_PM_BIT.addr()
            }
        }
    }
}

///Clock information
#[derive(Debug, Copy, Clone)]
pub struct Clock {
    pub hours: Hours,
    pub minutes: u8,
    pub seconds: u8,
}
///Calendar information

#[derive(Debug, Copy, Clone)]
pub struct Calendar {
    pub day: u8,
    pub date: u8,
    pub month: u8,
    pub year: u16,
}

// Swap format from bcd to decmial
pub(crate) fn bcd_to_decimal(bcd: u8) -> u8 {
    ((bcd & 0xF0) >> 4) * 10 + (bcd & 0x0F)
}

// Swap format from decimal to bcd
pub(crate) fn decimal_to_bcd(decimal: u8) -> u8 {
    ((decimal / 10) << 4) + (decimal % 10)
}

///SCLK,CE都为 OutPutPin设置
/// DAT为 数据输出端口需要设置为 PushPullOutput
/// 参考链接: https://github.com/task-jp/ch32v003j4m6-ds1302/blob/main/src/main.rs

///Input. CE signal must be asserted high during a read or a write
///Note: Previous data sheet revisions referred to CE as RST

pub struct DS1302<I1, I2, I3, D, const TIMER_HZ: u32>
where
    I1: PinId + ValidFunction<FunctionSio<SioInput>> + ValidFunction<FunctionSio<SioOutput>>,
    I2: PinId + ValidFunction<FunctionSio<SioInput>> + ValidFunction<FunctionSio<SioOutput>>,
    I3: PinId + ValidFunction<FunctionSio<SioInput>> + ValidFunction<FunctionSio<SioOutput>>,
{
    pub io: Option<Pin<I1, FunctionSioOutput, PullDown>>,
    pub ce: Pin<I2, FunctionSioOutput, PullDown>,
    pub sclk: Pin<I3, FunctionSioOutput, PullDown>,
    pub delay: D,
}

impl<I1, I2, I3, D, const TIMER_HZ: u32> DS1302<I1, I2, I3, D, TIMER_HZ>
where
    I1: PinId + ValidFunction<FunctionSio<SioInput>> + ValidFunction<FunctionSio<SioOutput>>,
    I2: PinId + ValidFunction<FunctionSio<SioInput>> + ValidFunction<FunctionSio<SioOutput>>,
    I3: PinId + ValidFunction<FunctionSio<SioInput>> + ValidFunction<FunctionSio<SioOutput>>,
    D: Delay<TIMER_HZ>,
{
    pub fn new(
        ce: Pin<I2, FunctionSioOutput, PullDown>,
        io: Pin<I1, FunctionSioOutput, PullDown>,
        sclk: Pin<I3, FunctionSioOutput, PullDown>,
        mode: Mode,
        delay: D,
    ) -> Result<Self, Ds1302Error> {
        let mut ds1302 = DS1302 {
            io: Some(io),
            ce,
            sclk,
            delay,
        };
        // Check CLOCK HALT FLAG bit
        let byte = ds1302.read_reg(RegisterRead::SECONDS.addr())?;
        // Reset CLOCK HALT FLAG bit, power on device
        if (byte & CmdFlag::CLOCK_HALT.addr()) != 0 {
            ds1302.write_reg(RegisterWrite::SECONDS.addr(), 0)?;
            let byte = ds1302.read_reg(RegisterRead::SECONDS.addr())?;
            if (byte & CmdFlag::CLOCK_HALT.addr()) != 0 {
                Err(Ds1302Error::Unknown)
            } else {
                ds1302.set_clock_mode(mode)?;
                Ok(ds1302)
            }
        } else {
            ds1302.set_clock_mode(mode)?;
            Ok(ds1302)
        }
    }

    pub fn set_running(&mut self, is_running: bool) -> Result<(), Ds1302Error> {
        let mut seconds = self.read_reg(RegisterRead::SECONDS.addr())?;
        if is_running {
            seconds &= 0b0111_1111;
        } else {
            seconds |= 0b1000_0000;
        }
        self.write_reg(RegisterWrite::SECONDS.addr(), seconds)?;
        Ok(())
    }

    fn read_reg(&mut self, reg: u8) -> Result<u8, Ds1302Error> {
        self.sclk.set_low().map_err(|_| Ds1302Error::ClockError)?;
        self.ce
            .set_high()
            .map_err(|_| Ds1302Error::ChipSelectError)?;
        self.delay.start(4.micros()).ok(); // tCC = 4us for 2V
        self.write_byte(reg)?;
        let data = self.read_byte()?;
        self.delay.start(300.nanos()).ok(); // tCCH = 240ns for 2V
        self.ce
            .set_low()
            .map_err(|_| Ds1302Error::ChipSelectError)?;
        self.delay.start(4.micros()).ok(); // tCC = 4us for 2V
                                           // println!("read:  addr: {:02X}({:08b} <=> {:08b}), data: {:02X}({:08b})", addr, addr, command_byte, data, data);
        Ok(data)
    }

    fn read_byte(&mut self) -> Result<u8, Ds1302Error> {
        let mut data = 0;
        self.sclk.set_low().map_err(|_| Ds1302Error::ClockError)?;
        let mut pin = self.io.take().unwrap().into_pull_up_input();
        for i in 0..8 {
            let bit = self.read_bit(&mut pin)?;
            data |= match bit {
                true => 1,
                false => 0,
            } << i;
        }

        let io: Pin<_, FunctionSioOutput, PullDown> = pin.reconfigure(); //RecConfigure the PinState
        self.io = Some(io);
        Ok(data)
    }

    fn read_bit(&mut self, pin: &mut impl InputPin) -> Result<bool, Ds1302Error> {
        self.delay.start(300.nanos()).ok(); // tCCZ = 280ns for 2V
        self.sclk.set_high().map_err(|_| Ds1302Error::ClockError)?;
        let bit = pin.is_high().map_err(|_| Ds1302Error::ReadError)?;
        self.delay.start(2000.nanos()).ok(); // tCH = 1000ns for 2V
        self.sclk.set_low().map_err(|_| Ds1302Error::ClockError)?;
        self.delay.start(1700.nanos()).ok(); // tCL = 1000ns for 2V
        Ok(bit)
    }

    fn write_byte(&mut self, byte: u8) -> Result<(), Ds1302Error> {
        self.sclk.set_low().map_err(|_| Ds1302Error::ClockError)?;
        let mut pin = self.io.take().unwrap();
        for i in 0..8 {
            self.write_bit(&mut pin, ((byte >> i) & 1) == 1)?;
        }
        self.io = Some(pin);
        Ok(())
    }

    fn write_bit(&mut self, pin: &mut impl OutputPin, bit: bool) -> Result<(), Ds1302Error> {
        let _ = pin.set_state(bit.into());
        self.delay.start(350.nanos()).ok(); // tDC = 200ns for 2V
        self.sclk.set_high().map_err(|_| Ds1302Error::ClockError)?;
        self.delay.start(2000.nanos()).ok(); // tCH = 1000ns for 2V
        self.sclk.set_low().map_err(|_| Ds1302Error::ClockError)?;
        self.delay.start(1700.nanos()).ok(); // tCL = 1000ns for 2V
        Ok(())
    }

    fn write_reg(&mut self, reg: u8, byte: u8) -> Result<(), Ds1302Error> {
        //Firstly Check WRITE_PROTECT_BIT
        let wp_read = self.read_reg(RegisterRead::WP.addr())?;
        if (wp_read & CmdFlag::WRITE_PROTECT.addr()) != 0 {
            //let bytes = [RegisterWrite::WP.addr(), 0];
            self.sclk.set_low().map_err(|_| Ds1302Error::ClockError)?;
            self.ce
                .set_high()
                .map_err(|_| Ds1302Error::ChipSelectError)?;
            self.delay.start(4.micros()).ok(); // tCC = 4us for 2V
            self.write_byte(RegisterWrite::WP.addr())?;
            self.write_byte(0x0)?; // Disable Write_Protect
            self.delay.start(300.nanos()).ok(); //tCCH = 240ns for 2V
            self.ce.set_low().ok();
            self.delay.start(4.micros()).ok();
        }
        //Then write current data to registers
        //nb::block!(self.delay.wait()).ok(); // wait CE inactive time min 4us
        self.sclk.set_low().map_err(|_| Ds1302Error::ClockError)?;
        self.ce
            .set_high()
            .map_err(|_| Ds1302Error::ChipSelectError)?;
        self.delay.start(4.micros()).ok(); // tCC = 4us for 2V
        self.write_byte(reg)?;
        self.write_byte(byte)?;
        self.delay.start(300.nanos()).ok(); //tCCH = 240ns for 2V
        self.ce.set_low().ok();
        self.delay.start(4.micros()).ok();
        Ok(())
    }

    /// write_protect(true: bool) -> Enable Write_Protect
    /// write_protect(false: bool) -> Disable Write_Protect
    pub fn write_protect(&mut self, enable: bool) -> Result<(), Ds1302Error> {
        //Firstly Check WRITE_PROTECT_BIT
        let wp_read = self.read_reg(RegisterRead::WP.addr())?;
        // wp_state == 0 -> disable_write_protect
        // wp_state != 0 (== 0x80) -> enable_write_protect
        let wp_state = wp_read & CmdFlag::WRITE_PROTECT.addr();

        if enable {
            if wp_state == 0 {
                self.sclk.set_low().map_err(|_| Ds1302Error::ClockError)?;
                self.ce
                    .set_high()
                    .map_err(|_| Ds1302Error::ChipSelectError)?;
                self.delay.start(4.micros()).ok(); // tCC = 4us for 2V
                self.write_byte(RegisterWrite::WP.addr())?;
                self.write_byte(CmdFlag::WRITE_PROTECT.addr())?; // Enable Write_Protect
                self.delay.start(300.nanos()).ok(); //tCCH = 240ns for 2V
                self.ce.set_low().ok();
                self.delay.start(4.micros()).ok();
            }
        } else {
            if wp_state != 0 {
                self.sclk.set_low().map_err(|_| Ds1302Error::ClockError)?;
                self.ce
                    .set_high()
                    .map_err(|_| Ds1302Error::ChipSelectError)?;
                self.delay.start(4.micros()).ok(); // tCC = 4us for 2V
                self.write_byte(RegisterWrite::WP.addr())?;
                self.write_byte(CmdFlag::DISABLE_WRITE_PROTECT.addr())?; // Disable Write_Protect
                self.delay.start(300.nanos()).ok(); //tCCH = 240ns for 2V
                self.ce.set_low().ok();
                self.delay.start(4.micros()).ok();
            }
        }

        Ok(())
    }

    ///Return current information about seconds
    pub fn get_seconds(&mut self) -> Result<u8, Ds1302Error> {
        self.read_reg(RegisterRead::SECONDS.addr())
            .map(|b| bcd_to_decimal(b))
    }
    ///Return current information about minutes
    pub fn get_minutes(&mut self) -> Result<u8, Ds1302Error> {
        self.read_reg(RegisterRead::MINUTES.addr())
            .map(|b| bcd_to_decimal(b))
    }
    ///Return current information about hours
    pub fn get_hours(&mut self) -> Result<Hours, Ds1302Error> {
        self.read_reg(RegisterRead::HOURS.addr()).map(|b| b.into())
    }
    ///Return current information about date
    pub fn get_date(&mut self) -> Result<u8, Ds1302Error> {
        self.read_reg(RegisterRead::DATE.addr())
            .map(|b| bcd_to_decimal(b))
    }
    ///Return current information about month
    pub fn get_month(&mut self) -> Result<u8, Ds1302Error> {
        self.read_reg(RegisterRead::MONTH.addr())
            .map(|b| bcd_to_decimal(b))
    }
    ///Return current information about year
    pub fn get_year(&mut self) -> Result<u16, Ds1302Error> {
        self.read_reg(RegisterRead::YEAR.addr())
            .map(|b| 2000_u16 + (bcd_to_decimal(b) as u16))
    }
    ///Return current information about day of the week
    pub fn get_day(&mut self) -> Result<u8, Ds1302Error> {
        self.read_reg(RegisterRead::DAY.addr())
            .map(|b| bcd_to_decimal(b))
    }

    ///Return current information about hours, minutes and seconds
    pub fn get_clock(&mut self) -> Result<Clock, Ds1302Error> {
        let mut status_response: [u8; 3] = [0; 3];
        self.sclk.set_low().ok();
        //nb::block!(self.delay.wait()).ok(); // wait CE inactive time min 4us
        self.ce.set_high().ok();
        self.delay.start(4.micros()).ok();
        self.write_byte(RegisterRead::CLKBURS.addr())?;

        for idx in 0..3 {
            status_response[idx] = self.read_byte()?;
        }

        self.sclk.set_high().ok();
        self.ce.set_low().ok();
        self.delay.start(4.micros()).ok();

        let clock = Clock {
            seconds: bcd_to_decimal(status_response[0]),
            minutes: bcd_to_decimal(status_response[1]),
            hours: status_response[2].into(),
        };

        Ok(clock)
    }

    ///Return current information about date, day of the week, month and year
    pub fn get_calendar(&mut self) -> Result<Calendar, Ds1302Error> {
        let mut status_response: [u8; 7] = [0; 7];
        self.sclk.set_low().ok();
        self.ce.set_high().ok();
        self.delay.start(4.micros()).ok(); // wait CE inactive time min 4us
        self.write_byte(RegisterRead::CLKBURS.addr())?;

        for idx in 0..7 {
            status_response[idx] = self.read_byte()?;
        }

        self.delay.start(300.nanos()).ok(); //tCCH = 240ns for 2V
        self.ce.set_low().ok();
        self.delay.start(4.micros()).ok();

        let calendar = Calendar {
            date: bcd_to_decimal(status_response[3]),
            month: bcd_to_decimal(status_response[4]),
            day: bcd_to_decimal(status_response[5]),
            year: (2000_u16 + (bcd_to_decimal(status_response[6]) as u16)),
        };

        Ok(calendar)
    }

    ///Return current information date and time
    pub fn get_clock_calendar(&mut self) -> Result<(Clock, Calendar), Ds1302Error> {
        let mut status_response: [u8; 7] = [0; 7];
        self.sclk.set_low().ok();
        self.ce.set_high().ok();
        self.delay.start(4.micros()).ok(); // wait CE inactive time min 4us

        self.write_byte(RegisterRead::CLKBURS.addr())?;

        for idx in 0..7 {
            status_response[idx] = self.read_byte()?;
        }

        self.delay.start(300.nanos()).ok(); //tCCH = 240ns for 2V
        self.ce.set_low().ok();
        self.delay.start(4.micros()).ok(); // tCWH = 4us for 2V

        let clock = Clock {
            seconds: bcd_to_decimal(status_response[0]),
            minutes: bcd_to_decimal(status_response[1]),
            hours: status_response[2].into(),
        };

        let calendar = Calendar {
            date: bcd_to_decimal(status_response[3]),
            month: bcd_to_decimal(status_response[4]),
            day: bcd_to_decimal(status_response[5]),
            year: (2000_u16 + (bcd_to_decimal(status_response[6]) as u16)),
        };

        Ok((clock, calendar))
    }

    ///Set seconds to defined value
    pub fn set_seconds(&mut self, seconds: u8) -> Result<(), Ds1302Error> {
        self.write_reg(RegisterWrite::SECONDS.addr(), decimal_to_bcd(seconds))
    }
    ///Set minutes to defined value
    pub fn set_minutes(&mut self, minutes: u8) -> Result<(), Ds1302Error> {
        self.write_reg(RegisterWrite::MINUTES.addr(), decimal_to_bcd(minutes))
    }
    ///Set hours to defined value
    pub fn set_hours(&mut self, hours: Hours) -> Result<(), Ds1302Error> {
        self.write_reg(RegisterWrite::HOURS.addr(), hours.into())
    }
    ///Set date to defined value
    pub fn set_date(&mut self, date: u8) -> Result<(), Ds1302Error> {
        self.write_reg(RegisterWrite::DATE.addr(), decimal_to_bcd(date))
    }
    ///Set month to defined value
    pub fn set_month(&mut self, month: u8) -> Result<(), Ds1302Error> {
        self.write_reg(RegisterWrite::MONTH.addr(), decimal_to_bcd(month))
    }
    ///Set day of the week to defined value
    pub fn set_day(&mut self, day: u8) -> Result<(), Ds1302Error> {
        self.write_reg(RegisterWrite::DAY.addr(), decimal_to_bcd(day))
    }
    ///Set year to defined value
    pub fn set_year(&mut self, year: u16) -> Result<(), Ds1302Error> {
        let y = if year < 2000 { 0 } else { year - 2000 };
        self.write_reg(RegisterWrite::YEAR.addr(), decimal_to_bcd(y as u8))
    }
    ///Set clock to defined values
    pub fn set_clock(&mut self, clock: Clock) -> Result<(), Ds1302Error> {
        //Not burst mode, because it changes the calendar registers
        self.set_hours(clock.hours)?;
        self.set_minutes(clock.minutes)?;
        self.set_seconds(clock.seconds)
    }

    ///Set calendar to defined values
    pub fn set_calendar(&mut self, calendar: Calendar) -> Result<(), Ds1302Error> {
        //Not burst mode, because it changes the clock registers
        self.set_year(calendar.year)?;
        self.set_month(calendar.month)?;
        self.set_date(calendar.date)?;
        self.set_day(calendar.day)
    }
    ///Set clock and calendar to defined values
    pub fn set_clock_calendar(
        &mut self,
        clock: Clock,
        calendar: Calendar,
    ) -> Result<(), Ds1302Error> {
        //Writing in burst mode, it changes all the clock and calendar registers
        let mut bytes = [0_u8; 9];
        bytes[0] = RegisterWrite::CLKBURS.addr();
        bytes[1] = decimal_to_bcd(clock.seconds);
        bytes[2] = decimal_to_bcd(clock.minutes);
        bytes[3] = clock.hours.into();
        bytes[4] = decimal_to_bcd(calendar.date);
        bytes[5] = decimal_to_bcd(calendar.month);
        bytes[6] = decimal_to_bcd(calendar.day);
        let y = if calendar.year < 2000 {
            0
        } else {
            calendar.year - 2000
        };
        bytes[7] = decimal_to_bcd(y as u8);

        self.sclk.set_low().ok();
        //nb::block!(self.delay.wait()).ok(); // wait CE inactive time min 4us
        self.ce.set_high().ok();
        self.delay.start(4.micros()).ok();
        self.write_byte(RegisterWrite::CLKBURS.addr())?;

        for idx in 1..=7 {
            self.write_byte(bytes[idx])?;
        }

        // self.sclk.set_high().ok();
        self.delay.start(300.nanos()).ok(); //tCCH = 240ns for 2V
        self.ce.set_low().ok();
        self.delay.start(4.micros()).ok();
        Ok(())
    }

    ///Switch between 12-hour (AM/PM) and 24-hour mode
    pub fn set_clock_mode(&mut self, mode: Mode) -> Result<(), Ds1302Error> {
        let hr = self.get_hours()?; // save current hours data
        match hr {
            Hours::Hour24(_h) => {
                if mode == Mode::Hour12 {
                    self.set_hours(hr.convert())
                } else {
                    Ok(())
                }
            }
            _ => {
                if mode == Mode::Hour24 {
                    self.set_hours(hr.convert())
                } else {
                    Ok(())
                }
            }
        }
    }

    /// Enable trickle-charge.
    /// Ds (diode drop voltage 0.7 or 1.4)
    /// Rs (2k or 4k or 8k)
    /// The maximum current = (Vcc - Ds) / Rs.
    pub fn tc_enable(&mut self, ds: Ds, rs: Rs) -> Result<(), Ds1302Error> {
        self.write_reg(RegisterWrite::TCS.addr(), TrickleCharger::enable(ds, rs))
    }

    /// Disable trickle-charge.
    pub fn tc_disable(&mut self) -> Result<(), Ds1302Error> {
        self.write_reg(RegisterWrite::TCS.addr(), TrickleCharger::disable())
    }

    /// Get the configuration of the trickle-charge register.
    pub fn tc_get(&mut self) -> Result<(bool, Option<Ds>, Option<Rs>), Ds1302Error> {
        let v = self.read_reg(RegisterRead::TCS.addr())?;
        Ok(TrickleCharger::from(v).get())
    }

    /// Whether to enable charging.
    pub fn tc_is_enabled(&mut self) -> Result<bool, Ds1302Error> {
        let v = self.read_reg(RegisterRead::TCS.addr())?;
        Ok(TrickleCharger::from(v).is_enabled())
    }

    /// Read DS1302 internal RAM. The static RAM is 31 x 8 bytes, index 0..=30.
    pub fn read_ram(&mut self, index: u8) -> Result<u8, Ds1302Error> {
        if index > 30 {
            return Err(Ds1302Error::Parameter);
        }
        self.read_reg(RegisterRead::RAM.addr() + index * 2)
    }

    /// Write DS1302 internal RAM. The static RAM is 31 x 8 bytes, index 0..=31.
    pub fn write_ram(&mut self, index: u8, value: u8) -> Result<(), Ds1302Error> {
        if index > 30 {
            return Err(Ds1302Error::Parameter);
        }
        self.write_reg(RegisterWrite::RAM.addr() + index * 2, value)
    }

    /// Read DS1302 internal RAM burst mode. Start at 0 index.
    /// The length is determined by the buf, but cannot exceed 31.
    pub fn read_ram_burst(&mut self, buf: &mut [u8]) -> Result<(), Ds1302Error> {
        let mut status_response = [0_u8; 32];
        self.sclk.set_low().ok();
        //nb::block!(self.delay.wait()).ok(); // wait CE inactive time min 4us
        self.ce.set_high().ok();
        self.delay.start(4.micros()).ok();
        self.write_byte(RegisterRead::RAMBURS.addr())?;

        for idx in 0..buf.len() + 1 {
            status_response[idx] = self.read_byte()?;
        }

        //self.sclk.set_high().ok();
        self.delay.start(300.nanos()).ok(); //tCCH = 240ns for 2V
        self.ce.set_low().ok();
        self.delay.start(4.micros()).ok();

        buf.copy_from_slice(&status_response[1..(buf.len() + 1)]);
        Ok(())
    }

    /// Write DS1302 internal RAM burst mode. Start at 0 index.
    /// The length is determined by the buf, but cannot exceed 31.
    pub fn write_ram_burst(&mut self, buf: &[u8]) -> Result<usize, Ds1302Error> {
        let mut bytes = [0_u8; 32];
        bytes[0] = RegisterWrite::RAMBURS.addr();
        let ll = buf.len();
        let ll = if ll > 31 { 31 } else { ll };

        self.sclk.set_low().ok();
        //nb::block!(self.delay.wait()).ok(); // wait CE inactive time min 4us
        self.ce.set_high().ok();
        self.delay.start(4.micros()).ok();
        self.write_byte(RegisterWrite::RAMBURS.addr())?;

        for idx in 0..ll + 1 {
            self.write_byte(buf[idx])?;
        }

        //self.sclk.set_high().ok();
        self.delay.start(300.nanos()).ok(); //tCCH = 240ns for 2V
        self.ce.set_low().ok();
        self.delay.start(4.micros()).ok();

        Ok(ll)
    }
}
