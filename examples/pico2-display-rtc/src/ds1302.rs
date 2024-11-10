// for RTC
use ds1302_drv::Delay as ds1302_delay;
use hal::timer::TimerDevice;
use hal::Timer;
use rp235x_hal as hal;

pub struct MyClock<TIM: TimerDevice, const TIMER_HZ: u32> {
    _timer: Timer<TIM>,
}

impl<TIM: TimerDevice, const TIMER_HZ: u32> MyClock<TIM, TIMER_HZ> {
    pub fn new(timer: Timer<TIM>) -> Self {
        Self { _timer: timer }
    }
}

impl<TIM: TimerDevice, const TIMER_HZ: u32> ds1302_delay<TIMER_HZ> for MyClock<TIM, TIMER_HZ> {
    type Error = core::convert::Infallible;

    fn now(&mut self) -> fugit::TimerInstantU32<TIMER_HZ> {
        fugit::TimerInstantU32::from_ticks(0)
    }

    fn start(&mut self, _duration: fugit::TimerDurationU32<TIMER_HZ>) -> Result<(), Self::Error> {
        Ok(())
    }

    fn wait(&mut self) -> nb::Result<(), Self::Error> {
        Ok(())
    }
}
