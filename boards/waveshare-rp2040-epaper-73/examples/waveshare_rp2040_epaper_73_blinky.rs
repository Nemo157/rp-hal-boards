//! Simple activity LED blinking example for the Waveshare e-Paper 7.3" board
#![no_std]
#![no_main]

use embedded_hal::{delay::DelayNs, digital::OutputPin};
use panic_halt as _;

use waveshare_rp2040_epaper_73::{
    entry,
    hal::{clocks::init_clocks_and_plls, pac, timer::Timer, watchdog::Watchdog, Sio},
    LedActivity, Pins, XOSC_CRYSTAL_FREQ,
};

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led: LedActivity = pins.led_activity.reconfigure();

    loop {
        led.set_high().unwrap();
        timer.delay_ms(1000);
        led.set_low().unwrap();
        timer.delay_ms(1000);
    }
}
