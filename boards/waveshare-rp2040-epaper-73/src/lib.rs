#![no_std]

pub use rp2040_hal as hal;

#[cfg(feature = "rt")]
extern crate cortex_m_rt;
#[cfg(feature = "rt")]
pub use hal::entry;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[cfg(feature = "boot2")]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

pub use hal::pac;

hal::bsp_pins!(
    /// GPIO 2 is connected to the SD-Card SPI Clock signal
    Gpio2 {
        name: sd_spi_clock,
        aliases: {
            FunctionSpi, PullNone: SdSpiClock
        }
    },

    /// GPIO 3 is connected to the SD-Card SPI TX signal
    Gpio3 {
        name: sd_spi_tx,
        aliases: {
            FunctionSpi, PullNone: SdSpiTx
        }
    },

    /// GPIO 4 is connected to the SD-Card SPI RX signal
    Gpio4 {
        name: sd_spi_rx,
        aliases: {
            FunctionSpi, PullNone: SdSpiRx
        }
    },

    /// GPIO 5 is connected to the SD-Card SPI CS signal
    Gpio5 {
        name: sd_spi_cs,
        aliases: {
            FunctionSioOutput, PullUp: SdSpiCs
        }
    },

    /// GPIO 6 is connected to the RTC Interrupt signal
    Gpio6 {
        name: rtc_interrupt,
        aliases: {
            FunctionSioInput, PullUp: RtcInterrupt
        }
    },

    /// GPIO 8 is connected to the EPD DC signal
    Gpio8 {
        name: epd_dc,
        aliases: {
            FunctionSioOutput, PullNone: EpdDc
        }
    },

    /// GPIO 9 is connected to the EPD SPI CS signal
    Gpio9 {
        name: epd_spi_cs,
        aliases: {
            FunctionSioOutput, PullUp: EpdSpiCs
        }
    },

    /// GPIO 10 is connected to the EPD SPI Clock signal
    Gpio10 {
        name: epd_spi_clock,
        aliases: {
            FunctionSpi, PullNone: EpdSpiClock
        }
    },

    /// GPIO 11 is connected to the EPD SPI TX signal
    Gpio11 {
        name: epd_spi_tx,
        aliases: {
            FunctionSpi, PullNone: EpdSpiTx
        }
    },

    /// GPIO 12 is connected to the EPD Reset signal
    Gpio12 {
        name: epd_reset,
        aliases: {
            FunctionSioOutput, PullNone: EpdReset
        }
    },

    /// GPIO 13 is connected to the EPD Busy signal
    Gpio13 {
        name: epd_busy,
        aliases: {
            FunctionSioInput, PullNone: EpdBusy
        }
    },

    /// GPIO 14 is connected to the RTC I2C SDA signal
    Gpio14 {
        name: rtc_i2c_sda,
        aliases: {
            FunctionI2c, PullNone: RtcI2cSda
        }
    },

    /// GPIO 15 is connected to the RTC I2C SCL signal
    Gpio15 {
        name: rtc_i2c_scl,
        aliases: {
            FunctionI2c, PullNone: RtcI2cScl
        }
    },

    /// GPIO 16 is connected to the EPD Power Enable signal
    Gpio16 {
        name: epd_power_enable,
        aliases: {
            FunctionSioOutput, PullNone: EpdPowerEnable
        }
    },

    /// GPIO 17 is connected to the Charge State signal
    Gpio17 {
        name: charge_state,
        aliases: {
            FunctionSioInput, PullUp: ChargeState
        }
    },

    /// GPIO 18 is connected to the Battery Off signal
    Gpio18 {
        name: battery_off,
        aliases: {
            FunctionSioOutput, PullNone: BatteryOff
        }
    },

    /// GPIO 19 is connected to the NEXT button.
    Gpio19 {
        name: next_button,
        aliases: {
            FunctionSioInput, PullUp: NextButton
        }
    },

    /// GPIO 23 is connected to the Power Mode (and appears to be unconfigured??).
    Gpio23 {
        name: power_mode,
        aliases: {
            FunctionNull, PullNone: PowerMode
        }
    },

    /// GPIO 24 is connected to the VBUS signal.
    Gpio24 {
        name: vbus,
        aliases: {
            FunctionSioInput, PullNone: Vbus
        }
    },

    /// GPIO 25 is connected to the Activity LED.
    Gpio25 {
        name: led_activity,
        aliases: {
            FunctionSioOutput, PullNone: LedActivity
        }
    },

    /// GPIO 26 is connected to the Power LED.
    Gpio26 {
        name: led_power,
        aliases: {
            FunctionSioOutput, PullNone: LedPower
        }
    },

    /// GPIO 29 is connected to the Battery Voltage.
    Gpio29 {
        name: battery_voltage,
        aliases: {
            FunctionNull, PullNone: BatteryVoltage
        }
    },

);

pub const RTC_I2C_ADDRESS: u8 = 0x51;

// TODO: Verify
pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
