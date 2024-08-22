//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::{digital::OutputPin, i2c::I2c};
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico::{
    self as bsp,
    hal::{fugit::RateExtU32, I2C},
};
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{FunctionI2C, Pin},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

// Helper function to convert a single byte to its hexadecimal representation.
fn byte_to_hex(byte: u8) -> (u8, u8) {
    const HEX_DIGITS: &[u8; 16] = b"0123456789ABCDEF";

    let high = HEX_DIGITS[(byte >> 4) as usize];
    let low = HEX_DIGITS[(byte & 0x0F) as usize];

    (high, low)
}

fn format_array_to_string(arr: [u8; 6]) -> Result<&'static str, ()> {
    static mut BUFFER: [u8; 12] = [0; 12]; // 6 bytes * 2 chars per byte
    let buffer = unsafe { &mut BUFFER };

    for (i, &byte) in arr.iter().enumerate() {
        let (high, low) = byte_to_hex(byte);
        buffer[i * 2] = high;
        buffer[i * 2 + 1] = low;
    }

    // Convert the buffer to a string slice (safe because it only contains valid ASCII)
    let result_str = unsafe { core::str::from_utf8_unchecked(&buffer[..]) };

    Ok(result_str)
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure two pins as being I²C, not GPIO
    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio18.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio19.reconfigure();
    // let not_an_scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio20.reconfigure();

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let mut i2c = I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // if i2c.write(0x52u8, &[0x40, 0]).is_err() {
    //     error!("cannot send handshake start");
    // }
    // if i2c.write(0x52u8, &[0]).is_err() {
    //     error!("cannot send handshake finish");
    // }

    if i2c.write(0x52u8, &[0xf0, 0x55]).is_err() {
        error!("cannot send handshake start");
    }
    if i2c.write(0x52u8, &[0xfb, 0]).is_err() {
        error!("cannot send handshake finish");
    }

    loop {
        let mut chuck: [u8; 6] = [0; 6];
        if i2c.write(0x52u8, &[0]).is_err() {
            error!("cannot write the address");
        }
        if let Err(_e) = i2c.read(0x52u8, &mut chuck) {
            error!("cannot get the readings");
        } else {
            let chuck_str = format_array_to_string(chuck);
            info!("{}", chuck_str);
            let z_button = chuck[5] & 1 != 0;
            let c_button = chuck[5] & 2 != 0;
            let button_state = |b: bool| {
                if b {
                    "OFF"
                } else {
                    "ON"
                }
            };
            info!(
                "Z: {}  C: {}",
                button_state(z_button),
                button_state(c_button)
            );
            let accel_x = ((chuck[5] >> 2) & 0b11) | (chuck[2] << 2);
            let accel_y = ((chuck[5] >> 4) & 0b11) | (chuck[3] << 2);
            let accel_z = ((chuck[5] >> 6) & 0b11) | (chuck[4] << 2);
            info!("X: {}, Y: {}, Z: {}", accel_x, accel_y, accel_z);
        }
        delay.delay_ms(10);
    }

    // let mut led_pin = pins.led.into_push_pull_output();

    // loop {
    //     info!("on!");
    //     led_pin.set_high().unwrap();
    //     delay.delay_ms(500);
    //     info!("off!");
    //     led_pin.set_low().unwrap();
    //     delay.delay_ms(500);
    // }
}

// End of file
