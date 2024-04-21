//! Blinks the LED on a Pico board
//!
#![no_std]
#![no_main]
extern crate alloc;

use core::fmt::Write;

use alloc_cortex_m::CortexMHeap;
use bsp::{
    entry,
    hal::{fugit::HertzU32, gpio::FunctionI2C},
};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
const HEAP_SIZE: usize = 1024 * 256;

use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;
use bitcoin::{blockdata::constants::genesis_block, hashes::Hash};
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use ssd1306::{size::DisplaySize128x64, I2CDisplayInterface, Ssd1306};

#[entry]
fn main() -> ! {
    info!("allocating");
    unsafe { ALLOCATOR.init(cortex_m_rt::heap_start() as usize, HEAP_SIZE) }
    info!("start");
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

    let scl_pin = pins.gpio3.into_function::<FunctionI2C>();
    let sda_pin = pins.gpio2.into_function::<FunctionI2C>();

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!

    let i2c = bsp::hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        HertzU32::kHz(400),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(
        interface,
        DisplaySize128x64,
        ssd1306::rotation::DisplayRotation::Rotate0,
    )
    .into_terminal_mode();

    display.clear().unwrap();
    // Flash a pin to signify the start of the program.

    let mut led_pin = pins.led.into_push_pull_output();
    delay.delay_ms(1_000);
    led_pin.set_high().unwrap();
    delay.delay_ms(500);
    led_pin.set_low().unwrap();
    delay.delay_ms(1_000);

    let genesis_block = genesis_block(bitcoin::Network::Signet);
    let mut header = genesis_block.header;
    let mut nonce = 0;

    loop {
        header.nonce = nonce;
        let target = header.target();
        let h = header.block_hash().as_raw_hash().to_byte_array();
        display
            .write_str(unsafe { core::str::from_utf8_unchecked(&h) })
            .unwrap();
        let pow = header.validate_pow(target);
        match pow {
            Ok(_block) => {
                info!("found a valid block!");
                led_pin.set_high().unwrap();
                delay.delay_ms(1_000_000);
                led_pin.set_low().unwrap();
            }
            Err(_) => {
                led_pin.set_high().unwrap();
                delay.delay_ms(100);
                led_pin.set_low().unwrap();
            }
        }
        nonce = nonce.wrapping_add(1);
        display.clear().unwrap();
    }
}
