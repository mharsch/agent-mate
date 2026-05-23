#![no_std]
#![no_main]

use rp235x_hal as hal;
use hal::block::ImageDef;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};

//Panic Handler
use {panic_probe as _};
// Defmt Logging
use defmt_rtt as _;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();
/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[hal::entry]
fn main() -> ! {    
    // Grab our singleton objects
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
    let mut led_pin = pins.gpio15.into_push_pull_output();
    let mut button = pins.gpio26.into_pull_up_input();

    let mut blinking = true;
    let mut led_state = false;
    let mut last_blink_us: u64 = timer.get_counter().ticks();
    let mut button_prev_pressed = false;
    const BLINK_HALF_PERIOD_US: u64 = 300_000;

    loop {
        let now: u64 = timer.get_counter().ticks();

        let button_pressed = button.is_low().unwrap_or(false);
        if button_prev_pressed && !button_pressed {
            blinking = !blinking;
            if !blinking {
                led_pin.set_low().unwrap();
                led_state = false;
            }
        }
        button_prev_pressed = button_pressed;

        if blinking && (now - last_blink_us >= BLINK_HALF_PERIOD_US) {
            led_state = !led_state;
            if led_state { led_pin.set_high().unwrap(); }
            else         { led_pin.set_low().unwrap();  }
            last_blink_us = now;
        }

        timer.delay_ms(10);
    }
}

// Program metadata for `picotool info`.
// This isn't needed, but it's recommended to have these minimal entries.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"your program description"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
