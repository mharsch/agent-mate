#![no_std]
#![no_main]

use rp235x_hal as hal;
use hal::block::ImageDef;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};

use rp235x_hal::clocks::ClockSource;
use rp235x_hal::fugit::RateExtU32;
use rotary_encoder_hal::{Direction, Rotary};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};

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

#[derive(Clone, Copy, PartialEq)]
enum Mode { Led, Mic, Gyro, Magneto, Temp, Optical, Pir, Buzz, Rgb }

impl Mode {
    fn label(self) -> &'static str {
        match self {
            Mode::Led     => "LED",
            Mode::Mic     => "Mic",
            Mode::Gyro    => "Gyro",
            Mode::Magneto => "Magneto",
            Mode::Temp    => "Temp",
            Mode::Optical => "Optical",
            Mode::Pir     => "PIR",
            Mode::Buzz    => "Buzz",
            Mode::Rgb     => "RGB",
        }
    }
}

const MODES: &[Mode] = &[
    Mode::Led, Mode::Mic, Mode::Gyro, Mode::Magneto, Mode::Temp,
    Mode::Optical, Mode::Pir, Mode::Buzz, Mode::Rgb,
];

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

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

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // LED (GP15, external breadboard) + button (GP26)
    let mut led_pin = pins.gpio15.into_push_pull_output();
    let mut button = pins.gpio26.into_pull_up_input();

    // Rotary encoder (GP6 A, GP7 B)
    let enc_a = pins.gpio6.into_pull_up_input();
    let enc_b = pins.gpio7.into_pull_up_input();
    let mut encoder = Rotary::new(enc_a, enc_b);

    // I2C0 for OLED (GP16 SDA, GP17 SCL)
    let sda_pin = pins.gpio16.into_function();
    let scl_pin = pins.gpio17.into_function();
    let i2c = hal::I2C::new_controller(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400_000u32.Hz(),
        &mut pac.RESETS,
        clocks.system_clock.get_freq(),
    );

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    // LED mode state
    let mut blinking = true;
    let mut led_state = false;
    let mut last_blink_us: u64 = timer.get_counter().ticks();
    let mut button_prev_pressed = false;
    const BLINK_HALF_PERIOD_US: u64 = 300_000;

    // Mode selection state
    let mut mode_idx: usize = 0;
    let mut display_dirty = true;

    loop {
        let now: u64 = timer.get_counter().ticks();

        // Rotary encoder → cycle mode
        match encoder.update() {
            Ok(Direction::Clockwise) => {
                mode_idx = (mode_idx + 1) % MODES.len();
                button_prev_pressed = false;
                display_dirty = true;
            }
            Ok(Direction::CounterClockwise) => {
                mode_idx = (mode_idx + MODES.len() - 1) % MODES.len();
                button_prev_pressed = false;
                display_dirty = true;
            }
            _ => {}
        }

        // LED mode: button toggle + blink logic
        if MODES[mode_idx] == Mode::Led {
            let button_pressed = button.is_low().unwrap_or(false);
            if button_prev_pressed && !button_pressed {
                blinking = !blinking;
                if !blinking {
                    led_pin.set_low().unwrap();
                    led_state = false;
                }
                display_dirty = true;
            }
            button_prev_pressed = button_pressed;

            if blinking && (now - last_blink_us >= BLINK_HALF_PERIOD_US) {
                led_state = !led_state;
                if led_state { led_pin.set_high().unwrap(); }
                else         { led_pin.set_low().unwrap();  }
                last_blink_us = now;
            }
        }

        // Redraw display only when content changed
        if display_dirty {
            let mode = MODES[mode_idx];
            let label_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
            let status_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

            display.clear(BinaryColor::Off).unwrap();
            Text::new(mode.label(), Point::new(0, 20), label_style)
                .draw(&mut display)
                .unwrap();

            let status = match mode {
                Mode::Led => if blinking { "Blinking" } else { "Override" },
                _         => "",
            };
            Text::new(status, Point::new(0, 50), status_style)
                .draw(&mut display)
                .unwrap();

            display.flush().unwrap();
            display_dirty = false;
        }

        timer.delay_ms(10);
    }
}

// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"agent-mate peripheral demo"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];
