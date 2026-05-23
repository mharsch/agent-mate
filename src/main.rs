#![no_std]
#![no_main]

use rp235x_hal as hal;
use hal::block::ImageDef;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use heapless::String as HString;
use core::fmt::Write;

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

struct Lsm6ds3<I2C> { i2c: I2C }

impl<I2C: embedded_hal::i2c::I2c> Lsm6ds3<I2C> {
    const ADDR: u8 = 0x6A;

    fn new(i2c: I2C) -> Self { Self { i2c } }

    fn init(&mut self) -> Result<(), I2C::Error> {
        self.i2c.write(Self::ADDR, &[0x11, 0x60])?; // CTRL2_G: 416 Hz ODR, ±250 dps
        self.i2c.write(Self::ADDR, &[0x10, 0x60])   // CTRL1_XL: 416 Hz ODR, ±2 g
    }

    fn read_gyro(&mut self) -> Result<(i16, i16, i16), I2C::Error> {
        let mut buf = [0u8; 6];
        self.i2c.write_read(Self::ADDR, &[0x22], &mut buf)?;
        Ok((
            i16::from_le_bytes([buf[0], buf[1]]),
            i16::from_le_bytes([buf[2], buf[3]]),
            i16::from_le_bytes([buf[4], buf[5]]),
        ))
    }
}

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

    // LED (GP20, external breadboard) + button (GP26)
    let mut led_pin = pins.gpio20.into_push_pull_output();
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

    // I2C1 for IMU (GP14 SDA, GP15 SCL)
    let imu_sda = pins.gpio14.into_function();
    let imu_scl = pins.gpio15.into_function();
    let i2c1 = hal::I2C::new_controller(
        pac.I2C1,
        imu_sda,
        imu_scl,
        400_000u32.Hz(),
        &mut pac.RESETS,
        clocks.system_clock.get_freq(),
    );
    let mut imu = Lsm6ds3::new(i2c1);
    imu.init().unwrap();

    // LED mode state
    let mut blinking = true;
    let mut led_state = false;
    let mut last_blink_us: u64 = timer.get_counter().ticks();
    let mut button_prev_pressed = false;
    const BLINK_HALF_PERIOD_US: u64 = 300_000;

    // Mode selection state
    let mut mode_idx: usize = 0;
    let mut display_dirty = true;

    // Gyro mode state
    let mut gx: i16 = 0;
    let mut gy: i16 = 0;
    let mut gz: i16 = 0;
    let mut gyro_ticks: u32 = 0;

    loop {
        let now: u64 = timer.get_counter().ticks();

        // Rotary encoder → cycle mode
        match encoder.update() {
            Ok(Direction::Clockwise) => {
                mode_idx = (mode_idx + 1) % MODES.len();
                button_prev_pressed = false;
                gyro_ticks = 0;
                display_dirty = true;
            }
            Ok(Direction::CounterClockwise) => {
                mode_idx = (mode_idx + MODES.len() - 1) % MODES.len();
                button_prev_pressed = false;
                gyro_ticks = 0;
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

        // Gyro mode: read sensor every ~100 ms
        if MODES[mode_idx] == Mode::Gyro {
            gyro_ticks += 1;
            if gyro_ticks >= 10 {
                gyro_ticks = 0;
                if let Ok((x, y, z)) = imu.read_gyro() {
                    gx = x; gy = y; gz = z;
                    display_dirty = true;
                }
            }
        }

        // Redraw display only when content changed
        if display_dirty {
            let mode = MODES[mode_idx];
            let label_style  = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
            let status_style = MonoTextStyle::new(&FONT_6X10,  BinaryColor::On);

            display.clear(BinaryColor::Off).unwrap();
            Text::new(mode.label(), Point::new(0, 20), label_style)
                .draw(&mut display).unwrap();

            match mode {
                Mode::Gyro => {
                    let mut xb: HString<16> = HString::new();
                    let mut yb: HString<16> = HString::new();
                    let mut zb: HString<16> = HString::new();
                    write!(xb, "X:{:>+7}", gx).ok();
                    write!(yb, "Y:{:>+7}", gy).ok();
                    write!(zb, "Z:{:>+7}", gz).ok();
                    Text::new(&xb, Point::new(0, 35), status_style).draw(&mut display).unwrap();
                    Text::new(&yb, Point::new(0, 47), status_style).draw(&mut display).unwrap();
                    Text::new(&zb, Point::new(0, 59), status_style).draw(&mut display).unwrap();
                }
                _ => {
                    let status = match mode {
                        Mode::Led => if blinking { "Blinking" } else { "Override" },
                        _         => "",
                    };
                    Text::new(status, Point::new(0, 50), status_style)
                        .draw(&mut display).unwrap();
                }
            }

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
