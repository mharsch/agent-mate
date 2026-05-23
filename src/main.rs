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
use rp235x_hal::pio::PIOExt;
use embedded_hal::pwm::SetDutyCycle;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
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

/// HSV → RGB with S=255. Returns (r, g, b), each 0-v.
fn hue_to_rgb(h: u8, v: u8) -> (u8, u8, u8) {
    let region = h / 43;
    let frac = ((h % 43) as u32 * v as u32 / 43) as u8;
    match region {
        0 => (v,        frac,    0),
        1 => (v - frac, v,       0),
        2 => (0,        v,       frac),
        3 => (0,        v - frac,v),
        4 => (frac,     0,       v),
        _ => (v,        0,       v - frac),
    }
}

struct Lsm6ds3;

impl Lsm6ds3 {
    const ADDR: u8 = 0x6A;

    fn init<I: embedded_hal::i2c::I2c>(i2c: &mut I) -> Result<(), I::Error> {
        i2c.write(Self::ADDR, &[0x11, 0x60])?; // CTRL2_G: 416 Hz ODR, ±250 dps
        i2c.write(Self::ADDR, &[0x10, 0x60])   // CTRL1_XL: 416 Hz ODR, ±2 g
    }

    fn read_gyro<I: embedded_hal::i2c::I2c>(i2c: &mut I) -> Result<(i16, i16, i16), I::Error> {
        let mut buf = [0u8; 6];
        i2c.write_read(Self::ADDR, &[0x22], &mut buf)?;
        Ok((
            i16::from_le_bytes([buf[0], buf[1]]),
            i16::from_le_bytes([buf[2], buf[3]]),
            i16::from_le_bytes([buf[4], buf[5]]),
        ))
    }
}

struct Mmc5603;

impl Mmc5603 {
    const ADDR: u8 = 0x30;

    fn init<I: embedded_hal::i2c::I2c, D: embedded_hal::delay::DelayNs>(
        i2c: &mut I,
        delay: &mut D,
    ) -> Result<(), I::Error> {
        i2c.write(Self::ADDR, &[0x1C, 0x80])?; // CTRL1: SW_RST
        delay.delay_ms(20);
        Ok(())
    }

    // Returns (x_mGauss, y_mGauss, z_mGauss)
    fn read<I: embedded_hal::i2c::I2c, D: embedded_hal::delay::DelayNs>(
        i2c: &mut I,
        delay: &mut D,
    ) -> Result<(i32, i32, i32), I::Error> {
        i2c.write(Self::ADDR, &[0x1B, 0x01])?; // CTRL0: TM_M (trigger measurement)
        delay.delay_ms(10);
        let mut buf = [0u8; 9];
        i2c.write_read(Self::ADDR, &[0x00], &mut buf)?;
        let x_raw = ((buf[0] as u32) << 12) | ((buf[1] as u32) << 4) | ((buf[6] as u32) >> 4);
        let y_raw = ((buf[2] as u32) << 12) | ((buf[3] as u32) << 4) | ((buf[6] as u32) & 0x0F);
        let z_raw = ((buf[4] as u32) << 12) | ((buf[5] as u32) << 4) | ((buf[7] as u32) >> 4);
        Ok((
            ((x_raw as i32) - 524288) / 16, // 1/16 mG per LSB, offset from 2^19
            ((y_raw as i32) - 524288) / 16,
            ((z_raw as i32) - 524288) / 16,
        ))
    }
}

struct Ltr381rgb;

impl Ltr381rgb {
    const ADDR: u8 = 0x53;

    fn init<I: embedded_hal::i2c::I2c>(i2c: &mut I) -> Result<(), I::Error> {
        i2c.write(Self::ADDR, &[0x05, 0x04])?; // ALS_CS_GAIN: 18x gain
        i2c.write(Self::ADDR, &[0x04, 0x40])   // ALS_CS_MEAS_RATE: 16-bit res, 25 ms rate
    }

    // Returns (r, g, b) normalized to 0-255
    fn read<I: embedded_hal::i2c::I2c, D: embedded_hal::delay::DelayNs>(
        i2c: &mut I,
        delay: &mut D,
    ) -> Result<(u8, u8, u8), I::Error> {
        i2c.write(Self::ADDR, &[0x00, 0x06])?;  // MAIN_CTRL: enable RGB/CS mode
        // Poll MAIN_STATUS (0x07) for CS data ready (bit 3)
        let mut status = [0u8; 1];
        for _ in 0..20u8 {
            delay.delay_ms(5);
            i2c.write_read(Self::ADDR, &[0x07], &mut status)?;
            if status[0] & 0x08 != 0 { break; }
        }
        let mut buf = [0u8; 9];
        i2c.write_read(Self::ADDR, &[0x0D], &mut buf)?; // burst read G, R, B (3 bytes each)
        i2c.write(Self::ADDR, &[0x00, 0x00])?;          // MAIN_CTRL: standby
        let g_raw = (buf[0] as u32) | ((buf[1] as u32) << 8) | ((buf[2] as u32) << 16);
        let r_raw = (buf[3] as u32) | ((buf[4] as u32) << 8) | ((buf[5] as u32) << 16);
        let b_raw = (buf[6] as u32) | ((buf[7] as u32) << 8) | ((buf[8] as u32) << 16);
        Ok((
            (r_raw.min(65535) * 255 / 65535) as u8,
            (g_raw.min(65535) * 255 / 65535) as u8,
            (b_raw.min(65535) * 255 / 65535) as u8,
        ))
    }
}

struct Sht30;

impl Sht30 {
    const ADDR: u8 = 0x44;

    // Returns (temp_fahrenheit_x10, humidity_pct)
    fn measure<I: embedded_hal::i2c::I2c, D: embedded_hal::delay::DelayNs>(
        i2c: &mut I,
        delay: &mut D,
    ) -> Result<(i32, u32), I::Error> {
        i2c.write(Self::ADDR, &[0x24, 0x00])?; // single-shot, high repeatability, no clock stretch
        delay.delay_ms(20);
        let mut buf = [0u8; 6];
        i2c.read(Self::ADDR, &mut buf)?;
        let t_raw = u16::from_be_bytes([buf[0], buf[1]]);
        let h_raw = u16::from_be_bytes([buf[3], buf[4]]);
        let t_x10 = (3150u32 * t_raw as u32 / 65535) as i32 - 490; // °F × 10
        let h_pct = 100u32 * h_raw as u32 / 65535;
        Ok((t_x10, h_pct))
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

    // PIR sensor (GP28, AS312 — active-high output)
    let mut pir_pin = pins.gpio28.into_pull_down_input();

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
    let mut i2c1 = hal::I2C::new_controller(
        pac.I2C1,
        imu_sda,
        imu_scl,
        400_000u32.Hz(),
        &mut pac.RESETS,
        clocks.system_clock.get_freq(),
    );
    Lsm6ds3::init(&mut i2c1).unwrap();
    Mmc5603::init(&mut i2c1, &mut timer).unwrap();
    Ltr381rgb::init(&mut i2c1).unwrap();

    // Buzzer PWM (GP27 → PWM slice 1, channel B on RP2350)
    let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let mut buzz_pwm = pwm_slices.pwm5;
    buzz_pwm.set_div_int(125u8); // 125 MHz / 125 = 1 MHz PWM clock
    buzz_pwm.set_top(999u16);    // 1 MHz / 1000 = 1 kHz carrier
    let _buzz_pin = buzz_pwm.channel_b.output_to(pins.gpio27);

    // WS2812 on GP22 via PIO0 SM0
    // T1=3 T2=4 T3=3 (10 cycles/bit at 8 MHz → 800 kHz; 375ns/875ns timing, within WS2812B spec)
    let ws2812_prog = pio_proc::pio_asm!(
        ".side_set 1",
        ".wrap_target",
        "bitloop:",
        "out x, 1       side 0 [2]",   // T3 cycles LOW, shift 1 bit into x
        "jmp !x do_zero side 1 [2]",   // T1 cycles HIGH; jump if 0-bit
        "jmp bitloop    side 1 [3]",   // T2 cycles HIGH (1-bit, loop)
        "do_zero:",
        "nop            side 0 [3]",   // T2 cycles LOW (0-bit)
        ".wrap",
    );
    let _ws_pin = pins.gpio22.into_function::<hal::gpio::FunctionPio0>();
    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio0.install(&ws2812_prog.program).unwrap();
    let (mut sm0, _, mut ws_tx) = hal::pio::PIOBuilder::from_installed_program(installed)
        .side_set_pin_base(22)
        .out_shift_direction(hal::pio::ShiftDirection::Left)
        .autopull(true)
        .pull_threshold(24)
        .clock_divisor_fixed_point(15u16, 160u8)
        .build(sm0);
    sm0.set_pindirs([(22u8, hal::pio::PinDir::Output)]);
    sm0.start();

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

    // Temp mode state (temp_ticks starts at threshold so first read is immediate)
    let mut temp_x10: i32 = 0;
    let mut hum_pct: u32 = 0;
    let mut temp_ticks: u32 = 100;

    // Magneto mode state (magneto_ticks starts at threshold for immediate first read)
    let mut mx: i32 = 0;
    let mut my: i32 = 0;
    let mut mz: i32 = 0;
    let mut magneto_ticks: u32 = 20;

    // Optical mode state
    let mut opt_r: u8 = 0;
    let mut opt_g: u8 = 0;
    let mut opt_b: u8 = 0;
    let mut optical_ticks: u32 = 20;

    // PIR mode state
    let mut pir_detected = false;

    // RGB mode state
    let mut rgb_hue: u8 = 0;
    let mut rgb_ticks: u32 = 0;
    let mut rgb_active = false;

    // Buzz mode state
    let mut buzz_duty: u16 = 0;
    let mut buzz_display_ticks: u32 = 0;
    let mut buzz_active = false;

    loop {
        let now: u64 = timer.get_counter().ticks();

        // Rotary encoder → cycle mode
        match encoder.update() {
            Ok(Direction::Clockwise) => {
                mode_idx = (mode_idx + 1) % MODES.len();
                button_prev_pressed = false;
                gyro_ticks = 0;
                temp_ticks = 100;
                magneto_ticks = 20;
                optical_ticks = 20;
                rgb_hue = 0;
                rgb_ticks = 0;
                buzz_duty = 0;
                buzz_display_ticks = 0;
                display_dirty = true;
            }
            Ok(Direction::CounterClockwise) => {
                mode_idx = (mode_idx + MODES.len() - 1) % MODES.len();
                button_prev_pressed = false;
                gyro_ticks = 0;
                temp_ticks = 100;
                magneto_ticks = 20;
                optical_ticks = 20;
                rgb_hue = 0;
                rgb_ticks = 0;
                buzz_duty = 0;
                buzz_display_ticks = 0;
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
                if let Ok((x, y, z)) = Lsm6ds3::read_gyro(&mut i2c1) {
                    gx = x; gy = y; gz = z;
                    display_dirty = true;
                }
            }
        }

        // Magneto mode: read sensor every ~200 ms
        if MODES[mode_idx] == Mode::Magneto {
            magneto_ticks += 1;
            if magneto_ticks >= 20 {
                magneto_ticks = 0;
                if let Ok((x, y, z)) = Mmc5603::read(&mut i2c1, &mut timer) {
                    mx = x; my = y; mz = z;
                    display_dirty = true;
                }
            }
        }

        // Optical mode: read sensor every ~200 ms
        if MODES[mode_idx] == Mode::Optical {
            optical_ticks += 1;
            if optical_ticks >= 20 {
                optical_ticks = 0;
                if let Ok((r, g, b)) = Ltr381rgb::read(&mut i2c1, &mut timer) {
                    opt_r = r; opt_g = g; opt_b = b;
                    display_dirty = true;
                }
            }
        }

        // PIR mode: update display on state change
        if MODES[mode_idx] == Mode::Pir {
            let detected = pir_pin.is_high().unwrap_or(false);
            if detected != pir_detected {
                pir_detected = detected;
                display_dirty = true;
            }
        }

        // RGB mode: cycle hue through rainbow, send to WS2812 on GP22
        if MODES[mode_idx] == Mode::Rgb {
            rgb_active = true;
            rgb_ticks += 1;
            if rgb_ticks >= 3 {  // update every ~30 ms
                rgb_ticks = 0;
                rgb_hue = rgb_hue.wrapping_add(10);
                let (r, g, b) = hue_to_rgb(rgb_hue, 32);
                let word = ((g as u32) << 24) | ((r as u32) << 16) | ((b as u32) << 8);
                ws_tx.write(word);
                display_dirty = true;
            }
        } else if rgb_active {
            ws_tx.write(0u32); // send black once on mode exit
            rgb_active = false;
        }

        // Buzz mode: sweep duty 0..999 over 2 seconds using timer; disable PWM when not active
        if MODES[mode_idx] == Mode::Buzz {
            if !buzz_active {
                buzz_pwm.enable();
                buzz_active = true;
            }
            let new_duty = ((now % 2_000_000u64) / 2000) as u16;
            if new_duty != buzz_duty {
                buzz_duty = new_duty;
                buzz_pwm.channel_b.set_duty_cycle(buzz_duty).ok();
            }
            buzz_display_ticks += 1;
            if buzz_display_ticks >= 5 {
                buzz_display_ticks = 0;
                display_dirty = true;
            }
        } else if buzz_active {
            buzz_pwm.channel_b.set_duty_cycle(0).ok();
            buzz_pwm.disable();
            buzz_duty = 0;
            buzz_active = false;
        }

        // Temp mode: read sensor every ~1 s (temp_ticks starts at threshold for quick first read)
        if MODES[mode_idx] == Mode::Temp {
            temp_ticks += 1;
            if temp_ticks >= 100 {
                temp_ticks = 0;
                if let Ok((t, h)) = Sht30::measure(&mut i2c1, &mut timer) {
                    temp_x10 = t;
                    hum_pct = h;
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
                Mode::Magneto => {
                    let mut xb: HString<16> = HString::new();
                    let mut yb: HString<16> = HString::new();
                    let mut zb: HString<16> = HString::new();
                    write!(xb, "X:{:>+7}", mx).ok();
                    write!(yb, "Y:{:>+7}", my).ok();
                    write!(zb, "Z:{:>+7}", mz).ok();
                    Text::new(&xb, Point::new(0, 35), status_style).draw(&mut display).unwrap();
                    Text::new(&yb, Point::new(0, 47), status_style).draw(&mut display).unwrap();
                    Text::new(&zb, Point::new(0, 59), status_style).draw(&mut display).unwrap();
                }
                Mode::Optical => {
                    let mut rb: HString<16> = HString::new();
                    let mut gb: HString<16> = HString::new();
                    let mut bb: HString<16> = HString::new();
                    write!(rb, "R:{:>4}", opt_r).ok();
                    write!(gb, "G:{:>4}", opt_g).ok();
                    write!(bb, "B:{:>4}", opt_b).ok();
                    Text::new(&rb, Point::new(0, 35), status_style).draw(&mut display).unwrap();
                    Text::new(&gb, Point::new(0, 47), status_style).draw(&mut display).unwrap();
                    Text::new(&bb, Point::new(0, 59), status_style).draw(&mut display).unwrap();
                }
                Mode::Temp => {
                    let mut tb: HString<16> = HString::new();
                    let mut hb: HString<16> = HString::new();
                    let (sign, t_abs) = if temp_x10 < 0 {
                        ("-", (-temp_x10) as u32)
                    } else {
                        ("", temp_x10 as u32)
                    };
                    write!(tb, "{}{}.{}F", sign, t_abs / 10, t_abs % 10).ok();
                    write!(hb, "{}%RH", hum_pct).ok();
                    Text::new(&tb, Point::new(0, 42), label_style).draw(&mut display).unwrap();
                    Text::new(&hb, Point::new(0, 59), status_style).draw(&mut display).unwrap();
                }
                Mode::Rgb => {
                    let (r, g, b) = hue_to_rgb(rgb_hue, 32);
                    let mut line1: HString<16> = HString::new();
                    let mut line2: HString<16> = HString::new();
                    write!(line1, "R:{:>3} G:{:>3}", r, g).ok();
                    write!(line2, "B:{:>3} H:{:>3}", b, rgb_hue).ok();
                    Text::new(&line1, Point::new(0, 40), status_style).draw(&mut display).unwrap();
                    Text::new(&line2, Point::new(0, 55), status_style).draw(&mut display).unwrap();
                }
                Mode::Buzz => {
                    let bar_px = (buzz_duty as u32 * 126 / 999) as u32;
                    Rectangle::new(Point::new(1, 35), Size::new(126, 12))
                        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                        .draw(&mut display).unwrap();
                    if bar_px > 0 {
                        Rectangle::new(Point::new(1, 35), Size::new(bar_px, 12))
                            .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
                            .draw(&mut display).unwrap();
                    }
                }
                _ => {
                    let status = match mode {
                        Mode::Led => if blinking { "Blinking" } else { "Override" },
                        Mode::Pir => if pir_detected { "Motion!" } else { "Clear" },
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
