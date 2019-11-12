#![no_main]
#![no_std]

extern crate panic_halt;

use lpc8xx_hal::gpio::direction::Output;
use lpc8xx_hal::swm::{self, pin_state::Gpio, Pin};
use lpc8xx_hal::{cortex_m_rt::entry, delay::Delay, prelude::*, Peripherals};

struct RGBLed<'gpio> {
    delay: Delay,
    red: Pin<swm::PIO1_2, Gpio<'gpio, Output>>,
    green: Pin<swm::PIO1_0, Gpio<'gpio, Output>>,
    blue: Pin<swm::PIO1_1, Gpio<'gpio, Output>>,
}

impl<'gpio> RGBLed<'gpio> {
    fn display(&mut self, red: u8, green: u8, blue: u8) {
        const TIME_FRAME: f32 = 1_000.0;

        let red = red as f32;
        let green = green as f32;
        let blue = blue as f32;

        let luminance = red + green + blue;

        let (red_time_frame, green_time_frame, blue_time_frame) = if luminance > 0.0 {
            (
                (red * TIME_FRAME) / luminance,
                (green * TIME_FRAME) / luminance,
                (blue * TIME_FRAME) / luminance,
            )
        } else {
            (0.0, 0.0, 0.0)
        };

        fn map_to_time(color: f32, time_frame: f32) -> (u32, u32) {
            let on = ((color * color) * time_frame) as u32 >> 16;
            let off = time_frame as u32 - on;
            (on, off)
        }

        if red_time_frame > 0.0 {
            // display red
            let (on, off) = map_to_time(red, red_time_frame);
            self.red.set_low().unwrap();
            self.delay.delay_us(on);
            self.red.set_high().unwrap();
            self.delay.delay_us(off);
        }

        if green_time_frame > 0.0 {
            // display green
            let (on, off) = map_to_time(green, green_time_frame);
            self.green.set_low().unwrap();
            self.delay.delay_us(on);
            self.green.set_high().unwrap();
            self.delay.delay_us(off);
        }

        if blue_time_frame > 0.0 {
            // display blue
            let (on, off) = map_to_time(blue, blue_time_frame);
            self.blue.set_low().unwrap();
            self.delay.delay_us(on);
            self.blue.set_high().unwrap();
            self.delay.delay_us(off);
        }

        // remaining time
        self.delay
            .delay_us((TIME_FRAME - red_time_frame - green_time_frame - blue_time_frame) as u32)
    }
}

struct Xorwow {
    a: u32,
    b: u32,
    c: u32,
    d: u32,
    counter: u32,
}

impl Xorwow {
    fn gen(&mut self) -> u32 {
        let mut t = self.d;
        let s = self.a;
        self.d = self.c;
        self.c = self.b;
        self.b = s;

        t ^= t >> 2;
        t ^= t << 1;
        t ^= s ^ (s << 4);
        self.a = t;

        self.counter = self.counter.wrapping_add(362_437);
        t.wrapping_add(self.counter)
    }
}

#[entry]
fn main() -> ! {
    // Get access to the device's peripherals. Since only one instance of this
    // struct can exist, the call to `take` returns an `Option<Peripherals>`.
    // If we tried to call the method a second time, it would return `None`, but
    // we're only calling it the one time here, so we can safely `unwrap` the
    // `Option` without causing a panic.
    let p = Peripherals::take().unwrap();

    // Initialize the APIs of the peripherals we need.
    let swm = p.SWM.split();
    let delay = Delay::new(p.SYST);
    #[cfg(feature = "82x")]
        let gpio = p.GPIO; // GPIO is initialized by default on LPC82x.
    #[cfg(feature = "845")]
        let gpio = {
        let mut syscon = p.SYSCON.split();
        p.GPIO.enable(&mut syscon.handle)
    };

    // Select pin for LED
    let mut led = RGBLed {
        delay,
        green: swm.pins.pio1_0.into_gpio_pin(&gpio).into_output(),
        blue: swm.pins.pio1_1.into_gpio_pin(&gpio).into_output(),
        red: swm.pins.pio1_2.into_gpio_pin(&gpio).into_output(),
    };

    led.red.set_high().unwrap();
    led.green.set_high().unwrap();
    led.blue.set_high().unwrap();

    let mut rng = Xorwow {
        a: 1,
        b: 2,
        c: 3,
        d: 4,
        counter: 0,

    };

    // Blink the LED using the systick with the delay traits
    loop {
        let color = rng.gen();
        let (red, green, blue) = (color & 0xFF0000 >> 16, color & 0x00FF00 >> 8, color & 0x0000FF);
        // Display the color for 300ms (display runs in 1ms.)
        for _ in 0..300 {
            led.display(red as u8, green as u8, blue as u8);
        }
    }
}