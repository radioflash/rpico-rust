//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{FunctionPio0, Pin},
    pac,
    pio::PIOExt,
    sio::Sio,
    watchdog::Watchdog,
};

fn add_numbers(a: i32, b: i32) -> i32 {
    a + b
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

    // configure LED pin for Pio0.
    let _led: Pin<_, FunctionPio0> = pins.gpio15.into_mode();
    // PIN id for use inside of PIO
    let led_pin_id = add_numbers(15, 0) as u8;

    // Define some simple PIO program.
    let mut a = pio::Assembler::<32>::new();
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();

    // Set pin as Out
    a.set(pio::SetDestination::PINDIRS, 1);
    // Define begin of program loop
    a.bind(&mut wrap_target);
    a.set(pio::SetDestination::PINS, 0);
    a.set(pio::SetDestination::PINS, 1);
    a.bind(&mut wrap_source);

    // The labels wrap_target and wrap_source, as set above,
    // define a loop which is executed repeatedly by the PIO
    // state machine.
    let program = a.assemble_with_wrap(wrap_source, wrap_target);

    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program).unwrap();
    let (int, frac) = (1, 1); // as slow as possible (0 is interpreted as 65536)
    let (sm, _, _) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .set_pins(led_pin_id, 1)
        .clock_divisor_fixed_point(int, frac)
        .build(sm0);
    sm.start();

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead. If you have
    // a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here.
    // let mut led_pin = pins.led.into_push_pull_output();

    // let mut led_pin = pins.gpio14.into_push_pull_output(); //led.into_push_pull_output();

    loop {
        // info!("on!");
        // led_pin.set_high().unwrap();
        // delay.delay_ms(500);
        // info!("off!");
        // led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
