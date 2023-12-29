#![no_main]
#![no_std]

use cortex_m::delay::Delay;
use embedded_hal::digital::v2::OutputPin;
use rp_pico::{
    hal::{
        self,
        gpio::{FunctionPio0, Pin, PullUp},
        pio::{PIOExt as _, ShiftDirection},
        Clock as _, Sio,
    },
    pac, XOSC_CRYSTAL_FREQ, entry,
};
use z80rp2040 as _; // global logger + panicking-behavior + memory layout

const ROM_SIZE: usize = 0x1000;
const RAM_SIZE: usize = 0x10000 - ROM_SIZE;
const ROM: &[u8] = include_bytes!("../hello.bin");

#[entry]
fn entry() -> ! {
    defmt::println!("Hello, world!");

    let core = cortex_m::Peripherals::take().unwrap();
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
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
    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut reset_pin = pins.gpio29.into_push_pull_output();
    reset_pin.set_low().unwrap();

    let (mut pio, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);

    {
        let clk_pin: Pin<_, FunctionPio0, _> = pins.gpio28.into_function();
        let clk_pin_id = clk_pin.id().num;
        let clk_pg = pio_proc::pio_asm! {
            ".wrap_target",
            "set pins, 0",
            "set pins, 1",
            ".wrap",
        };
        let installed = pio.install(&clk_pg.program).unwrap();
        let (int, frac) = (50, 0); // 125MHz / 50 = 1.25MHz
        let (mut sm, _, _tx) = hal::pio::PIOBuilder::from_program(installed)
            .set_pins(clk_pin_id, 1)
            .clock_divisor_fixed_point(int, frac)
            .build(sm0);
        sm.set_pindirs([(clk_pin_id, hal::pio::PinDir::Output)]);
        sm.start();
    }

    let d0_pin: Pin<_, FunctionPio0, _> = pins.gpio16.into_function();
    let _d1_pin: Pin<_, FunctionPio0, _> = pins.gpio17.into_function();
    let _d2_pin: Pin<_, FunctionPio0, _> = pins.gpio18.into_function();
    let _d3_pin: Pin<_, FunctionPio0, _> = pins.gpio19.into_function();
    let _d4_pin: Pin<_, FunctionPio0, _> = pins.gpio20.into_function();
    let _d5_pin: Pin<_, FunctionPio0, _> = pins.gpio21.into_function();
    let _d6_pin: Pin<_, FunctionPio0, _> = pins.gpio22.into_function();
    let _d7_pin: Pin<_, FunctionPio0, _> = pins.gpio23.into_function();
    let rfsh_pin: Pin<_, FunctionPio0, PullUp> = pins.gpio24.into_function().into_pull_type();
    let mreq_pin: Pin<_, FunctionPio0, PullUp> = pins.gpio25.into_function().into_pull_type();
    let rd_pin: Pin<_, FunctionPio0, PullUp> = pins.gpio26.into_function().into_pull_type();
    let wait_pin: Pin<_, FunctionPio0, _> = pins.gpio27.into_function();
    let d0_pin_id = d0_pin.id().num;
    let rfsh_pin_id = rfsh_pin.id().num;
    let mreq_pin_id = mreq_pin.id().num;
    let rd_pin_id = rd_pin.id().num;
    let wait_pin_id = wait_pin.id().num;
    let mem_pg = pio_proc::pio_asm! {
        ".side_set 1 opt",
        "wait 1 gpio 25",
        "nop side 1",
        ".wrap_target",
        "wait 0 gpio 25",
        "jmp pin nonrfsh",
        "jmp next",
        "nonrfsh:",
        "nop side 0",
        "in pins, 1", // read nRD
        "push",
        "pull ifempty block",
        "out pindirs, 8",
        "out pins, 8",
        "next:",
        "nop side 1",
        "wait 1 gpio 25",
        // clear data bus
        "out pins, 8",
        "out pindirs, 8",
        ".wrap",
    };

    let installed = pio.install(&mem_pg.program).unwrap();
    let (int, frac) = (1, 0); // 125MHz / 1 = 125MHz
    let (mut sm, mut rx, mut tx) = hal::pio::PIOBuilder::from_program(installed)
        .side_set_pin_base(wait_pin_id)
        .out_pins(d0_pin_id, 8)
        .in_pin_base(rd_pin_id)
        .jmp_pin(rfsh_pin_id)
        .clock_divisor_fixed_point(int, frac)
        .out_shift_direction(ShiftDirection::Right)
        .build(sm1);
    sm.set_pindirs(
        [
            (rfsh_pin_id, hal::pio::PinDir::Input),
            (mreq_pin_id, hal::pio::PinDir::Input),
            (rd_pin_id, hal::pio::PinDir::Input),
            (wait_pin_id, hal::pio::PinDir::Output),
        ]
        .into_iter()
        .chain((0..8).map(|i| (d0_pin_id + i, hal::pio::PinDir::Input))),
    );
    sm.start();

    let _a0 = pins.gpio0.into_floating_input();
    let _a1 = pins.gpio1.into_floating_input();
    let _a2 = pins.gpio2.into_floating_input();
    let _a3 = pins.gpio3.into_floating_input();
    let _a4 = pins.gpio4.into_floating_input();
    let _a5 = pins.gpio5.into_floating_input();
    let _a6 = pins.gpio6.into_floating_input();
    let _a7 = pins.gpio7.into_floating_input();
    let _a8 = pins.gpio8.into_floating_input();
    let _a9 = pins.gpio9.into_floating_input();
    let _a10 = pins.gpio10.into_floating_input();
    let _a11 = pins.gpio11.into_floating_input();
    let _a12 = pins.gpio12.into_floating_input();
    let _a13 = pins.gpio13.into_floating_input();
    let _a14 = pins.gpio14.into_floating_input();
    let _a15 = pins.gpio15.into_floating_input();

    delay.delay_ms(1);
    reset_pin.set_high().unwrap();

    let gpio_in_addr: u32 = 0xd0000000 + 0x004;
    let gpio_in_ptr = gpio_in_addr as *const u32;

    let ram = cortex_m::singleton!(: [u8; RAM_SIZE] = [0; RAM_SIZE]).unwrap();

    loop {
        if let Some(n_rd) = rx.read() {
            if n_rd == 0 {
                // READ
                let gpio = unsafe { core::ptr::read_volatile(gpio_in_ptr) };
                let bus_addr = (gpio & 0xFFFF) as usize;
                let value = if bus_addr < ROM.len() {
                    ROM[bus_addr]
                } else if ROM_SIZE < bus_addr {
                    ram[bus_addr - ROM_SIZE]
                } else {
                    0x00
                };
                defmt::println!("READ: {:04x} -> {:02x}", bus_addr, value);
                tx.write((value as u32) << 8 | 0xFF);
            } else {
                // WRITE
                defmt::println!("WRITE");
                tx.write(0);
            }
        }
    }

    //rpi_hello::exit()
}
