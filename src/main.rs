#![no_std]
#![no_main]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt as rt;
// extern crate cortex_m_semihosting as sh;
// extern crate panic_semihosting;
// extern crate stm32l4xx_hal as hal;
// #[macro_use(block)]
// extern crate nb;

// use crate::hal::delay::Delay;
// use crate::hal::prelude::*;
// use crate::hal::serial::Serial;
// use crate::rt::entry;
use crate::rt::ExceptionFrame;
use core::panic::PanicInfo;
// use crate::sh::hio;
// use core::fmt::Write;
// use cortex_m::asm;


use rtic::app;

#[app(device = stm32l4xx_hal::pac, peripherals = true)]
mod app {

    use stm32l4xx_hal::{
        gpio::{gpioa::PA0, gpioc::PC6, Alternate, Edge, Input, Output, Pin, PullUp, PushPull},
        serial::Serial,
        delay::Delay,
        prelude::*,
    };
    use systick_monotonic::{fugit::Duration, Systick};

    // A monotonic timer to enable scheduling in RTIC
    #[monotonic(binds = SysTick, default = true)]
    type Tonic = Systick<1000>; // 100 Hz / 10 ms granularity

    // Resources shared between tasks
    #[shared]
    struct Shared {
        // s1: u32,
        // s2: i32,
    }

    // Local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {
        // l1: u8,
        // l2: i8,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // let systick = cx.core.SYST;

        // Initialize the monotonic (SysTick rate in QEMU is 12 MHz)
        // let mono = Systick::new(systick, 12_000_000);

        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let mut pwr = cx.device.PWR.constrain(&mut rcc.apb1r1);

        // let mut flash = dp.FLASH.constrain(); // .constrain();
        // let mut rcc = dp.RCC.constrain();
        // let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

        let clocks = rcc
            .cfgr
            .sysclk(80.MHz())
            .pclk1(80.MHz())
            .pclk2(80.MHz())
            .freeze(&mut flash.acr, &mut pwr);

        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb2);

        // let mut txp = gpioa.pa2.into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
        // let rxp = gpioa
        //     .pa3
        //     .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
        //
        // let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);
        // let mut led = gpiob
        //     .pb3
        //     .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
        // led.set_high();
        //
        // let mut timer = Delay::new(cp.SYST, clocks);
        //
        // let serial = Serial::usart2(dp.USART2, (txp, rxp), 115_200.bps(), clocks, &mut rcc.apb1r1);
        // let (mut tx, mut rx) = serial.split();
        //
        // let sent = b'X';
        // let mut value: u8 = 0;
        //
        // writeln!(tx, "value: {:02}\r", value).unwrap();

        let mono = Systick::new(cx.core.SYST, 48_000_000);

        (Shared {}, Local {}, init::Monotonics(mono))
    }
}

/*
#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = hal::stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain(); // .constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    // Try a different clock configuration
    // let clocks = rcc.cfgr.hclk(8.MHz()).freeze(&mut flash.acr, &mut pwr);
    // let clocks = rcc.cfgr
    //     .sysclk(64.MHz())
    //     .pclk1(32.MHz())
    //     .freeze(&mut flash.acr, &mut pwr);
    let clocks = rcc
        .cfgr
        .sysclk(80.MHz())
        .pclk1(80.MHz())
        .pclk2(80.MHz())
        .freeze(&mut flash.acr, &mut pwr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);

    let mut txp = gpioa.pa2.into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let rxp = gpioa
        .pa3
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);
    let mut led = gpiob
        .pb3
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    led.set_high();

    let mut timer = Delay::new(cp.SYST, clocks);

    let serial = Serial::usart2(dp.USART2, (txp, rxp), 115_200.bps(), clocks, &mut rcc.apb1r1);
    let (mut tx, mut rx) = serial.split();

    let sent = b'X';
    let mut value: u8 = 0;

    loop {
        // block!(timer.wait()).unwrap();
        timer.delay_ms(1000_u32);
        led.set_high();
        // block!(timer.wait()).unwrap();
        timer.delay_ms(1000_u32);
        led.set_low();

        // tx.write(b" over the lazy dog.").wait();

        writeln!(tx, "value: {:02}\r", value).unwrap();
       // block!(tx.write(sent)).ok();
    }
}
*/
#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
