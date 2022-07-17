#![no_std]
#![no_main]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt as rt;
// extern crate cortex_m_semihosting as sh;
// extern crate panic_semihosting;
extern crate stm32l4xx_hal as hal;
#[macro_use(block)]
extern crate nb;

use crate::hal::delay::Delay;
use crate::hal::prelude::*;
use crate::hal::serial::Serial;
use crate::rt::entry;
use crate::rt::ExceptionFrame;
use core::panic::PanicInfo;
// use crate::sh::hio;
use core::fmt::Write;
use cortex_m::asm;

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

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
