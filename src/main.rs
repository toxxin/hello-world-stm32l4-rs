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

use crate::rt::ExceptionFrame;
use core::panic::PanicInfo;

use rtic::app;

#[app(device = stm32l4xx_hal::pac, peripherals = true, dispatchers = [EXTI0, EXTI1])]
mod app {

    use stm32l4xx_hal::{gpio::{gpioa::PA2, gpioa::PA15, gpiob::PB3, gpiob::PB6, gpiob::PB7, Alternate, Edge, Input, Output, Pin, PullUp, PushPull, OpenDrain}, i2c::I2c, serial::Serial, delay::Delay, prelude::*, hal, pac};
    use systick_monotonic::{fugit::Duration, Systick};
    use core::fmt::Write;
    use defmt_bbq::DefmtConsumer;
    use stm32l4xx_hal::timer::{Event, Timer};
    use stm32l4xx_hal::serial::{Event as SerialEvent};
    use stm32l4xx_hal::serial;
    use stm32l4xx_hal::i2c;

    // A monotonic timer to enable scheduling in RTIC
    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>; // 1000 Hz / 1 ms granularity

    #[shared]
    struct Shared {
        tx: serial::Tx<stm32l4xx_hal::pac::USART2>,
        i2c1: i2c::I2c<pac::I2C1, (PB6<Alternate<OpenDrain, 4>>, PB7<Alternate<OpenDrain, 4>>)>,
        consumer: DefmtConsumer,
    }

    #[local]
    struct Local {
        led: PB3<Output<PushPull>>,
        state: bool,
        tim: Timer<pac::TIM2>,
        rx: serial::Rx<stm32l4xx_hal::pac::USART2>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // let systick = cx.core.SYST;

        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let mut pwr = cx.device.PWR.constrain(&mut rcc.apb1r1);

        let clocks = rcc
            .cfgr
            .sysclk(80.MHz())
            .pclk1(80.MHz())
            .pclk2(80.MHz())
            .freeze(&mut flash.acr, &mut pwr);

        let mut tim = Timer::tim2(cx.device.TIM2, 100.Hz(), clocks, &mut rcc.apb1r1);
        tim.listen(Event::TimeOut);

        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb2);

        let txp = gpioa.pa2.into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
        let rxp = gpioa.pa15.into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

        let mut gpiob = cx.device.GPIOB.split(&mut rcc.ahb2);
        let mut led = gpiob
            .pb3
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
        led.set_high();

        // let mut timer = Delay::new(cp.SYST, clocks);

        let mut serial = Serial::usart2(cx.device.USART2, (txp, rxp), 115_200.bps(), clocks, &mut rcc.apb1r1);

        serial.listen(SerialEvent::Rxne);
        let (tx, rx) = serial.split();

        let mut consumer = defmt_bbq::init().unwrap();

        // defmt::println!("Hello, world+++++!");

        // I2C configuration
        let mut scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let mut sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

        scl.internal_pull_up(&mut gpiob.pupdr, true);
        sda.internal_pull_up(&mut gpiob.pupdr, true);

        let mut i2c1 = I2c::i2c1(cx.device.I2C1, (scl, sda), i2c::Config::new(100.kHz(), clocks), &mut rcc.apb1r1);

        let mono = Systick::new(cx.core.SYST, 80_000_000);

        // Schedule the blinking task
        blink::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();
        heart::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();
        logger::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();
        indication_tick::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();

        (Shared{tx, i2c1, consumer}, Local { rx, led, state: false, tim }, init::Monotonics(mono))
    }

    #[task(local = [led, state], shared = [tx])]
    fn blink(cx: blink::Context) {
        if *cx.local.state {
            cx.local.led.set_high();
            *cx.local.state = false;
        } else {
            cx.local.led.set_low();
            *cx.local.state = true;
        }

        let mut serial = cx.shared.tx;
        serial.lock(|s: &mut serial::Tx<stm32l4xx_hal::pac::USART2>| {
            let value: u8 = 1;
            writeln!(s, "Hello, World! {:02}\r", value).unwrap();
        });

        blink::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();
    }

    #[task(shared = [tx])]
    fn heart(cx: heart::Context) {
        let mut serial = cx.shared.tx;
        serial.lock(|s: &mut serial::Tx<stm32l4xx_hal::pac::USART2>| {
            static mut value: u8 = 1;
            writeln!(s, "Heart{{bit}} task! {:02}\r", unsafe {value} ).unwrap();
        });

        heart::spawn_after(Duration::<u64, 1, 1000>::from_ticks(500)).unwrap();
    }

    #[task(binds = TIM2, priority = 1, local = [tim], shared = [tx])]
    fn timer(cx: timer::Context) {
        let _ = cx.local.tim.clear_interrupt(Event::TimeOut);
        let mut serial = cx.shared.tx;
        serial.lock(|_: &mut serial::Tx<stm32l4xx_hal::pac::USART2>| {
            // writeln!(s, "timer7\r").unwrap();
        });
    }

    #[task(binds = USART2, priority = 3, shared = [tx], local = [rx])]
    fn uart_rx(cx: uart_rx::Context) {
        if let Ok(b) = cx.local.rx.read() {
            let mut stx = cx.shared.tx;
            stx.lock(|s: &mut serial::Tx<stm32l4xx_hal::pac::USART2>| {
                writeln!(s, "uart_rx: {}\r", b as char).unwrap();
            });
        }
    }

    #[task(priority = 2, shared = [tx, i2c1])]
    fn indication_tick(cx: indication_tick::Context)
    {
        let mut i2c_bus = cx.shared.i2c1;
        let mut buffer = [0u8; 7];
        i2c_bus.lock(|iface: &mut i2c::I2c<stm32l4xx_hal::pac::I2C1, (PB6<Alternate<OpenDrain, 4>>, PB7<Alternate<OpenDrain, 4>>)>| {
            iface.write(0x3c, &buffer).unwrap();
        });

        indication_tick::spawn_after(Duration::<u64, 1, 1000>::from_ticks(100)).unwrap();
    }

    #[task(priority = 1, shared = [tx, consumer])]
    fn logger(cx: logger::Context)
    {
        let mut con = cx.shared.consumer;
        con.lock(|c: &mut DefmtConsumer| {
            if let Ok(grant) = c.read() {
                let mut stx = cx.shared.tx;

                stx.lock(|s: &mut serial::Tx<stm32l4xx_hal::pac::USART2>| {

                    for byte in grant.buf() {
                        // block!(cx.local.serial2.write(*byte)).unwrap();
                        writeln!(s, "{}\r", *byte).unwrap();
                    }
                });

                // Then when done, make sure you release the grant
                // to free the space for future logging.
                let glen = grant.len();
                grant.release(glen);
            }
        });

        logger::spawn_after(Duration::<u64, 1, 1000>::from_ticks(100)).unwrap();
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
