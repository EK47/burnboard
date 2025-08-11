#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use core::fmt::Write;

use ch32_hal::usart;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::ParityType;
use hal::gpio::{AnyPin, Level, Output, Input, Pin};
use hal::usart::{Uart, Parity, StopBits, DataBits};
use embassy_executor::Spawner;
use {ch32_hal as hal, panic_halt as _};

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    // Initialize and configure settings
    //ch32_hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();

    config.rcc = hal::rcc::Config::SYSCLK_FREQ_96MHZ_HSI;
    // p is for peripherals!
    let mut p = hal::init(config);

    let mut uart_cfg = usart::Config::default();
    uart_cfg.baudrate = 115200;
    uart_cfg.parity = Parity::ParityNone;
    uart_cfg.stop_bits = StopBits::STOP1;
    uart_cfg.data_bits = DataBits::DataBits8;

    let mut uart = Uart::new_blocking(p.USART2, p.PA3, p.PA2, uart_cfg).unwrap();

    // Writes "Burnboard" to the name of the bluetooth device
    loop {
        Timer::after_millis(1000).await; 
    
        uart.blocking_write(b"3448011189718465583770293407674980");
    }
}

