#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use defmt::*;
use defmt_rtt as _;

use ch32_hal::{bind_interrupts, peripherals, println};
use ch32_hal::rcc::HseMode;
use ch32_hal::usbd::{self, Driver};

use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor, KeyboardUsage};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embassy_usb::class::hid::{self, HidWriter, RequestHandler};
use embassy_usb::Builder;
use embassy_futures::join::join;

use heapless::Vec;

use hal::gpio::{Level, Output, Input, Pull};
use qingke::riscv;
use {ch32_hal as hal, panic_halt as _};

// helpful references:
// https://wiki.osdev.org/USB_Human_Interface_Devices#USB_keyboard


bind_interrupts!(struct Irqs {
    USB_LP_CAN1_RX0 => ch32_hal::usbd::InterruptHandler<peripherals::USBD>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    // Initialize and configure settings
    let mut config = hal::Config::default();

    {
        use hal::rcc::*;

        // External oscillator is 16 MHz
        config.rcc.hse = Some(Hse {
            freq: ch32_hal::time::Hertz(16_000_000),
            mode: HseMode::Oscillator 
        });

        config.rcc.pll_src = PllSource::HSE;
        // Set to 128MHz max
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL8, 
        });

        config.rcc.sys = Sysclk::PLL;
    }

    // p is for peripherals!
    let p = hal::init(config);

    // USB Driver configured to the proper pins.
    let usb_driver = Driver::new(p.USBD, Irqs, p.PA12, p.PA11);

    // New config with vendor ID FIAAAAAA and product Id BOA(R)D
    let mut config = embassy_usb::Config::new(0xF144, 0xB0AD);
    config.manufacturer = Some("Catchfire");
    config.product = Some("Burnboard");
    config.serial_number = Some("0000_0001");
    config.self_powered = true;
    config.max_power = 350; //mA
    config.max_packet_size_0 = 16;

    // Window compat
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut request_handler = MyRequestHandler {};

    let mut state = hid::State::new();

    let mut builder = Builder::new(
        usb_driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buf,
        );

    let mut config = hid::Config {
       report_descriptor: KeyboardReport::desc(),
       request_handler: Some(&mut request_handler),
       poll_ms: 60,
       max_packet_size: 8,
    };
    
    // maximum 8 byte writer, 6 for keycodes and two for modifier
    let mut writer = HidWriter::<_, 8>::new(&mut builder, &mut state, config);

    // Starts the USB engine... forever!
    let mut usb = builder.build();
    let usb_fut = usb.run();

    // Currently reports the letter "a" every 500ms
    let hid_fut = async {
        loop {
            Timer::after_millis(500).await;

            let mut report = KeyboardReport::default();

            report.keycodes = [0u8; 6];

            // Letter A 
            report.keycodes[0] = KeyboardUsage::KeyboardAa as u8;

            match writer.write_serialize(&report).await {
                Ok(()) => {}
                Err(e) => warn!("Failed to send report: {:?}", e),
            }
        }
    };

    // STOPS AFTER THIS! (FOR NOW)
    join(usb_fut, hid_fut).await;

    // Internally pull-down the Input ports
    let mut row1 = Input::new(p.PB7, Pull::Down);
    let mut row2 = Input::new(p.PB6, Pull::Down);
    let mut row3 = Input::new(p.PB5, Pull::Down);
    let mut row4 = Input::new(p.PB4, Pull::Down);
    let mut row5 = Input::new(p.PB3, Pull::Down);

    let mut inputs = Vec::<Input<'static>, 5>::new();

    let _ = inputs.push(row1);
    let _ = inputs.push(row2);
    let _ = inputs.push(row3);
    let _ = inputs.push(row4);
    let _ = inputs.push(row5);
    
    // Instantiate all twelve columns and all five rows
    let mut col1 = Output::new(p.PA1, Level::Low, Default::default());
    let mut col2 = Output::new(p.PB1, Level::Low, Default::default());
    let mut col3 = Output::new(p.PA0, Level::Low, Default::default());
    let mut col4 = Output::new(p.PA4, Level::Low, Default::default());
    let mut col5 = Output::new(p.PA5, Level::Low, Default::default());
    let mut col6 = Output::new(p.PA6, Level::Low, Default::default());
    let mut col7 = Output::new(p.PA7, Level::Low, Default::default());
    let mut col8 = Output::new(p.PB0, Level::Low, Default::default());
    let mut col9 = Output::new(p.PA9, Level::Low, Default::default());
    let mut col10 = Output::new(p.PA13, Level::Low, Default::default());
    let mut col11 = Output::new(p.PA14, Level::Low, Default::default());
    let mut col12 = Output::new(p.PA15, Level::Low, Default::default());
 
    let mut outputs = Vec::<Output<'static>, 12>::new(); 
                                          
    let _ = outputs.push(col1);
    let _ = outputs.push(col2);
    let _ = outputs.push(col3);
    let _ = outputs.push(col4);
    let _ = outputs.push(col5);
    let _ = outputs.push(col6);
    let _ = outputs.push(col7);
    let _ = outputs.push(col8);
    let _ = outputs.push(col9);
    let _ = outputs.push(col10);
    let _ = outputs.push(col11);
    let _ = outputs.push(col12);

    let mut matrix = KeyMatrix {
        inputs,
        outputs,
    };

    // Main loop
    let main_fut = async {
        loop {
            // Scans the key matrix
            let mut scan_data = scan_matrix(&mut matrix).await;
           
            // HID modifier and keycode array
            let mut modifier: u16 = 0x0000;
            let mut keycodes = [0u8; 6];

            // scan_data length is guaranteed to be smaller than or equal to keycodes length
            for i in 0..scan_data.len() {
                if let Some(&keycode) = scan_data.get(i) {
                    // Standard keys
                    if keycode < 0xE0 && keycode != 0 {
                        keycodes[i] = keycode as u8;
                    // Standard Modifier keys
                    // a la
                    // https://docs.zephyrproject.org/apidoc/latest/group__usb__hid__mk__report__desc.html
                    } else if keycode >= 0xE0 {
                       match keycode {
                           0xE0 => modifier |= 0x01, // left ctrl
                           0xE1 => modifier |= 0x02, // left shift
                           0xE2 => modifier |= 0x04, // left alt
                           0xE3 => modifier |= 0x08, // left super
                           0xE4 => modifier |= 0x10, // right ctrl 
                           0xE5 => modifier |= 0x20, // right shift
                           0xE6 => modifier |= 0x40, // right alt
                           0xE7 => modifier |= 0x80, // right super
                           0x100 => modifier |= 0x100, // layer
                           0x200 => modifier |= 0x200, // function
                           0x400 => modifier |= 0x400, // meta
                           0x800 => modifier |= 0x800, // hyper
                           _ => info!("WARNING - Impossible modifier keycode"),
                       } 
                    }
                }
            }

            for i in 0..keycodes.len() {
                
            }

            // Generate a report with the given keycodes
            let report = KeyboardReport {
                modifier,
                reserved: 0,
                leds: 0,
                keycodes 
            };

            // Sends report to computer about the keys being pressed
            match writer.write_serialize(&report).await {
                Ok(()) => {}
                Err(e) => warn!("Failed to send report: {:?}", e),
            } 
        }
    };

    

    loop {
        info!("WARNING! Impossible point reached implying system failure");
    }
}

async fn scan_matrix(key_matrix: &mut KeyMatrix) -> Vec::<u16, 6> {
    
    // Allocate 6 keys to be pressed
    let mut keys_pressed = Vec::<u16, 6>::new();

    // Clean the list of keys pressed
    keys_pressed.clear();

    // First, scan the key matrix 
    // TODO: Add debouncing, investigate interrupt-based system
    for o in 0..(key_matrix.outputs.len()) {
        let pin_o: &mut Output = &mut key_matrix.outputs[o];
        // enable pin output
        pin_o.toggle();
        Timer::after_micros(50).await;

        for i in 0..key_matrix.inputs.len() {
            let pin_i: &mut Input = &mut key_matrix.inputs[i];
            // poll input
            if pin_i.is_high() {
                if let Err(_unpushed) = keys_pressed.push(interpret_coordinate(i, o).unwrap_or(0)) {
                    // Overflow detected, set phantom case and immediately return
                    keys_pressed.clear();
                    for j in 0..keys_pressed.len() {
                        keys_pressed[j] = 0x01;
                    }
                    return keys_pressed;
                }
            }
        } 

        // disable pin output
        pin_o.toggle();
        Timer::after_micros(50).await;
    }

    keys_pressed
}

// Prepare for some disgusting HID stuff. This is the primary keymapping system that takes actual
// HID codes and remaps them into my custom keymapping. I'm including support for my other
// alternative keys (like Meta and Hyper) alongside the currently implemented shift and layer keys.
fn keycode_mapping(original_modifier: u8, keycode: u8) -> (u8, u8) {
    // Unmodified
    if(original_modifier == 0x000) {
        match keycode {
            0x2F => return (original_modifier | 0x02, 0x2F),
            _ => return (original_modifier, keycode),
        }
    }
    
    // Layer handler
    if(original_modifier & 0x100 == 0x100) {

    }


    // Shift Key handler
    if(original_modifier & 0x01 == 0x02 || original_modifier & 0x010 == 0x020) {
        match keycode {
                    // 
            _ => return (original_modifier, keycode)
        }
    }

    (0,0)
}

// Add special handling for Layer 1 (0x100), Fn (0x200), Layer 2 (0x100), Meta (0x400), and Hyper (0x800)
fn interpret_coordinate(input: usize, output: usize) -> Option<u16> {
    match input {
        0 => match output {
            0 => Some(0x29), // ESC
            1 => Some(0x1E), // 1
            2 => Some(0x1F), // 2
            3 => Some(0x20), // 3
            4 => Some(0x21), // 4
            5 => Some(0x22), // 5
            6 => Some(0x23), // 6
            7 => Some(0x24), // 7
            8 => Some(0x25), // 8
            9 => Some(0x26), // 9
            10 => Some(0x27), // 0
            11 => Some(0x2E), // =
            _ => { info!("WARNING - Unexpected output value in interpret_coordinate"); return None },
        }
        1 => match output {
            0 => Some(0x2B), // Tab
            1 => Some(0x14), // Q
            2 => Some(0x1A), // W
            3 => Some(0x08), // E
            4 => Some(0x15), // R
            5 => Some(0x17), // T
            6 => Some(0x1C), // Y
            7 => Some(0x18), // U
            8 => Some(0x0C), // I
            9 => Some(0x12), // O
            10 => Some(0x13), // P
            11 => Some(0x2F), // {
            _ => { info!("WARNING - Unexpected output value in interpret_coordinate"); return None },
        }
        2 => match output {
            0 => Some(0x100), // Layer 1
            1 => Some(0x04), // A
            2 => Some(0x16), // S
            3 => Some(0x07), // D
            4 => Some(0x09), // F
            5 => Some(0x0A), // G
            6 => Some(0x0B), // H
            7 => Some(0x0D), // J
            8 => Some(0x0E), // K
            9 => Some(0x0F), // L
            10 => Some(0x33), // ;
            11 => Some(0x2A), // Backspace
            _ => { info!("WARNING - Unexpected output value in interpret_coordinate"); return None },
        }
        3 => match output {
            0 => Some(0xE1), // Shift
            1 => Some(0x1D), // Z
            2 => Some(0x1B), // X
            3 => Some(0x06), // C
            4 => Some(0x19), // V
            5 => Some(0x05), // B
            6 => Some(0x11), // N
            7 => Some(0x10), // M
            8 => Some(0x36), // ,
            9 => Some(0x38), // /
            10 => Some(0xE5), // Shift
            11 => Some(0x28), // Enter 
            _ => { info!("WARNING - Unexpected output value in interpret_coordinate"); return None },
        }
        4 => match output {
            0 => Some(0xE0), // Ctrl
            1 => Some(0xE3), // 👽 
            2 => Some(0xE2), // Alt
            3 => Some(0x200), // Fn 
            4 => Some(0x2C), // Spacebar
            5 => None, // No key
            6 => None, // No key
            7 => Some(0x400), // Meta
            8 => Some(0x800), // Hyper
            9 => Some(0x100), // Layer2
            10 => Some(0x4C), // Delete
            11 => None, // No Key
            _ => { info!("WARNING - Unexpected output value in interpret_coordinate"); return None },
        }
        _ => { info!("WARNING - Unexpected input value in interpret_coordinate"); return None },
    }
}

struct KeyMatrix {
    inputs: Vec<Input<'static>, 5>,
    outputs: Vec<Output<'static>, 12>
}


// TODO - What the heck is this and do I need to implement more useful behavior?
struct MyRequestHandler {}

impl RequestHandler for MyRequestHandler {
    fn get_report(&mut self, id: hid::ReportId, _buf: &mut [u8]) -> Option<usize> {
        info!("Get report for {:?}", id);
        None
    }

    fn set_report(&mut self, id: hid::ReportId, data: &[u8]) -> embassy_usb::control::OutResponse {
        info!("Set report for {:?}: {=[u8]}", id, data);
        embassy_usb::control::OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, id: Option<hid::ReportId>, duration_ms: u32) {
        info!("Set idle rate for {:?} to {:?}", id, duration_ms);
    }

    fn get_idle_ms(&mut self, id: Option<hid::ReportId>) -> Option<u32> {
        info!("Get idle rate for {:?}", id);
        None
    }
}
