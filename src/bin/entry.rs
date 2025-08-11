#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use defmt::*;
use defmt_rtt as _;

use ch32_hal::{bind_interrupts, peripherals};
use ch32_hal::usbd::Driver;
use ch32_hal::gpio::{Level, Output, Input, Pull};
use ch32_hal::usart::UartTx;

use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};

use embassy_executor::Spawner;
use embassy_time::Timer;
use embassy_usb::class::hid::{self, HidWriter};
use embassy_usb::{Builder, Handler};
use embassy_futures::join::join;

use heapless::Vec;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, Ordering};

use {ch32_hal as hal, panic_halt as _};

static DEBOUNCE_TIME: i32 = 20; // 20ms

// helpful references:
// https://wiki.osdev.org/USB_Human_Interface_Devices#USB_keyboard


bind_interrupts!(struct Irqs {
    USB_LP_CAN1_RX0 => ch32_hal::usbd::InterruptHandler<peripherals::USBD>;
});


#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    // Initialize and configure settings
    //ch32_hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();

    /*{
            use hal::rcc::*;

        config.rcc.hse = None;
        config.rcc.sys = Sysclk::PLL;
        config.rcc.pll_src = PllSource::HSI;
        // Multiply internal 8MHz by 6 to get 48MHz
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL6,
        });
        config.rcc.pllx = None;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        config.rcc.hspll_src = HsPllSource::HSI;
        config.rcc.hspll = Some(HsPll {
            pre: HsPllPrescaler::DIV2,
        });

        // External oscillator is 16 MHz
        //config.rcc.hse = Some(Hse {
        //    freq: ch32_hal::time::Hertz(16_000_000),
        //    mode: HseMode::Oscillator 
        //});
    }*/

    config.rcc = hal::rcc::Config::SYSCLK_FREQ_96MHZ_HSI;
    // p is for peripherals!
    let mut p = hal::init(config);

    // This allows pins 13 and 14 to be used as GPIO (standard is SWD)
    unsafe {
        // Voltatile read and write
        let mut values = read_volatile(0x40010004 as *mut u32);

        write_volatile(0x40010004 as *mut u32, values | (0b0100_0000_0000_0000_0000_0000_0000_u32));
    }

    // USB Driver configured to the proper pins.
    let usb_driver = Driver::new(p.USBD, Irqs, p.PA12, p.PA11);

    // New config with vendor ID FIAAAAAA and product Id BOA(R)D
    //let mut usb_config = embassy_usb::Config::new(0xF144, 0xB0AD);
    let mut usb_config = embassy_usb::Config::new(0xF144, 0xB0AD);
    usb_config.manufacturer = Some("Catchfire");
    usb_config.product = Some("Burnboard");
    usb_config.serial_number = Some("0000_0001");
    usb_config.self_powered = true;
    usb_config.max_power = 200; //mA
    usb_config.max_packet_size_0 = 64;

    // Window compat
    usb_config.device_class = 0xEF;
    usb_config.device_sub_class = 0x02;
    usb_config.device_protocol = 0x01;
    usb_config.composite_with_iads = true;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut device_handler = MyDeviceHandler::new();

    let mut state = hid::State::new();

    let mut builder = Builder::new(
        usb_driver,
        usb_config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
        );

    builder.handler(&mut device_handler);
    
    let mut hid_config = hid::Config {
       report_descriptor: KeyboardReport::desc(),
       request_handler: None,
       poll_ms: 60,
       max_packet_size: 64,
    };
    
    // maximum 10 byte writer, 6 for keycodes array, 3 for others
    let mut writer = HidWriter::<_, 10>::new(&mut builder, &mut state, hid_config);

    // Starts the USB engine... forever!
    let mut usb = builder.build();
    let usb_fut = usb.run();
    
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
            let scan_data = scan_matrix(&mut matrix).await;
           
            // HID modifier and keycode array
            let mut modifier: u16 = 0x0000;
            let mut keycodes = [0u8; 6];

            // scan_data length is guaranteed to be smaller than or equal to keycodes length
            for i in 0..scan_data.len() {
                if let Some(&keycode) = scan_data.get(i) {
                    // Standard typing keys
                    if keycode < 0xE0 && keycode != 0 {
                        keycodes[i] = keycode as u8;
                    // Modifier keys
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
                           _ => info!("WARNING - Unhandled modifier keycode"),
                       } 
                    }
                }
            }

            let mut new_modifier = modifier;

            // Adjust for my custom keymapping
            for i in 0..keycodes.len() {
                (new_modifier, keycodes[i]) = keycode_mapping(modifier, keycodes[i]); 
            }

            // Generate a report with the given keycodes
            let report = KeyboardReport {
                modifier: (new_modifier & 0b11111111) as u8,
                reserved: 0,
                leds: 0,
                keycodes: [keycodes[0], keycodes[1], keycodes[2], keycodes[3], keycodes[4], keycodes[5]], 
            };

            // Sends report to computer about the keys being pressed
            match writer.write_serialize(&report).await {
                Ok(()) => {}
                Err(e) => warn!("Failed to send report: {:?}", e),
            } 
        } //async loop
    }; //let main_fut

    join(usb_fut, main_fut).await;

    loop {
        Timer::after_secs(1).await;
        warn!("WARNING! Impossible point reached implying major system failure");
    }
}

async fn scan_matrix(key_matrix: &mut KeyMatrix) -> Vec::<u16, 6> {
    
    // Allocate 6 possible keys that can be pressed
    let mut keys_pressed = Vec::<u16, 6>::new();

    // Clean the list of keys pressed
    keys_pressed.clear();

    // Scan the key matrix 
    // TODO: Auto-repeat, investigate advanced debounce and interrupt-based system
    for o in 0..(key_matrix.outputs.len()) {
        let pin_o: &mut Output = &mut key_matrix.outputs[o];
        pin_o.set_high();

        // Ensure that the output has time to rise (and previous pin has fallen)
        Timer::after_nanos(20).await;

        // Scan thru set of inputs for detected key
        for i in 0..(key_matrix.inputs.len()) {
            let pin_i: &mut Input = &mut key_matrix.inputs[i];
            if pin_i.is_high() {
                // Simple debounce - check if the button is still pressed 20ms later
                Timer::after_millis(DEBOUNCE_TIME as u64).await;
                if pin_i.is_high() { 
                    // Overflow keypress detector, sets phantom case and immediately returns
                    //println!("i: {}, o: {}", i, o);
                    if let Err(_unpushed) = keys_pressed.push(interpret_coordinate(i, o).unwrap_or(0)) {
                        keys_pressed.clear();
                        for j in 0..keys_pressed.len() {
                            keys_pressed[j] = 0x01;
                        }
                        return keys_pressed;
                    }
                } // debounce pin_i.is_high()
            } // pin_i.is_high()
        }  // for i in 0..(key_matrix.inputs.len())

        // Pin fall time is handled by next pin's rise timer
        pin_o.set_low();
    }

    keys_pressed
}

// Prepare for some disgusting HID stuff. This is the primary keymapping system that takes
// my custom keymapping and remaps it to HID. I'm including support for my other
// alternative keys (like Meta and Hyper) alongside the currently implemented shift and layer keys.
fn keycode_mapping(original_modifier: u16, keycode: u8) -> (u16, u8) {
    // Unmodified
    if original_modifier == 0x000 {
        match keycode {
            0x2F => return (original_modifier &0b11011111, 0x2F), // Default { instead of [
            _ => return (original_modifier, keycode),
        }
    }
    
    // Layer handler
    if original_modifier & 0b100000000 == 0b100000000 {
        // Check if shift is being held with layer
        if original_modifier & 0b00000010 == 0x02 || original_modifier & 0b00100000 == 0x020 {
            match keycode {
                0x29 => return (original_modifier, 0x35), // Shift+Layer+Esc = ~
                0x2E => return (original_modifier, 0x2D), // Layer+= = -
                0x2F => return (original_modifier & 0b11011101, 0x30), // Shift+Layer+{ = ]
                0x33 => return (original_modifier, 0x34), // Shift+Layer+; = "
                0x36 => return (original_modifier, 0x37), // Shift+Layer+, = >
                0x38 => return (original_modifier, 0x31), // Shift+Layer+/ = |
                _ => return (original_modifier, keycode)
            }
        } else { // Shift isn't being held
            match keycode {
                0x29 => return (original_modifier, 0x35), // Layer+Esc = `
                0x2E => return (original_modifier, 0x2D), // Layer+= = -
                0x2F => return (original_modifier | 0b00100010, 0x30), // Layer+{ = }
                0x0B => return (original_modifier, 0x50), // Layer+h = left key 
                0x0D => return (original_modifier, 0x51), // Layer+j = down key
                0x0E => return (original_modifier, 0x52), // Layer+k = up key
                0x0F => return (original_modifier, 0x4F), // Layer+l = right key
                0x33 => return (original_modifier, 0x34), // Layer+; = '
                0x36 => return (original_modifier, 0x37), // Layer+, = .
                0x38 => return (original_modifier, 0x31), // Layer+/ = \
                _ => return (original_modifier, keycode)
            }
        }
    }

    // Shift Key handler
    if original_modifier & 0b00000010 == 0x02 || original_modifier & 0b00100000 == 0x020 {
        match keycode { 
            0x2F => return (original_modifier &0b11011101, 0x2F), // Shift+{ = [
            _ => return (original_modifier, keycode)
        }
    }

    (original_modifier, keycode)
}

// Interprets the keys as they are on the keymap
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
            _ => { info!("WARNING - Unexpected output value in interpret_coordinate"); None },
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
            _ => { info!("WARNING - Unexpected output value in interpret_coordinate"); None },
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
            _ => { info!("WARNING - Unexpected output value in interpret_coordinate"); None },
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
            _ => { info!("WARNING - Unexpected output value in interpret_coordinate"); None },
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
            9 => Some(0x4C), // Delete
            10 => Some(0x100), // Layer2
            11 => None, // No Key
            _ => { info!("WARNING - Unexpected output value in interpret_coordinate"); None },
        }
        _ => { info!("WARNING - Unexpected input value in interpret_coordinate"); None },
    }
}

struct KeyMatrix {
    inputs: Vec<Input<'static>, 5>,
    outputs: Vec<Output<'static>, 12>
}

struct MyDeviceHandler {
    configured: AtomicBool,
}

impl MyDeviceHandler {
    fn new() -> Self {
        MyDeviceHandler {
            configured: AtomicBool::new(false),
        }
    }
}

impl Handler for MyDeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        self.configured.store(false, Ordering::Relaxed);
        if enabled {
            info!("Device enabled");
        } else {
            info!("Device disabled");
        }
    }

    fn reset(&mut self) {
        self.configured.store(false, Ordering::Relaxed);
        info!("Bus reset, the Vbus current limit is 100mA");
    }

    fn addressed(&mut self, addr: u8) {
        self.configured.store(false, Ordering::Relaxed);
        info!("Usb address set to: {}", addr);
    }

    fn configured(&mut self, configured: bool) {
        self.configured.store(configured, Ordering::Relaxed);
        if configured {
            info!("Device configured, it may draw up to the configured current limit");
        } else {
            info!("Device no longer configured, Vbus current limit is 100mA");
        }
    }
}
