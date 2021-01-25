#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use cortex_m_rt::entry;
use stm32f3xx_hal as hal;
use hal::pac as pac;
use hal::{
    prelude::*,
    usb::{Peripheral, UsbBus},
};
use usb_device::prelude::*;

use usbd_midi::{
    data::{
        midi::{message::Message, notes::Note},
        usb::constants::USB_CLASS_NONE,
        usb_midi::midi_packet_reader::MidiPacketBufferReader,
    },
    midi_device::MidiClass,
};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .freeze(&mut flash.acr);

    assert!(clocks.usbclk_valid());

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    let mut led = gpioe
        .pe11
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

    let usb = Peripheral {
        usb: dp.USB,
        pin_dm: gpioa.pa11.into_af14(&mut gpioa.moder, &mut gpioa.afrh),
        pin_dp: gpioa.pa12.into_af14(&mut gpioa.moder, &mut gpioa.afrh),
    };

    let usb_bus = UsbBus::new(usb);

    let mut midi = MidiClass::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x5e4))
        .product("usbd-midi example")
        .device_class(USB_CLASS_NONE)
        .serial_number("serial_number")
        .manufacturer("manufacturer")
        .build();

    loop {
        if !usb_dev.poll(&mut [&mut midi]) {
            continue;
        }

        let mut buffer = [0; 64];

        if let Ok(size) = midi.read(&mut buffer) {
            let buffer_reader = MidiPacketBufferReader::new(&buffer, size);
            for packet in buffer_reader.into_iter() {
                if let Ok(packet) = packet {
                    match packet.message {
                        Message::NoteOn(Channel1, Note::C2, ..) => {
                            led.set_low().unwrap();
                        }
                        Message::NoteOff(Channel1, Note::C2, ..) => {
                            led.set_high().unwrap();
                        }
                        _ => {}
                    }
                }
            }
        }
    }
}
