#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

#[macro_use]
extern crate lazy_static;

use core::{alloc::Layout, cell::RefCell};

use alloc_cortex_m::CortexMHeap;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use hal::{gpio::{Input, PullDown, gpioa::PA0}, pac};
use pac::interrupt;

use hal::{
    prelude::*,
    usb::{Peripheral, UsbBus},
};
use panic_halt as _;
use ringbuf::{Consumer, Producer, RingBuffer};
use stm32f3xx_hal as hal;
use usb_device::prelude::*;

use usbd_midi::{
    data::{
        byte::u7::U7,
        midi::{message::Message, notes::Note},
        usb::constants::USB_CLASS_NONE,
        usb_midi::{
            cable_number::CableNumber, midi_packet_reader::MidiPacketBufferReader,
            usb_midi_event_packet::UsbMidiEventPacket,
        },
    },
    midi_device::MidiClass,
};

lazy_static! {
    static ref MUTEX_BUTTON_PIN: Mutex<RefCell<Option<PA0<Input<PullDown>>>>> = Mutex::new(RefCell::new(None));
    static ref MUTEX_EXTI: Mutex<RefCell<Option<pac::EXTI>>> = Mutex::new(RefCell::new(None));
    static ref FIFO_PRODUCER: Mutex<RefCell<Option<Producer<Message>>>> = Mutex::new(RefCell::new(None));
    static ref FIFO_CONSUMER: Mutex<RefCell<Option<Consumer<Message>>>> = Mutex::new(RefCell::new(None));
}

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

#[interrupt]
fn EXTI0() {
    cortex_m::interrupt::free(|cs| {
        let exti = MUTEX_EXTI.borrow(cs).borrow(); // acquire Mutex
        exti.as_ref()
            .unwrap() // unwrap RefCell
            .pr1
            .modify(|_, w| w.pr0().set_bit()); // clear the EXTI line 0 pending bit
    });

    let button_state = cortex_m::interrupt::free(|cs| {
        let pin = MUTEX_BUTTON_PIN.borrow(cs).borrow(); // acquire Mutex
        pin
            .as_ref()
            .unwrap() // unwrap RefCell
            .is_high()
            .unwrap()
    });
    let message = if button_state {
        Message::NoteOn(
            usbd_midi::data::midi::channel::Channel::Channel1,
            Note::A4,
            U7::MAX,
        )
    } else {
        Message::NoteOff(
            usbd_midi::data::midi::channel::Channel::Channel1,
            Note::A4,
            U7::MAX,
        )
    };

    cortex_m::interrupt::free(|cs| {
        let mut producer = FIFO_PRODUCER.borrow(cs).borrow_mut();
        let p = producer.as_mut().unwrap();
        let _ = p.push(message);
    });
}

#[entry]
fn main() -> ! {
    // Initialize the allocator
    let start = cortex_m_rt::heap_start() as usize;
    let size = 1024; // in bytes
    unsafe { ALLOCATOR.init(start, size) }

    let dp = pac::Peripherals::take().unwrap();
    {
        let rcc = &dp.RCC;
        rcc.ahbenr.modify(|_, w| w.iopaen().set_bit()); // enable clock for GPIOA
        rcc.apb2enr.modify(|_, w| w.syscfgen().set_bit()); // enable clock for SYSCFG

        let gpioa = &dp.GPIOA;
        gpioa.moder.modify(|_, w| w.moder0().input()); // moder0 corresponds to pin 0 on GPIOA
        gpioa.pupdr.modify(|_, w| unsafe { w.pupdr0().bits(0b10) }); // set mode to pull-down

        let syscfg = &dp.SYSCFG;
        syscfg
            .exticr1
            .modify(|_, w| unsafe { w.exti0().bits(0b000) }); // connect EXTI0 to PA0 pin

        let exti = &dp.EXTI;
        exti.imr1.modify(|_, w| w.mr0().set_bit()); // unmask interrupt

        exti.rtsr1.modify(|_, w| w.tr0().set_bit()); // trigger on rising-edge
        exti.ftsr1.modify(|_, w| w.tr0().set_bit()); // trigger on falling-edge
    }

    // Configure clocks
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
    let mut led2 = gpioe
        .pe12
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

    let pa11 = gpioa.pa11;
    let pa12 = gpioa.pa12;
    let pa0 = gpioa.pa0.into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr);
    let usb = Peripheral {
        usb: dp.USB,
        pin_dm: pa11.into_af14(&mut gpioa.moder, &mut gpioa.afrh),
        pin_dp: pa12.into_af14(&mut gpioa.moder, &mut gpioa.afrh),
    };

    let usb_bus = UsbBus::new(usb);

    let mut midi = MidiClass::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x5e4))
        .product("usbd-midi example")
        .device_class(USB_CLASS_NONE)
        .serial_number("serial_number")
        .manufacturer("manufacturer")
        .build();

    let exti = dp.EXTI;
    cortex_m::interrupt::free(|cs| {
        MUTEX_BUTTON_PIN.borrow(cs).replace(Some(pa0));
        MUTEX_EXTI.borrow(cs).replace(Some(exti));
        let fifo = RingBuffer::<Message>::new(16);
        let (p, c) = fifo.split();
        FIFO_CONSUMER.borrow(cs).replace(Some(c));
        FIFO_PRODUCER.borrow(cs).replace(Some(p));
    });

    // Enable interrupt
    {
        let cortexm_peripherals = cortex_m::Peripherals::take().unwrap();
        let mut nvic = cortexm_peripherals.NVIC;
        nvic.enable(pac::Interrupt::EXTI0);
    }

    loop {
        cortex_m::interrupt::free(|cs| {
            let mut c = FIFO_CONSUMER.borrow(cs).borrow_mut();
            let consumer = c.as_mut().unwrap();
            loop {
                match consumer.pop() {
                    Some(message) => {
                        match message {
                            Message::NoteOn(_, _, _) => {
                                led2.set_high();
                            }
                            _ => {
                                led2.set_low();
                            }
                        }
                        midi.send_message(UsbMidiEventPacket::from_midi(CableNumber::Cable0, message));
                    }
                    _ => break,
                }
            }
        });

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
                            led.set_high().unwrap();
                        }
                        Message::NoteOff(Channel1, Note::C2, ..) => {
                            led.set_low().unwrap();
                        }
                        _ => {}
                    }
                }
            }
        }
    }
}
