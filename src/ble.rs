#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, delay::Delay, peripherals::Peripherals, rng::Rng, system::SystemControl,
    timer::timg::TimerGroup,
};

use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    asynch::Ble,
    att::Uuid,
};
use esp_wifi::{ble::controller::asynch::BleConnector, initialize, EspWifiInitFor};
use log::info;

extern crate alloc;
use core::mem::MaybeUninit;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

#[embassy_executor::task]
async fn run() {
    loop {
        esp_println::println!("Hello world from embassy using esp-hal-async!");
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();
    let delay = Delay::new(&clocks);
    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);

    init_heap();

    esp_println::logger::init_logger(log::LevelFilter::Info);
    let init = initialize(
        EspWifiInitFor::Ble,
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

    let mut bluetooth = peripherals.BT;

    let connector = BleConnector::new(&init, &mut bluetooth);
    let mut ble = Ble::new(connector, esp_wifi::current_millis);
    info!("Connector created");

    info!("{:?}", ble.init().await);
    info!("{:?}", ble.cmd_set_le_advertising_parameters().await);
    info!(
        "{:?}",
        ble.cmd_set_le_advertising_data(
            create_advertising_data(&[
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x1809)]),
                AdStructure::CompleteLocalName(esp_hal::chip!()),
            ])
            .unwrap()
        )
        .await
    );
    info!("{:?}", ble.cmd_set_le_advertise_enable(true).await);

    info!("started advertising");
    loop {
        delay.delay_millis(2000);
        info!("BLE task is running");
    }
}
