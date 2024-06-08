#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use as7331_rs::as7331::As7331;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, delay::Delay, gpio::Io, i2c::I2C, peripherals::Peripherals, prelude::*,
    system::SystemControl, timer::timg::TimerGroup,
};

use log::{error, info};

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

#[main]
async fn main(_spawner: Spawner) -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timer_group0);

    let delay = Delay::new(&clocks);
    init_heap();

    esp_println::logger::init_logger(log::LevelFilter::Info);

    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    io.pins.gpio3.set_high();
    delay.delay_millis(500);
    io.pins.gpio3.set_low();

    let i2c0 = I2C::new_async(
        peripherals.I2C0,
        io.pins.gpio4,
        io.pins.gpio5,
        400.kHz(),
        &clocks,
    );
    let lsb_a = 304.69 / ((1 << (11 - 8)) as f32) / ((1 << 9) as f32 / 1024.0) / 1000.0;
    let lsb_b = 398.44 / ((1 << (11 - 8)) as f32) / ((1 << 9) as f32 / 1024.0) / 1000.0;
    let lsb_c = 191.41 / ((1 << (11 - 8)) as f32) / ((1 << 9) as f32 / 1024.0) / 1000.0;
    let mut as7331_sensor = As7331::new(i2c0, delay);

    let _ = as7331_sensor.power_up().await;
    let _ = as7331_sensor.reset().await;
    delay.delay_millis(100);
    let chip_id = as7331_sensor.get_chip_id().await.unwrap();

    if chip_id == 0x21 {
        let _ = as7331_sensor.set_configuration_mode().await;
        let _ = as7331_sensor.init(0, 0, 0x01, 40, 8, 9).await;
        delay.delay_millis(100);
        let _ = as7331_sensor.set_measurement_mode().await;
    } else {
        error!("Wrong chip id: {}", chip_id);
    }

    loop {
        let status = as7331_sensor.get_status().await.unwrap();
        if (status & 0x0008) != 0 {
            let all_data = as7331_sensor.read_all_data().await.unwrap();
            let temp = all_data[0];
            let uv_a = all_data[1];
            let uv_b = all_data[2];
            let uv_c = all_data[3];

            info!("AS7331 UV DATA:");
            info!("AS7331 UVA: {:.2} (uW/cm^2)", uv_a as f32 * lsb_a);
            info!("AS7331 UVB: {:.2} (uW/cm^2)", uv_b as f32 * lsb_b);
            info!("AS7331 UVC: {:.2} (uW/cm^2)", uv_c as f32 * lsb_c);
            info!(
                "AS7331 Temperature: {:.2} (Celcius)",
                temp as f32 * 0.05 - 66.9
            );
        }
    }
}
