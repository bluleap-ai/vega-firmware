#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, delay::Delay, gpio::Io, i2c::I2C, peripherals::Peripherals, prelude::*,
    system::SystemControl, timer::timg::TimerGroup,
};

use bme688_rs::{bme::Bme, *};
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

    {
        // reset Vsen when powering up
        io.pins.gpio10.set_high();
        delay.delay_millis(500);
        io.pins.gpio10.set_low();
    }

    let i2c0 = I2C::new_async(
        peripherals.I2C0,
        io.pins.gpio6,
        io.pins.gpio7,
        400.kHz(),
        &clocks,
    );

    let mut bme_sensor = Bme::new(i2c0, delay);
    let _ = bme_sensor.init().await;
    let _ = bme_sensor
        .set_config(
            DeviceConfig::default()
                .filter(Filter::Off)
                .odr(Odr::StandbyNone)
                .oversample_humidity(Sample::Once)
                .oversample_pressure(Sample::X16)
                .oversample_temperature(Sample::X2),
        )
        .await;

    // configure heater
    let _ = bme_sensor
        .set_gas_heater_conf(
            OperationMode::Forced,
            GasHeaterConfig::default()
                .enable()
                .heater_temp(300)
                .heater_duration(100),
        )
        .await;
    let mut sample_count = 0;
    loop {
        let _ = bme_sensor.set_op_mode(OperationMode::Forced).await;
        let del_period = bme_sensor
            .get_measure_duration(OperationMode::Forced)
            .wrapping_add(300_u32 * 1000);
        bme_sensor.delay.delay_micros(del_period);

        // Get the sensor data
        let sensor_data = bme_sensor.get_data(OperationMode::Forced).await.unwrap();

        info!(
            "Count: {} - temperature: {:.2}, pressure: {:.2}, humidity: {:.2}, gas_resistance: {:.2}, status: {:x}",
            sample_count,
            sensor_data.temperature,
            sensor_data.pressure,
            sensor_data.humidity,
            sensor_data.gas_resistance,
            sensor_data.status,
        );
        sample_count += 1;
    }
}
