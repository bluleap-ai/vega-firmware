#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, delay::Delay, embassy, gpio::Io, i2c::I2C, peripherals::Peripherals,
    prelude::*, system::SystemControl, timer::timg::TimerGroup,
};

mod zmod4510;
use log::{error, info};
use zmod4510::{
    commands::Command,
    types::{Oaq2ndGenHandle, Oaq2ndGenInputs, Oaq2ndGenResults, ZmodDev},
    Zmod,
};

extern crate alloc;
use core::mem::MaybeUninit;

extern "C" {
    pub fn init_oaq_2nd_gen(oaq_handle: &mut Oaq2ndGenHandle) -> i8;
    pub fn calc_oaq_2nd_gen(oaq_handle: &mut Oaq2ndGenHandle, zmod_handle: &mut ZmodDev, algo_input: &Oaq2ndGenInputs, results: &Oaq2ndGenResults) -> i8;
}

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
async fn main(spawner: Spawner) -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0);

    let delay = Delay::new(&clocks);
    init_heap();

    esp_println::logger::init_logger(log::LevelFilter::Debug);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c0 = I2C::new_async(
        peripherals.I2C0,
        io.pins.gpio4,
        io.pins.gpio5,
        400.kHz(),
        &clocks,
    );

    let mut oaq_handle = Oaq2ndGenHandle {
        sample_cnt: 0,
        smooth_rmox: 0.0,
        gcda: 0.0,
        o3_conc_ppb: 0.0,
        o3_1h_ppb: 0.0,
        o3_8h_ppb: 0.0,
    };

    let mut oaq_inputs = Oaq2ndGenInputs{
        adc_result: [0;18],
        humidity_pct: 0.0,
        temperature_degc: 0.0,
    };

    unsafe {
        init_oaq_2nd_gen(&mut oaq_handle);
    }

    let mut zmod_sensor = Zmod::new(i2c0, delay).await;
    match zmod_sensor.read_info().await {
        true => {
            info!("read_info successfully!");
        }
        false => {
            panic!("Failed to read_info");
        }
    }

    let mut tracking_num: [u8; 6] = [0x00; 6];
    match zmod_sensor.read_tracking_number(&mut tracking_num).await {
        Ok(_) => info!("Read tracking number successfully!"),
        Err(e) => {
            panic!("Failed to read tracking number: {:?}", e);
        }
    }
    info!(
        "Sensor tracking numner: 0x0000{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}",
        tracking_num[0],
        tracking_num[1],
        tracking_num[2],
        tracking_num[3],
        tracking_num[4],
        tracking_num[5]
    );
    info!("Sensor trimming data: {:?}", zmod_sensor.prod_data);

    match zmod_sensor.init().await {
        Ok(_) => {
            info!("Init successfully!");
        }
        Err(e) => {
            panic!("Failed to Init ZMOD: {:?}", e);
        }
    }
    match zmod_sensor.init_meas().await {
        Ok(_) => {
            info!("Init measurement successfully!");
        }
        Err(e) => {
            panic!("Failed to Init measurement: {:?}", e);
        }
    }

    loop {
        match zmod_sensor.start_meas().await {
            Ok(_) => {
                info!("start measurement successfully!");
            }
            Err(e) => {
                panic!("Failed to start measurement: {:?}", e);
            }
        }
        delay.delay_millis(2000);

        let status = match zmod_sensor.read_status().await {
            Ok(ret) => ret,
            Err(_) => {
                panic!("I2C ERR: Failed to read status");
            }
        };

        if (Command::StatusSequencerRunningMask.as_byte() & status) != 0 {
            zmod_sensor.delay.delay_millis(50);
            continue;
        }
        let mut data = [0x00; 18];
        let mut rmox: [f32; 32] = [0.0; 32];
        let _ = zmod_sensor.read_adc(&mut data).await;

        let _ = zmod_sensor.calc_rmox(&data, &mut rmox).await;
        info!("ADC:    {:?}", data);
        info!("RMOX:   {:?}", rmox);

        oaq_inputs.adc_result = data;
        oaq_inputs.humidity_pct = 50.0;
        oaq_inputs.temperature_degc = 20.0;

        let prod = zmod_sensor.prod_data.clone();
        let init_cfg = zmod_sensor.init_conf.clone();
        let meas_cfg = zmod_sensor.meas_conf.clone();

        let mut dev = ZmodDev {
            i2c_addr: 0x33,
            config: zmod_sensor.config.clone(),
            mox_er: zmod_sensor.mox_er.clone(),
            mox_lr: zmod_sensor.mox_lr.clone(),
            pid: zmod_sensor.pid,
            prod_data: &prod,
            init_config: &init_cfg,
            meas_config: &meas_cfg,
            delay: &Zmod::delay_ms,
            read: &Zmod::read,
            write: &Zmod::write,
        };

        unsafe {
            let mut oaq_result = Oaq2ndGenResults {
                rmox: [0.0; 8],
                o3_conc_ppb: 0.0,
                fast_aqi: 0,
                epa_aqi: 0,
            };
            let aqi = calc_oaq_2nd_gen(&mut oaq_handle, &mut dev, &oaq_inputs, &mut oaq_result);
            info!("AQI: {}", aqi);
        }
    }
}
