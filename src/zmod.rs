#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, delay::Delay, gpio::Io, i2c::I2C, peripherals::Peripherals, prelude::*,
    system::SystemControl, timer::timg::TimerGroup,
};

use log::{debug, info, warn};
use zmod4510_rs::{
    commands::Command, zmod::Zmod, Oaq2ndGenHandle, Oaq2ndGenInputs, Oaq2ndGenResults, ZmodDev,
};

extern crate alloc;
use core::{borrow::Borrow, mem::MaybeUninit};

extern "C" {
    pub fn init_oaq_2nd_gen(oaq_handle: *const Oaq2ndGenHandle) -> i8;
    pub fn calc_oaq_2nd_gen(
        oaq_handle: *const Oaq2ndGenHandle,
        zmod_handle: *const ZmodDev,
        algo_input: *const Oaq2ndGenInputs,
        results: *const Oaq2ndGenResults,
    ) -> i8;
}

unsafe extern "C" fn i2c_delay_ms(_t: u32) {
    warn!("i2c_delay_ms was called from Renesas lib");
}

unsafe extern "C" fn i2c_write(_addr: u8, _reg: u8, _data: *const u8, _len: u8) -> i8 {
    warn!("i2c_write was called from Renesas lib");
    0
}

unsafe extern "C" fn i2c_read(_addr: u8, _reg: u8, _data: *const u8, _len: u8) -> i8 {
    warn!("i2c_read was called from Renesas lib");
    0
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
#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timg0.timer0);

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

    let oaq_handle = Oaq2ndGenHandle {
        sample_cnt: 0,
        smooth_rmox: 0.0,
        gcda: 0.0,
        o3_conc_ppb: 0.0,
        o3_1h_ppb: 0.0,
        o3_8h_ppb: 0.0,
    };

    unsafe {
        init_oaq_2nd_gen(&oaq_handle);
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
    info!(
        "Sensor trimming data: {:?}",
        zmod_sensor.zmod_data.prod_data
    );

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
            panic!("Error during reading status register");
        }

        let mut data = [0x00; 18];
        let _ = zmod_sensor.read_adc(&mut data).await;
        info!("ADC:    {:?}", data);

        // Check validity of the ADC results.
        if !zmod_sensor.check_error_event().await {
            panic!("Error during reading status register");
        }

        let oaq_inputs = Oaq2ndGenInputs {
            adc_result: data.as_mut_ptr(),
            humidity_pct: 50.0,
            temperature_degc: 20.0,
        };

        let mut prod = zmod_sensor.zmod_data.prod_data;
        let init_cfg = zmod_sensor.zmod_data.init_conf.clone();
        let meas_cfg = zmod_sensor.zmod_data.meas_conf.clone();

        let dev = ZmodDev {
            i2c_addr: 0x33,
            config: zmod_sensor.zmod_data.config,
            mox_er: zmod_sensor.zmod_data.mox_er,
            mox_lr: zmod_sensor.zmod_data.mox_lr,
            pid: zmod_sensor.zmod_data.pid,
            prod_data: prod.as_mut_ptr(),
            init_config: init_cfg.borrow(),
            meas_config: meas_cfg.borrow(),
            delay: i2c_delay_ms,
            read: i2c_read,
            write: i2c_write,
        };

        unsafe {
            let oaq_result = Oaq2ndGenResults {
                rmox: [0.0; 8],
                o3_conc_ppb: 0.0,
                fast_aqi: 0,
                epa_aqi: 0,
            };
            let ret = calc_oaq_2nd_gen(&oaq_handle, &dev, &oaq_inputs, &oaq_result);
            debug!("OAQ HANDLE: {:?}", oaq_handle);
            if ret != 0 && ret != 1 {
                panic!("ERROR {} during calculateing algorithm, exit", ret);
            } else {
                info!("------------ Measurement result ------------");
                if ret == 0 {
                    info!("ZMOD4510: Valid data");
                } else {
                    info!("ZMOD4510: Warm up");
                }
                info!(" Rmox0 : {:.3} kOhm", oaq_result.rmox[0] / 1e3);
                info!(" Rmox1 : {:.3} kOhm", oaq_result.rmox[1] / 1e3);
                info!(" Rmox2 : {:.3} kOhm", oaq_result.rmox[2] / 1e3);
                info!(" Rmox3 : {:.3} kOhm", oaq_result.rmox[3] / 1e3);
                info!(" Rmox4 : {:.3} kOhm", oaq_result.rmox[4] / 1e3);
                info!(" Rmox5 : {:.3} kOhm", oaq_result.rmox[5] / 1e3);
                info!(" Rmox6 : {:.3} kOhm", oaq_result.rmox[6] / 1e3);
                info!(" Rmox7 : {:.3} kOhm", oaq_result.rmox[7] / 1e3);
                info!(" O3_conc_ppb = {:.3}", oaq_result.o3_conc_ppb);
                info!(" Fast AQI = {}", oaq_result.fast_aqi);
                info!(" EPA AQI = {}", oaq_result.epa_aqi);
            }
        }
    }
}
