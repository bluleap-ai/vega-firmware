#![no_std]
#![no_main]
#[cfg(feature = "as7331")]
use as7331_rs::as7331::As7331;
use bleps::asynch::Ble;
#[cfg(any(feature = "bme688", feature = "zmod", feature = "as7331"))]
use bleps::Data;
#[cfg(feature = "bme688")]
use bme688_rs::{
    bme::Bme, BmeData, DeviceConfig, Filter, GasHeaterConfig, Odr, OperationMode, Sample,
};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, delay::Delay, peripherals::Peripherals, rng::Rng, system::SystemControl,
    timer::timg::TimerGroup,
};
#[cfg(any(feature = "as7331", feature = "bme688", feature = "zmod"))]
use esp_hal::{gpio::Io, i2c::I2C};
use esp_wifi::{ble::controller::asynch::BleConnector, initialize, EspWifiInitFor};
#[cfg(any(feature = "as7331", feature = "bme688", feature = "zmod"))]
use log::error;
use log::info;
#[cfg(feature = "zmod")]
use log::{debug, warn};
#[cfg(feature = "zmod")]
use zmod4510_rs::{
    commands::Command, zmod::Zmod, Oaq2ndGenHandle, Oaq2ndGenInputs, Oaq2ndGenResults, ZmodData,
    ZmodDev,
};

extern crate alloc;
#[cfg(feature = "zmod")]
use core::borrow::Borrow;
use core::mem::MaybeUninit;

#[cfg(feature = "zmod")]
extern "C" {
    pub fn init_oaq_2nd_gen(oaq_handle: *const Oaq2ndGenHandle) -> i8;
    pub fn calc_oaq_2nd_gen(
        oaq_handle: *const Oaq2ndGenHandle,
        zmod_handle: *const ZmodDev,
        algo_input: *const Oaq2ndGenInputs,
        results: *const Oaq2ndGenResults,
    ) -> i8;
}
#[cfg(feature = "zmod")]
unsafe extern "C" fn i2c_delay_ms(_t: u32) {
    warn!("i2c_delay_ms was called from Renesas lib");
}
#[cfg(feature = "zmod")]
unsafe extern "C" fn i2c_write(_addr: u8, _reg: u8, _data: *const u8, _len: u8) -> i8 {
    warn!("i2c_write was called from Renesas lib");
    0
}
#[cfg(feature = "zmod")]
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
    let clocks = ClockControl::max(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);

    init_heap();

    esp_println::logger::init_logger(log::LevelFilter::Debug);

    //==================================================================================//
    //                             Initialize for Bluetooth                             //
    //==================================================================================//
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
    #[cfg(any(feature = "bme688", feature = "zmod", feature = "as7331"))]
    let mut adv_data: [u8; 31] = [
        // Device name
        0x05, 0x09, b'V', b'E', b'G', b'A', // Manufacturing data
        0x18, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ];

    //==================================================================================//
    //                       Initialize for I2C Interface                               //
    //==================================================================================//
    let delay = Delay::new(&clocks);

    #[cfg(any(feature = "as7331", feature = "bme688", feature = "zmod"))]
    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    #[cfg(any(feature = "as7331", feature = "bme688", feature = "zmod"))]
    {
        // reset Vsen when powering up
        io.pins.gpio10.set_high();
        delay.delay_millis(500);
        io.pins.gpio10.set_low();
    }

    #[cfg(any(feature = "as7331", feature = "bme688", feature = "zmod"))]
    let mut i2c0 = I2C::new_async(
        peripherals.I2C0,
        io.pins.gpio6,
        io.pins.gpio7,
        400.kHz(),
        &clocks,
    );

    info!("Initialize the I2C OK");
    //==================================================================================//
    //                       Initialize for BME688 Sensor                               //
    //==================================================================================//
    #[cfg(feature = "bme688")]
    let mut bme_data: BmeData;
    #[cfg(feature = "bme688")]
    {
        info!("Start initializing for BME688 sensor");
        let mut bme_sensor = Bme::new(i2c0, delay);
        match bme_sensor.init().await {
            Ok(()) => info!("Initialize for BME sensor OK"),
            Err(e) => error!("Failed to init BME sensor {:?}", e),
        }
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
        (i2c0, bme_data) = bme_sensor.destroy();
    }

    //==================================================================================//
    //                       Initialize for AS7331 sensor                               //
    //==================================================================================//
    #[cfg(feature = "as7331")]
    {
        info!("Start initializing for AS7331 sensor");
        let mut as7331_sensor = As7331::new(i2c0, delay);

        let _ = as7331_sensor.power_up().await;
        let _ = as7331_sensor.reset().await;
        delay.delay_millis(100);
        let chip_id = as7331_sensor.get_chip_id().await.unwrap();

        if chip_id == 0x21 {
            match as7331_sensor.set_configuration_mode().await {
                Ok(()) => info!("Set configuration mode for AS7331 OK"),
                Err(e) => error!("I2C FAILED: {:?}", e),
            }
            match as7331_sensor.init(0, 0, 0x01, 40, 8, 9).await {
                Ok(()) => info!("Init for AS7331 OK"),
                Err(e) => error!("I2C FAILED: {:?}", e),
            }
            delay.delay_millis(100);
            match as7331_sensor.set_measurement_mode().await {
                Ok(()) => info!("set measurement mode for AS7331 OK"),
                Err(e) => error!("I2C FAILED: {:?}", e),
            }
        } else {
            error!("Wrong chip id: {}", chip_id);
        }
        delay.delay_millis(100);
        i2c0 = as7331_sensor.destroy();
    }

    //==================================================================================//
    //                       Initialize for ZMOD4510 sensor                             //
    //==================================================================================//
    #[cfg(feature = "zmod")]
    let mut zmod_data: ZmodData;
    #[cfg(feature = "zmod")]
    let oaq_handle = Oaq2ndGenHandle {
        sample_cnt: 0,
        smooth_rmox: 0.0,
        gcda: 0.0,
        o3_conc_ppb: 0.0,
        o3_1h_ppb: 0.0,
        o3_8h_ppb: 0.0,
    };

    #[cfg(feature = "zmod")]
    unsafe {
        init_oaq_2nd_gen(&oaq_handle);
    }
    #[cfg(feature = "zmod")]
    {
        info!("Start initializing for ZMOD4510 sensor");
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
        (i2c0, zmod_data) = zmod_sensor.destroy();
    }

    //==================================================================================//
    //                          MAIN LOOP FOR APPLICATION                               //
    //==================================================================================//
    loop {
        #[cfg(feature = "bme688")]
        {
            let mut bme_sensor = Bme::new_with_data(i2c0, delay, bme_data);
            let _ = bme_sensor.set_op_mode(OperationMode::Forced).await;
            let del_period = bme_sensor
                .get_measure_duration(OperationMode::Forced)
                .wrapping_add(300_u32 * 1000);
            bme_sensor.delay.delay_micros(del_period);

            // Get the sensor data
            let sensor_data = bme_sensor.get_data(OperationMode::Forced).await.unwrap();

            info!(
                    "BME688: temperature: {:.2}, pressure: {:.2}, humidity: {:.2}, gas_resistance: {:.2}, status: {:x}",
                    sensor_data.temperature,
                    sensor_data.pressure / 1000.0,
                    sensor_data.humidity,
                    sensor_data.gas_resistance,
                    sensor_data.status,
                );
            adv_data[8] = sensor_data.temperature as u8;
            adv_data[9] = sensor_data.humidity as u8;
            adv_data[10] = (sensor_data.pressure / 1000.0) as u8;
            adv_data[11..15].copy_from_slice(&sensor_data.gas_resistance.to_le_bytes());
            (i2c0, bme_data) = bme_sensor.destroy();
        }
        #[cfg(feature = "zmod")]
        {
            let mut zmod_sensor = Zmod::new_with_data(i2c0, delay, zmod_data);
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
                    adv_data[27] = ((oaq_result.o3_conc_ppb as u16 >> 8) & 0xff) as u8;
                    adv_data[28] = (oaq_result.o3_conc_ppb as u16 & 0xff) as u8;
                    adv_data[29] = ((oaq_result.fast_aqi >> 8) & 0xff) as u8;
                    adv_data[30] = (oaq_result.fast_aqi & 0xff) as u8;
                }
            }
            (i2c0, zmod_data) = zmod_sensor.destroy();
        }
        #[cfg(feature = "as7331")]
        {
            let lsb_a = 304.69 / ((1 << (11 - 8)) as f32) / ((1 << 9) as f32 / 1024.0) / 1000.0;
            let lsb_b = 398.44 / ((1 << (11 - 8)) as f32) / ((1 << 9) as f32 / 1024.0) / 1000.0;
            let lsb_c = 191.41 / ((1 << (11 - 8)) as f32) / ((1 << 9) as f32 / 1024.0) / 1000.0;
            let mut as7331_sensor = As7331::new(i2c0, delay);
            let status = as7331_sensor.get_status().await.unwrap();
            if (status & 0x0008) != 0 {
                let all_data = as7331_sensor.read_all_data().await.unwrap();
                let temp = all_data[0];
                let uv_a = all_data[1];
                let uv_b = all_data[2];
                let uv_c = all_data[3];
                let convert_uv_a = uv_a as f32 * lsb_a;
                let convert_uv_b = uv_b as f32 * lsb_b;
                let convert_uv_c = uv_c as f32 * lsb_c;
                info!("AS7331 UV DATA:");
                info!("AS7331 UVA: {:.2} (uW/cm^2)", convert_uv_a);
                info!("AS7331 UVB: {:.2} (uW/cm^2)", convert_uv_b);
                info!("AS7331 UVC: {:.2} (uW/cm^2)", convert_uv_c);
                info!(
                    "AS7331 Temperature: {:.2} (Celcius)",
                    temp as f32 * 0.05 - 66.9
                );
                adv_data[15..19].copy_from_slice(&convert_uv_a.to_le_bytes());
                adv_data[19..23].copy_from_slice(&convert_uv_b.to_le_bytes());
                adv_data[23..27].copy_from_slice(&convert_uv_c.to_le_bytes());
            } else {
                warn!("AS7331 status is not 0x0008: {}", status);
            }
            i2c0 = as7331_sensor.destroy();
        }
        delay.delay_millis(2000);
        #[cfg(any(feature = "bme688", feature = "zmod", feature = "as7331"))]
        {
            let mut data = Data::default();
            data.append(&[0]);

            for item in adv_data.iter() {
                data.append(&[*item]);
            }

            let len = data.len - 1;
            data.set(0, len as u8);

            if len > 31 {
                error!("ERROR: Advertise data is too long");
            }

            for _ in 0..(31 - len) {
                data.append(&[0]);
            }
            // create_advertising_data
            info!("{:?}", ble.cmd_set_le_advertising_data(data).await);
            info!("{:?}", ble.cmd_set_le_advertise_enable(true).await);
            info!("Advertising data {:?}", data);
            info!("started advertising");
        }
    }
}
