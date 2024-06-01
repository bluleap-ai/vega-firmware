use core::result::Result::{self, Err, Ok};
use esp_hal::i2c::Error;
use esp_hal::i2c::Error::ExecIncomplete;
use esp_hal::peripherals::I2C0;
use esp_hal::Async;
use esp_hal::{delay::Delay, i2c::I2C};
use log::debug;
use log::error;

use crate::{BmeData, DeviceConfig, GasHeaterConfig, OperationMode, SensorData};

const BME_I2C_ADDRESS: u8 = 0x76;
const BME_SOFT_RST_ADDR: u8 = 0xe0;
const BME_GET_DEVICE_ADDR: u8 = 0xd0;
const BME_GET_VARIANT_ADDR: u8 = 0xf0;
const BME_REG_COEFF1: u8 = 0x8a;
const BME_REG_COEFF2: u8 = 0xe1;
const BME_REG_COEFF3: u8 = 0x00;

pub struct Bme<'a> {
    pub i2c: I2C<'a, I2C0, Async>,
    pub delay: Delay,
    pub bme_data: BmeData,
}

#[allow(dead_code)]
impl<'a> Bme<'a> {
    pub fn new(i2c: I2C<'a, I2C0, Async>, delay: Delay) -> Self {
        Bme {
            i2c,
            delay,
            bme_data: BmeData::default(),
        }
    }

    pub fn new_with_data(i2c: I2C<'a, I2C0, Async>, delay: Delay, bme_data: BmeData) -> Self {
        Bme {
            i2c,
            delay,
            bme_data,
        }
    }

    pub fn destroy(self) -> (I2C<'a, I2C0, Async>, BmeData) {
        (self.i2c, self.bme_data)
    }

    pub async fn get_op_mode(&mut self) -> Result<OperationMode, Error> {
        let op_mode = self.i2c_get_reg(0x74).await?;
        match op_mode[0] {
            0 => Ok(OperationMode::Sleep),
            1 => Ok(OperationMode::Forced),
            2 => Ok(OperationMode::Parallel),
            _ => Err(ExecIncomplete),
        }
    }

    pub fn get_measure_duration(&mut self, op_mode: OperationMode) -> u32 {
        let mut meas_dur;
        let mut meas_cycles;
        let os_to_meas_cycle: [u8; 6] = [0, 1, 2, 4, 8, 16];

        self.boundary_check();

        meas_cycles = os_to_meas_cycle[self.bme_data.config.os_temp as usize] as u32;
        meas_cycles += os_to_meas_cycle[self.bme_data.config.os_pres as usize] as u32;
        meas_cycles += os_to_meas_cycle[self.bme_data.config.os_hum as usize] as u32;

        meas_dur = meas_cycles * 1963;
        meas_dur += 477 * 4;
        meas_dur += 477 * 5;

        if op_mode != OperationMode::Parallel {
            meas_dur += 1000;
        }

        meas_dur
    }

    pub async fn get_data(&mut self, op_mode: OperationMode) -> Result<SensorData, Error> {
        let mut n_fields = 0u8;
        let mut data = [SensorData::default(); 3];
        self.get_raw_data(op_mode, &mut data, &mut n_fields).await?;
        Ok(data[0])
    }

    pub async fn get_raw_data(
        &mut self,
        op_mode: OperationMode,
        data: &mut [SensorData],
        n_data: &mut u8,
    ) -> Result<(), Error> {
        match op_mode {
            OperationMode::Sleep => {
                error!("Can't get raw data in sleep mode");
                return Err(ExecIncomplete);
            }
            OperationMode::Forced => {
                self.read_field_data(0, &mut data[0]).await?;
                *n_data = 1;
            }
            OperationMode::Parallel | OperationMode::Sequential => {
                self.read_all_field_data(data).await?;
                for i in 0..3 {
                    *n_data = 0;
                    if data[i].status & 0x80 != 0 {
                        *n_data += 1;
                    }
                }
            }
        }
        Ok(())
    }

    async fn read_field_data(&mut self, index: u8, data: &mut SensorData) -> Result<(), Error> {
        let read = self.i2c_get_reg(0x1d + 17 * index).await?;
        data.status = read[0] & 0x80;
        data.gas_index = read[0] & 0x0f;
        data.meas_index = read[1];
        let adc_pres =
            ((read[2] as u32) * 4096) | ((read[3] as u32) * 16) | ((read[4] as u32) / 16);
        let adc_temp =
            ((read[5] as u32) * 4096) | ((read[6] as u32) * 16) | ((read[7] as u32) / 16);
        let adc_hum = (((read[8] as u32) * 256) | (read[9] as u32)) as u16;
        let adc_gas_res_low = (((read[13] as u32) * 4) | ((read[14] as u32) / 64)) as u16;
        let adc_gas_res_high = (((read[15] as u32) * 4) | ((read[16] as u32) / 64)) as u16;
        let gas_range_l = read[14] & 0x0f;
        let gas_range_h = read[16] & 0x0f;

        if self.bme_data.variant_id == 0x01 {
            data.status |= read[16] & 0x20;
            data.status |= read[16] & 0x10;
        } else {
            data.status |= read[14] & 0x20;
            data.status |= read[14] & 0x10;
        }

        if data.status & 0x80 != 0 {
            let res_heat = self.i2c_get_reg(0x5a + data.gas_index).await?;
            data.res_heat = res_heat[0];
            let idac = self.i2c_get_reg(0x50 + data.gas_index).await?;
            data.idac = idac[0];
            let gas_wait = self.i2c_get_reg(0x64 + data.gas_index).await?;
            data.gas_wait = gas_wait[0];
            data.temperature = self.calc_temperature(adc_temp);
            data.pressure = self.calc_pressure(adc_pres);
            data.humidity = self.calc_humidity(adc_hum);

            if self.bme_data.variant_id == 0x01 {
                data.gas_resistance = self.calc_gas_resistance_high(adc_gas_res_high, gas_range_h);
            } else {
                data.gas_resistance = self.calc_gas_resistance_low(adc_gas_res_low, gas_range_l);
            }
        }

        Ok(())
    }

    async fn read_all_field_data(&mut self, data: &mut [SensorData]) -> Result<(), Error> {
        let buff = self.i2c_get_reg(0x1d).await?;
        let set_val = self.i2c_get_reg(0x50).await?;
        let mut i = 0 as usize;
        while i < 3 {
            let off = (i * 17) as u8;
            (data[i]).status = (buff[off as usize] as i32 & 0x80 as i32) as u8;
            (data[i]).gas_index = (buff[off as usize] as i32 & 0xf as i32) as u8;
            (data[i]).meas_index = buff[(off as i32 + 1 as i32) as usize];
            let adc_pres = (buff[(off as i32 + 2 as i32) as usize] as u32)
                .wrapping_mul(4096 as i32 as u32)
                | (buff[(off as i32 + 3 as i32) as usize] as u32).wrapping_mul(16 as i32 as u32)
                | (buff[(off as i32 + 4 as i32) as usize] as u32).wrapping_div(16 as i32 as u32);
            let adc_temp = (buff[(off as i32 + 5 as i32) as usize] as u32)
                .wrapping_mul(4096 as i32 as u32)
                | (buff[(off as i32 + 6 as i32) as usize] as u32).wrapping_mul(16 as i32 as u32)
                | (buff[(off as i32 + 7 as i32) as usize] as u32).wrapping_div(16 as i32 as u32);
            let adc_hum = ((buff[(off as i32 + 8 as i32) as usize] as u32)
                .wrapping_mul(256 as i32 as u32)
                | buff[(off as i32 + 9 as i32) as usize] as u32) as u16;
            let adc_gas_res_low = ((buff[(off as i32 + 13 as i32) as usize] as u32)
                .wrapping_mul(4 as i32 as u32)
                | (buff[(off as i32 + 14 as i32) as usize] as u32).wrapping_div(64 as i32 as u32))
                as u16;
            let adc_gas_res_high = ((buff[(off as i32 + 15 as i32) as usize] as u32)
                .wrapping_mul(4 as i32 as u32)
                | (buff[(off as i32 + 16 as i32) as usize] as u32).wrapping_div(64 as i32 as u32))
                as u16;
            let gas_range_l = (buff[(off as i32 + 14 as i32) as usize] as i32 & 0xf as i32) as u8;
            let gas_range_h = (buff[(off as i32 + 16 as i32) as usize] as i32 & 0xf as i32) as u8;
            if self.bme_data.variant_id == 0x01 {
                data[i].status |= buff[off as usize + 16] & 0x20;
                data[i].status |= buff[off as usize + 16] & 0x10;
            } else {
                data[i].status |= buff[off as usize + 14] & 0x20;
                data[i].status |= buff[off as usize + 14] & 0x10;
            }

            data[i].idac = set_val[data[i].gas_index as usize];
            data[i].res_heat = set_val[10 + data[i].gas_index as usize];
            data[i].gas_wait = set_val[20 + data[i].gas_index as usize];
            data[i].temperature = self.calc_temperature(adc_temp);
            data[i].pressure = self.calc_pressure(adc_pres);
            data[i].humidity = self.calc_humidity(adc_hum);
            if self.bme_data.variant_id == 0x01 {
                data[i].gas_resistance =
                    self.calc_gas_resistance_high(adc_gas_res_high, gas_range_h);
            } else {
                data[i].gas_resistance =
                    self.calc_gas_resistance_high(adc_gas_res_low, gas_range_l);
            }
            i = i.wrapping_add(1);
        }
        Ok(())
    }

    fn calc_gas_resistance_high(&mut self, gas_res_adc: u16, gas_range: u8) -> f32 {
        let var1 = (262144 >> gas_range) as u32;
        let mut var2: i32 = gas_res_adc as i32 - 512;
        var2 *= 3;
        var2 = 4096 + var2;
        1000000.0 * var1 as f32 / var2 as f32
    }

    fn calc_gas_resistance_low(&mut self, gas_res_adc: u16, gas_range: u8) -> f32 {
        let gas_res_f: f32 = gas_res_adc as f32;
        let gas_range_f: f32 = ((1 as u32) << gas_range as i32) as f32;
        let lookup_k1_range: [f32; 16] = [
            0.0f32, 0.0f32, 0.0f32, 0.0f32, 0.0f32, -1.0f32, 0.0f32, -0.8f32, 0.0f32, 0.0f32,
            -0.2f32, -0.5f32, 0.0f32, -1.0f32, 0.0f32, 0.0f32,
        ];
        let lookup_k2_range: [f32; 16] = [
            0.0f32, 0.0f32, 0.0f32, 0.0f32, 0.1f32, 0.7f32, 0.0f32, -0.8f32, -0.1f32, 0.0f32,
            0.0f32, 0.0f32, 0.0f32, 0.0f32, 0.0f32, 0.0f32,
        ];
        let var1 = 1340.0f32 + 5.0f32 * self.bme_data.calib.range_sw_err as i32 as f32;
        let var2 = var1 * (1.0f32 + lookup_k1_range[gas_range as usize] / 100.0f32);
        let var3 = 1.0f32 + lookup_k2_range[gas_range as usize] / 100.0f32;
        1.0f32 / (var3 * 0.000000125f32 * gas_range_f * ((gas_res_f - 512.0f32) / var2 + 1.0f32))
    }

    fn calc_temperature(&mut self, temp_adc: u32) -> f32 {
        let var1 = ((temp_adc as f32 / 16384.0) - (self.bme_data.calib.par_t1 as f32 / 1024.0))
            * (self.bme_data.calib.par_t2 as f32);
        let var2 = ((temp_adc as f32 / 131072.0) - (self.bme_data.calib.par_t1 as f32 / 8192.0))
            * ((temp_adc as f32 / 131072.0) - (self.bme_data.calib.par_t1 as f32 / 8192.0))
            * (self.bme_data.calib.par_t3 as f32 * 16.0);
        self.bme_data.calib.t_fine = var1 + var2;
        self.bme_data.calib.t_fine / 5120.0
    }

    fn calc_pressure(&mut self, pres_adc: u32) -> f32 {
        let mut var1 = (self.bme_data.calib.t_fine / 2.0) - 64000.0;
        let mut var2 = var1 * var1 * ((self.bme_data.calib.par_p6 as f32) / 131072.0);
        var2 = var2 + (var1 * (self.bme_data.calib.par_p5 as f32) * 2.0);
        var2 = (var2 / 4.0) + ((self.bme_data.calib.par_p4 as f32) * 65536.0);
        var1 = ((((self.bme_data.calib.par_p3 as f32) * var1 * var1) / 16384.0)
            + ((self.bme_data.calib.par_p2 as f32) * var1))
            / 524288.0;
        var1 = (1.0 + (var1 / 32768.0)) * (self.bme_data.calib.par_p1 as f32);
        let mut calc_pres = 1048576.0 - (pres_adc as f32);

        if var1 != 0.0 {
            calc_pres = ((calc_pres - (var2 / 4096.0)) * 6250.0) / var1;
            var1 = ((self.bme_data.calib.par_p9 as f32) * calc_pres * calc_pres) / 2147483648.0;
            var2 = calc_pres * ((self.bme_data.calib.par_p8 as f32) / 32768.0);
            let var3 = (calc_pres / 256.0)
                * (calc_pres / 256.0)
                * (calc_pres / 256.0)
                * ((self.bme_data.calib.par_p10 as f32) / 131072.0);
            calc_pres + (var1 + var2 + var3 + ((self.bme_data.calib.par_p7 as f32) / 128.0)) / 16.0
        } else {
            0.0
        }
    }

    fn calc_humidity(&mut self, hum_adc: u16) -> f32 {
        let temp_comp = self.bme_data.calib.t_fine / 5120.0;
        let var1 = (hum_adc as f32)
            - ((self.bme_data.calib.par_h1 as f32 * 16.0)
                + (self.bme_data.calib.par_p3 as f32 / 2.0 * temp_comp));
        let var2 = var1
            * ((self.bme_data.calib.par_h2 as f32 / 262144.0)
                * (1.0
                    + (self.bme_data.calib.par_h4 as f32 / 16384.0 * temp_comp)
                    + (self.bme_data.calib.par_h5 as f32 / 1048576.0 * temp_comp * temp_comp)));
        let var3 = self.bme_data.calib.par_h6 as f32 / 16384.0;
        let var4 = self.bme_data.calib.par_h7 as f32 / 2097152.0;

        let calc_hum = var2 + (var3 + var4 * temp_comp) * var2 * var2;
        if calc_hum > 100.0 {
            100.0
        } else if calc_hum < 0.0 {
            0.0
        } else {
            calc_hum
        }
    }

    pub async fn set_op_mode(&mut self, op_mode: OperationMode) -> Result<(), Error> {
        let mut tmp_pow_mode;
        loop {
            let data = self.i2c_get_reg(0x74).await?;
            tmp_pow_mode = data[0];
            let pow_mode = tmp_pow_mode & 0x03;
            if pow_mode != 0 {
                tmp_pow_mode = tmp_pow_mode & !0x03;
                self.i2c_set_reg(0x74, tmp_pow_mode).await?;
                self.delay.delay_millis(10);
            } else {
                break;
            }
        }

        if op_mode != OperationMode::Sleep {
            tmp_pow_mode = (tmp_pow_mode & !0x03) | ((op_mode as u8) & 0x03);
            self.i2c_set_reg(0x74, tmp_pow_mode).await?;
        }
        Ok(())
    }

    pub fn boundary_check(&mut self) {
        if self.bme_data.config.filter > 7 {
            self.bme_data.config.filter = 7;
            self.bme_data.info_msg |= 0x01;
        }

        if self.bme_data.config.os_temp > 5 {
            self.bme_data.config.os_temp = 5;
            self.bme_data.info_msg |= 0x01;
        }

        if self.bme_data.config.os_pres > 5 {
            self.bme_data.config.os_pres = 5;
            self.bme_data.info_msg |= 0x01;
        }

        if self.bme_data.config.os_hum > 5 {
            self.bme_data.config.os_hum = 5;
            self.bme_data.info_msg |= 0x01;
        }

        if self.bme_data.config.odr > 8 {
            self.bme_data.config.odr = 8;
            self.bme_data.info_msg |= 0x01;
        }
    }

    pub async fn set_config(&mut self, conf: DeviceConfig) -> Result<(), Error> {
        let mut odr20: u8 = 0;
        let mut odr3: u8 = 1;
        self.bme_data.config = conf;
        let cur_op_mode = self.get_op_mode().await?;
        self.set_op_mode(OperationMode::Sleep).await?;
        let mut data: [u8; 32] = self.i2c_get_reg(0x71).await?;

        self.boundary_check();

        data[4] = (data[4] & !0x1C) | ((self.bme_data.config.filter << 2) & 0x1C);
        data[3] = (data[3] & !0xE0) | ((self.bme_data.config.os_temp << 5) & 0xE0);
        data[3] = (data[3] & !0x1C) | ((self.bme_data.config.os_pres << 2) & 0x1C);
        data[1] = (data[1] & !0x07) | ((self.bme_data.config.os_hum << 2) & 0x07);

        if self.bme_data.config.odr != 8 {
            odr20 = self.bme_data.config.odr;
            odr3 = 0;
        }

        data[4] = (data[4] & !0xE0) | ((odr20 << 5) & 0xE0);
        data[0] = (data[0] & !0x80) | ((odr3 << 7) & 0x80);

        self.i2c_set_reg(0x71, data[0]).await?;
        self.i2c_set_reg(0x72, data[1]).await?;
        self.i2c_set_reg(0x73, data[2]).await?;
        self.i2c_set_reg(0x74, data[3]).await?;
        self.i2c_set_reg(0x75, data[4]).await?;

        if cur_op_mode != OperationMode::Sleep {
            self.set_op_mode(cur_op_mode).await?;
        }
        Ok(())
    }

    pub async fn set_gas_heater_conf(
        &mut self,
        op_mode: OperationMode,
        conf: GasHeaterConfig,
    ) -> Result<(), Error> {
        let mut rh_reg_addr = [0u8; 10];
        let mut rh_reg_data = [0u8; 10];
        let mut gw_reg_addr = [0u8; 10];
        let mut gw_reg_data = [0u8; 10];
        let hctrl: u8;
        let run_gas: u8;
        let mut nb_conv = 0u8;
        let mut write_len = 0u8;
        self.bme_data.gas_heater_config = conf;
        self.set_op_mode(OperationMode::Sleep).await?;
        match op_mode {
            OperationMode::Forced => {
                rh_reg_addr[0] = 0x5a;
                rh_reg_data[0] = self.calc_res_heat(self.bme_data.gas_heater_config.heatr_temp);
                gw_reg_addr[0] = 0x64;
                gw_reg_data[0] = self.calc_gas_wait(self.bme_data.gas_heater_config.heatr_dur);
                nb_conv = 0;
                write_len = 1;
            }
            OperationMode::Parallel => {
                if self.bme_data.gas_heater_config.shared_heatr_dur == 0 {
                    return Err(ExecIncomplete);
                }

                let mut count = 0 as usize;
                while count < self.bme_data.gas_heater_config.profile_len as usize {
                    rh_reg_addr[count] = 0x5a + count as u8;
                    rh_reg_data[count] = self.calc_res_heat(unsafe {
                        *self.bme_data.gas_heater_config.heatr_temp_prof.add(count)
                    });
                    gw_reg_addr[count] = 0x64 + count as u8;
                    gw_reg_data[count] =
                        unsafe { *self.bme_data.gas_heater_config.heatr_dur_prof.add(count) as u8 };
                    count += 1;
                }
                nb_conv = self.bme_data.gas_heater_config.profile_len;
                write_len = self.bme_data.gas_heater_config.profile_len;
                let shared_dur =
                    self.calc_heatr_dur_shared(self.bme_data.gas_heater_config.shared_heatr_dur);
                self.i2c_set_reg(0x6e, shared_dur).await?;
            }
            OperationMode::Sequential => {
                let mut count = 0 as usize;
                while count < self.bme_data.gas_heater_config.profile_len as usize {
                    rh_reg_addr[count] = 0x5a;
                    rh_reg_data[count] = self.calc_res_heat(unsafe {
                        *self.bme_data.gas_heater_config.heatr_temp_prof.add(count)
                    });
                    gw_reg_addr[count] = 0x64 + count as u8;
                    gw_reg_data[count] = self.calc_gas_wait(unsafe {
                        *self.bme_data.gas_heater_config.heatr_dur_prof.add(count)
                    });
                    count += 1;
                }
            }
            OperationMode::Sleep => return Err(ExecIncomplete),
        }
        for i in 0..write_len {
            self.i2c_set_reg(rh_reg_addr[i as usize], rh_reg_data[i as usize])
                .await?;
            self.i2c_set_reg(gw_reg_addr[i as usize], gw_reg_data[i as usize])
                .await?;
        }

        let mut ctrl_gas_data = self.i2c_get_reg(0x70).await?;

        if self.bme_data.gas_heater_config.enable == 0x01 {
            hctrl = 0;
            if self.bme_data.variant_id == 0x01 {
                run_gas = 0x02;
            } else {
                run_gas = 0x01;
            }
        } else {
            hctrl = 1;
            run_gas = 0;
        }

        ctrl_gas_data[0] = ((ctrl_gas_data[0] & !0x08) | ((hctrl << 3) & 0x08)) as u8;
        ctrl_gas_data[1] = ((ctrl_gas_data[1] & !0x0f) | (nb_conv & 0x0f)) as u8;
        ctrl_gas_data[1] = ((ctrl_gas_data[1] & !0x30) | ((run_gas << 4) & 0x30)) as u8;

        self.i2c_set_reg(0x70, ctrl_gas_data[0]).await?;
        self.i2c_set_reg(0x71, ctrl_gas_data[1]).await?;

        Ok(())
    }

    pub fn calc_res_heat(&mut self, mut temp: u16) -> u8 {
        if temp > 400 {
            temp = 400;
        }

        let var1 = (self.bme_data.calib.par_gh1 as f32 / 16.0) + 49.0;
        let var2 = (self.bme_data.calib.par_gh2 as f32 / 32768.0 * 0.0005) + 0.00235;
        let var3 = self.bme_data.calib.par_gh3 as f32 / 1024.0;
        let var4 = var1 * (1.0 + var2 * (temp as f32));
        let var5 = var4 + var3 * (self.bme_data.amb_temp as f32);
        let res_heat = (3.4
            * (var5
                * (4.0 / (4.0 + self.bme_data.calib.res_heat_range as f32))
                * (1.0 / (1.0 + (self.bme_data.calib.res_heat_val as f32) * 0.002))
                - 25.0)) as u8;
        res_heat
    }

    pub fn calc_gas_wait(&mut self, mut dur: u16) -> u8 {
        let mut factor = 0u8;
        let durval;

        if dur > 0x0FC0 {
            durval = 0xFF;
        } else {
            while dur > 0x3f {
                dur /= 4;
                factor += 1;
            }
            durval = (dur as u8 + factor * 64) as u8;
        }
        durval
    }

    pub fn calc_heatr_dur_shared(&mut self, mut dur: u16) -> u8 {
        if dur > 0x0783 {
            0xff
        } else {
            dur = ((dur as u32) * 1000 / 477) as u16;
            let mut factor = 0u8;
            while dur < 0x3f {
                dur = dur >> 2;
                factor = factor + 1;
            }
            ((dur as u8) + factor * 64) as u8
        }
    }

    pub async fn init(&mut self) -> Result<(), Error> {
        self.soft_reset().await?;
        let read = self.i2c_get_reg(BME_GET_DEVICE_ADDR).await?;

        if read[0] == 0x61 {
            self.get_variant_id().await?;
            self.get_calib_data().await?;
        } else {
            return Err(ExecIncomplete);
        }
        Ok(())
    }

    async fn get_calib_data(&mut self) -> Result<(), Error> {
        let mut data_array = [0u8; 42];

        let read = self.i2c_get_reg(BME_REG_COEFF1).await?;
        data_array[0..(23)].copy_from_slice(&read[..(23)]);

        let read = self.i2c_get_reg(BME_REG_COEFF2).await?;
        data_array[23..(37)].copy_from_slice(&read[..(14)]);

        let read = self.i2c_get_reg(BME_REG_COEFF3).await?;
        data_array[37..].copy_from_slice(&read[..(5)]);

        self.bme_data.calib.par_t1 = ((data_array[32] as u16) << 8) | (data_array[31] as u16);
        self.bme_data.calib.par_t2 =
            (((data_array[1] as u16) << 8) | (data_array[0] as u16)) as i16;
        self.bme_data.calib.par_t3 = data_array[2] as i8;
        self.bme_data.calib.par_p1 = ((data_array[5] as u16) << 8) | (data_array[4] as u16);
        self.bme_data.calib.par_p2 =
            (((data_array[7] as u16) << 8) | (data_array[6] as u16)) as i16;
        self.bme_data.calib.par_p3 = data_array[8] as i8;
        self.bme_data.calib.par_p4 =
            (((data_array[11] as u16) << 8) | (data_array[10] as u16)) as i16;
        self.bme_data.calib.par_p5 =
            (((data_array[13] as u16) << 8) | (data_array[12] as u16)) as i16;
        self.bme_data.calib.par_p6 = data_array[15] as i8;
        self.bme_data.calib.par_p7 = data_array[14] as i8;
        self.bme_data.calib.par_p8 =
            (((data_array[19] as u16) << 8) | (data_array[18] as u16)) as i16;
        self.bme_data.calib.par_p9 =
            (((data_array[21] as u16) << 8) | (data_array[20] as u16)) as i16;
        self.bme_data.calib.par_p10 = data_array[22];
        self.bme_data.calib.par_h1 =
            ((data_array[25] as u16) << 4) | ((data_array[24] as u16) & 0xF);
        self.bme_data.calib.par_h2 =
            ((data_array[23] as u16) << 4) | ((data_array[24] as u16) >> 4);
        self.bme_data.calib.par_h3 = data_array[26] as i8;
        self.bme_data.calib.par_h4 = data_array[27] as i8;
        self.bme_data.calib.par_h5 = data_array[28] as i8;
        self.bme_data.calib.par_h6 = data_array[29];
        self.bme_data.calib.par_h7 = data_array[30] as i8;
        self.bme_data.calib.par_gh1 = data_array[35] as i8;
        self.bme_data.calib.par_gh2 =
            (((data_array[34] as u16) << 8) | (data_array[33] as u16)) as i16;
        self.bme_data.calib.par_gh3 = data_array[36] as i8;
        self.bme_data.calib.res_heat_range = (data_array[39] & 0x30) / 16;
        self.bme_data.calib.res_heat_val = data_array[37] as i8;
        self.bme_data.calib.range_sw_err = ((data_array[41] & 0xF0) as i8) / 16;
        Ok(())
    }

    async fn get_variant_id(&mut self) -> Result<u8, Error> {
        let read = self.i2c_get_reg(BME_GET_VARIANT_ADDR).await?;
        self.bme_data.variant_id = read[0];
        Ok(read[0])
    }

    // need a 10ms delay after reset
    async fn soft_reset(&mut self) -> Result<(), Error> {
        let cmd = 0xb6;
        self.i2c_set_reg(BME_SOFT_RST_ADDR, cmd).await?;
        self.delay.delay_millis(10);
        Ok(())
    }

    async fn i2c_set_reg(&mut self, reg: u8, cmd: u8) -> Result<(), Error> {
        match self.i2c.write(BME_I2C_ADDRESS, &[reg, cmd]).await {
            Ok(_) => debug!("I2C_WRITE - ADDR: 0x{:02X} - DATA: 0x{:02X}", reg, cmd),
            Err(e) => return Err(e),
        }
        Ok(())
    }

    async fn i2c_get_reg(&mut self, reg: u8) -> Result<[u8; 32], Error> {
        let mut read_data = [0u8; 32];
        match self
            .i2c
            .write_read(BME_I2C_ADDRESS, &[reg], &mut read_data)
            .await
        {
            Ok(_) => debug!("I2C_READ - ADDR: 0x{:02X} - DATA: {:?}", reg, read_data),
            Err(e) => return Err(e),
        }
        Ok(read_data)
    }
}
