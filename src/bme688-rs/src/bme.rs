use core::result::Result::{self, Err, Ok};
use esp_hal::i2c::Error;
use esp_hal::i2c::Error::ExecIncomplete;
use esp_hal::peripherals::I2C0;
use esp_hal::Async;
use esp_hal::{delay::Delay, i2c::I2C};
use log::debug;

use crate::{CalibrationData, DeviceConfig, GasHeaterConfig, OperationMode};

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
    pub variant_id: u8,
    pub amb_temp: i8,
    pub calib: CalibrationData,
    pub config: DeviceConfig,
    pub gas_heater_config: GasHeaterConfig,
    pub info_msg: u8,
}

#[allow(dead_code)]
impl<'a> Bme<'a> {
    fn new(i2c: I2C<'a, I2C0, Async>, delay: Delay) -> Self {
        Bme {
            i2c,
            delay,
            variant_id: 0,
            amb_temp: 0,
            calib: CalibrationData::default(),
            config: DeviceConfig::default(),
            info_msg: 0,
            gas_heater_config: GasHeaterConfig::default(),
        }
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
        if self.config.filter > 7 {
            self.config.filter = 7;
            self.info_msg |= 0x01;
        }

        if self.config.os_temp > 5 {
            self.config.os_temp = 5;
            self.info_msg |= 0x01;
        }

        if self.config.os_pres > 5 {
            self.config.os_pres = 5;
            self.info_msg |= 0x01;
        }

        if self.config.os_hum > 5 {
            self.config.os_hum = 5;
            self.info_msg |= 0x01;
        }

        if self.config.odr > 8 {
            self.config.odr = 8;
            self.info_msg |= 0x01;
        }
    }

    pub async fn set_config(&mut self, conf: DeviceConfig) -> Result<(), Error> {
        let mut odr20: u8 = 0;
        let mut odr3: u8 = 1;
        self.config = conf;
        let cur_op_mode = self.get_op_mode().await?;
        self.set_op_mode(OperationMode::Sleep).await?;
        let mut data = self.i2c_get_reg(0x71).await?;
        self.boundary_check();

        data[4] = (data[4] & !0x1C) | ((self.config.filter << 2) & 0x1C);
        data[3] = (data[3] & !0xE0) | ((self.config.os_temp << 5) & 0xE0);
        data[2] = (data[2] & !0x1C) | ((self.config.os_pres << 2) & 0x1C);
        data[1] = (data[1] & !0x07) | ((self.config.os_hum << 2) & 0x07);

        if self.config.odr != 8 {
            odr20 = self.config.odr;
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
        self.gas_heater_config = conf;
        self.set_op_mode(OperationMode::Sleep).await?;
        match op_mode {
            OperationMode::Forced => {
                rh_reg_addr[0] = 0x5a;
                rh_reg_data[0] = self.calc_res_heat(self.gas_heater_config.heatr_temp);
                gw_reg_addr[0] = 0x64;
                gw_reg_data[0] = self.calc_gas_wait(self.gas_heater_config.heatr_dur);
                nb_conv = 0;
                write_len = 1;
            }
            OperationMode::Parallel => {
                if self.gas_heater_config.shared_heatr_dur == 0 {
                    return Err(ExecIncomplete);
                }

                let mut count = 0 as usize;
                while count < self.gas_heater_config.profile_len as usize {
                    rh_reg_addr[count] = 0x5a + count as u8;
                    rh_reg_data[count] = self.calc_res_heat(unsafe {
                        *self.gas_heater_config.heatr_temp_prof.add(count)
                    });
                    gw_reg_addr[count] = 0x64 + count as u8;
                    gw_reg_data[count] =
                        unsafe { *self.gas_heater_config.heatr_dur_prof.add(count) as u8 };
                    count += 1;
                }
                nb_conv = self.gas_heater_config.profile_len;
                write_len = self.gas_heater_config.profile_len;
                let shared_dur =
                    self.calc_heatr_dur_shared(self.gas_heater_config.shared_heatr_dur);
                self.i2c_set_reg(0x6e, shared_dur).await?;
            }
            OperationMode::Sequential => {
                let mut count = 0 as usize;
                while count < self.gas_heater_config.profile_len as usize {
                    rh_reg_addr[count] = 0x5a;
                    rh_reg_data[count] = self.calc_res_heat(unsafe {
                        *self.gas_heater_config.heatr_temp_prof.add(count)
                    });
                    gw_reg_addr[count] = 0x64 + count as u8;
                    gw_reg_data[count] = self.calc_gas_wait(unsafe {
                        *self.gas_heater_config.heatr_dur_prof.add(count)
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

        if self.gas_heater_config.enable == 0x01 {
            hctrl = 0;
            if self.variant_id == 0x01 {
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

        let var1 = (self.calib.par_gh1 as f32 / 16.0) + 49.0;
        let var2 = (self.calib.par_gh2 as f32 / 32768.0 * 0.0005) + 49.0;
        let var3 = self.calib.par_gh3 as f32 / 1024.0;
        let var4 = var1 * (1.0 + var2 * (temp as f32));
        let var5 = var4 + var3 * (self.amb_temp as f32);
        let res_heat = (3.4
            * (var5
                * (4.0 / (4.0 + self.calib.res_heat_range as f32))
                * (1.0 / (1.0 + (self.calib.res_heat_val as f32) * 0.002))
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
        self.amb_temp = 25;
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

        self.calib.par_t1 = ((data_array[32] as u16) << 8) | (data_array[31] as u16);
        self.calib.par_t2 = (((data_array[1] as u16) << 8) | (data_array[0] as u16)) as i16;
        self.calib.par_t3 = data_array[2] as i8;
        self.calib.par_p1 = ((data_array[5] as u16) << 8) | (data_array[4] as u16);
        self.calib.par_p2 = (((data_array[7] as u16) << 8) | (data_array[6] as u16)) as i16;
        self.calib.par_p3 = data_array[8] as i8;
        self.calib.par_p4 = (((data_array[11] as u16) << 8) | (data_array[10] as u16)) as i16;
        self.calib.par_p5 = (((data_array[13] as u16) << 8) | (data_array[12] as u16)) as i16;
        self.calib.par_p6 = data_array[15] as i8;
        self.calib.par_p7 = data_array[14] as i8;
        self.calib.par_p8 = (((data_array[19] as u16) << 8) | (data_array[18] as u16)) as i16;
        self.calib.par_p9 = (((data_array[21] as u16) << 8) | (data_array[20] as u16)) as i16;
        self.calib.par_p10 = data_array[22];
        self.calib.par_h1 = ((data_array[25] as u16) << 4) | ((data_array[24] as u16) & 0xF);
        self.calib.par_h2 = ((data_array[23] as u16) << 4) | ((data_array[24] as u16) >> 4);
        self.calib.par_h3 = data_array[26] as i8;
        self.calib.par_h4 = data_array[27] as i8;
        self.calib.par_h5 = data_array[28] as i8;
        self.calib.par_h6 = data_array[29];
        self.calib.par_h7 = data_array[30] as i8;
        self.calib.par_gh1 = data_array[35] as i8;
        self.calib.par_gh2 = (((data_array[34] as u16) << 8) | (data_array[33] as u16)) as i16;
        self.calib.par_gh3 = data_array[36] as i8;
        self.calib.res_heat_range = (data_array[39] & 0x30) / 16;
        self.calib.res_heat_val = data_array[37] as i8;
        self.calib.range_sw_err = ((data_array[41] & 0xF0) as i8) / 16;
        Ok(())
    }

    async fn get_variant_id(&mut self) -> Result<u8, Error> {
        let read = self.i2c_get_reg(BME_GET_VARIANT_ADDR).await?;
        self.variant_id = read[0];
        Ok(read[0])
    }

    // need a 10ms delay after reset
    async fn soft_reset(&mut self) -> Result<(), Error> {
        let cmd = 0xb6;
        self.i2c_set_reg(BME_SOFT_RST_ADDR, cmd).await
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
