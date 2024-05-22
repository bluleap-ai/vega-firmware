use esp_hal::i2c::Error;
use esp_hal::i2c::Error::ExecIncomplete;
use esp_hal::peripherals::I2C0;
use esp_hal::Async;
use esp_hal::{delay::Delay, i2c::I2C};
use log::{debug, error};
use core::result::Result::{self, Ok, Err};

use super::commands::{self, Command};
use super::types::*;

const ZMOD_I2C_ADDRESS: u8 = 0x33;
const ZMOD4510_PID: u16 = 0x6320;

pub struct Zmod<'a> {
    pub i2c: I2C<'a, I2C0, Async>,
    pub delay: Delay,
    pub config: [u8; 6],
    pub prod_data: [u8; 10],
    pub mox_er: u16,
    pub mox_lr: u16,
    pub pid: u16,
    pub init_conf: ZmodConf,
    pub meas_conf: ZmodConf,
}

impl<'a> Zmod<'a> {
    pub async fn new(i2c: I2C<'a, I2C0, Async>, delay: Delay) -> Self {
        Zmod {
            i2c,
            delay,
            config: [0; 6],
            mox_er: 0,
            mox_lr: 0,
            prod_data: [0; 10],
            pid: ZMOD4510_PID,
            init_conf: ZmodConf::default(),
            meas_conf: ZmodConf::default(),
        }
    }

    pub async fn read_tracking_number(&mut self, track_num: &mut [u8]) -> Result<(), Error> {
        self.i2c
            .write_read(
                ZMOD_I2C_ADDRESS,
                &[Command::ZmodAddrTracking.as_byte()],
                track_num,
            )
            .await
    }

    pub async fn read_info(&mut self) -> bool {
        debug!("Start read information of ZMOD sensor");
        // wait for sensor ready
        let mut status: u8 = 0x80;
        let mut counter: u32 = 0;
        while counter < 1000 && (status & 0x80 != 0) {
            match self
                .i2c
                .write(ZMOD_I2C_ADDRESS, &[Command::ZmodAddrCmd.as_byte()])
                .await
            {
                Ok(_) => debug!("Send ZmodAddrCmd OK"),
                Err(e) => {
                    error!("FAILED I2C: {:?}", e);
                    counter += 1;
                    continue;
                }
            }

            match self.read_status().await {
                Ok(ret) => {
                    status = ret;
                    debug!("STATUS: {}", ret)
                }
                Err(e) => error!("Failed to read status, {:?}", e),
            };
            self.delay.delay_millis(200);
            counter += 1;
        }

        if counter >= 1000 {
            return false;
        }

        let mut data = [0u8; 2];

        match self
            .i2c
            .write_read(
                ZMOD_I2C_ADDRESS,
                &[Command::ZmodAddrPid.as_byte()],
                &mut data,
            )
            .await
        {
            Ok(_) => debug!("Send ZmodAddrPid OK"),
            Err(e) => {
                error!("FAILED I2C: {:?}", e);
                return false;
            }
        }
        let pro_id = (data[0] as u16) << 8 | data[1] as u16;
        if self.pid != pro_id {
            error!("Unsupported sensor PID: {}", pro_id);
            return false;
        }

        match self
            .i2c
            .write_read(
                ZMOD_I2C_ADDRESS,
                &[Command::ZmodAddrGeneralPupose.as_byte()],
                &mut self.prod_data,
            )
            .await
        {
            Ok(_) => debug!("Send ZmodAddrGeneralPupose OK"),
            Err(e) => {
                error!("FAILED I2C: {:?}", e);
                return false;
            }
        }

        match self
            .i2c
            .write_read(
                ZMOD_I2C_ADDRESS,
                &[Command::ZmodAddrConf.as_byte()],
                &mut self.config,
            )
            .await
        {
            Ok(_) => debug!("Send ZmodAddrConf OK"),
            Err(e) => {
                error!("FAILED I2C: {:?}", e);
                return false;
            }
        }

        match self.pid {
            ZMOD4510_PID => {
                self.meas_conf = ZmodConf::zmod4510_measurement();
                self.init_conf = ZmodConf::zmod4510_init();
            }
            _ => {
                error!("Unsupported PID {}", self.pid);
                return false;
            }
        }

        true
    }

    pub async fn init(&mut self) -> Result<(), Error> {
        debug!("Inititalize ZMOD sensor");
        let mut data_r = [0u8; 32];
        let mut hsp: [u8; 16] = [0; 16];
        match self
            .i2c
            .write_read(ZMOD_I2C_ADDRESS, &[0xB7], &mut data_r)
            .await
        {
            Ok(_) => debug!("Send 0xB7 OK"),
            Err(e) => {
                return Err(e);
            }
        }

        if self
            .calc_factor(self.init_conf.clone(), &mut hsp)
            .await
            .is_err()
        {
            error!("Failed to calculate factor value");
            return Err(ExecIncomplete);
        }

        match self
            .i2c_write(
                self.init_conf.h.addr,
                &mut hsp,
                self.init_conf.h.len as usize,
            )
            .await
        {
            Ok(_) => debug!("Send init configuration->h OK"),
            Err(e) => {
                return Err(e);
            }
        }

        let mut d_data = self.init_conf.d.data;
        match self
            .i2c_write(
                self.init_conf.d.addr,
                &mut d_data,
                self.init_conf.d.len as usize,
            )
            .await
        {
            Ok(_) => debug!("Send init configuration->d OK"),
            Err(e) => {
                return Err(e);
            }
        }

        let mut m_data = self.init_conf.m.data;
        match self
            .i2c_write(
                self.init_conf.m.addr,
                &mut m_data,
                self.init_conf.m.len as usize,
            )
            .await
        {
            Ok(_) => debug!("Send init configuration->m OK"),
            Err(e) => {
                return Err(e);
            }
        }

        let mut s_data = self.init_conf.s.data;
        match self
            .i2c_write(
                self.init_conf.s.addr,
                &mut s_data,
                self.init_conf.s.len as usize,
            )
            .await
        {
            Ok(_) => debug!("Send init configuration->s OK"),
            Err(e) => {
                return Err(e);
            }
        }

        match self
            .i2c_write(
                Command::ZmodAddrCmd.as_byte(),
                &mut [self.init_conf.start],
                1,
            )
            .await
        {
            Ok(_) => debug!("Send init configuration->start OK"),
            Err(e) => {
                return Err(e);
            }
        }
        let mut status: u8 = 0x80;
        while status & commands::Command::StatusSequencerRunningMask.as_byte() != 0 {
            status = match self.read_status().await {
                Ok(ret) => ret,
                Err(e) => return Err(e),
            };
            self.delay.delay_millis(50);
        }

        match self
            .i2c
            .write_read(ZMOD_I2C_ADDRESS, &[self.init_conf.r.addr], &mut data_r)
            .await
        {
            Ok(_) => debug!("Send init configuration->r OK"),
            Err(e) => return Err(e),
        }

        debug!("Calculate LR ER: {:?}", data_r);

        self.mox_lr = ((data_r[0] as u16) << 8) | data_r[1] as u16;
        self.mox_er = ((data_r[2] as u16) << 8) | data_r[3] as u16;

        Ok(())
    }

    pub async fn i2c_write(&mut self, addr: u8, data: &mut [u8], len: usize) -> Result<(), Error> {
        let mut send_data = [0x00; 32];
        send_data[0] = addr;

        send_data[1..(len + 1)].copy_from_slice(&data[..(len)]);

        debug!(
            "I2C Write: ADDR {} - DATA {:?}",
            send_data[0],
            &send_data[1..len + 1]
        );
        self.i2c
            .write(ZMOD_I2C_ADDRESS, &send_data[0..len + 1])
            .await
    }

    pub async fn init_meas(&mut self) -> Result<(), Error> {
        debug!("Initialize measurement configuration for ZMOD sensor");
        let mut hsp = [0u8; 16];

        let _ = self.calc_factor(self.meas_conf.clone(), &mut hsp).await;

        match self
            .i2c_write(
                self.meas_conf.h.addr,
                &mut hsp,
                self.meas_conf.h.len as usize,
            )
            .await
        {
            Ok(_) => debug!("Send meas_conf->h OK"),
            Err(e) => return Err(e),
        }

        let mut d_data = self.meas_conf.d.data;
        match self
            .i2c_write(
                self.meas_conf.d.addr,
                &mut d_data,
                self.meas_conf.d.len as usize,
            )
            .await
        {
            Ok(_) => debug!("Send meas_conf->d OK"),
            Err(e) => return Err(e),
        }

        let mut m_data = self.meas_conf.m.data;
        match self
            .i2c_write(
                self.meas_conf.m.addr,
                &mut m_data,
                self.meas_conf.m.len as usize,
            )
            .await
        {
            Ok(_) => debug!("Send meas_conf->m OK"),
            Err(e) => return Err(e),
        }

        let mut s_data = self.meas_conf.s.data;
        match self
            .i2c_write(
                self.meas_conf.s.addr,
                &mut s_data,
                self.meas_conf.s.len as usize,
            )
            .await
        {
            Ok(_) => debug!("Send meas_conf->s OK"),
            Err(e) => return Err(e),
        }
        Ok(())
    }

    pub async fn start_meas(&mut self) -> Result<(), Error> {
        debug!("Start measurement of ZMOD sensor");
        self.i2c_write(
            Command::ZmodAddrCmd.as_byte(),
            &mut [self.meas_conf.start],
            1,
        )
        .await
    }

    pub async fn read_adc(&mut self, data: &mut [u8]) -> Result<(), Error> {
        match self
            .i2c
            .write_read(ZMOD_I2C_ADDRESS, &[self.meas_conf.r.addr], data)
            .await
        {
            Ok(_) => debug!("Read ADC OK"),
            Err(e) => return Err(e),
        }

        Ok(())
    }

    pub async fn read_status(&mut self) -> Result<u8, Error> {
        let mut status = [0u8; 1];
        match self
            .i2c
            .write_read(
                ZMOD_I2C_ADDRESS,
                &[Command::ZmodAddrStatus.as_byte()],
                &mut status,
            )
            .await
        {
            Ok(_) => Ok(status[0]),
            Err(e) => Err(e),
        }
    }

    pub async fn calc_factor(&mut self, zmod_config: ZmodConf, hsp: &mut [u8]) -> Result<(), ()> {
        let mut hsp_temp: [i16; 8] = [0; 8];
        let mut hspf: f32;
        let mut count: usize = 0;

        debug!("zmod->config: {:?}", self.config);

        while count < zmod_config.h.len as usize {
            hsp_temp[count / 2] =
                ((zmod_config.h.data[count] as i16) << 8) + zmod_config.h.data[count + 1] as i16;
            hspf = (-(((self.config[2] as f32) * 256.0) + self.config[3] as f32)
                * (((self.config[4] as f32) + 640.0)
                    * ((self.config[5] as f32) + (hsp_temp[count / 2] as f32))
                    - 512000.0))
                / 12288000.0;
            hsp[count] = ((hspf as u16) >> 8) as u8;
            hsp[count + 1] = ((hspf as u16) & 0x00FF) as u8;
            count += 2;
        }

        debug!("hsp result {:?}", hsp);

        Ok(())
    }

    pub async fn check_error_event(&mut self) -> bool {
        let mut data = [0u8; 1];
        match self
            .i2c
            .write_read(ZMOD_I2C_ADDRESS, &[0xB7], &mut data)
            .await
        {
            Ok(_) => debug!("Send error event OK"),
            Err(e) => {
                error!("Failed to send error event: {:?}", e);
                return false;
            }
        }

        if data[0] != 0 {
            if data[0] & Command::StatusPorEventMask.as_byte() != 0 {
                error!("ERROR_POR_EVENT");
                return false;
            } else if data[0] & Command::StatusAccessConflictMask.as_byte() != 0 {
                error!("ERROR_ACCESS_CONFLICT");
                return false;
            }
        }

        true
    }
    // pub async fn calc_rmox(&mut self, adc_result: &[u8], rmox: &mut [f32]) -> Result<(), ()> {
    //     let mut count: usize = 0;
    //     let mut rmox_index: usize = 0;

    //     info!("MOX_LR: {}", self.mox_lr);
    //     info!("MOX_ER: {}", self.mox_er);

    //     while count < (self.meas_conf.r.len as usize) {
    //         let adc_value = ((adc_result[count] as u16) << 8) | (adc_result[count + 1] as u16);
    //         if 0.0 >= (adc_value as f32 - self.mox_lr as f32) {
    //             rmox[rmox_index] = 1e-3;
    //             rmox_index += 1;
    //         } else if 0.0 >= (self.mox_er as f32 - adc_value as f32) {
    //             rmox[rmox_index] = 10e9;
    //             rmox_index += 1;
    //         } else {
    //             rmox[rmox_index] =
    //                 (self.config[0] as f32) * 1e3 * (adc_value as f32 - self.mox_lr as f32)
    //                     / (self.mox_er as f32 - adc_value as f32);
    //             rmox_index += 1;
    //         }

    //         count += 2;
    //     }

    //     Ok(())
    // }
}
