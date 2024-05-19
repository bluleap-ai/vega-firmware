use esp_hal::peripherals::I2C0;
use esp_hal::Async;
// use embedded_hal_02::{delay::DelayNs, i2c::Error, i2c::I2c};
use log::{error, info};
use esp_hal::{
    clock::ClockControl, delay::Delay, embassy, gpio::Io, i2c::I2C, peripherals::Peripherals,
    prelude::*, system::SystemControl, timer::timg::TimerGroup,
};

use super::commands::{self, Command};
use super::types::*;

const ZMOD_I2C_ADDRESS: u8 = 0x33;
const ZMOD4510_PID: u16 = 0x6320;

pub struct Zmod<'a> {
    i2c: I2C<'a, I2C0, Async>,
    pub delay: Delay,
    config: [u8; 6],
    general_purpose: [u8; 9],
    mox_er: u16,
    mox_lr: u16,
    pid: u16,
    init_conf: zmod_conf,
    meas_conf: zmod_conf,
}

impl<'a> Zmod<'a>
{
    pub async fn new(i2c: I2C<'a, I2C0, Async>, delay: Delay) -> Self {
        Zmod {
            i2c,
            delay,
            config: [0, 0, 0, 0, 0, 0],
            general_purpose: [0, 0, 0, 0, 0, 0, 0, 0, 0],
            mox_er: 0,
            mox_lr: 0,
            pid: 0,
            init_conf: zmod_conf::default(),
            meas_conf: zmod_conf::default(),
        }
    }

    pub async fn destroy(self) -> I2C<'a, I2C0, Async> {
        self.i2c
    }

    pub async fn read_info(&mut self) -> bool {
        // wait for sensor ready
        let mut status: u8 = 0x80;
        let mut counter: u32 = 0;
        while counter < 1000 && (status & 0x80 != 0) {
            match self
                .i2c
                .write(ZMOD_I2C_ADDRESS, &[Command::ZmodAddrCmd.as_byte()])
                .await
            {
                Ok(_) => info!("NICE"),
                Err(e) => {
                    error!("FAILED I2C: {:?}", e);
                    continue;
                }
            }

            match self.read_status().await {
                Ok(ret) => {
                    status = ret;
                    info!("STATUS: {}", ret)
                },
                Err(_) => error!("Failed to read status"),
            };
            
            counter += 1;
        }

        if counter >= 1000 {
            return false;
        }

        let mut data = [0u8; 2];

        if self
            .i2c
            .write_read(
                ZMOD_I2C_ADDRESS,
                &[Command::ZmodAddrPid.as_byte()],
                &mut data,
            )
            .await
            .is_err()
        {
            error!("FAILED I2C");
            return false;
        }

        self.pid = (data[0] as u16) << 8 | data[1] as u16;

        if self
            .i2c
            .write_read(
                ZMOD_I2C_ADDRESS,
                &[Command::ZmodAddrGeneralPupose.as_byte()],
                &mut self.general_purpose,
            )
            .await
            .is_err()
        {
            error!("FAILED I2C");
            return false;
        }

        if self
            .i2c
            .write_read(
                ZMOD_I2C_ADDRESS,
                &[Command::ZmodAddrConf.as_byte()],
                &mut self.config,
            )
            .await
            .is_err()
        {
            error!("FAILED I2C");
            return false;
        }

        match self.pid {
            ZMOD4510_PID => {
                self.meas_conf = zmod_conf::zmod4510();
                self.init_conf = zmod_conf::zmod45xxi();
            }
            _ => {
                error!("Unsupported!");
                return false;
            }
        }

        true
    }

    pub async fn init(&mut self) -> Result<(), ()> {
        let mut data = [0u8; 4];
        if self
            .i2c
            .write_read(ZMOD_I2C_ADDRESS, &[0xB7], &mut data)
            .await
            .is_err()
        {
            return Err(());
        }

        if self.calc_factor(80.0, &mut data).await.is_err() {
            return Err(());
        }

        if !self.i2c_write(
            self.init_conf.h.addr,
            &mut data,
            self.init_conf.h.len as usize,
        ).await {
            return Err(());
        }

        let mut d_data = self.init_conf.d.data;
        if !self.i2c_write(
            self.init_conf.d.addr,
            &mut d_data,
            self.init_conf.d.len as usize,
        ).await {
            return Err(());
        }

        let mut m_data = self.init_conf.m.data;
        if !self.i2c_write(
            self.init_conf.m.addr,
            &mut m_data,
            self.init_conf.m.len as usize,
        ).await {
            return Err(());
        }

        let mut s_data = self.init_conf.s.data;
        if !self.i2c_write(
            self.init_conf.s.addr,
            &mut s_data,
            self.init_conf.s.len as usize,
        ).await {
            return Err(());
        }

        if !self.i2c_write(
            0x93,
            &mut [self.init_conf.start],
            self.init_conf.h.len as usize,
        ).await {
            return Err(());
        }
        let mut status: u8 = 0;
        while status & commands::Command::StatusSequencerRunningMask.as_byte() == 0 {
            status = match self.read_status().await {
                Ok(ret) => ret,
                Err(_) => return Err(()),
            };
            self.delay.delay_millis(50);
        }

        if self
            .i2c
            .write_read(ZMOD_I2C_ADDRESS, &[self.init_conf.r.addr], &mut data)
            .await
            .is_err()
        {
            return Err(());
        }

        self.mox_lr = ((data[0] as u16) << 8) | data[1] as u16;
        self.mox_er = ((data[2] as u16) << 8) | data[3] as u16;

        if self
            .i2c
            .write_read(ZMOD_I2C_ADDRESS, &[0xB7], &mut data)
            .await
            .is_err()
        {
            return Err(());
        }

        if data[0] != 0 {
            if Command::StatusAccessConflictMask.as_byte() & data[0] != 0 {
                return Err(());
            } else if Command::StatusPorEventMask.as_byte() & data[0] != 0 {
                return Err(());
            }
        }

        Ok(())
    }

    pub async fn i2c_write(&mut self, addr: u8, data: &mut [u8], len: usize) -> bool {
        let mut send_data = [0x00; 32];
        send_data[0] = addr;
        send_data[1..len].copy_from_slice(&data[0..(len - 1)]);

        self.i2c.write(ZMOD_I2C_ADDRESS, &send_data[0..len]).await.is_ok()
    }

    pub async fn init_meas(&mut self) {
        let mut data = [0x00; 10];
        let _ = self.i2c.write_read(ZMOD_I2C_ADDRESS, &[0xB7], &mut data).await;
        let _ = self.calc_factor(-440.0, &mut data).await;
        let _ = self.calc_factor(-490.0, &mut data[2..]).await;
        let _ = self.calc_factor(-540.0, &mut data[4..]).await;
        let _ = self.calc_factor(-590.0, &mut data[6..]).await;
        let _ = self.calc_factor(-640.0, &mut data[8..]).await;
        let mut h_data = self.init_conf.m.data;
        if !self.i2c_write(
            self.meas_conf.h.addr,
            &mut h_data,
            self.meas_conf.h.len as usize,
        ).await {
            error!("I2C ERR");
        }
        let mut d_data = self.init_conf.m.data;
        if !self.i2c_write(
            self.meas_conf.d.addr,
            &mut d_data,
            self.meas_conf.d.len as usize,
        ).await {
            error!("I2C ERR");
        }
        let mut m_data = self.init_conf.m.data;
        if !self.i2c_write(
            self.meas_conf.m.addr,
            &mut m_data,
            self.meas_conf.m.len as usize,
        ).await {
            error!("I2C ERR");
        }
        let mut s_data = self.init_conf.m.data;
        if !self.i2c_write(
            self.meas_conf.s.addr,
            &mut s_data,
            self.meas_conf.s.len as usize,
        ).await {
            error!("I2C ERR");
        }
    }

    pub async fn start_meas(&mut self) {
        if !self.i2c_write(
            Command::ZmodAddrCmd.as_byte(),
            &mut [self.meas_conf.start],
            1,
        ).await {

        }
    }

    pub async fn read_adc(&mut self, data: &mut [u8]) -> bool {
        let mut read_data = [0x00; 1];
        let _ = self.i2c
            .write_read(ZMOD_I2C_ADDRESS, &[self.meas_conf.r.addr], data).await;

        let _ = self.i2c
            .write_read(ZMOD_I2C_ADDRESS, &[0xB7], &mut read_data).await;

        if read_data[0] != 0 {
            return false;
        }
        true
    }

    pub async fn read_status(&mut self) -> Result<u8, ()> {
        let mut status = [0u8; 1];
        if self
            .i2c
            .write_read(
                ZMOD_I2C_ADDRESS,
                &[Command::ZmodAddrStatus.as_byte()],
                &mut status,
            )
            .await
            .is_err()
        {
            error!("I2C ERROR");
            return Err(());
        }

        Ok(status[0])
    }

    pub async fn calc_factor(&mut self, factor: f32, data: &mut [u8]) -> Result<(), ()> {
        let hspf: f32;
        hspf = (-((self.config[2] as f32) * 256.0 + (self.config[3] as f32))
            * (((self.config[4] as f32) + 640.0) * ((self.config[5] as f32) + factor) - 512000.0))
            / 12288000.0;

        if 0.0 > hspf || 4096.0 < hspf {
            return Err(());
        }

        data[0] = (((hspf as u32) >> 8) | 0xFF) as u8;
        data[1] = ((hspf as u32) | 0xFF) as u8;

        Ok(())
    }

    pub async fn calc_rmox(&mut self, adc_result: &[u8], rmox: &mut [f32]) -> Result<(), ()> {
        let mut count: usize = 0;
        let mut adc_value: u16 = 0;
        let mut rmox_local: f32 = 0.0;

        while count < (self.meas_conf.r.len as usize) {
            adc_value = ((adc_result[count] as u16) << 8) | (adc_result[count + 1] as u16);
            if 0.0 >= (adc_value as f32 - self.mox_lr as f32) {
                rmox_local = 1e-3;
            } else if 0.0 >= (self.mox_er as f32 - adc_value as f32) {
                rmox_local = 1e12;
            } else {
                rmox_local =
                    (self.config[0] as f32) * 1e3 * (adc_value as f32 - self.mox_lr as f32)
                        / (self.mox_er as f32 - adc_value as f32);
            }

            if 1e12 < rmox_local {
                rmox_local = 1e12;
            }

            rmox[count / 2] = rmox_local;

            count += 2;
        }

        Ok(())
    }
}
