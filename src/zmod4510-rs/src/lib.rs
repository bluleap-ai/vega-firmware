#![no_std]

pub mod commands;
pub mod zmod;

use core::clone::Clone;
use core::fmt::Debug;
use core::marker::Copy;
use core::prelude::v1::derive;

pub struct ZmodData {
    pub config: [u8; 6],
    pub prod_data: [u8; 10],
    pub mox_er: u16,
    pub mox_lr: u16,
    pub pid: u16,
    pub init_conf: ZmodConf,
    pub meas_conf: ZmodConf,
}

impl ZmodData {
    fn default() -> Self {
        Self {
            config: [0; 6],
            mox_er: 0,
            mox_lr: 0,
            prod_data: [0; 10],
            pid: 0x6320,
            init_conf: ZmodConf::default(),
            meas_conf: ZmodConf::default(),
        }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Oaq2ndGenHandle {
    pub sample_cnt: u32,
    pub smooth_rmox: f32,
    pub gcda: f32,
    pub o3_conc_ppb: f32,
    pub o3_1h_ppb: f32,
    pub o3_8h_ppb: f32,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Oaq2ndGenInputs {
    pub adc_result: *const u8,
    pub humidity_pct: f32,
    pub temperature_degc: f32,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Oaq2ndGenResults {
    pub rmox: [f32; 8],
    pub o3_conc_ppb: f32,
    pub fast_aqi: u16,
    pub epa_aqi: u16,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct ZmodDev {
    pub i2c_addr: u8,
    pub config: [u8; 6],
    pub mox_er: u16,
    pub mox_lr: u16,
    pub pid: u16,
    pub prod_data: *const u8,
    pub read: unsafe extern "C" fn(u8, u8, *const u8, u8) -> i8,
    pub write: unsafe extern "C" fn(u8, u8, *const u8, u8) -> i8,
    pub delay: unsafe extern "C" fn(u32),
    pub init_config: *const ZmodConf,
    pub meas_config: *const ZmodConf,
}

#[repr(C)]
#[derive(Debug, Clone)]
pub struct ZmodConfStr {
    pub addr: u8,
    pub len: u8,
    pub data: [u8; 30],
}

#[repr(C)]
#[derive(Debug, Clone)]
pub struct ZmodConf {
    pub start: u8,
    pub h: ZmodConfStr,
    pub d: ZmodConfStr,
    pub m: ZmodConfStr,
    pub s: ZmodConfStr,
    pub r: ZmodConfStr,
    pub prod_data_len: u8,
}

impl ZmodConfStr {
    fn default() -> Self {
        ZmodConfStr {
            addr: 0x00,
            len: 0x00,
            data: [0; 30],
        }
    }
}

impl ZmodConf {
    pub fn default() -> Self {
        ZmodConf {
            start: 0x00,
            h: ZmodConfStr::default(),
            d: ZmodConfStr::default(),
            m: ZmodConfStr::default(),
            s: ZmodConfStr::default(),
            r: ZmodConfStr::default(),
            prod_data_len: 0,
        }
    }

    pub fn zmod4510_measurement() -> Self {
        ZmodConf {
            start: 0x80,
            h: ZmodConfStr {
                addr: 0x40,
                len: 4,
                data: [
                    0x00, 0x50, 0xFE, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00,
                ],
            },
            d: ZmodConfStr {
                addr: 0x50,
                len: 2,
                data: [
                    0x00, 0x10, 0xA0, 0x18, 0xC0, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00,
                ],
            },
            m: ZmodConfStr {
                addr: 0x60,
                len: 2,
                data: [
                    0x23, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00,
                ],
            },
            s: ZmodConfStr {
                addr: 0x68,
                len: 18,
                data: [
                    0x00, 0x00, 0x06, 0x41, 0x06, 0x41, 0x06, 0x41, 0x06, 0x41, 0x06, 0x41, 0x06,
                    0x41, 0x06, 0x41, 0x86, 0x41, 0x00, 0x03, 0x00, 0x0B, 0x00, 0x13, 0x00, 0x04,
                    0x00, 0x0C, 0x80, 0x14,
                ],
            },
            r: ZmodConfStr {
                addr: 0x97,
                len: 18,
                data: [
                    0x00, 0x00, 0x00, 0x08, 0x00, 0x10, 0x00, 0x01, 0x00, 0x09, 0x00, 0x11, 0x00,
                    0x02, 0x00, 0x0A, 0x00, 0x12, 0x00, 0x03, 0x00, 0x0B, 0x00, 0x13, 0x00, 0x04,
                    0x00, 0x0C, 0x80, 0x14,
                ],
            },
            prod_data_len: 10,
        }
    }

    pub fn zmod4510_init() -> Self {
        ZmodConf {
            start: 0x80,
            h: ZmodConfStr {
                addr: 0x40,
                len: 2,
                data: [
                    0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00,
                ],
            },
            d: ZmodConfStr {
                addr: 0x50,
                len: 2,
                data: [
                    0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00,
                ],
            },
            m: ZmodConfStr {
                addr: 0x60,
                len: 2,
                data: [
                    0xC3, 0xE3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00,
                ],
            },
            s: ZmodConfStr {
                addr: 0x68,
                len: 4,
                data: [
                    0x00, 0x00, 0x80, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00,
                ],
            },
            r: ZmodConfStr {
                addr: 0x97,
                len: 4,
                data: [
                    0x00, 0x00, 0x80, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00,
                ],
            },
            prod_data_len: 0,
        }
    }
}
