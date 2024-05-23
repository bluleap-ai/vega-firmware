use core::ffi;

#[derive(Debug, Copy, Clone, Default)]
/// Sensor settings structure
pub struct DeviceConfig {
    os_hum: u8,
    /// Temperature oversampling.
    os_temp: u8,
    /// Pressure oversampling.
    os_pres: u8,
    /// Filter coefficient.
    filter: u8,
    /// Standby time between sequential mode measurement profiles.
    odr: u8,
}

#[derive(Debug, Copy, Clone)]
#[repr(C)]
pub struct GasHeaterConfig {
    pub(crate) enable: u8,
    pub(crate) heatr_temp: u16,
    pub(crate) heatr_dur: u16,
    pub(crate) heatr_temp_prof: *mut u16,
    pub(crate) heatr_dur_prof: *mut u16,
    pub(crate) profile_len: u8,
    pub(crate) shared_heatr_dur: u16,
}

/// Oversampling setting
pub enum Sample {
    /// Switch off measurements
    Off = 0,
    /// Perform 1 measurement
    Once = 1,
    /// Perform 2 measurements
    X2 = 2,
    /// Perform 4 measurements
    X4 = 3,
    /// Perform 8 measurements
    X8 = 4,
    /// Perform 16 measurements
    X16 = 5,
}

/// Possible IIR Filter settings
#[repr(u8)]
pub enum Filter {
    /// Switch off the filter
    Off = 0u8,
    /// Filter coefficient of 2
    Size1 = 1,
    /// Filter coefficient of 4
    Size3 = 2,
    /// Filter coefficient of 8
    Size7 = 3,
    /// Filter coefficient of 16
    Size15 = 4,
    /// Filter coefficient of 32
    Size31 = 5,
    /// Filter coefficient of 64
    Size63 = 6,
    /// Filter coefficient of 128
    Size127 = 7,
}

/// ODR/Standby time macros
#[repr(u8)]
pub enum Odr {
    /// Standby time of 0.59ms
    Standby0_59Ms = 0,
    /// Standby time of 62.5ms
    Standby62_5Ms = 1,
    /// Standby time of 125ms
    Standby125Ms = 2,
    /// Standby time of 250ms
    Standby250Ms = 3,
    /// Standby time of 500ms
    Standby500Ms = 4,
    /// Standby time of 1s
    Standby1000Ms = 5,
    /// Standby time of 10ms
    Standby10Ms = 6,
    /// Standby time of 20ms
    Standby20Ms = 7,
    /// No standby time
    StandbyNone = 8,
}

/// Operation mode of the sensor.
#[derive(PartialEq)]
pub enum OperationMode {
    /// No measurements are performed. Minimal power consumption.
    Sleep = 0,
    /// Single TPHG cycle is performed. Gas sensor heater only operates during gas measurement.
    /// Returns to Sleep afterwards.
    Forced = 1,
    /// Multiple TPHG cycles are performed. Gas sensor heater operates in parallel to TPH
    /// measurement. Does not return to Sleep Mode.
    Parallel = 2,
    Sequential = 3,
}

#[derive(Debug, Copy, Clone, Default)]
#[repr(C)]
pub struct CalibrationData {
    pub par_h1: u16,
    pub par_h2: u16,
    pub par_h3: i8,
    pub par_h4: i8,
    pub par_h5: i8,
    pub par_h6: u8,
    pub par_h7: i8,
    pub par_gh1: i8,
    pub par_gh2: i16,
    pub par_gh3: i8,
    pub par_t1: u16,
    pub par_t2: i16,
    pub par_t3: i8,
    pub par_p1: u16,
    pub par_p2: i16,
    pub par_p3: i8,
    pub par_p4: i16,
    pub par_p5: i16,
    pub par_p6: i8,
    pub par_p7: i8,
    pub par_p8: i16,
    pub par_p9: i16,
    pub par_p10: u8,
    pub t_fine: ffi::c_float,
    pub res_heat_range: u8,
    pub res_heat_val: i8,
    pub range_sw_err: i8,
}

impl DeviceConfig {
    pub fn filter(&self, filter: Filter) -> Self {
        let mut conf = *self;
        conf.filter = filter as u8;
        conf
    }
    pub fn odr(&self, odr: Odr) -> Self {
        let mut conf = *self;
        conf.odr = odr as u8;
        conf
    }
    pub fn oversample_humidity(&self, h: Sample) -> Self {
        let mut conf = *self;
        conf.os_hum = h as u8;
        conf
    }
    pub fn oversample_pressure(&self, p: Sample) -> Self {
        let mut conf = *self;
        conf.os_pres = p as u8;
        conf
    }
    pub fn oversample_temperature(&self, t: Sample) -> Self {
        let mut conf = *self;
        conf.os_temp = t as u8;
        conf
    }
}

impl GasHeaterConfig {
    pub fn enable(&self) -> Self {
        let mut conf = *self;
        conf.enable = true as u8;
        conf
    }
    pub fn heater_temp(&self, temp: u16) -> Self {
        let mut conf = *self;
        conf.heatr_temp = temp;
        conf
    }
    pub fn heater_duration(&self, duration: u16) -> Self {
        let mut conf = *self;
        conf.heatr_dur = duration;
        conf
    }
    pub fn disable(&self) -> Self {
        let mut conf = *self;
        conf.enable = false as u8;
        conf
    }
}

impl Default for GasHeaterConfig {
    fn default() -> Self {
        Self {
            enable: 0,
            heatr_temp: 0,
            heatr_dur: 0,
            heatr_temp_prof: 0 as *mut u16,
            heatr_dur_prof: 0 as *mut u16,
            profile_len: 0,
            shared_heatr_dur: 0,
        }
    }
}