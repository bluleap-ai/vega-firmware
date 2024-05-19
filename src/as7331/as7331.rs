use embedded_hal_02::{delay::DelayNs, i2c::Error, i2c::I2c};

const AS7331_I2C_ADDRESS: u8 = 0x74;

#[derive(Debug)]
pub struct As7331<I2C, D> {
    i2c: I2C,
    pub delay: D,
}

impl<I2C, D, E> As7331<I2C, D>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
{
    pub fn new(i2c: I2C, delay: D) -> Self {
        As7331 {
            i2c,
            delay,
        }
    }
}