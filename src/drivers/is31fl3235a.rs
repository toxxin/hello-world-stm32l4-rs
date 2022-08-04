//! # IS31FL3235A library
//! A rust-embedded driver for the Lumissil Microsystems IS31FL3235A LED driver

#![no_std]
// #![deny(warnings)]

use embedded_hal::blocking::i2c::Write;

/// Is31fl3235a Error.
#[derive(Debug, Copy, Clone)]
pub enum Error<E> {
    /// Arguments error.
    Param,
    /// Underlying bus error.
    Bus(E),
}

pub enum Register {
    /// Shutdown register
    /// The Shutdown Register sets software shutdown mode of IS31FL3235A.
    /// During shutdown mode all registers retain their data.
    Shutdown = 0x00,
    /// Channel [28:1] PWM register byte
    Pwm = 0x05,
    /// Update the PWM
    Update = 0x25,
    /// Channel [28:1] PWM register byte
    LedControl = 0x2A,
    /// Control Control register
    GlobalControl = 0x4A,
    /// OutputFrequency register
    OutputFrequency = 0x4B,
    /// Reset all registers
    Reset = 0x4F,
}

pub enum SoftwareShutdownMode {
    Shutdown = 0x00,
    Normal = 0x01,
}

pub enum OutputCurrent {
    Max = 0x0,
    /// Imax/2
    Max2 = 0x1,
    /// Imax/3
    Max3 = 0x2,
    /// Imax/4
    Max4 = 0x3,
}

pub enum OutputFrequency {
    Freq3Hz = 0x0,
    Freq22kHz = 0x1,
}

/// Is31fl3235a driver
pub struct Is31fl3235a<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C, E> Is31fl3235a<I2C>
    where
        I2C: Write<Error = E>,
{
    pub fn new(_i2c: I2C, address: u8) -> Result<Self, E> {
        let drv = Is31fl3235a {
            i2c: _i2c,
            addr: address,
        };

        Ok(drv)
    }

    pub fn power_on(&mut self) -> Result<(), E> {

        // Reset all internal registers
        let mut temp = [Register::Reset as u8, 0x0];
        self.i2c.write(self.addr, &temp)?;

        // Put the driver in the normal mode
        temp[0] = Register::Shutdown as u8;
        temp[1] = 1 as u8;
        self.i2c.write(self.addr, &temp)?;

        // Enable all channels
        for i in 0..28 {
            temp[0] = (Register::LedControl as u8) + i;
            temp[1] = 0x1;
            self.i2c.write(self.addr, &temp)?;
        }

        self.apply_changes()?;

        temp[0] = Register::GlobalControl as u8;
        temp[1] = 0x0;
        self.i2c.write(self.addr, &temp)?;

        Ok(())
    }

    pub fn power_off(&mut self) -> Result<(), E> {
        let mut temp = [Register::Shutdown as u8, 0x0];
        self.i2c.write(self.addr, &temp)?;
        Ok(())
    }

    pub fn set_frequency(&mut self, f: OutputFrequency) -> Result<(), E> {
        let mut temp = [Register::OutputFrequency as u8, f as u8];
        self.i2c.write(self.addr, &temp)?;
        Ok(())
    }

    pub fn set_pwm(&mut self, ch: u8, value: u8) -> Result<(), E> {

        // TODO: add channel number validation

        let mut temp = [(Register::Pwm as u8) + ch - 1, value];
        self.i2c.write(self.addr, &temp)?;

        self.apply_changes()?;

        Ok(())
    }

    #[inline]
    fn apply_changes(&mut self) -> Result<(), E> {
        let mut temp = [Register::Update as u8, 0x0];
        self.i2c.write(self.addr, &temp)?;
        Ok(())
    }
}
