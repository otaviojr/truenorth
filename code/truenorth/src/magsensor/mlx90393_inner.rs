use std::sync::mpsc::{Receiver, Sender};
use std::sync::{mpsc, Arc, Mutex};
use std::{thread, time::Duration};

use esp_idf_hal::delay::BLOCK;
use esp_idf_hal::gpio::AnyIOPin;
use esp_idf_hal::i2c::I2cDriver;

use crate::magsensor::mlx90393_defs::*;
use crate::TrueNorthParameters;

use super::{MagSensorEvent, MagSensorHandlerPtr, MagSensorState};

// HALLCONF - 0x00
// is the same table applying a scale factor of 98/75
const GAIN_RES_CONVERSION: [[(f32, f32); 8];4] = [
    // HALLCONF - 0xC
    [
        (0.751, 1.210),
        (0.601, 0.968),
        (0.451, 0.726),
        (0.376, 0.605),
        (0.300, 0.484),
        (0.250, 0.403),
        (0.200, 0.323),
        (0.150, 0.242),
    ],
    [
        (1.502, 2.420),
        (1.202, 1.936),
        (0.901, 1.452),
        (0.751, 1.210),
        (0.601, 0.968),
        (0.501, 0.807),
        (0.401, 0.645),
        (0.300, 0.484),
    ],
    [
        (3.004, 4.840),
        (2.403, 3.872),
        (1.803, 2.904),
        (1.502, 2.420),
        (1.202, 1.936),
        (1.001, 1.613),
        (0.801, 1.291),
        (0.601, 0.968),
    ],
    [
        (6.009, 9.680),
        (4.840, 7.744),
        (3.605, 5.808),
        (3.004, 4.840),
        (2.403, 3.872),
        (2.003, 3.227),
        (1.602, 2.581),
        (1.202, 1.936),
    ],
];

pub struct MLX90393Internal {
    pub current_gain: Option<MLX90393GAIN>,
    pub current_resolution: Option<u16>,
    pub current_filter: Option<MLX90393FILTER>,
    pub current_oversampling: Option<MLX90393OVERSAMPLING>,
    pub state: MagSensorState,
    pub last_state: MagSensorState,
    pub channel: Arc<Mutex<(Sender<bool>,Receiver<bool>)>>,
    pub handlers: Vec<Arc<Mutex<MagSensorHandlerPtr>>>,
}

impl Default for MLX90393Internal {
    fn default() -> Self {
        let (tx, rx) = mpsc::channel::<bool>();
        Self {
            current_gain: None,
            current_resolution: None,
            current_filter: None,
            current_oversampling: None,
            state: MagSensorState::Idle,
            last_state: MagSensorState::Idle,
            channel: Arc::new(Mutex::new((tx, rx))),
            handlers: Vec::new(),
        }
    }
}

pub struct MLX90393Inner {
    pub i2c: Option<I2cDriver<'static>>,
    pub int: AnyIOPin,
    pub slave_address: u8,
    pub parameters: Arc<TrueNorthParameters>,
    pub internal: MLX90393Internal,
}

impl MLX90393Inner {

    pub fn send_event(&mut self, event: MagSensorEvent) -> Result<(), Box<dyn std::error::Error>> {
        let handlers = self.internal.handlers.iter();
        for handler in handlers {
            handler.lock().unwrap()(event);
        }
        Ok(())
    }

    pub fn set_state(&mut self, state: MagSensorState) {
        self.internal.last_state = self.internal.state;
        self.internal.state = state;
    }

    #[allow(dead_code)]
    pub fn read_register(&mut self, register: MLX90393REG) -> Result<u16, Box<dyn std::error::Error>> {
        let tx_buf: [u8; 2] = [MLX90393CMD::RR.into(), (register as u8) << 2];
        let mut rx_buf: [u8; 3] = [0; 3];

        let slave_address = self.slave_address;

        self.i2c.as_mut().unwrap().write(slave_address, &tx_buf, BLOCK)?;
        thread::sleep(Duration::from_millis(10));
        self.i2c.as_mut().unwrap().read(slave_address, &mut rx_buf, BLOCK)?;

        let status = rx_buf[0];
        let error = status & 0x10;
        let ret = (rx_buf[1] as u16) << 8 | rx_buf[2] as u16;   

        if error != 0 {
            return Err(Box::new(std::io::Error::new(std::io::ErrorKind::Other, format!("MLX90393: read_register failed, status: {}", status))));
        }

        Ok(ret)
    }

    #[allow(dead_code)]
    pub fn write_register(&mut self, register: MLX90393REG, value: u16) -> Result<(), Box<dyn std::error::Error>> {
        let tx_buf: [u8; 4] = [MLX90393CMD::WR.into(), ((value >> 8) & 0xFF) as u8, (value & 0xFF) as u8, (register as u8) << 2];
        let mut rx_buf: [u8; 1] = [0; 1];

        let slave_address = self.slave_address;

        self.i2c.as_mut().unwrap().write(slave_address, &tx_buf, BLOCK)?;
        thread::sleep(Duration::from_millis(10));
        self.i2c.as_mut().unwrap().read(slave_address, &mut rx_buf, BLOCK)?;

        let status = rx_buf[0];
        let error = status & 0x10;
        if error != 0 {
            return Err(Box::new(std::io::Error::new(std::io::ErrorKind::Other, format!("MLX90393: write_register failed, status: {}", status))));
        }

        Ok(())
    }


    #[allow(dead_code)]
    pub fn read_measurement(&mut self) -> Result<[f32; 3], Box<dyn std::error::Error>> {
        let tx_buf: [u8; 1] = [MLX90393CMD::RM as u8 | MLX90393AXIS::ALL as u8];
        let mut rx_buf: [u8; 9] = [0; 9];

        let slave_address = self.slave_address;

        self.i2c.as_mut().unwrap().write(slave_address, &tx_buf, BLOCK)?;
        thread::sleep(Duration::from_millis(10));
        self.i2c.as_mut().unwrap().read(slave_address, &mut rx_buf, BLOCK)?;

        let status = rx_buf[0];
        let error = status & 0x10;
        let val = [
            (rx_buf[3] as i16) << 8 | rx_buf[4] as i16,
            (rx_buf[5] as i16) << 8 | rx_buf[6] as i16,
            (rx_buf[7] as i16) << 8 | rx_buf[8] as i16,
        ];

        let gain = self.get_gain()?;
        let x_resolution = self.get_resolution(MLX90393AXIS::X)?;
        let y_resolution = self.get_resolution(MLX90393AXIS::Y)?;
        let z_resolution = self.get_resolution(MLX90393AXIS::Z)?;

        let ret = [
            val[0] as f32 * GAIN_RES_CONVERSION[x_resolution as usize][gain as usize].0,
            val[1] as f32 * GAIN_RES_CONVERSION[y_resolution as usize][gain as usize].0,
            val[2] as f32 * GAIN_RES_CONVERSION[z_resolution as usize][gain as usize].1,
        ];

        if error != 0 {
            return Err(Box::new(std::io::Error::new(std::io::ErrorKind::Other, format!("MLX90393: read_measurement failed, status: {}", status))));
        }

        Ok(ret)
    }

    #[allow(dead_code)]
    pub fn set_gain(&mut self, new_gain: MLX90393GAIN) -> Result<(), Box<dyn std::error::Error>> {
        let mut gain = self.read_register(MLX90393REG::CONF1)?;
        gain &= !0x0070;
        gain |= (new_gain as u16) << 4;

        self.write_register(MLX90393REG::CONF1, gain)?;

        self.internal.current_gain = Some(new_gain);

        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_gain(&mut self) -> Result<MLX90393GAIN, Box<dyn std::error::Error>> {

        if self.internal.current_gain.is_some() {
            return Ok(self.internal.current_gain.unwrap());
        }

        let mut gain = self.read_register(MLX90393REG::CONF1)?;
        gain &= 0x0070;

        Ok(MLX90393GAIN::from((gain >> 4) as u8))
    }

    #[allow(dead_code)]
    pub fn set_resolution(&mut self, axis: MLX90393AXIS, new_resolution: MLX90393RESOLUTION) -> Result<(), Box<dyn std::error::Error>> {
        let mut resolution = self.read_register(MLX90393REG::CONF3)?;

        match axis {
            MLX90393AXIS::X => {
                resolution &= !0x0060;
                resolution |= (new_resolution as u16) << 5;
            },
            MLX90393AXIS::Y => {
                resolution &= !0x0180;
                resolution |= (new_resolution as u16) << 7;
            },
            MLX90393AXIS::Z => {
                resolution &= !0x0600;
                resolution |= (new_resolution as u16) << 9;
            },
            MLX90393AXIS::ALL => return Err(Box::new(std::io::Error::new(std::io::ErrorKind::Other, "MLX90393: set_resolution failed, axis ALL not allowed here."))),
        }

        self.write_register(MLX90393REG::CONF3, resolution)?;

        self.internal.current_resolution = Some(resolution);

        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_resolution(&mut self, axis: MLX90393AXIS) -> Result<MLX90393RESOLUTION, Box<dyn std::error::Error>> {
        let resolution = if self.internal.current_resolution.is_some() {
            self.internal.current_resolution.unwrap()
        } else {
            self.read_register(MLX90393REG::CONF3)?
        };

        Ok(MLX90393RESOLUTION::from(match axis {
            MLX90393AXIS::X => (((resolution & 0x0060) >> 5) & 0x03) as u8,
            MLX90393AXIS::Y => (((resolution & 0x0180) >> 7) & 0x03) as u8,
            MLX90393AXIS::Z => (((resolution & 0x0600) >> 9) & 0x03) as u8,
            MLX90393AXIS::ALL => return Err(Box::new(std::io::Error::new(std::io::ErrorKind::Other, "MLX90393: get_resolution failed, axis ALL not allowed here."))),
        }))
    }

    #[allow(dead_code)]
    pub fn set_filter(&mut self, new_filter: MLX90393FILTER) -> Result<(), Box<dyn std::error::Error>> {
        let mut filter = self.read_register(MLX90393REG::CONF3)?;
        filter &= !0x1C;
        filter |= (new_filter as u16) << 2;
        self.write_register(MLX90393REG::CONF3, filter)?;

        self.internal.current_filter = Some(new_filter);
        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_filter(&mut self) -> Result<MLX90393FILTER, Box<dyn std::error::Error>> {
        if self.internal.current_filter.is_some() {
            return Ok(self.internal.current_filter.unwrap());
        }

        let filter = self.read_register(MLX90393REG::CONF3)?;
        Ok(MLX90393FILTER::from(((filter & 0x1C) >> 2) as u8))
    }

    #[allow(dead_code)]
    pub fn set_oversampling(&mut self, new_oversampling: MLX90393OVERSAMPLING) -> Result<(), Box<dyn std::error::Error>> {
        let mut oversampling = self.read_register(MLX90393REG::CONF3)?;
        oversampling &= !0x03;
        oversampling |= new_oversampling as u16;
        self.write_register(MLX90393REG::CONF3, oversampling)?;

        self.internal.current_oversampling = Some(new_oversampling);
        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_oversampling(&mut self) -> Result<MLX90393OVERSAMPLING, Box<dyn std::error::Error>> {
        if self.internal.current_oversampling.is_some() {
            return Ok(self.internal.current_oversampling.unwrap());
        }

        let oversampling = self.read_register(MLX90393REG::CONF3)?;
        Ok(MLX90393OVERSAMPLING::from((oversampling & 0x03) as u8))
    }

    #[allow(dead_code)]
    pub fn set_trigger_interval(&mut self, state: bool) -> Result<(), Box<dyn std::error::Error>> {
        let mut trigger = self.read_register(MLX90393REG::CONF2)?;
        trigger &= !0x8000;
        if state {
            trigger |= 0x8000;
        }
        self.write_register(MLX90393REG::CONF2, trigger)?;

        Ok(())
    }

    #[allow(dead_code)]
    pub fn start_single_measurement(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let tx_buf: [u8; 1] = [MLX90393CMD::SM as u8 | MLX90393AXIS::ALL as u8];
        let mut rx_buf: [u8; 1] = [0; 1];

        let slave_address = self.slave_address;

        self.i2c.as_mut().unwrap().write(slave_address, &tx_buf, BLOCK)?;
        thread::sleep(Duration::from_millis(10));
        self.i2c.as_mut().unwrap().read(slave_address, &mut rx_buf, BLOCK)?;

        let status = rx_buf[0];
        let error = status & 0x10;
        let sm_mode = status & 0x20;

        if error != 0 {
            return Err(Box::new(std::io::Error::new(std::io::ErrorKind::Other, format!("MLX90393: start_single_measurement failed, status: {}", status))));
        }

        if sm_mode == 0 {
            return Err(Box::new(std::io::Error::new(std::io::ErrorKind::Other, format!("MLX90393: start_single_measurement failed, status: {}", status))));
        }

        Ok(())
    }

    #[allow(dead_code)]
    pub fn start_burst_measurement(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let tx_buf: [u8; 1] = [MLX90393CMD::SB as u8 | MLX90393AXIS::ALL as u8];
        let mut rx_buf: [u8; 1] = [0; 1];

        let slave_address = self.slave_address;

        self.i2c.as_mut().unwrap().write(slave_address, &tx_buf, BLOCK)?;
        thread::sleep(Duration::from_millis(10));
        self.i2c.as_mut().unwrap().read(slave_address, &mut rx_buf, BLOCK)?;

        let status = rx_buf[0];
        let error = status & 0x10;
        let bm_mode = status & 0x80;

        if error != 0 {
            return Err(Box::new(std::io::Error::new(std::io::ErrorKind::Other, format!("MLX90393: start_burst_measurement failed, status: {}", status))));
        }

        if bm_mode == 0 {
            return Err(Box::new(std::io::Error::new(std::io::ErrorKind::Other, format!("MLX90393: start_burst_measurement failed, status: {}", status))));
        }

        Ok(())
    }

    pub fn set_wakeup_comparator(&mut self, comparator: bool) -> Result<(), Box<dyn std::error::Error>> {
        let mut comparator_register = self.read_register(MLX90393REG::CONF2)?;
        if comparator {
            comparator_register |= 0x10;
        } else {
            comparator_register &= !0x10;
        }
        self.write_register(MLX90393REG::CONF2, comparator_register)?;
        Ok(())
    }

    #[allow(dead_code)]
    pub fn start_wakeup_measurement(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let tx_buf: [u8; 1] = [MLX90393CMD::SW as u8 | MLX90393AXIS::ALL as u8];
        let mut rx_buf: [u8; 1] = [0; 1];

        let slave_address = self.slave_address;

        self.i2c.as_mut().unwrap().write(slave_address, &tx_buf, BLOCK)?;
        thread::sleep(Duration::from_millis(10));
        self.i2c.as_mut().unwrap().read(slave_address, &mut rx_buf, BLOCK)?;

        let status = rx_buf[0];
        let error = status & 0x10;
        let wu_mode = status & 0x40;

        if error != 0 {
            return Err(Box::new(std::io::Error::new(std::io::ErrorKind::Other, format!("MLX90393: start_wakeup_measurement failed, status: {}", status))));
        }

        if wu_mode == 0 {
            return Err(Box::new(std::io::Error::new(std::io::ErrorKind::Other, format!("MLX90393: start_wakeup_measurement failed, status: {}", status))));
        }

        Ok(())
    }

    #[allow(dead_code)]
    pub fn exit_mode(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let tx_buf: [u8; 1] = [MLX90393CMD::EX as u8];
        let mut rx_buf: [u8; 1] = [0; 1];

        let slave_address = self.slave_address;

        self.i2c.as_mut().unwrap().write(slave_address, &tx_buf, BLOCK)?;
        thread::sleep(Duration::from_millis(10));
        self.i2c.as_mut().unwrap().read(slave_address, &mut rx_buf, BLOCK)?;

        let status = rx_buf[0];
        let error = status & 0x10;

        if error != 0 {
            return Err(Box::new(std::io::Error::new(std::io::ErrorKind::Other, format!("MLX90393: exit_mode failed, status: {}", status))));
        }

        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let tx_buf: [u8; 1] = [MLX90393CMD::RT as u8];
        let mut rx_buf: [u8; 1] = [0; 1];

        let slave_address = self.slave_address;

        self.i2c.as_mut().unwrap().write(slave_address, &tx_buf, BLOCK)?;
        thread::sleep(Duration::from_millis(10));
        self.i2c.as_mut().unwrap().read(slave_address, &mut rx_buf, BLOCK)?;

        let status = rx_buf[0];
        let error = status & 0x10;

        if error != 0 {
            return Err(Box::new(std::io::Error::new(std::io::ErrorKind::Other, format!("MLX90393: reset failed, status: {}", status))));
        }

        Ok(())
    }

    pub fn add_handler(&mut self, handler: MagSensorHandlerPtr) -> Result<(), Box<dyn std::error::Error>> {
        self.internal.handlers.push(Arc::new(Mutex::new(handler)));
        Ok(())
    }
}