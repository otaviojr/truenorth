use std::{pin::pin, sync::{mpsc::{self, Receiver, Sender}, Arc, Mutex}, thread, time::Duration};

use async_executor::LocalExecutor;
use esp_idf_hal::delay::BLOCK;
use esp_idf_hal::{gpio::AnyIOPin, i2c::{I2c, I2cDriver}, peripheral::Peripheral, units::Hertz};

use crate::{magsensor::MagSensor, Endable};


#[derive(Debug)]
pub enum MLX90393REG {
    CONF1 = 0x00,
    CONF2 = 0x01,
    CONF3 = 0x02
}

impl From<MLX90393REG> for u8 {
    fn from(reg: MLX90393REG) -> Self {
        reg as u8
    }
}

#[derive(Debug)]
pub enum MLX90393CMD {
    SB = 0x10,  // Start burst mode.
    SW = 0x20,  // Start wakeup on change mode.
    SM = 0x30,  // Start single-meas mode.
    RM = 0x40,  // Read measurement.
    RR = 0x50,  // Read register.
    WR = 0x60,  // Write register.
    EX = 0x80,  // Exit mode.
    HR = 0xD0,  // Memory recall.
    HS = 0x70,  // Memory store.
    RT = 0xF0,  // Reset.
    NOP = 0x00, // NOP.
}

impl From<MLX90393CMD> for u8 {
    fn from(cmd: MLX90393CMD) -> Self {
        cmd as u8
    }
}

impl From<u8> for MLX90393CMD {
    fn from(cmd: u8) -> Self {
        match cmd {
            0x10 => MLX90393CMD::SB,
            0x20 => MLX90393CMD::SW,
            0x30 => MLX90393CMD::SM,
            0x40 => MLX90393CMD::RM,
            0x50 => MLX90393CMD::RR,
            0x60 => MLX90393CMD::WR,
            0x80 => MLX90393CMD::EX,
            0xD0 => MLX90393CMD::HR,
            0x70 => MLX90393CMD::HS,
            0xF0 => MLX90393CMD::RT,
            0x00 => MLX90393CMD::NOP,
            _ => panic!("Invalid MLX90393CMD"),
        }
    }
}


pub enum MLX90393GAIN {
    GAIN5X = (0x00),
    GAIN4X,
    GAIN3X,
    GAIN2_5X,
    GAIN2X,
    GAIN1_67X,
    GAIN1_33X,
    GAIN1X
}

impl From<MLX90393GAIN> for u8 {
    fn from(gain: MLX90393GAIN) -> Self {
        gain as u8
    }
}

impl From<u8> for MLX90393GAIN {
    fn from(gain: u8) -> Self {
        match gain {
            0x00 => MLX90393GAIN::GAIN5X,
            0x01 => MLX90393GAIN::GAIN4X,
            0x02 => MLX90393GAIN::GAIN3X,
            0x03 => MLX90393GAIN::GAIN2_5X,
            0x04 => MLX90393GAIN::GAIN2X,
            0x05 => MLX90393GAIN::GAIN1_67X,
            0x06 => MLX90393GAIN::GAIN1_33X,
            0x07 => MLX90393GAIN::GAIN1X,
            _ => panic!("Invalid MLX90393GAIN"),
        }
    }
}

pub enum MLX90393RESOLUTION {
    RES16,
    RES17,
    RES18,
    RES19,
}

impl From<MLX90393RESOLUTION> for u8 {
    fn from(resolution: MLX90393RESOLUTION) -> Self {
        resolution as u8
    }
}

impl From<u8> for MLX90393RESOLUTION {
    fn from(resolution: u8) -> Self {
        match resolution {
            0x00 => MLX90393RESOLUTION::RES16,
            0x01 => MLX90393RESOLUTION::RES17,
            0x02 => MLX90393RESOLUTION::RES18,
            0x03 => MLX90393RESOLUTION::RES19,
            _ => panic!("Invalid MLX90393RESOLUTION"),
        }
    }
}

#[derive(Debug)]
pub enum MLX90393AXIS {
    X = 0x02,
    Y = 0x04,
    Z = 0x08,
    ALL = 0x0E
}

impl From<MLX90393AXIS> for u8 {
    fn from(axis: MLX90393AXIS) -> Self {
        axis as u8
    }
}

impl From<u8> for MLX90393AXIS {
    fn from(axis: u8) -> Self {
        match axis {
            0x02 => MLX90393AXIS::X,
            0x04 => MLX90393AXIS::Y,
            0x08 => MLX90393AXIS::Z,
            0x0E => MLX90393AXIS::ALL,
            _ => panic!("Invalid MLX90393AXIS"),
        }
    }
}

pub enum MLX90393FILTER {
    FILTER0,
    FILTER1,
    FILTER2,
    FILTER3,
    FILTER4,
    FILTER5,
    FILTER6,
    FILTER7,
}

impl From<MLX90393FILTER> for u8 {
    fn from(filter: MLX90393FILTER) -> Self {
        filter as u8
    }
}

impl From<u8> for MLX90393FILTER {
    fn from(filter: u8) -> Self {
        match filter {
            0x00 => MLX90393FILTER::FILTER0,
            0x01 => MLX90393FILTER::FILTER1,
            0x02 => MLX90393FILTER::FILTER2,
            0x03 => MLX90393FILTER::FILTER3,
            0x04 => MLX90393FILTER::FILTER4,
            0x05 => MLX90393FILTER::FILTER5,
            0x06 => MLX90393FILTER::FILTER6,
            0x07 => MLX90393FILTER::FILTER7,
            _ => panic!("Invalid MLX90393FILTER"),
        }
    }
}

pub enum MLX90393OVERSAMPLING {
    OSR0,
    OSR1,
    OSR2,
    OSR3,
}

impl From<MLX90393OVERSAMPLING> for u8 {
    fn from(oversampling: MLX90393OVERSAMPLING) -> Self {
        oversampling as u8
    }
}

impl From<u8> for MLX90393OVERSAMPLING {
    fn from(oversampling: u8) -> Self {
        match oversampling {
            0x00 => MLX90393OVERSAMPLING::OSR0,
            0x01 => MLX90393OVERSAMPLING::OSR1,
            0x02 => MLX90393OVERSAMPLING::OSR2,
            0x03 => MLX90393OVERSAMPLING::OSR3,
            _ => panic!("Invalid MLX90393OVERSAMPLING"),
        }
    }
}
  
pub struct MLX90393Config {
    slave_address: u8,
    sda: AnyIOPin,
    scl: AnyIOPin,
}

impl MLX90393Config {
    pub fn new(slave_address: u8, sda: AnyIOPin, scl: AnyIOPin) -> Arc<Mutex<Self>> {
        let me = Self { slave_address, sda, scl };
        Arc::new(Mutex::new(me))
    }
}

pub struct MLX90393<'a> {
    i2c: Option<I2cDriver<'a>>,
    slave_address: u8,
    tx_end: Sender<bool>,
    rx_end: Arc<Mutex<Receiver<bool>>>,
}

impl<'a> MLX90393<'a> {
    #[allow(dead_code)]
    pub fn new(i2c: impl Peripheral<P = impl I2c> + 'a, config: Arc<Mutex<MLX90393Config>>) -> Result<Self, Box<dyn std::error::Error>> {
        let (tx_end, rx_end) = mpsc::channel::<bool>();
        let i2c = Self::init_i2c(i2c, config.clone())?;
        let me = Self { i2c: Some(i2c), slave_address: config.lock().unwrap().slave_address, tx_end, rx_end: Arc::new(Mutex::new(rx_end)) };
        me.init()?;
        Ok(me)
    }

    fn init_i2c(i2c: impl Peripheral<P = impl I2c> + 'a, config: Arc<Mutex<MLX90393Config>>) -> Result<I2cDriver<'a>, Box<dyn std::error::Error>> {
        let sda = unsafe { config.lock().unwrap().sda.clone_unchecked() };
        let scl = unsafe { config.lock().unwrap().scl.clone_unchecked() };
        let config = esp_idf_hal::i2c::I2cConfig::new().baudrate(Hertz(100000));
        let i2c = I2cDriver::new(i2c, sda, scl, &config)?;
        Ok(i2c)
    }

    #[allow(dead_code)]
    pub fn read_register(&mut self, register: MLX90393REG) -> Result<u16, Box<dyn std::error::Error>> {
        let tx_buf: [u8; 2] = [MLX90393CMD::RR.into(), (register as u8) << 2];
        let mut rx_buf: [u8; 3] = [0; 3];

        self.i2c.as_mut().unwrap().write(self.slave_address, &tx_buf, BLOCK)?;
        thread::sleep(Duration::from_millis(10));
        self.i2c.as_mut().unwrap().read(self.slave_address, &mut rx_buf, BLOCK)?;

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
        self.i2c.as_mut().unwrap().write(self.slave_address, &tx_buf, BLOCK)?;
        thread::sleep(Duration::from_millis(10));
        self.i2c.as_mut().unwrap().read(self.slave_address, &mut rx_buf, BLOCK)?;

        let status = rx_buf[0];
        let error = status & 0x10;
        if error != 0 {
            return Err(Box::new(std::io::Error::new(std::io::ErrorKind::Other, format!("MLX90393: write_register failed, status: {}", status))));
        }

        Ok(())
    }

    #[allow(dead_code)]
    pub fn set_gain(&mut self, new_gain: MLX90393GAIN) -> Result<(), Box<dyn std::error::Error>> {
        let mut gain = self.read_register(MLX90393REG::CONF1)?;
        gain &= !0x0070;
        gain |= (new_gain as u16) << 4;

        self.write_register(MLX90393REG::CONF1, gain)?;

        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_gain(&mut self) -> Result<MLX90393GAIN, Box<dyn std::error::Error>> {
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

        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_resolution(&mut self, axis: MLX90393AXIS) -> Result<MLX90393RESOLUTION, Box<dyn std::error::Error>> {
        let resolution = self.read_register(MLX90393REG::CONF3)?;
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
        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_filter(&mut self) -> Result<MLX90393FILTER, Box<dyn std::error::Error>> {
        let filter = self.read_register(MLX90393REG::CONF3)?;
        Ok(MLX90393FILTER::from(((filter & 0x1C) >> 2) as u8))
    }

    #[allow(dead_code)]
    pub fn set_oversampling(&mut self, new_oversampling: MLX90393OVERSAMPLING) -> Result<(), Box<dyn std::error::Error>> {
        let mut oversampling = self.read_register(MLX90393REG::CONF3)?;
        oversampling &= !0x03;
        oversampling |= new_oversampling as u16;
        self.write_register(MLX90393REG::CONF3, oversampling)?;
        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_oversampling(&mut self) -> Result<MLX90393OVERSAMPLING, Box<dyn std::error::Error>> {
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

        self.i2c.as_mut().unwrap().write(self.slave_address, &tx_buf, BLOCK)?;
        thread::sleep(Duration::from_millis(10));
        self.i2c.as_mut().unwrap().read(self.slave_address, &mut rx_buf, BLOCK)?;

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

        self.i2c.as_mut().unwrap().write(self.slave_address, &tx_buf, BLOCK)?;
        thread::sleep(Duration::from_millis(10));
        self.i2c.as_mut().unwrap().read(self.slave_address, &mut rx_buf, BLOCK)?;

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

    #[allow(dead_code)]
    pub fn start_wakeup_measurement(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let tx_buf: [u8; 1] = [MLX90393CMD::SW as u8 | MLX90393AXIS::ALL as u8];
        let mut rx_buf: [u8; 1] = [0; 1];

        self.i2c.as_mut().unwrap().write(self.slave_address, &tx_buf, BLOCK)?;
        thread::sleep(Duration::from_millis(10));
        self.i2c.as_mut().unwrap().read(self.slave_address, &mut rx_buf, BLOCK)?;

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

        self.i2c.as_mut().unwrap().write(self.slave_address, &tx_buf, BLOCK)?;
        thread::sleep(Duration::from_millis(10));
        self.i2c.as_mut().unwrap().read(self.slave_address, &mut rx_buf, BLOCK)?;

        let status = rx_buf[0];
        let error = status & 0x10;

        if error != 0 {
            return Err(Box::new(std::io::Error::new(std::io::ErrorKind::Other, format!("MLX90393: exit_mode failed, status: {}", status))));
        }

        Ok(())
    }

    fn init(&self) -> Result<(), Box<dyn std::error::Error>> {
        let rx_end = self.rx_end.clone();
        
        thread::Builder::new().stack_size(1024 * 20).spawn(move || {
            let executor = LocalExecutor::new();

            async fn sensor_thread(_executor: &LocalExecutor<'_>, rx_end: Arc<Mutex<Receiver<bool>>>) -> Result<(), Box<dyn std::error::Error>> {
                loop {
                    if let Ok(end) = rx_end.lock().unwrap().try_recv() {
                        if end {
                            break;
                        }
                    }
                    log::debug!("MLX90393: sensor_thread");
                    thread::sleep(Duration::from_millis(1000));
                }
                Ok(())
            }
    
            let fut = &mut pin!(sensor_thread(&executor, rx_end));
    
            if let Err(e) = async_io::block_on(executor.run(fut)) {
                log::error!("Error MLX90393 thread: {}", e);
            }

            log::info!("MLX90393: thread ended");
        })?;

        Ok(())
    }
}

impl<'a> MagSensor for MLX90393<'a> {
    fn get_angle(&self) -> Result<i32, Box<dyn std::error::Error>> {
        Ok(0)
    }
}

impl<'a> Drop for MLX90393<'a> {
    fn drop(&mut self) {
        log::debug!("MLX90393: drop");
    }
}

impl<'a> Endable for MLX90393<'a> {
    fn end(&self) {
        log::debug!("MLX90393: end");
        if let Err(e) = self.tx_end.send(true) {
            log::error!("Error sending end signal: {}", e);
        }
    }
}