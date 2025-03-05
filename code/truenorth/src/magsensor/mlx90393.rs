
use std::{pin::pin, sync::{Arc, Mutex}, thread};

use async_executor::LocalExecutor;
use esp_idf_hal::{gpio::AnyIOPin, i2c::{I2c, I2cDriver}, peripheral::Peripheral, units::Hertz};

use crate::magsensor::MagSensor;
use crate::magsensor::mlx90393_defs::*;
use crate::magsensor::mlx90393_inner::MLX90393Inner;

  
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


pub struct MLX90393 {
    inner: Arc<Mutex<MLX90393Inner>>,
}

impl MLX90393 {
    #[allow(dead_code)]
    pub fn new(i2c: impl Peripheral<P = impl I2c> + 'static, config: Arc<Mutex<MLX90393Config>>) -> Result<Self, Box<dyn std::error::Error>> {
        let i2c = Self::init_i2c(i2c, config.clone())?;
        
        let me = Self { 
            inner: Arc::new(Mutex::new(MLX90393Inner { 
                i2c: Some(i2c), 
                slave_address: config.lock().unwrap().slave_address, 
                current_gain: None, 
                current_resolution: None, 
                current_filter: None, 
                current_oversampling: None 
            }))
        };
        
        me.init()?;

        Ok(me)
    }

    fn init(&self) -> Result<(), Box<dyn std::error::Error>> {
        let shared_self = self.inner.clone();

        thread::sleep(std::time::Duration::from_millis(10));
        if let Err(e) = self.exit_mode() {
            log::warn!("Error exiting mode: {}", e);
        }
        thread::sleep(std::time::Duration::from_millis(10));
        if let Err(e) = self.reset(){
            log::warn!("Error resetting magnetometer: {}", e);
        }
        thread::sleep(std::time::Duration::from_millis(100));

        self.set_gain(MLX90393GAIN::GAIN1X)?;
        self.set_resolution(MLX90393AXIS::X, MLX90393RESOLUTION::RES17)?;
        self.set_resolution(MLX90393AXIS::Y, MLX90393RESOLUTION::RES17)?;
        self.set_resolution(MLX90393AXIS::Z, MLX90393RESOLUTION::RES16)?;
        self.set_oversampling(MLX90393OVERSAMPLING::OSR3)?;
        self.set_filter(MLX90393FILTER::FILTER5)?;
        self.start_burst_measurement()?;

        async fn monitor(_executor: &LocalExecutor<'_>, me: Arc<Mutex<MLX90393Inner>>) -> Result<(), Box<dyn std::error::Error>> {        
            loop {
                log::debug!("monitor thread...");
                thread::sleep(std::time::Duration::from_secs(1));
            }
        }

        let shared_self_clone = Arc::clone(&shared_self);
        thread::Builder::new().stack_size(20000).spawn(move || {
            let executor = LocalExecutor::new();
            let fut = &mut pin!(monitor(&executor, shared_self_clone));

            async_io::block_on(executor.run(fut)).unwrap();
        })?;

        Ok(())
    }

    fn init_i2c(i2c: impl Peripheral<P = impl I2c> + 'static, config: Arc<Mutex<MLX90393Config>>) -> Result<I2cDriver<'static>, Box<dyn std::error::Error>> {
        let sda = unsafe { config.lock().unwrap().sda.clone_unchecked() };
        let scl = unsafe { config.lock().unwrap().scl.clone_unchecked() };
        let config = esp_idf_hal::i2c::I2cConfig::new().baudrate(Hertz(100000));
        let i2c = I2cDriver::new(i2c, sda, scl, &config)?;
        Ok(i2c)
    }

    pub fn read_register(&self, register: MLX90393REG) -> Result<u16, Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().read_register(register)
    }

    pub fn write_register(&self, register: MLX90393REG, value: u16) -> Result<(), Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().write_register(register, value)
    }

    pub fn read_measurement(&self) -> Result<[f32; 3], Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().read_measurement()
    }

    pub fn set_gain(&self, new_gain: MLX90393GAIN) -> Result<(), Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().set_gain(new_gain)
    }

    pub fn get_gain(&self) -> Result<MLX90393GAIN, Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().get_gain()
    }

    pub fn set_resolution(&self, axis: MLX90393AXIS, new_resolution: MLX90393RESOLUTION) -> Result<(), Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().set_resolution(axis, new_resolution)
    }

    pub fn get_resolution(&self, axis: MLX90393AXIS) -> Result<MLX90393RESOLUTION, Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().get_resolution(axis)
    }

    pub fn set_filter(&self, new_filter: MLX90393FILTER) -> Result<(), Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().set_filter(new_filter)
    }

    pub fn get_filter(&self) -> Result<MLX90393FILTER, Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().get_filter()
    }

    pub fn set_oversampling(&self, new_oversampling: MLX90393OVERSAMPLING) -> Result<(), Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().set_oversampling(new_oversampling)
    }

    pub fn get_oversampling(&self) -> Result<MLX90393OVERSAMPLING, Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().get_oversampling()
    }

    pub fn set_trigger_interval(&self, state: bool) -> Result<(), Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().set_trigger_interval(state)
    }

    pub fn start_single_measurement(&self) -> Result<(), Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().start_single_measurement()
    }

    pub fn start_burst_measurement(&self) -> Result<(), Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().start_burst_measurement()
    }

    pub fn start_wakeup_measurement(&self) -> Result<(), Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().start_wakeup_measurement()
    }

    pub fn exit_mode(&self) -> Result<(), Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().exit_mode()
    }

    pub fn reset(&self) -> Result<(), Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().reset()
    }    
}

impl MagSensor for MLX90393 {
    fn get_angle(&self) -> Result<i32, Box<dyn std::error::Error>> {
        Ok(0)
    }
}

impl Drop for MLX90393 {
    fn drop(&mut self) {
        log::debug!("MLX90393: drop");
    }
}