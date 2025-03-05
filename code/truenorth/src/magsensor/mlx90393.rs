
use std::{num::NonZero, pin::pin, sync::{Arc, Mutex}, thread};

use async_executor::LocalExecutor;
use esp_idf_svc::hal::{gpio::{AnyIOPin, PinDriver, Pull, InterruptType}, i2c::{I2c, I2cDriver}, peripheral::Peripheral, units::Hertz};
use esp_idf_svc::hal::task::notification::Notification;


use crate::{magsensor::MagSensor, TrueNorthParameters};
use crate::magsensor::mlx90393_defs::*;
use crate::magsensor::mlx90393_inner::MLX90393Inner;

pub struct MLX90393Config {
    slave_address: u8,
    sda: AnyIOPin,
    scl: AnyIOPin,
    int: AnyIOPin,
    parameters: Arc<TrueNorthParameters>,
}

impl MLX90393Config {
    pub fn new(parameters: Arc<TrueNorthParameters>, slave_address: u8, sda: AnyIOPin, scl: AnyIOPin, int: AnyIOPin) -> Arc<Mutex<Self>> {
        let me = Self { parameters, slave_address, sda, scl, int };
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
        
        let mut config = config.lock().unwrap();

        let me = Self { 
            inner: Arc::new(Mutex::new(MLX90393Inner { 
                i2c: Some(i2c), 
                int: unsafe { config.int.clone_unchecked() },
                slave_address: config.slave_address, 
                current_gain: None, 
                current_resolution: None, 
                current_filter: None, 
                current_oversampling: None,
                parameters: config.parameters.clone()
            })),
        };
        
        me.init()?;

        Ok(me)
    }

    fn init(&self) -> Result<(), Box<dyn std::error::Error>> {
        let shared_self = self.inner.clone();

        self.configure()?;
    
        async fn monitor(_executor: &LocalExecutor<'_>, me: Arc<Mutex<MLX90393Inner>>) -> Result<(), Box<dyn std::error::Error>> {        

            let int = unsafe { me.lock().unwrap().int.clone_unchecked() };

            let mut interrupt_pin = {
                if let Ok(iopin) = PinDriver::input(int) {
                    iopin
                } else {
                    log::error!("Error setting up interruption");
                    return Err(Box::new(std::io::Error::new(std::io::ErrorKind::Other, "Error setting up interruption")));
                }
            };
        
            interrupt_pin.set_pull(Pull::Down).unwrap();
            interrupt_pin.set_interrupt_type(InterruptType::PosEdge).unwrap();
        
            let notification = Notification::new();
            let waker = notification.notifier();

            unsafe {
                interrupt_pin
                    .subscribe_nonstatic(move || {
                        waker.notify(NonZero::new(1).unwrap());
                    })
                    .unwrap();
            }

            me.lock().unwrap().start_burst_measurement()?;

            loop {

                log::debug!("monitor thread...");

                if let Err(e) = interrupt_pin.enable_interrupt() {
                    log::error!("Error enabling interrupt: {}", e);
                }
        
                notification.wait_any();

                {
                    let mut lock_me = me.lock().unwrap();
        
                    match lock_me.read_measurement() {
                        Ok(measurement) => {
                            log::debug!("Measurement: {:?}", measurement);
                            let x = measurement[0];
                            let y = measurement[1];
                            let z = measurement[2];
    
                            let parameters = lock_me.parameters.clone();
    
                            let mut max_x = parameters.max_x.lock().unwrap(); 
                            let mut min_x = parameters.min_x.lock().unwrap();
                            let mut max_y = parameters.max_y.lock().unwrap();
                            let mut min_y = parameters.min_y.lock().unwrap();
                            let mut max_z = parameters.max_z.lock().unwrap();
                            let mut min_z = parameters.min_z.lock().unwrap();
    
                            if x > *max_x.get() {
                                if let Err(e) = max_x.set(x) {
                                    log::error!("Error setting max_x: {}", e);
                                }
                            }
                            if x < *min_x.get() {
                                if let Err(e) = min_x.set(x) {
                                    log::error!("Error setting min_x: {}", e);
                                }
                            }
                            if y > *max_y.get() {
                                if let Err(e) = max_y.set(y) {
                                    log::error!("Error setting max_y: {}", e);
                                }
                            }
                            if y < *min_y.get() {
                                if let Err(e) = min_y.set(y) {
                                    log::error!("Error setting min_y: {}", e);
                                }
                            }
                            if z > *max_z.get() {
                                if let Err(e) = max_z.set(z) {
                                    log::error!("Error setting max_z: {}", e);
                                }
                            }
                            if z < *min_z.get() {
                                if let Err(e) = min_z.set(z) {
                                    log::error!("Error setting min_z: {}", e);
                                }
                            }
    
                            let mut heading =  (y.atan2(x) * 180.0) / std::f32::consts::PI;
                            log::debug!("Heading 1: {}", heading);
                            heading = heading.abs();
                            log::debug!("Heading 2: {}", heading);
                            if heading > 180.0 {
                                heading = heading - 180.0;
                            }
                            log::debug!("Heading 3: {}", heading);
                        }
                        Err(e) => log::error!("Error reading measurement: {}", e),
                    }        
                        
                }
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

    fn configure(&self) -> Result<(), Box<dyn std::error::Error>> {
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

        Ok(())
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