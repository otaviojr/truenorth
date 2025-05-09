use core::f32;
use std::{
    num::NonZero,
    sync::{Arc, Mutex},
    thread,
    time::{Duration, Instant},
};

use esp_idf_svc::hal::task::notification::Notification;
use esp_idf_svc::hal::{
    gpio::{AnyIOPin, InterruptType, PinDriver, Pull},
    i2c::{I2c, I2cDriver},
    peripheral::Peripheral,
    units::Hertz,
};

use super::MagSensorHandlerPtr;
use crate::magsensor::mlx90393_defs::*;
use crate::magsensor::mlx90393_inner::{MLX90393Inner, MLX90393Internal};
use crate::math::{LowPassFilter, Vector3};
use crate::{
    magsensor::{MagSensor, MagSensorEvent, MagSensorState},
    Endable, TrueNorthParameters,
};

const CALIBRATION_SAMPLES: usize = 30;
const MEASUREMENT_SAMPLES: usize = 100;

const CALIBRATION_SAMPLE_TIME: u128 = 10;
const MEASUREMENT_SAMPLE_TIME: u128 = 1000;

pub struct MLX90393Config {
    slave_address: u8,
    sda: AnyIOPin,
    scl: AnyIOPin,
    int: AnyIOPin,
    parameters: Arc<TrueNorthParameters>,
}

impl MLX90393Config {
    pub fn new(
        parameters: Arc<TrueNorthParameters>,
        slave_address: u8,
        sda: AnyIOPin,
        scl: AnyIOPin,
        int: AnyIOPin,
    ) -> Arc<Mutex<Self>> {
        let me = Self {
            parameters,
            slave_address,
            sda,
            scl,
            int,
        };
        Arc::new(Mutex::new(me))
    }
}

pub struct MLX90393 {
    inner: Arc<Mutex<MLX90393Inner>>,
}

impl MLX90393 {
    #[allow(dead_code)]
    pub fn new(
        i2c: impl Peripheral<P = impl I2c> + 'static,
        config: Arc<Mutex<MLX90393Config>>,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let i2c = Self::init_i2c(i2c, config.clone())?;

        let mut config = config.lock().unwrap();

        let me = Self {
            inner: Arc::new(Mutex::new(MLX90393Inner {
                i2c: Some(i2c),
                int: unsafe { config.int.clone_unchecked() },
                slave_address: config.slave_address,
                parameters: config.parameters.clone(),
                internal: MLX90393Internal::default(),
            })),
        };

        me.init()?;

        Ok(me)
    }

    fn init(&self) -> Result<(), Box<dyn std::error::Error>> {
        let shared_self = self.inner.clone();

        self.configure()?;

        thread::Builder::new().spawn(move || {
            let int = unsafe { shared_self.lock().unwrap().int.clone_unchecked() };

            let mut interrupt_pin = {
                if let Ok(iopin) = PinDriver::input(int) {
                    iopin
                } else {
                    log::error!("Error setting up interruption");
                    return Err(Box::new(std::io::Error::new(
                        std::io::ErrorKind::Other,
                        "Error setting up interruption",
                    )));
                }
            };

            interrupt_pin.set_pull(Pull::Down).unwrap();
            interrupt_pin
                .set_interrupt_type(InterruptType::PosEdge)
                .unwrap();

            let notification = Notification::new();
            let waker = notification.notifier();

            unsafe {
                interrupt_pin
                    .subscribe_nonstatic(move || {
                        waker.notify(NonZero::new(1).unwrap());
                    })
                    .unwrap();
            }

            //me.lock().unwrap().start_burst_measurement()?;

            let mut value = LowPassFilter::new(0.5);
            let mut pool = vec![];
            let mut avg = Vector3::new(0.0, 0.0, 0.0);

            let mut measure_event = MagSensorEvent::HeadingChanged(0);

            let mut current_time = Instant::now();

            'thread_loop: loop {
                {
                    let lock_me = shared_self.lock().unwrap();
                    let lock_channel = lock_me.internal.channel.lock().unwrap();

                    if let Ok(end) = lock_channel.1.try_recv() {
                        if end {
                            break 'thread_loop;
                        }
                    }
                }

                //log::debug!("monitor thread...");

                if let Err(e) = interrupt_pin.enable_interrupt() {
                    log::error!("Error enabling interrupt: {}", e);
                }

                if let Some(_ret) = notification.wait(100) {
                    let mut lock_me = shared_self.lock().unwrap();

                    match lock_me.read_measurement() {
                        Ok(measurement) => {
                            //log::debug!("Measurement: {:?}", measurement);
                            let x = measurement[0];
                            let y = measurement[1];
                            let z = measurement[2];

                            if current_time.elapsed().as_millis()
                                > if lock_me.internal.state == MagSensorState::Calibrating {
                                    CALIBRATION_SAMPLE_TIME
                                } else {
                                    MEASUREMENT_SAMPLE_TIME
                                }
                            {
                                pool.push(value.update(Vector3 { x, y, z }));
                                current_time = Instant::now();

                                if pool.len() > MEASUREMENT_SAMPLES {
                                    pool.remove(0);
                                }

                                avg = {
                                    let mut sum_x = 0.0;
                                    let mut sum_y = 0.0;
                                    let mut sum_z = 0.0;

                                    let len =
                                        if lock_me.internal.state == MagSensorState::Calibrating {
                                            if pool.len() < CALIBRATION_SAMPLES {
                                                pool.len()
                                            } else {
                                                CALIBRATION_SAMPLES
                                            }
                                        } else {
                                            pool.len()
                                        };

                                    for i in pool.len() - len..pool.len() {
                                        sum_x += pool[i].x;
                                        sum_y += pool[i].y;
                                        sum_z += pool[i].z;
                                    }

                                    Vector3 {
                                        x: sum_x / len as f32,
                                        y: sum_y / len as f32,
                                        z: sum_z / len as f32,
                                    }
                                };
                            }

                            let event = MagSensorEvent::RawChanged(avg);
                            if let Err(e) = lock_me.send_event(event) {
                                log::error!("Error sending event: {}", e);
                            }

                            let parameters = lock_me.parameters.clone();

                            let mut max_x = parameters.max_x.lock().unwrap();
                            let mut min_x = parameters.min_x.lock().unwrap();
                            let mut max_y = parameters.max_y.lock().unwrap();
                            let mut min_y = parameters.min_y.lock().unwrap();
                            let mut max_z = parameters.max_z.lock().unwrap();
                            let mut min_z = parameters.min_z.lock().unwrap();

                            if lock_me.internal.state == MagSensorState::Calibrating {
                                let mut changed = false;
                                if avg.x > *max_x.get() {
                                    if let Err(e) = max_x.set(avg.x) {
                                        log::error!("Error setting max_x: {}", e);
                                    }
                                    changed = true;
                                }
                                if avg.x < *min_x.get() {
                                    if let Err(e) = min_x.set(avg.x) {
                                        log::error!("Error setting min_x: {}", e);
                                    }
                                    changed = true;
                                }
                                if avg.y > *max_y.get() {
                                    if let Err(e) = max_y.set(avg.y) {
                                        log::error!("Error setting max_y: {}", e);
                                    }
                                    changed = true;
                                }
                                if avg.y < *min_y.get() {
                                    if let Err(e) = min_y.set(avg.y) {
                                        log::error!("Error setting min_y: {}", e);
                                    }
                                    changed = true;
                                }
                                if avg.z > *max_z.get() {
                                    if let Err(e) = max_z.set(avg.z) {
                                        log::error!("Error setting max_z: {}", e);
                                    }
                                    changed = true;
                                }
                                if avg.z < *min_z.get() {
                                    if let Err(e) = min_z.set(avg.z) {
                                        log::error!("Error setting min_z: {}", e);
                                    }
                                    changed = true;
                                }

                                if changed {
                                    let event = MagSensorEvent::CalibratedChanged(
                                        (*max_x.get(), *min_x.get()),
                                        (*max_y.get(), *min_y.get()),
                                        (*max_z.get(), *min_z.get()),
                                    );

                                    if let Err(e) = lock_me.send_event(event) {
                                        log::error!("Error sending event: {}", e);
                                    }
                                }
                            } else if lock_me.internal.state == MagSensorState::Measuring {
                                let calc_x =
                                    (x + avg.x) / 2.0 - ((*max_x.get() + *min_x.get()) / 2.0);
                                let calc_y =
                                    (y + avg.y) / 2.0 - ((*max_y.get() + *min_y.get()) / 2.0);

                                let mut heading =
                                    (calc_x.atan2(calc_y) * 180.0) / std::f32::consts::PI;

                                if heading < 0.0 {
                                    heading = heading + 360.0;
                                }

                                let value = match measure_event.clone() {
                                    MagSensorEvent::HeadingChanged(value) => value,
                                    _ => 0,
                                };

                                // Calculate the difference between the current heading and the previous heading in degrees
                                let diff = if (value - heading.round() as i32).abs() > 180 {
                                    360 - (value - heading.round() as i32).abs()
                                } else {
                                    (value - heading.round() as i32).abs()
                                };

                                if diff > 2 {
                                    measure_event =
                                        MagSensorEvent::HeadingChanged(heading.round() as i32);
                                    if let Err(e) = lock_me.send_event(measure_event.clone()) {
                                        log::error!("Error sending event: {}", e);
                                    }
                                }
                            }
                        }
                        Err(e) => log::error!("Error reading measurement: {}", e),
                    }
                }
            }

            log::info!("MLX90393: thread ended");
            Ok(())
        })?;

        Ok(())
    }

    fn init_i2c(
        i2c: impl Peripheral<P = impl I2c> + 'static,
        config: Arc<Mutex<MLX90393Config>>,
    ) -> Result<I2cDriver<'static>, Box<dyn std::error::Error>> {
        let sda = unsafe { config.lock().unwrap().sda.clone_unchecked() };
        let scl = unsafe { config.lock().unwrap().scl.clone_unchecked() };
        let config = esp_idf_hal::i2c::I2cConfig::new().baudrate(Hertz(100000));
        let i2c = I2cDriver::new(i2c, sda, scl, &config)?;
        Ok(i2c)
    }

    fn configure(&self) -> Result<(), Box<dyn std::error::Error>> {
        thread::sleep(std::time::Duration::from_millis(100));
        if let Err(e) = self.exit_mode() {
            log::warn!("Error exiting mode: {}", e);
        }
        thread::sleep(std::time::Duration::from_millis(100));
        if let Err(e) = self.reset() {
            log::warn!("Error resetting magnetometer: {}", e);
        }
        thread::sleep(std::time::Duration::from_millis(2000));

        self.set_gain(MLX90393GAIN::GAIN1X)?;
        self.set_resolution(MLX90393AXIS::X, MLX90393RESOLUTION::RES19)?;
        self.set_resolution(MLX90393AXIS::Y, MLX90393RESOLUTION::RES19)?;
        self.set_resolution(MLX90393AXIS::Z, MLX90393RESOLUTION::RES16)?;
        self.set_oversampling(MLX90393OVERSAMPLING::OSR3)?;
        self.set_filter(MLX90393FILTER::FILTER5)?;

        Ok(())
    }

    pub fn read_register(&self, register: MLX90393REG) -> Result<u16, Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().read_register(register)
    }

    pub fn write_register(
        &self,
        register: MLX90393REG,
        value: u16,
    ) -> Result<(), Box<dyn std::error::Error>> {
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

    pub fn set_resolution(
        &self,
        axis: MLX90393AXIS,
        new_resolution: MLX90393RESOLUTION,
    ) -> Result<(), Box<dyn std::error::Error>> {
        self.inner
            .lock()
            .unwrap()
            .set_resolution(axis, new_resolution)
    }

    pub fn get_resolution(
        &self,
        axis: MLX90393AXIS,
    ) -> Result<MLX90393RESOLUTION, Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().get_resolution(axis)
    }

    pub fn set_filter(&self, new_filter: MLX90393FILTER) -> Result<(), Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().set_filter(new_filter)
    }

    pub fn get_filter(&self) -> Result<MLX90393FILTER, Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().get_filter()
    }

    pub fn set_oversampling(
        &self,
        new_oversampling: MLX90393OVERSAMPLING,
    ) -> Result<(), Box<dyn std::error::Error>> {
        self.inner
            .lock()
            .unwrap()
            .set_oversampling(new_oversampling)
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
    fn calibrate(&self, timeout: Duration) -> Result<(), Box<dyn std::error::Error>> {
        {
            let mut inner_lock = self.inner.lock().unwrap();

            if let Err(e) = inner_lock.exit_mode() {
                log::warn!("Error exiting mode: {}", e);
            }

            thread::sleep(Duration::from_millis(100));

            inner_lock.start_burst_measurement()?;
            inner_lock.set_state(MagSensorState::Calibrating);

            log::debug!("Magnetometer: Calibrating");
        }
        thread::sleep(timeout);
        {
            let mut inner_lock = self.inner.lock().unwrap();
            if let Err(e) = inner_lock.exit_mode() {
                log::warn!("Error exiting mode: {}", e);
            }
            thread::sleep(Duration::from_millis(100));
            inner_lock.set_state(MagSensorState::Idle);
            log::debug!("Magnetometer: Calibration complete");
        }
        Ok(())
    }

    fn add_handler(&self, handler: MagSensorHandlerPtr) -> Result<(), Box<dyn std::error::Error>> {
        self.inner.lock().unwrap().add_handler(handler)
    }

    fn start(&self) -> Result<(), Box<dyn std::error::Error>> {
        let mut inner_lock = self.inner.lock().unwrap();
        if let Err(e) = inner_lock.exit_mode() {
            log::warn!("Error exiting mode: {}", e);
        }
        thread::sleep(Duration::from_millis(100));
        inner_lock.set_wakeup_comparator(true)?;
        inner_lock.start_wakeup_measurement()?;
        inner_lock.set_state(MagSensorState::Measuring);

        log::debug!("Magnetometer: Measurement started");
        Ok(())
    }
}

impl Endable for MLX90393 {
    fn end(&self) {
        let mut inner_lock = self.inner.lock().unwrap();
        inner_lock.set_state(MagSensorState::Idle);
        if let Err(e) = inner_lock.internal.channel.lock().unwrap().0.send(true) {
            log::error!("Error sending end signal: {}", e);
        }
        log::debug!("MLX90393: end");
    }
}

impl Drop for MLX90393 {
    fn drop(&mut self) {
        log::debug!("MLX90393: drop");
    }
}
