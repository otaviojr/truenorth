use std::{pin::pin, sync::mpsc::{self, Receiver, Sender}, thread};
use std::sync::{Arc, Mutex};

use async_executor::LocalExecutor;
use esp_idf_svc::hal::prelude::*;
use esp_idf_hal::{gpio::AnyIOPin, ledc::{config::TimerConfig, LedcChannel, LedcDriver, LedcTimer, LedcTimerDriver, LowSpeed, Resolution}, peripheral::Peripheral};

use crate::Endable;

pub struct Motor<T, C> 
where 
    T: LedcTimer<SpeedMode = LowSpeed> + Peripheral + 'static,
    <T as Peripheral>::P: LedcTimer<SpeedMode = LowSpeed> + Peripheral + 'static,
    <<T as Peripheral>::P as Peripheral>::P: LedcTimer<SpeedMode = LowSpeed> + Peripheral + 'static,
    C: LedcChannel<SpeedMode = LowSpeed> + Peripheral + 'static,
    <C as Peripheral>::P: LedcChannel<SpeedMode = LowSpeed> + Peripheral + 'static,
    <<C as Peripheral>::P as Peripheral>::P: LedcChannel<SpeedMode = LowSpeed> + Peripheral + 'static
{
    pin: AnyIOPin,
    timer: T,
    channel: C,
    angle: u32,
    rx: Arc<Mutex<Receiver<u32>>>,
    tx: Sender<u32>,
    end_tx: Sender<bool>,
    end_rx: Arc<Mutex<Receiver<bool>>>
}

impl<T, C> Motor<T, C> 
where 
    T: LedcTimer<SpeedMode = LowSpeed> + Peripheral + 'static,
    <T as Peripheral>::P: LedcTimer<SpeedMode = LowSpeed> + Peripheral + 'static,
    <<T as Peripheral>::P as Peripheral>::P: LedcTimer<SpeedMode = LowSpeed> + Peripheral + 'static,
    C: LedcChannel<SpeedMode = LowSpeed> + Peripheral + 'static,
    <C as Peripheral>::P: LedcChannel<SpeedMode = LowSpeed> + Peripheral + 'static,
    <<C as Peripheral>::P as Peripheral>::P: LedcChannel<SpeedMode = LowSpeed> + Peripheral + 'static
{
    #[allow(dead_code)]
    pub fn new(pin: AnyIOPin, timer: T, channel: C) -> Result<Self, Box<dyn std::error::Error>> {
        let (tx_motor, rx_motor) = mpsc::channel::<u32>();
        let (tx_end, rx_end) = mpsc::channel::<bool>();
        let mut me = Self { pin, timer, channel, angle: 0, rx: Arc::new(Mutex::new(rx_motor)), tx: tx_motor, end_tx: tx_end, end_rx: Arc::new(Mutex::new(rx_end)) };
        me.setup()?;
        Ok(me)
    }

    #[allow(dead_code)]
    fn setup(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let motor_pwm_pin = unsafe { self.pin.clone_unchecked() };
        let timer_driver = LedcTimerDriver::new(unsafe { self.timer.clone_unchecked() }, &TimerConfig::default().frequency(50.Hz().into()).resolution(Resolution::Bits13))?;
        let mut driver = LedcDriver::new(unsafe { self.channel.clone_unchecked() }, timer_driver, motor_pwm_pin)?;
    
        let rx = self.rx.clone();
        let end_rx = self.end_rx.clone();

        thread::Builder::new().stack_size(1024 * 20).spawn(move || {
            let executor = LocalExecutor::new();

            async fn send(_executor: &LocalExecutor<'_>, rx: Arc<Mutex<Receiver<u32>>>, end_rx: Arc<Mutex<Receiver<bool>>>, driver: &mut LedcDriver<'_>) -> Result<(), Box<dyn std::error::Error>> {
                loop {
                    if let Ok(end) = end_rx.lock().unwrap().try_recv() {
                        if end {
                            break;
                        }
                    }

                    if let Ok(angle) = rx.lock().unwrap().try_recv() {
                        let time = ((angle * (2500 - 500)) / 180) + 500;
    
                        log::debug!("angle: {}", angle);
                        log::debug!("time: {}us", time);
    
                        let max_duty = driver.get_max_duty();
                        let duty_value = (time * (max_duty as u32)) / 20000;
    
                        log::debug!("max_duty: {}", max_duty);
                        log::debug!("duty_value: {}", duty_value);
    
                        driver.set_duty(duty_value)?;
                    }

                    log::debug!("Motor: Sleeping");
                    thread::sleep(std::time::Duration::from_millis(1000));
                }

                Ok(())
            }
    
            let fut = &mut pin!(send(&executor, rx, end_rx, &mut driver));
    
            if let Err(e) = async_io::block_on(executor.run(fut)) {
                log::error!("Error running motor pwm thread: {}", e);
            }

            log::info!("Motor: thread ended");
        })?;
    
        Ok(())
    }

    #[allow(dead_code)]
    pub fn set_angle(&mut self, angle: i32) -> Result<(), Box<dyn std::error::Error>> {

        if angle > 180 || angle < 0 {
            return Err("Angle must be between 0 and 180".into());
        }   

        self.angle = angle as u32;
        self.tx.send(angle as u32)?;
        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_angle(&self) -> u32 {
        self.angle
    }
}

impl<T, C> Drop for Motor<T, C>
where
    T: LedcTimer<SpeedMode = LowSpeed> + Peripheral + 'static,
    <T as Peripheral>::P: LedcTimer<SpeedMode = LowSpeed> + Peripheral + 'static,
    <<T as Peripheral>::P as Peripheral>::P: LedcTimer<SpeedMode = LowSpeed> + Peripheral + 'static,
    C: LedcChannel<SpeedMode = LowSpeed> + Peripheral + 'static,
    <C as Peripheral>::P: LedcChannel<SpeedMode = LowSpeed> + Peripheral + 'static,
    <<C as Peripheral>::P as Peripheral>::P: LedcChannel<SpeedMode = LowSpeed> + Peripheral + 'static
{
    fn drop(&mut self) {
        log::debug!("Motor: drop");
    }
}

impl<T, C> Endable for Motor<T, C>
where
    T: LedcTimer<SpeedMode = LowSpeed> + Peripheral + 'static,
    <T as Peripheral>::P: LedcTimer<SpeedMode = LowSpeed> + Peripheral + 'static,
    <<T as Peripheral>::P as Peripheral>::P: LedcTimer<SpeedMode = LowSpeed> + Peripheral + 'static,
    C: LedcChannel<SpeedMode = LowSpeed> + Peripheral + 'static,
    <C as Peripheral>::P: LedcChannel<SpeedMode = LowSpeed> + Peripheral + 'static,
    <<C as Peripheral>::P as Peripheral>::P: LedcChannel<SpeedMode = LowSpeed> + Peripheral + 'static
{
    fn end(&self) {
        log::debug!("Motor: end");
        if let Err(e) = self.end_tx.send(true) {
            log::error!("Error sending end signal: {}", e);
        }
    }
}
