#![feature(impl_trait_in_bindings)]

pub mod motor;
pub mod smartvar;
pub mod magsensor;

use async_executor::LocalExecutor;
use esp32_nimble::utilities::BleUuid;
use esp32_nimble::NimbleProperties;
use magsensor::mlx90393::{MLX90393Config, MLX90393AXIS, MLX90393FILTER, MLX90393GAIN, MLX90393OVERSAMPLING, MLX90393RESOLUTION};
use std::any::Any;
use std::collections::HashMap;
use std::thread;
use rand::Rng;
use std::sync::Mutex;
use std::cell::RefCell;
use std::sync::Arc;
use std::pin::pin;
use async_io;
use esp_idf_svc::hal::prelude::*;
use esp32_nimble::{BLEDevice, BLEAdvertisementData, BLECharacteristic, enums::{ConnMode, DiscMode, AuthReq, SecurityIOCap}};

use crate::motor::Motor;
use crate::smartvar::SmartVar;

use crate::magsensor::mlx90393::MLX90393;

thread_local! {
    #[allow(clippy::thread_local_initializer_can_be_made_const)]
    static TAG_NAMESPACE:RefCell<&'static str> =  RefCell::new("truenorth");
    static TAG_DECLINATION:RefCell<&'static str> =  RefCell::new("declination");
}

pub struct TrueNorthParameters {
    pub declination: Arc<Mutex<SmartVar<i32>>>,
}

pub trait Endable {
    fn end(&self);
}

pub struct EndableHandler {
    pub objects: Vec<Arc<Mutex<dyn Endable>>>
}

impl EndableHandler {
    pub fn new() -> Self {
        Self { objects: Vec::new() }
    }

    #[allow(dyn_drop)]
    pub fn add(&mut self, object: Arc<Mutex<dyn Endable>>) {
        self.objects.push(object);
    }

    pub fn end_all(&mut self) {
        loop {
            let object = self.objects.pop();
            if let Some(object) = object {
                object.lock().unwrap().end();
            } else {
                break;
            }
        }
    }
}

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let mut endable = EndableHandler::new();

    let parameters = Arc::new(TrueNorthParameters {
        declination: SmartVar::new(0),
    });

    #[allow(unused)]

    let mut peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    let motor = match Motor::new(pins.gpio0.into(), peripherals.ledc.timer0, peripherals.ledc.channel0) {
        Ok(motor) => Arc::new(Mutex::new(motor)),
        Err(error) => {
            log::error!("Error setting up motor: {}", error);
            halt_system(&mut endable);
            return;
        }
    };

    endable.add(motor.clone());

    let config = MLX90393Config::new(0x0C, pins.gpio8.into(), pins.gpio9.into());
    
    let mag = match MLX90393::new(peripherals.i2c0, config) {
        Ok(mag) => Arc::new(Mutex::new(mag)),
        Err(_error) => {
            halt_system(&mut endable);
            return;
        }
    };

    endable.add(mag.clone());

    if let Err(error) = configure_mag(mag.clone()) {
        log::error!("Error configuring magnetometer: {}", error);
        halt_system(&mut endable);
        return;
    }
    
    if let Err(err) = setup_bt_server(parameters.clone()) {
        log::error!("Error setting up advertisement: {}", err);
    }

    /*
        Setup storage after all services are running.

        This will ensure that all loaded values from NVS partition will be sended to all handlers.
    */
    if let Err(err) = parameters.clone().declination.lock().unwrap().setup_storage(TAG_NAMESPACE.take().to_string(), TAG_DECLINATION.take().to_string()) {
        log::error!("Error setting up declination storage: {}", err);
    }

    loop {
        if let Err(e) = mag.lock().unwrap().start_single_measurement() {
            log::error!("Error starting single measurement: {}", e);
        }
        thread::sleep(std::time::Duration::from_millis(1000));
        match mag.lock().unwrap().read_measurement() {
            Ok(measurement) => {
                log::debug!("Measurement: {:?}", measurement);
                let x = measurement[0];
                let y = measurement[1];
                //let z = measurement[2];
                
                let mut heading =  (y.atan2(x) * 180.0) / std::f32::consts::PI;
                log::debug!("Heading 1: {}", heading);
                heading = heading.abs();
                log::debug!("Heading 2: {}", heading);
                if heading > 180.0 {
                    heading = heading - 180.0;
                }
                log::debug!("Heading 3: {}", heading);

                if let Err(e) = motor.lock().unwrap().set_angle(heading as i32) {
                    log::error!("Error sending message: {}", e);
                }
            }
            Err(e) => log::error!("Error reading measurement: {}", e),
        }
        thread::sleep(std::time::Duration::from_millis(1000));

        /*if let Err(e) = motor.lock().unwrap().set_angle(0) {
            log::error!("Error sending message: {}", e);
        }
        thread::sleep(std::time::Duration::from_secs(60));
        if let Err(e) = motor.lock().unwrap().set_angle(90) {
            log::error!("Error sending message: {}", e);
        }
        thread::sleep(std::time::Duration::from_secs(60));
        if let Err(e) = motor.lock().unwrap().set_angle(180) {
            log::error!("Error sending message: {}", e);
        }
        thread::sleep(std::time::Duration::from_secs(60));
        
        let angle = rand::thread_rng().gen_range(0..181);
        if let Err(e) = motor.lock().unwrap().set_angle(angle) {
            log::error!("Error sending message: {}", e);
        }
        thread::sleep(std::time::Duration::from_secs(60));*/
    }
}

#[allow(unused)]
fn halt_system(endable: &mut EndableHandler) {
    log::error!("Halting system");
    endable.end_all();
    loop {
        thread::sleep(std::time::Duration::from_secs(60));
    }
}

#[allow(unused)]
fn configure_mag(mag: Arc<Mutex<MLX90393>>) -> Result<(), Box<dyn std::error::Error>> {
    mag.lock().unwrap().set_gain(MLX90393GAIN::GAIN1X)?;
    mag.lock().unwrap().set_resolution(MLX90393AXIS::X, MLX90393RESOLUTION::RES17)?;
    mag.lock().unwrap().set_resolution(MLX90393AXIS::Y, MLX90393RESOLUTION::RES17)?;
    mag.lock().unwrap().set_resolution(MLX90393AXIS::Z, MLX90393RESOLUTION::RES16)?;
    mag.lock().unwrap().set_oversampling(MLX90393OVERSAMPLING::OSR3)?;
    mag.lock().unwrap().set_filter(MLX90393FILTER::FILTER5)?;

    Ok(()) 
}

fn setup_bt_server(parameters: Arc<TrueNorthParameters>) -> Result<(), Box<dyn std::error::Error>> {

    let ble_device = Arc::new(Mutex::new(BLEDevice::take()));

    async fn ble_server(_executor: &LocalExecutor<'_>, ble_device: Arc<Mutex<&'static mut BLEDevice>>, parameters: Arc<TrueNorthParameters>) -> Result<(), Box<dyn std::error::Error>> {        
        let ble_advertiser = ble_device.lock().unwrap().get_advertising();

        ble_device
            .lock().unwrap()
            .security()
            .set_auth(AuthReq::all())
            .set_passkey(123456)
            .set_io_cap(SecurityIOCap::NoInputNoOutput)
            .resolve_rpa();


        let server = ble_device.lock().unwrap().get_server();

        server.on_connect(|server, clntdesc| {
            // Print connected client data
            log::debug!("{:?}", clntdesc);
            // Update connection parameters
            server
                .update_conn_params(clntdesc.conn_handle(), 24, 48, 0, 60)
                .unwrap();
        });

        server.on_disconnect(|_desc, _reason| {
            println!("Disconnected, back to advertising");
        });

        let truenorth_service = server.create_service(BleUuid::from_uuid16(0x6969));

        // Create a characteristic to associate with created service
        let declination_characteristic = truenorth_service.lock().create_characteristic(
            BleUuid::from_uuid16(0x1000),
            NimbleProperties::READ | NimbleProperties::WRITE | NimbleProperties::NOTIFY);

        {
            let declination_parameter = parameters.declination.clone();

            declination_characteristic.lock().on_write(move|value| {
                let data = value.recv_data();
                log::debug!("Correction received: {:?}", data);
                if let Err(err) = declination_parameter.lock().unwrap().set(((data[3] as i32) << 24 | (data[2] as i32) << 16 | (data[1] as i32) << 8 | (data[0] as i32)) as i32) {
                    log::error!("Error setting declination: {}", err);
                }
                log::debug!("Correction set to: {}", declination_parameter.lock().unwrap().get());
                value.notify();
            });                
        }

        ble_advertiser.lock().set_data(BLEAdvertisementData::new()
            .name("TrueNorth")
            .add_service_uuid(BleUuid::from_uuid16(0x6969))
        )?;

        ble_advertiser
            .lock()
            .advertisement_type(ConnMode::Und)
            .disc_mode(DiscMode::Gen)
            .scan_response(false);
    
        // Start Advertising
        ble_advertiser.lock().start()?;
    
        log::info!("Advertisement Started");

        {
            let declination_parameter = parameters.declination.clone();

            declination_parameter.lock().unwrap().add_handler(Box::new(|value, parameters| {
                let dc = parameters.get("characteristic").unwrap().downcast_ref::<Arc<esp32_nimble::utilities::mutex::Mutex<BLECharacteristic>>>();
                if let Some(dc) = dc {
                    dc.lock().set_value(value.to_le_bytes().to_vec().as_slice()).notify();
                    log::debug!("BleCallback: Declination SmartVar changed to: {}", value);
                } else {
                    log::error!("BleCallback:Characteristic not found");
                }
            }), HashMap::from([("characteristic".to_string(), Box::new(declination_characteristic.clone()) as Box<dyn Any + Send>)]));    
        }

        loop {
            log::debug!("ble server thread...");
            thread::sleep(std::time::Duration::from_secs(10));
        }
    }

    thread::Builder::new().stack_size(20000).spawn(move || {
        let executor = LocalExecutor::new();

        let fut = &mut pin!(ble_server(&executor, ble_device, parameters));

        async_io::block_on(executor.run(fut)).unwrap();
    })?;

    Ok(())
}