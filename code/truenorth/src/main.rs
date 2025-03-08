#![feature(impl_trait_in_bindings)]

pub mod motor;
pub mod smartvar;
pub mod magsensor;
use crate::motor::Motor;
use crate::smartvar::SmartVar;

use std::any::Any;
use std::collections::HashMap;
use std::sync::mpsc::Receiver;
use std::thread;
//use rand::Rng;
use std::sync::{mpsc, Mutex};
use std::cell::RefCell;
use std::sync::Arc;

use esp32_nimble::utilities::BleUuid;
use esp32_nimble::NimbleProperties;
use esp32_nimble::{BLEDevice, BLEAdvertisementData, BLECharacteristic, enums::{ConnMode, DiscMode, AuthReq, SecurityIOCap}};
use esp_idf_svc::hal::prelude::*;

use magsensor::mlx90393::MLX90393Config;
use magsensor::mlx90393::MLX90393;
use magsensor::{MagSensor, MagSensorEvent};

thread_local! {
    #[allow(clippy::thread_local_initializer_can_be_made_const)]
    static TAG_NAMESPACE:RefCell<&'static str> =  RefCell::new("truenorth");
    static TAG_DECLINATION:RefCell<&'static str> =  RefCell::new("declination");
    static TAG_MAX_X:RefCell<&'static str> =  RefCell::new("max_x");
    static TAG_MAX_Y:RefCell<&'static str> =  RefCell::new("max_y");
    static TAG_MAX_Z:RefCell<&'static str> =  RefCell::new("max_z");
    static TAG_MIN_X:RefCell<&'static str> =  RefCell::new("min_x");
    static TAG_MIN_Y:RefCell<&'static str> =  RefCell::new("min_y");
    static TAG_MIN_Z:RefCell<&'static str> =  RefCell::new("min_z");
}

pub struct TrueNorthParameters {
    pub declination: Arc<Mutex<SmartVar<i32>>>,
    pub max_x: Arc<Mutex<SmartVar<f32>>>,
    pub max_y: Arc<Mutex<SmartVar<f32>>>,
    pub max_z: Arc<Mutex<SmartVar<f32>>>,
    pub min_x: Arc<Mutex<SmartVar<f32>>>,
    pub min_y: Arc<Mutex<SmartVar<f32>>>,
    pub min_z: Arc<Mutex<SmartVar<f32>>>
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
        max_x: SmartVar::new(f32::MIN), //0xFFFF7FFF
        max_y: SmartVar::new(f32::MIN), //0xFFFF7FFF
        max_z: SmartVar::new(f32::MIN), //0xFFFF7FFF
        min_x: SmartVar::new(f32::MAX), //0xFFFF7F7F
        min_y: SmartVar::new(f32::MAX), //0xFFFF7F7F
        min_z: SmartVar::new(f32::MAX)
    });

    endable.add(parameters.clone().declination.clone());
    endable.add(parameters.clone().max_x.clone());
    endable.add(parameters.clone().max_y.clone());
    endable.add(parameters.clone().max_z.clone());
    endable.add(parameters.clone().min_x.clone());
    endable.add(parameters.clone().min_y.clone());
    endable.add(parameters.clone().min_z.clone());

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

    let config = MLX90393Config::new(parameters.clone(), 0x0C, pins.gpio8.into(), pins.gpio9.into(), pins.gpio1.into());
    
    let mag = match MLX90393::new(peripherals.i2c0, config) {
        Ok(mag) => Arc::new(Mutex::new(mag)),
        Err(_error) => {
            halt_system(&mut endable);
            return;
        }
    };

    endable.add(mag.clone());

    if let Err(err) = mag.lock().unwrap().add_handler(Box::new(|event| {
        match event {
            MagSensorEvent::CalibratedChanged((max_x, min_x), (max_y, min_y), (max_z, min_z)) => {
                log::debug!("Calibrated: {:?}, {:?}, {:?}", (max_x, min_x), (max_y, min_y), (max_z, min_z));
            }
            MagSensorEvent::HeadingChanged(heading) => {
                log::debug!("Heading: {:?}", heading);
            }
            _ => {}
        }
    })) {
        log::error!("Error adding handler: {}", err);
    }
    
    let bt_receiver = match setup_bt_server(parameters.clone()) {
        Ok(receiver) => receiver,
        Err(err) => {
            log::error!("Error setting up advertisement: {}", err);
            halt_system(&mut endable);
            return;
        }
    };

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

    if let Err(err) = parameters.clone().max_x.lock().unwrap().setup_storage(TAG_NAMESPACE.take().to_string(), TAG_MAX_X.take().to_string()) {
        log::error!("Error setting up max_x storage: {}", err);
    }

    if let Err(err) = parameters.clone().max_y.lock().unwrap().setup_storage(TAG_NAMESPACE.take().to_string(), TAG_MAX_Y.take().to_string()) {
        log::error!("Error setting up max_y storage: {}", err);
    }

    if let Err(err) = parameters.clone().max_z.lock().unwrap().setup_storage(TAG_NAMESPACE.take().to_string(), TAG_MAX_Z.take().to_string()) {
        log::error!("Error setting up max_z storage: {}", err);
    }

    if let Err(err) = parameters.clone().min_x.lock().unwrap().setup_storage(TAG_NAMESPACE.take().to_string(), TAG_MIN_X.take().to_string()) {
        log::error!("Error setting up min_x storage: {}", err);
    }

    if let Err(err) = parameters.clone().min_y.lock().unwrap().setup_storage(TAG_NAMESPACE.take().to_string(), TAG_MIN_Y.take().to_string()) {
        log::error!("Error setting up min_y storage: {}", err);
    }

    if let Err(err) = parameters.clone().min_z.lock().unwrap().setup_storage(TAG_NAMESPACE.take().to_string(), TAG_MIN_Z.take().to_string()) {
        log::error!("Error setting up min_z storage: {}", err);
    }

    if let Err(err) = mag.lock().unwrap().start() {
        log::error!("Error starting mag: {}", err);
        halt_system(&mut endable);
        return;
    }

    //halt_system(&mut endable);

    loop {

        if let Ok(command) = bt_receiver.try_recv() {
            match command {
                BluetoothCommand::ResetCalibrationData => {
                    if let Err(err) = parameters.clone().max_x.lock().unwrap().set(f32::MIN) {
                        log::error!("Error setting max_x: {}", err);
                    }
                    if let Err(err) = parameters.clone().max_y.lock().unwrap().set(f32::MIN) {
                        log::error!("Error setting max_y: {}", err);
                    }
                    if let Err(err) = parameters.clone().max_z.lock().unwrap().set(f32::MIN) {
                        log::error!("Error setting max_z: {}", err);
                    }
                    if let Err(err) = parameters.clone().min_x.lock().unwrap().set(f32::MAX) {
                        log::error!("Error setting min_x: {}", err);
                    }
                    if let Err(err) = parameters.clone().min_y.lock().unwrap().set(f32::MAX) {
                        log::error!("Error setting min_y: {}", err);
                    }
                    if let Err(err) = parameters.clone().min_z.lock().unwrap().set(f32::MAX) {
                        log::error!("Error setting min_z: {}", err);
                    }
                }
                BluetoothCommand::Calibrate => {
                    if let Err(err) = mag.lock().unwrap().calibrate(std::time::Duration::from_secs(60)) {
                        log::error!("Error calibrating mag: {}", err);
                    }
                    if let Err(err) = mag.lock().unwrap().start() {
                        log::error!("Error starting mag: {}", err);
                    }
                }
                _ => {
                    log::error!("Unknown bluetooth command");
                }
            }
        }
        //if let Err(e) = mag.lock().unwrap().start_single_measurement() {
        //    log::error!("Error starting single measurement: {}", e);
        //}

        /*thread::sleep(std::time::Duration::from_millis(1000));*/

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
        }*/
        thread::sleep(std::time::Duration::from_millis(1000));
    }
}

#[allow(unused)]
fn halt_system(endable: &mut EndableHandler) {
    endable.end_all();
    log::error!("Halting system");
    loop {
        thread::sleep(std::time::Duration::from_secs(1));
    }
}

enum BluetoothCommand {
    Unknown = 0x00,
    ResetCalibrationData = 0x01,
    Calibrate = 0x02,
}

impl From<u8> for BluetoothCommand {
    fn from(value: u8) -> Self {
        match value {
            0x01 => BluetoothCommand::ResetCalibrationData,
            0x02 => BluetoothCommand::Calibrate,
            _ => BluetoothCommand::Unknown,
        }
    }
}

impl From<BluetoothCommand> for u8 {
    fn from(value: BluetoothCommand) -> Self {
        match value {
            BluetoothCommand::ResetCalibrationData => 0x01,
            BluetoothCommand::Calibrate => 0x02,
            _ => 0x00,
        }
    }
}

fn setup_bt_server(parameters: Arc<TrueNorthParameters>) -> Result<Receiver<BluetoothCommand>, Box<dyn std::error::Error>> {

    let (sender, receiver) = mpsc::channel::<BluetoothCommand>();
    
    let ble_device = Arc::new(Mutex::new(BLEDevice::take()));

    thread::Builder::new().spawn(move || {

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

        log::debug!("Creating BT service");

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

        let command_characteristic = truenorth_service.lock().create_characteristic(
            BleUuid::from_uuid16(0x1001),
            NimbleProperties::WRITE | NimbleProperties::NOTIFY);

        command_characteristic.lock().on_write(move|_value| {
            let data = _value.recv_data();
            log::debug!("Command received: {:?}", data);
            match BluetoothCommand::from(data[0]) {
                BluetoothCommand::ResetCalibrationData => {
                    sender.send(BluetoothCommand::ResetCalibrationData).unwrap();
                }
                BluetoothCommand::Calibrate => {
                    sender.send(BluetoothCommand::Calibrate).unwrap();
                }
                _ => log::debug!("Unknown command"),
            }
        });

        if let Err(err) = ble_advertiser.lock().set_data(BLEAdvertisementData::new()
            .name("TrueNorth")
            .add_service_uuid(BleUuid::from_uuid16(0x6969))
        ) {
            log::error!("Error setting advertisement data: {}", err);
            return;
        }

        ble_advertiser
            .lock()
            .advertisement_type(ConnMode::Und)
            .disc_mode(DiscMode::Gen)
            .scan_response(false);
    
        // Start Advertising
        if let Err(err) = ble_advertiser.lock().start() {
            log::error!("Error starting advertisement: {}", err);
            return;
        }
    
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
            thread::sleep(std::time::Duration::from_secs(1));
        }
    })?;

    Ok(receiver)
}