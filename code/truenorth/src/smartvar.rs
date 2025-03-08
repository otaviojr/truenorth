use std::{any::{self, Any}, collections::HashMap, sync::{mpsc::{self, Receiver, Sender}, Arc, Mutex, MutexGuard}, thread};

use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, EspNvsPartition, NvsDefault};

use crate::Endable;

#[derive(Debug)]
#[allow(dead_code)]
pub enum SmartVarEvent<T: Send> {
    Changed(T)
}

#[allow(dead_code)]
pub type SmartVarPtr<T> = Box<dyn for<'a> Fn(&'a T, MutexGuard<'a, HashMap<String, Box<dyn any::Any + Send>>>) + Send>;

pub struct SmartVarHandlerPtr<T> {
    handler: Arc<Mutex<SmartVarPtr<T>>>,
    parameters: Arc<Mutex<HashMap<String, Box<dyn any::Any + Send>>>>,
}

#[allow(dead_code)]
pub struct SmartVar<T: Send> {
    namespace: Option<String>,
    storage_name: Option<String>,
    value: T,
    handlers: Vec<SmartVarHandlerPtr<T>>,
    channel: (Sender<SmartVarEvent<T>>, Receiver<SmartVarEvent<T>>),
    end_channel: (Sender<bool>, Receiver<bool>)
}

impl<T: Clone +Send + 'static> SmartVar<T> {
    pub fn new(value: T) -> Arc<Mutex<Self>> {
        let (tx, rx) = mpsc::channel::<SmartVarEvent<T>>();
        let (tx_end, rx_end) = mpsc::channel::<bool>(); 
        let me = Arc::new(Mutex::new(Self { namespace: Option::None, storage_name: Option::None, value, handlers: Vec::new(), channel: (tx, rx), end_channel: (tx_end, rx_end) }));
        
        // Lock the mutex and call setup
        {
            let mut smart_var = me.lock().unwrap();
            smart_var.setup(me.clone());
        }
        
        me
    }

    fn setup(&mut self, me: Arc<Mutex<Self>>) {
        let me_shared = me.clone();
        if let Err(e) = thread::Builder::new().spawn(move || {
            'main_loop: loop {

                {
                    let mut lock_me = me_shared.lock().unwrap();

                    if let Ok(end) = lock_me.end_channel.1.try_recv() {
                        if end {
                            break 'main_loop;
                        }
                    }

                    if let Ok(event) = lock_me.channel.1.try_recv() {
                        match event {
                            SmartVarEvent::Changed(value) => {
                                //log::debug!("SmartVar:Changed");
                                for handler in lock_me.handlers.iter_mut() {
                                    handler.handler.lock().unwrap()(&value, handler.parameters.lock().unwrap());
                                }                    
                            }
                        }
                    }
                }

                thread::sleep(std::time::Duration::from_millis(100));
            }

            log::info!("SmartVar:updater thread ended");

        }) {
            log::error!("SmartVar:updater error: {}", e);
        }
    }

    pub fn setup_storage(&mut self, namespace: String, storage_name: String) -> Result<(), Box<dyn std::error::Error>> {
        self.namespace = Option::Some(namespace);
        self.storage_name = Option::Some(storage_name);
        
        if let Err(e) = self.load() {
            log::warn!("SmartVar: setup_storage: Error loading from storage:");
            log::warn!("Namespace: {}", self.namespace.as_ref().unwrap());
            log::warn!("Storage name: {}", self.storage_name.as_ref().unwrap());
            log::warn!("Error: {}", e);
        }

        Ok(())
    }

    fn get_partition_namespace(&self) -> Result<Arc<Mutex<EspNvs<NvsDefault>>>, Box<dyn std::error::Error>> {

        if self.namespace.is_none() {
            return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, "Namespace is not set")));
        }

        let nvs_default_partition: EspNvsPartition<NvsDefault> = EspDefaultNvsPartition::take().unwrap();
    
        let nvs = match EspNvs::new(nvs_default_partition, self.namespace.as_ref().unwrap(), true) {
            Ok(nvs) => {
                println!("Got namespace {:?} from default partition", self.namespace.as_ref().unwrap());
                nvs
            }
            Err(e) => panic!("Could't get namespace {:?}", e),
        };
    
        Ok(Arc::new(Mutex::new(nvs)))
    }
    

    fn load(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let nvs = self.get_partition_namespace()?;

        if self.storage_name.is_none() {
            return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, "Storage name is not set")));
        }

        if std::any::TypeId::of::<T>() == std::any::TypeId::of::<i32>() {
            let value = nvs.lock().unwrap().get_i32(self.storage_name.as_ref().unwrap()).unwrap();
            if let Some(value) = value {  
                if let Some(s) = (&mut self.value as &mut dyn Any).downcast_mut::<i32>() {
                    *s = value;
                } else {
                    return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, "SmartVar: self.value is not a String")));
                }
            }
        } else if std::any::TypeId::of::<T>() == std::any::TypeId::of::<u32>() {
            let value = nvs.lock().unwrap().get_u32(self.storage_name.as_ref().unwrap()).unwrap();
            if let Some(value) = value {  
                if let Some(s) = (&mut self.value as &mut dyn Any).downcast_mut::<u32>() {
                    *s = value;
                } else {
                    return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, "SmartVar: self.value is not a String")));
                }
            }
        } else if std::any::TypeId::of::<T>() == std::any::TypeId::of::<i16>() {
            let value = nvs.lock().unwrap().get_i16(self.storage_name.as_ref().unwrap()).unwrap();
            if let Some(value) = value {  
                if let Some(s) = (&mut self.value as &mut dyn Any).downcast_mut::<i16>() {
                    *s = value;
                } else {
                    return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, "SmartVar: self.value is not a String")));
                }
            }
        } else if std::any::TypeId::of::<T>() == std::any::TypeId::of::<u16>() {
            let value = nvs.lock().unwrap().get_u16(self.storage_name.as_ref().unwrap()).unwrap();
            if let Some(value) = value {  
                if let Some(s) = (&mut self.value as &mut dyn Any).downcast_mut::<u16>() {
                    *s = value;
                } else {
                    return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, "SmartVar: self.value is not a String")));
                }
            }
        } else if std::any::TypeId::of::<T>() == std::any::TypeId::of::<i8>() {
            let value = nvs.lock().unwrap().get_i8(self.storage_name.as_ref().unwrap()).unwrap();
            if let Some(value) = value {  
                if let Some(s) = (&mut self.value as &mut dyn Any).downcast_mut::<i8>() {
                    *s = value;
                } else {
                    return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, "SmartVar: self.value is not a String")));
                }
            }
        } else if std::any::TypeId::of::<T>() == std::any::TypeId::of::<u8>() {
            let value = nvs.lock().unwrap().get_u8(self.storage_name.as_ref().unwrap()).unwrap();
            if let Some(value) = value {  
                if let Some(s) = (&mut self.value as &mut dyn Any).downcast_mut::<u8>() {
                    *s = value;
                } else {
                    return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, "SmartVar: self.value is not a String")));
                }
            }
        } else if std::any::TypeId::of::<T>() == std::any::TypeId::of::<f32>() {
            let mut buffer = vec![0u8; 4];
            let value = nvs.lock().unwrap().get_raw(self.storage_name.as_ref().unwrap(), &mut buffer).unwrap();
            if let Some(value) = value {  
                if let Some(s) = (&mut self.value as &mut dyn Any).downcast_mut::<f32>() {
                    *s = f32::from_le_bytes(value.try_into().unwrap());
                }
            }
        } else if std::any::TypeId::of::<T>() == std::any::TypeId::of::<String>() {
            if let Ok(size) = nvs.lock().unwrap().str_len(self.storage_name.as_ref().unwrap()) {
                if let Some(size) = size {
                    log::debug!("SmartVar: load string of size: {}", size);
                    let mut buffer = vec![0u8; size];
                    let ret = nvs.lock().unwrap().get_str(self.storage_name.as_ref().unwrap(), &mut buffer);
                    if let Ok(value) = ret {
                        if let Some(value) = value {
                            if let Some(s) = (&mut self.value as &mut dyn Any).downcast_mut::<String>() {
                                *s = value.to_string();
                            } else {
                                return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, "SmartVar: self.value is not a String")));
                            }
                        } else {
                            return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, "SmartVar: Error getting storage value")));
                        }
                    } else {
                        return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, "SmartVar: Error getting storage value")));
                    }
                } else {
                    return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, "SmartVar: Error getting storage size")));
                }
            } else {
                return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, "SmartVar: Error getting storage size")));
            }
        } else {
            return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, "SmartVar: Type not supported")));
        }

        self.channel.0.send(SmartVarEvent::Changed(self.value.clone())).unwrap();

        Ok(())
    }

    fn save(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let nvs = self.get_partition_namespace()?;

        if std::any::TypeId::of::<T>() == std::any::TypeId::of::<i32>() {
            if let Some(s) = (&mut self.value as &mut dyn Any).downcast_mut::<i32>() {
                if let Err(err) = nvs.lock().unwrap().set_i32(self.storage_name.as_ref().unwrap(), *s) {
                    return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, format!("SmartVar: Error saving storage value: {}", err))));
                }
            }
        } else if std::any::TypeId::of::<T>() == std::any::TypeId::of::<u32>() {
            if let Some(s) = (&mut self.value as &mut dyn Any).downcast_mut::<u32>() {
                if let Err(err) = nvs.lock().unwrap().set_u32(self.storage_name.as_ref().unwrap(), *s) {
                    return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, format!("SmartVar: Error saving storage value: {}", err))));
                }
            }
        } else if std::any::TypeId::of::<T>() == std::any::TypeId::of::<i16>() {
            if let Some(s) = (&mut self.value as &mut dyn Any).downcast_mut::<i16>() {
                if let Err(err) = nvs.lock().unwrap().set_i16(self.storage_name.as_ref().unwrap(), *s) {
                    return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, format!("SmartVar: Error saving storage value: {}", err))));
                }
            }
        } else if std::any::TypeId::of::<T>() == std::any::TypeId::of::<u16>() {
            if let Some(s) = (&mut self.value as &mut dyn Any).downcast_mut::<u16>() {
                if let Err(err) = nvs.lock().unwrap().set_u16(self.storage_name.as_ref().unwrap(), *s) {
                    return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, format!("SmartVar: Error saving storage value: {}", err))));
                }
            }
        } else if std::any::TypeId::of::<T>() == std::any::TypeId::of::<i8>() {
            if let Some(s) = (&mut self.value as &mut dyn Any).downcast_mut::<i8>() {
                if let Err(err) = nvs.lock().unwrap().set_i8(self.storage_name.as_ref().unwrap(), *s) {
                    return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, format!("SmartVar: Error saving storage value: {}", err))));
                }
            }
        } else if std::any::TypeId::of::<T>() == std::any::TypeId::of::<u8>() {
            if let Some(s) = (&mut self.value as &mut dyn Any).downcast_mut::<u8>() {
                if let Err(err) = nvs.lock().unwrap().set_u8(self.storage_name.as_ref().unwrap(), *s) {
                    return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, format!("SmartVar: Error saving storage value: {}", err))));
                }
            }
        } else if std::any::TypeId::of::<T>() == std::any::TypeId::of::<f32>() {
            if let Some(s) = (&mut self.value as &mut dyn Any).downcast_mut::<f32>() {
                let mut buffer = vec![0u8; 4];
                buffer.copy_from_slice(&s.to_le_bytes());
                if let Err(err) = nvs.lock().unwrap().set_raw(self.storage_name.as_ref().unwrap(), &buffer) {
                    return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, format!("SmartVar: Error saving storage value: {}", err))));
                }
            }
        } else if std::any::TypeId::of::<T>() == std::any::TypeId::of::<String>() {
            if let Some(s) = (&mut self.value as &mut dyn Any).downcast_mut::<String>() {
                if let Err(err) = nvs.lock().unwrap().set_str(self.storage_name.as_ref().unwrap(), s) {
                    return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, format!("SmartVar: Error saving storage value: {}", err))));
                }
            }
        } else {
            return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidInput, "SmartVar: Type not supported")));
        }
        Ok(())
    }

    pub fn add_handler(&mut self, handler: SmartVarPtr<T>, parameters: HashMap<String, Box<dyn any::Any + Send>>) {
        self.handlers.push(SmartVarHandlerPtr { handler: Arc::new(Mutex::new(handler)), parameters: Arc::new(Mutex::new(parameters)) });
    }

    pub fn set(&mut self, value: T) -> Result<(), Box<dyn std::error::Error>> {
        self.value = value.clone();
        if self.storage_name.is_some() && self.namespace.is_some() {
            if let Err(e) = self.save() {
                log::warn!("SmartVar: set: Error saving to storage:");
                log::warn!("Namespace: {}", self.namespace.as_ref().unwrap());
                log::warn!("Storage name: {}", self.storage_name.as_ref().unwrap());
                log::warn!("Error: {}", e);
            }
        }
        self.channel.0.send(SmartVarEvent::Changed(value.clone()))?;
        Ok(())
    }

    pub fn get(&self) -> &T {
        &self.value
    }
}

impl<T: Clone +Send + 'static> Endable for SmartVar<T> {
    fn end(&self) {
        self.end_channel.0.send(true).unwrap();
    }
}