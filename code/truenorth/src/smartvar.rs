use std::{any::{self, Any}, collections::HashMap, pin::pin, sync::{mpsc::{self, Receiver, Sender}, Arc, Mutex, MutexGuard}, thread};

use async_executor::LocalExecutor;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, EspNvsPartition, NvsDefault};

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
    rx: Arc<Mutex<Receiver<SmartVarEvent<T>>>>,
    tx: Sender<SmartVarEvent<T>>,
}

impl<T: Clone +Send + 'static> SmartVar<T> {
    pub fn new(value: T) -> Arc<Mutex<Self>> {
        let (tx, rx) = mpsc::channel::<SmartVarEvent<T>>();
        let me = Arc::new(Mutex::new(Self { namespace: Option::None, storage_name: Option::None, value, handlers: Vec::new(), rx: Arc::new(Mutex::new(rx)), tx }));
        
        // Lock the mutex and call setup
        {
            let mut smart_var = me.lock().unwrap();
            smart_var.setup(me.clone());
        }
        
        me
    }

    fn setup(&mut self, me: Arc<Mutex<Self>>) {
        async fn updater<T: Clone +Send + 'static>(me: Arc<Mutex<SmartVar<T>>>, _executor: &LocalExecutor<'_>, rx: Arc<Mutex<Receiver<SmartVarEvent<T>>>>) -> Result<(), Box<dyn std::error::Error>> {
            loop {
                if let Ok(event) = rx.lock().unwrap().recv() {
                    match event {
                        SmartVarEvent::Changed(value) => {
                            //log::debug!("SmartVar:Changed: {}", value);
                            for handler in me.lock().unwrap().handlers.iter_mut() {
                                handler.handler.lock().unwrap()(&value, handler.parameters.lock().unwrap());
                            }                    
                        }
                    }
                }
            }
        }

        if let Err(e) = thread::Builder::new().stack_size(20000).spawn(move || {
            let executor = LocalExecutor::new();
    
            let fut = &mut pin!(updater(me.clone(), &executor, me.clone().lock().unwrap().rx.clone()));
    
            async_io::block_on(executor.run(fut)).unwrap();
        }) {
            log::error!("SmartVar:updater: {}", e);
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

        self.tx.send(SmartVarEvent::Changed(self.value.clone())).unwrap();

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
        self.tx.send(SmartVarEvent::Changed(value.clone()))?;
        Ok(())
    }

    pub fn get(&self) -> &T {
        &self.value
    }
}