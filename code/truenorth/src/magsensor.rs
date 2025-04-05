use crate::math::Vector3;
use std::time::Duration;

pub mod mlx90393;
pub mod mlx90393_defs;
pub mod mlx90393_inner;

pub type MagSensorHandlerPtr = Box<dyn Fn(MagSensorEvent) -> () + Send>;

#[derive(Debug, Clone, Copy)]
pub enum MagSensorEvent {
    RawChanged(Vector3),
    CalibratedChanged((f32, f32), (f32, f32), (f32, f32)),
    HeadingChanged(i32),
}

#[allow(unused)]
pub trait MagSensor {
    fn start(&self) -> Result<(), Box<dyn std::error::Error>>;
    fn calibrate(&self, timeout: Duration) -> Result<(), Box<dyn std::error::Error>>;
    fn add_handler(&self, handler: MagSensorHandlerPtr) -> Result<(), Box<dyn std::error::Error>>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MagSensorState {
    Idle,
    Calibrating,
    Measuring,
}

impl From<MagSensorState> for u8 {
    fn from(state: MagSensorState) -> Self {
        state as u8
    }
}

impl From<u8> for MagSensorState {
    fn from(state: u8) -> Self {
        match state {
            0x00 => MagSensorState::Idle,
            0x01 => MagSensorState::Calibrating,
            0x02 => MagSensorState::Measuring,
            _ => panic!("Invalid MagSensorState"),
        }
    }
}

impl From<MagSensorState> for &str {
    fn from(state: MagSensorState) -> Self {
        match state {
            MagSensorState::Idle => "Idle",
            MagSensorState::Calibrating => "Calibrating",
            MagSensorState::Measuring => "Measuring",
        }
    }
}

impl From<&str> for MagSensorState {
    fn from(state: &str) -> Self {
        match state {
            "Idle" => MagSensorState::Idle,
            "Calibrating" => MagSensorState::Calibrating,
            "Measuring" => MagSensorState::Measuring,
            _ => panic!("Invalid MagSensorState"),
        }
    }
}

