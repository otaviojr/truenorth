pub mod mlx90393;

#[allow(unused)]
pub trait MagSensor {
    fn get_angle(&self) -> Result<i32, Box<dyn std::error::Error>>;
}
