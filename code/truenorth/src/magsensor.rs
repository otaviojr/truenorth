pub mod mlx90393;
pub mod mlx90393_defs;
pub mod mlx90393_inner;

#[allow(unused)]
pub trait MagSensor {
    fn get_angle(&self) -> Result<i32, Box<dyn std::error::Error>>;
}
