#[derive(Debug, Clone, Copy)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
}

pub struct LowPassFilter {
    alpha: f32,
    state: Option<Vector3>,
}

impl LowPassFilter {
    pub fn new(alpha: f32) -> Self {
        Self { alpha, state: None }
    }

    pub fn update(&mut self, input: Vector3) -> Vector3 {
        let filtered = match self.state {
            Some(prev) => Vector3 {
                x: self.alpha * input.x + (1.0 - self.alpha) * prev.x,
                y: self.alpha * input.y + (1.0 - self.alpha) * prev.y,
                z: self.alpha * input.z + (1.0 - self.alpha) * prev.z,
            },
            None => input,
        };

        self.state = Some(filtered.clone());
        filtered
    }
}
