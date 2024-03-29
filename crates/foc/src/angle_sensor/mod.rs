pub enum SensorType {
    NoSensor,
    Encoder,
}

pub trait AngleSensor {
    fn get_angle(&self) -> f32;

    fn get_type(&self) -> SensorType {
        SensorType::Encoder
    }
}

pub struct NoAngleSensor {}

impl NoAngleSensor {
    #[allow(dead_code)]
    pub fn new() -> Self {
        Self {}
    }
}

impl AngleSensor for NoAngleSensor {
    fn get_angle(&self) -> f32 {
        0.
    }
    fn get_type(&self) -> SensorType {
        SensorType::NoSensor
    }
}
