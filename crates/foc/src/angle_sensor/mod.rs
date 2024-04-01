pub trait AngleSensor {
    fn get_angle(&mut self) -> f32;
}
