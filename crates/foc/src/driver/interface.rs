pub trait Adcs {
    fn get_voltage(&self) -> (f32, f32, f32);
}

pub trait Pwms {
    fn set_duty(&mut self, u: f32, v: f32, w: f32);
    fn enable(&mut self);
    fn disable(&mut self);
}
