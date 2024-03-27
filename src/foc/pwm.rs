trait PWMTrait {
    fn set_duty(&mut self, u: u16, v: u16, w: u16);
    fn enable(&mut self);
    fn disable(&mut self);
}
