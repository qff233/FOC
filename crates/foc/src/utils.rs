use micromath::F32Ext;

#[allow(dead_code)]
pub struct AngleSinCos {
    pub sin: f32,
    pub cos: f32,
}
impl AngleSinCos {
    #[allow(dead_code)]
    pub fn new(angle: f32) -> Self {
        let sin = angle.sin();
        let cos = angle.cos();
        Self { sin, cos }
    }
}

pub fn park(a: f32, b: f32, _c: f32, cos_sin: &AngleSinCos) -> (f32, f32) {
    let alpha = a;
    let beta = 3_f32.sqrt() / 3. * (a + 2. * b);

    let sin = cos_sin.sin;
    let cos = cos_sin.cos;
    let d = cos * alpha + sin * beta;
    let q = -sin * alpha + cos * beta;
    (d, q)
}

pub fn svpwm(d: f32, q: f32, cos_sin: &AngleSinCos) -> (f32, f32, f32) {
    let sin = cos_sin.sin;
    let cos = cos_sin.cos;

    let alpha = cos * d - sin * q;
    let beta = sin * d + cos * q;

    let a = alpha;
    let b = -0.5 * alpha + 0.5 * 3_f32.sqrt() * beta;
    let c = -0.5 * alpha - 0.5 * 3_f32.sqrt() * beta;

    // debug!("{} {} {}", a, b, c);
    let vmax = a.max(b).max(c);
    let vmin = a.min(b).min(c);
    let vcom = (vmax + vmin) / 2. + 0.5;

    (vcom - a, vcom - b, vcom - c)
}
