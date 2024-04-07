use micromath::F32Ext;

#[allow(dead_code)]
pub struct AngleSinCos {
    pub cos: f32,
    pub sin: f32,
}
impl AngleSinCos {
    #[allow(dead_code)]
    pub fn new(angle: f32) -> Self {
        let cos = angle.cos();
        let sin = angle.sin();
        Self { cos, sin }
    }
}

pub fn park(a: f32, b: f32, _c: f32, cos_sin: &AngleSinCos) -> (f32, f32) {
    let alpha = a;
    let beta = 3_f32.sqrt() / 3. * (a + 2. * b);

    let cos = cos_sin.cos;
    let sin = cos_sin.sin;
    let d = cos * alpha + sin * beta;
    let q = -sin * alpha + cos * beta;
    (d, q)
}

pub fn inv_park(d: f32, q: f32, cos_sin: &AngleSinCos) -> (f32, f32, f32) {
    let cos = cos_sin.cos;
    let sin = cos_sin.sin;
    let beta = cos * d - sin * q;
    let alpha = sin * d + cos * q;

    let a = alpha;
    let b = -0.5 * alpha + 0.5 * 3_f32.sqrt() * beta;
    let c = -0.5 * alpha - 0.5 * 3_f32.sqrt() * beta;
    // info!("{} {} {}", a, b, c);
    (a, b, c)
}

pub fn svpwm(a: f32, b: f32, c: f32) -> (f32, f32, f32) {
    let vmax = a.max(b).max(c);
    let vmin = a.min(b).min(c);
    let vcom = (vmax + vmin) / 2. + 0.5;

    (vcom - a, vcom - b, vcom - c)
}

// #[cfg(test)]
// mod tests {
//     use super::*;
//     #[test]
//     fn test_park() {
//         let result = park(1.0, 0., 0., 0.);
//         assert_eq!(result.0, 1);
//         assert_eq!(result.1, 0);
//     }

//     #[test]
//     fn test_inv_park() {
//     }

//     #[test]
//     fn test_svpwm() {
//     }
// }
