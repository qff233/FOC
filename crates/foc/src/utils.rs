use core::f32::consts::PI;

use micromath::F32Ext;

#[allow(dead_code)]
pub struct AngleSinCos {
    pub cos_alpha: f32,
    pub sin_alpha: f32,
    pub cos_beta: f32,
    pub sin_beta: f32,
    pub cos_gamma: f32,
    pub sin_gamma: f32,
}
impl AngleSinCos {
    #[allow(dead_code)]
    pub fn new(mut angle: f32) -> Self {
        angle = angle.min(2. * PI).max(0.);

        let theta = angle - 120_f32.to_radians(); // angle - 120
        let beta = angle + 120_f32.to_radians(); // angle + 120

        let cos_alpha = angle.cos();
        let sin_alpha = angle.sin();
        let cos_beta = theta.cos();
        let sin_beta = theta.sin();
        let cos_gamma = beta.cos();
        let sin_gamma = beta.sin();
        Self {
            cos_alpha,
            sin_alpha,
            cos_beta,
            sin_beta,
            cos_gamma,
            sin_gamma,
        }
    }
}

pub fn park(a: f32, b: f32, c: f32, cos_sin: &AngleSinCos) -> (f32, f32) {
    let a1 = cos_sin.cos_alpha;
    let a2 = cos_sin.cos_beta;
    let a3 = cos_sin.cos_gamma;
    let b1 = -cos_sin.sin_alpha;
    let b2 = -cos_sin.sin_beta;
    let b3 = -cos_sin.sin_gamma;

    let d = 2.0_f32 / 3. * (a1 * a + a2 * b + a3 * c);
    let q = 2.0_f32 / 3. * (b1 * a + b2 * b + b3 * c);
    (d, q)
}

pub fn inv_park(d: f32, q: f32, cos_sin: &AngleSinCos) -> (f32, f32, f32) {
    let a1 = cos_sin.cos_alpha;
    let a2 = -cos_sin.sin_alpha;
    let b1 = cos_sin.cos_beta;
    let b2 = -cos_sin.sin_beta;
    let c1 = cos_sin.cos_gamma;
    let c2 = -cos_sin.sin_gamma;

    let a = a1 * d + a2 * q;
    let b = b1 * d + b2 * q;
    let c = c1 * d + c2 * q;

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
