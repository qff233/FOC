use core::f32::consts::PI;

use defmt::info;
use micromath::F32Ext;

pub fn park(a: f32, b: f32, c: f32, mut angle: f32) -> (f32, f32) {
    angle += PI;
    angle = angle.min(2. * PI).max(0.);

    let theta = angle - 120_f32.to_radians(); // angle - 120
    let beta = angle + 120_f32.to_radians(); // angle + 120

    let a1 = angle.cos();
    let a2 = theta.cos();
    let a3 = beta.cos();
    let b1 = -angle.sin();
    let b2 = -theta.sin();
    let b3 = -beta.sin();

    let d = 2.0_f32 / 3. * (a1 * a + a2 * b + a3 * c);
    let q = 2.0_f32 / 3. * (b1 * a + b2 * b + b3 * c);
    (d, q)
}

pub fn inv_park(d: f32, q: f32, mut angle: f32) -> (f32, f32, f32) {
    angle += PI;
    angle = angle.min(2. * PI).max(0.);
    // trace!("{}", angle);
    let theta = angle - 120_f32.to_radians(); // angle - 120
    let beta = angle + 120_f32.to_radians(); // angle + 120

    // trace!(
    //     "{} {} {}",
    //     angle.to_degrees(),
    //     theta.to_degrees(),
    //     beta.to_degrees()
    // );

    let a1 = angle.cos();
    let a2 = -angle.sin();
    let b1 = theta.cos();
    let b2 = -theta.sin();
    let c1 = beta.cos();
    let c2 = -beta.sin();

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
