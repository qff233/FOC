use core::f32::consts::PI;

#[allow(dead_code)]
pub struct Pll2 {
    kp: f32,
    limit: f32,
    speed: f32,
    last_ang: f32,
    dt: f32,
}

#[allow(dead_code)]
impl Pll2 {
    pub fn new(kp: f32, limit: f32, dt: f32) -> Self {
        Self {
            kp,
            limit,
            speed: 0.0,
            last_ang: 0.0,
            dt,
        }
    }

    pub fn update(&mut self, angle: f32) -> f32 {
        let mut diff = angle - self.last_ang;
        if diff > PI {
            diff -= 2. * PI;
        } else if diff < -PI {
            diff += 2. * PI;
        }

        self.last_ang = angle;
        self.speed += (diff - self.speed) * self.kp;
        self.speed = self.speed.max(-self.limit).min(self.limit);

        self.speed / self.dt
    }
}
