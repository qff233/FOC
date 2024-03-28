use embedded_hal::i2c::I2c;

#[allow(dead_code)]
struct Mt6818<T: I2c> {
    i2c: T,
}

impl<T: I2c> Mt6818<T> {
    #[allow(dead_code)]
    fn new(i2c: T) -> Self {
        // TODO init mt6818
        Self { i2c }
    }
}
