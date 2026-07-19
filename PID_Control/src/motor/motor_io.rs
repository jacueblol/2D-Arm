#[allow(dead_code)]
#[derive(Debug)]
pub struct MotorLogSample {
    pub time_s: f64,
    pub position: f64,
    pub velocity: f64,
    pub voltage: f64,
    pub load_torque: f64,
}

pub struct MotorLogger {
    pub(crate) start_time: std::time::Instant,
    samples: Vec<MotorLogSample>,
}

#[allow(dead_code)]
impl MotorLogger {
    pub fn new() -> Self {
        Self {
            start_time: std::time::Instant::now(),
            samples: Vec::new(),
        }
    }

    pub fn log(&mut self, s: MotorLogSample) {
        self.samples.push(s);
    }

    pub fn write_csv(&self, path: &str) {
        use std::fs::File;
        use std::io::Write;

        let mut file = File::create(path).expect("Unable to create motor log file");
        writeln!(file, "time_s,position,velocity,voltage,load_torque").unwrap();
        for s in &self.samples {
            writeln!(
                file,
                "{:.6},{:.6},{:.6},{:.6},{:.6}",
                s.time_s, s.position, s.velocity, s.voltage, s.load_torque
            )
            .unwrap();
        }
    }
}

pub struct MotorIO {
    pub(crate) position: f64,
    pub(crate) velocity: f64,
    pub(crate) input_voltage: f64,
    pub logger: Option<MotorLogger>,
}

impl MotorIO {
    pub fn new() -> Self {
        Self {
            position: 0.0,
            velocity: 0.0,
            input_voltage: 0.0,
            logger: Some(MotorLogger::new()),
        }
    }
}

#[allow(dead_code)]
pub trait MotorFn {
    fn set_voltage(&mut self, volts: f64);
    fn reset(&mut self);
    fn get_position_rad(&self) -> f64;
    fn get_velocity_rad_s(&self) -> f64;
}
