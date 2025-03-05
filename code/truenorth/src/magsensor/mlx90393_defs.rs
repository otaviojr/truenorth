#[derive(Debug)]
pub enum MLX90393REG {
    CONF1 = 0x00,
    CONF2 = 0x01,
    CONF3 = 0x02
}

impl From<MLX90393REG> for u8 {
    fn from(reg: MLX90393REG) -> Self {
        reg as u8
    }
}

#[derive(Debug)]
pub enum MLX90393CMD {
    SB = 0x10,  // Start burst mode.
    SW = 0x20,  // Start wakeup on change mode.
    SM = 0x30,  // Start single-meas mode.
    RM = 0x40,  // Read measurement.
    RR = 0x50,  // Read register.
    WR = 0x60,  // Write register.
    EX = 0x80,  // Exit mode.
    HR = 0xD0,  // Memory recall.
    HS = 0x70,  // Memory store.
    RT = 0xF0,  // Reset.
    NOP = 0x00, // NOP.
}

impl From<MLX90393CMD> for u8 {
    fn from(cmd: MLX90393CMD) -> Self {
        cmd as u8
    }
}

impl From<u8> for MLX90393CMD {
    fn from(cmd: u8) -> Self {
        match cmd {
            0x10 => MLX90393CMD::SB,
            0x20 => MLX90393CMD::SW,
            0x30 => MLX90393CMD::SM,
            0x40 => MLX90393CMD::RM,
            0x50 => MLX90393CMD::RR,
            0x60 => MLX90393CMD::WR,
            0x80 => MLX90393CMD::EX,
            0xD0 => MLX90393CMD::HR,
            0x70 => MLX90393CMD::HS,
            0xF0 => MLX90393CMD::RT,
            0x00 => MLX90393CMD::NOP,
            _ => panic!("Invalid MLX90393CMD"),
        }
    }
}


#[derive(Debug, Clone, Copy)]
pub enum MLX90393GAIN {
    GAIN5X = (0x00),
    GAIN4X = (0x01),
    GAIN3X = (0x02),
    GAIN2_5X = (0x03),
    GAIN2X = (0x04),
    GAIN1_67X = (0x05),
    GAIN1_33X = (0x06),
    GAIN1X = (0x07),
}

impl From<MLX90393GAIN> for u8 {
    fn from(gain: MLX90393GAIN) -> Self {
        gain as u8
    }
}

impl From<u8> for MLX90393GAIN {
    fn from(gain: u8) -> Self {
        match gain {
            0x00 => MLX90393GAIN::GAIN5X,
            0x01 => MLX90393GAIN::GAIN4X,
            0x02 => MLX90393GAIN::GAIN3X,
            0x03 => MLX90393GAIN::GAIN2_5X,
            0x04 => MLX90393GAIN::GAIN2X,
            0x05 => MLX90393GAIN::GAIN1_67X,
            0x06 => MLX90393GAIN::GAIN1_33X,
            0x07 => MLX90393GAIN::GAIN1X,
            _ => panic!("Invalid MLX90393GAIN"),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum MLX90393RESOLUTION {
    RES16 = (0x00),
    RES17 = (0x01),
    RES18 = (0x02),
    RES19 = (0x03),
}

impl From<MLX90393RESOLUTION> for u8 {
    fn from(resolution: MLX90393RESOLUTION) -> Self {
        resolution as u8
    }
}

impl From<u8> for MLX90393RESOLUTION {
    fn from(resolution: u8) -> Self {
        match resolution {
            0x00 => MLX90393RESOLUTION::RES16,
            0x01 => MLX90393RESOLUTION::RES17,
            0x02 => MLX90393RESOLUTION::RES18,
            0x03 => MLX90393RESOLUTION::RES19,
            _ => panic!("Invalid MLX90393RESOLUTION"),
        }
    }
}

#[derive(Debug)]
pub enum MLX90393AXIS {
    X = 0x02,
    Y = 0x04,
    Z = 0x08,
    ALL = 0x0E
}

impl From<MLX90393AXIS> for u8 {
    fn from(axis: MLX90393AXIS) -> Self {
        axis as u8
    }
}

impl From<u8> for MLX90393AXIS {
    fn from(axis: u8) -> Self {
        match axis {
            0x02 => MLX90393AXIS::X,
            0x04 => MLX90393AXIS::Y,
            0x08 => MLX90393AXIS::Z,
            0x0E => MLX90393AXIS::ALL,
            _ => panic!("Invalid MLX90393AXIS"),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum MLX90393FILTER {
    FILTER0,
    FILTER1,
    FILTER2,
    FILTER3,
    FILTER4,
    FILTER5,
    FILTER6,
    FILTER7,
}

impl From<MLX90393FILTER> for u8 {
    fn from(filter: MLX90393FILTER) -> Self {
        filter as u8
    }
}

impl From<u8> for MLX90393FILTER {
    fn from(filter: u8) -> Self {
        match filter {
            0x00 => MLX90393FILTER::FILTER0,
            0x01 => MLX90393FILTER::FILTER1,
            0x02 => MLX90393FILTER::FILTER2,
            0x03 => MLX90393FILTER::FILTER3,
            0x04 => MLX90393FILTER::FILTER4,
            0x05 => MLX90393FILTER::FILTER5,
            0x06 => MLX90393FILTER::FILTER6,
            0x07 => MLX90393FILTER::FILTER7,
            _ => panic!("Invalid MLX90393FILTER"),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum MLX90393OVERSAMPLING {
    OSR0,
    OSR1,
    OSR2,
    OSR3,
}

impl From<MLX90393OVERSAMPLING> for u8 {
    fn from(oversampling: MLX90393OVERSAMPLING) -> Self {
        oversampling as u8
    }
}

impl From<u8> for MLX90393OVERSAMPLING {
    fn from(oversampling: u8) -> Self {
        match oversampling {
            0x00 => MLX90393OVERSAMPLING::OSR0,
            0x01 => MLX90393OVERSAMPLING::OSR1,
            0x02 => MLX90393OVERSAMPLING::OSR2,
            0x03 => MLX90393OVERSAMPLING::OSR3,
            _ => panic!("Invalid MLX90393OVERSAMPLING"),
        }
    }
}
