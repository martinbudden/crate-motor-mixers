#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum DshotProtocol {
    #[default]
    Dshot150 = 0,
    Dshot300 = 1,
    Dshot600 = 2,
    Dshot1200 = 3,
    Proshot = 4,
    W2818B = 255,
}

impl DshotProtocol {
    #[allow(unused)]
    #[must_use]
    pub const fn baud_rate(self) -> u32 {
        match self {
            Self::Dshot150 => 150_000,
            Self::Dshot300 => 300_000,
            Self::Dshot600 => 600_000,
            Self::Dshot1200 => 1_200_000,
            Self::Proshot => 1_000_000,
            Self::W2818B => 800_000,
        }
    }
}

impl TryFrom<u8> for DshotProtocol {
    type Error = ();

    /// Validating conversion from `u8` to `Protocol`. Invalid values return error.
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        let default = Self::default();
        if value == default as u8 {
            Ok(default)
        } else {
            let ret = Self::from_u8(value);
            if ret == default { Err(()) } else { Ok(ret) }
        }
    }
}

impl DshotProtocol {
    /// Forgiving conversion from `u8` to `Protocol`, converts invalid values to default.
    #[must_use]
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => Self::Dshot150,
            1 => Self::Dshot300,
            2 => Self::Dshot600,
            3 => Self::Dshot1200,
            4 => Self::Proshot,
            255 => Self::W2818B,
            _ => Self::default(),
        }
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full_eq<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + Eq + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full_eq::<DshotProtocol>();
    }
}
