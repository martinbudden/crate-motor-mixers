#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum Command {
    #[default]
    MotorStop = 0,
    /// Wait at least 260ms before next command.
    Beep1,
    /// Wait at least 260ms before next command.
    Beep2,
    /// Wait at least 260ms before next command.
    Beep3,
    /// Wait at least 260ms before next command.
    Beep4,
    /// Wait at least 260ms before next command.
    Beep5,
    /// Wait at least 12ms before next command.
    EscInfo,
    /// Needs 6 transmissions.
    SpinDirection1,
    /// Needs 6 transmissions.
    SpinDirection2,
    /// Needs 6 transmissions.
    ThreeDModeOn,
    /// Needs 6 transmissions.
    ThreeDModeOff,
    SettingsRequest,
    /// Needs 6 transmissions. Wait at least 35ms before next command.
    SettingsSave,
    /// Needs 6 transmissions.
    ExtendedTelemetryEnable,
    /// Needs 6 transmissions.
    ExtendedTelemetryDisable,

    // 15-19 are unassigned.
    /// Needs 6 transmissions.
    SpinDirectionNormal = 20,
    /// Needs 6 transmissions.
    SpinDirectionReversed,
    Led0On,
    Led1On,
    Led2On,
    Led3On,
    Led0Off,
    Led1Off,
    Led2Off,
    Led3Off,
    AudioStreamModeToggle,
    SilentModeToggle,
    /// Needs 6 transmissions. Enables individual signal line commands.
    SignalLineTelemetryEnable,
    /// Needs 6 transmissions. Disables individual signal line commands.
    SignalLineTelemetryDisable,
    /// Needs 6 transmissions. Enables individual signal line commands.
    SignalLineContinuousERPMTelemetry,
    /// Needs 6 transmissions. Enables individual signal line commands.
    SignalLineContinuousERPMPeriodTelemetry,

    // 36-41 are unassigned.
    /// 1ºC per LSB.
    SignalLineTemperatureTelemetry = 42,
    /// 10mV per LSB, 40.95V max.
    SignalLineVoltageTelemetry,
    /// 100mA per LSB, 409.5A max.
    SignalLineCurrentTelemetry,
    /// 10mAh per LSB, 40.95Ah max.
    SignalLineConsumptionTelemetry,
    /// 100erpm per LSB, 409500erpm max.
    SignalLineERPMTelemetry,
    /// 16us per LSB, 65520us max.
    SignalLineERPMPeriodTelemetry,
}

impl TryFrom<u8> for Command {
    type Error = ();

    /// Validating conversion from `u8` to `Command`. Invalid values return error.
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

impl Command {
    /// Forgiving conversion from `u8` to `Command`, converts invalid values to default.
    #[must_use]
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => Self::MotorStop,
            1 => Self::Beep1,
            2 => Self::Beep2,
            3 => Self::Beep3,
            4 => Self::Beep4,
            5 => Self::Beep5,
            6 => Self::EscInfo,
            7 => Self::SpinDirection1,
            8 => Self::SpinDirection2,
            9 => Self::ThreeDModeOn,
            10 => Self::ThreeDModeOff,
            11 => Self::SettingsRequest,
            12 => Self::SettingsSave,
            13 => Self::ExtendedTelemetryEnable,
            14 => Self::ExtendedTelemetryDisable,
            20 => Self::SpinDirectionNormal,
            21 => Self::SpinDirectionReversed,
            22 => Self::Led0On,
            23 => Self::Led1On,
            24 => Self::Led2On,
            25 => Self::Led3On,
            26 => Self::Led0Off,
            27 => Self::Led1Off,
            28 => Self::Led2Off,
            29 => Self::Led3Off,
            30 => Self::AudioStreamModeToggle,
            31 => Self::SilentModeToggle,
            32 => Self::SignalLineTelemetryEnable,
            33 => Self::SignalLineTelemetryDisable,
            34 => Self::SignalLineContinuousERPMTelemetry,
            35 => Self::SignalLineContinuousERPMPeriodTelemetry,
            // 36-41 are unassigned.
            42 => Self::SignalLineTemperatureTelemetry,
            43 => Self::SignalLineVoltageTelemetry,
            44 => Self::SignalLineCurrentTelemetry,
            45 => Self::SignalLineConsumptionTelemetry,
            46 => Self::SignalLineERPMTelemetry,
            47 => Self::SignalLineERPMPeriodTelemetry,
            _ => Self::default(),
        }
    }
}

impl Command {
    #[allow(unused)]
    pub const fn repetitions_required(self) -> u8 {
        match self {
            // Protocol docs often stat 6 repeats for these commands.
            // We use 10, like Betaflight, as a conservative measure.
            Self::SpinDirection1
            | Self::SpinDirection2
            | Self::ThreeDModeOn
            | Self::ThreeDModeOff
            | Self::SettingsSave
            | Self::ExtendedTelemetryEnable
            | Self::ExtendedTelemetryDisable
            | Self::SpinDirectionNormal
            | Self::SpinDirectionReversed
            | Self::SignalLineTelemetryEnable
            | Self::SignalLineTelemetryDisable
            | Self::SignalLineContinuousERPMTelemetry
            | Self::SignalLineContinuousERPMPeriodTelemetry => 10,
            _ => 1,
        }
    }

    #[allow(unused)]
    pub const fn delay_required_us(self) -> u32 {
        match self {
            Self::Beep1 | Self::Beep2 | Self::Beep3 | Self::Beep4 | Self::Beep5 => 260_000,
            Self::EscInfo => 12_000,
            Self::SettingsSave => 35_000,
            _ => 0,
        }
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full_eq<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + Eq + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full_eq::<Command>();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn u8_to_command() {
        assert_eq!(Command::from_u8(0), Command::MotorStop);
        assert_eq!(Command::from_u8(1), Command::Beep1);
        assert_eq!(Command::from_u8(35), Command::SignalLineContinuousERPMPeriodTelemetry);
        assert_eq!(Command::from_u8(36), Command::MotorStop);
        assert_eq!(Command::from_u8(41), Command::MotorStop);
        assert_eq!(Command::from_u8(42), Command::SignalLineTemperatureTelemetry);
        assert_eq!(Command::from_u8(47), Command::SignalLineERPMPeriodTelemetry);
        assert_eq!(Command::from_u8(48), Command::MotorStop);

        assert_eq!(Command::try_from(0), Ok(Command::MotorStop));
        assert_eq!(Command::try_from(1), Ok(Command::Beep1));
        assert_eq!(Command::try_from(35), Ok(Command::SignalLineContinuousERPMPeriodTelemetry));
        assert_eq!(Command::try_from(36), Err(()));
        assert_eq!(Command::try_from(41), Err(()));
        assert_eq!(Command::try_from(42), Ok(Command::SignalLineTemperatureTelemetry));
        assert_eq!(Command::try_from(47), Ok(Command::SignalLineERPMPeriodTelemetry));
        assert_eq!(Command::try_from(48), Err(()));
    }
}
