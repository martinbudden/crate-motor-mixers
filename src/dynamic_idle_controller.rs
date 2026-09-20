#[cfg(feature = "storage")]
use sequential_storage::map::PostcardValue;
#[cfg(feature = "serde")]
use {
    postcard::experimental::max_size::MaxSize,
    serde::{Deserialize, Serialize},
};

pub use pidsk_controller::{PidControllerf32, PidGainsf32};
pub use signal_filters::{Pt1Filterf32, SignalFilter};

/// Conversion between RPM and Hz.
pub trait RpmHz: Sized {
    /// Convert from rpm to Hz.
    #[must_use]
    fn to_hz(self) -> Self;
    /// Convert from Hz to rpm.
    #[must_use]
    fn to_rpm(self) -> Self;
}

impl RpmHz for f32 {
    fn to_hz(self) -> Self {
        self / 60.0
    }
    fn to_rpm(self) -> Self {
        self * 60.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
#[allow(missing_docs)]
pub struct DynamicIdleControllerConfig {
    pub dyn_idle_min_rpm_d100: u8, // multiply this by 100 to get the actual min RPM
    pub dyn_idle_p_gain_x100: u8,  // divide this by 100 to get the actual kp
    pub dyn_idle_i_gain_x100: u8,
    pub dyn_idle_d_gain_x100: u8,
    pub dyn_idle_max_increase: u8,
}

#[cfg(feature = "storage")]
impl PostcardValue<'_> for DynamicIdleControllerConfig {}

impl Default for DynamicIdleControllerConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl DynamicIdleControllerConfig {
    /// Constructor.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            dyn_idle_min_rpm_d100: 0,
            dyn_idle_p_gain_x100: 50,
            dyn_idle_i_gain_x100: 50,
            dyn_idle_d_gain_x100: 50,
            dyn_idle_max_increase: 150,
        }
    }
}

/// PID controller to boost motor speeds so that slowest motor does not go below minimum allowed RPM.
///
/// A minimum RPM is required because the ESC will desynchronize if the motors turn too slowly (since they won't generate
/// enough back EMF for the ESC know the position of the rotor relative to the windings).
///
/// Note that a simple minimum output value is not sufficient: consider the case where the throttle is cut while hovering,
/// the quad will start to fall and this falling will generate a reverse torque on the motors which will eventually
/// overcome the fixed output value. Many types of maneuver can generate this reverse torque.
///
/// Instead we have a PID controller that increases output to the motors as the slowest motor nears the minimum allowed RPM.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynamicIdleController {
    task_interval_microseconds: u32,
    minimum_allowed_motor_hz: f32, // minimum motor Hz, dynamically controlled
    max_increase: f32,
    pid: PidControllerf32, // PID to ensure slowest motor does not go below min RPS
    dterm_filter: Pt1Filterf32,
    config: DynamicIdleControllerConfig,
}

impl Default for DynamicIdleController {
    fn default() -> Self {
        Self::new(1000)
    }
}

impl DynamicIdleController {
    /// Constructor.
    #[must_use]
    pub fn new(task_interval_microseconds: u32) -> Self {
        Self {
            task_interval_microseconds,
            minimum_allowed_motor_hz: 0.0, // minimum motor Hz, dynamically controlled
            max_increase: 0.0,
            pid: PidControllerf32::default(), // PID for dynamic idle, ie to ensure slowest motor does not go below min RPS
            dterm_filter: Pt1Filterf32::new(),
            config: DynamicIdleControllerConfig::new(),
        }
    }

    #[inline]
    #[must_use]
    pub fn config(&self) -> DynamicIdleControllerConfig {
        self.config
    }

    pub fn set_config(&mut self, config: DynamicIdleControllerConfig) {
        self.config = config;

        // Convert max increase multiplier from thousandths to fractional float bounds
        self.max_increase = f32::from(self.config.dyn_idle_max_increase) * 0.001;

        // Convert RPM to Hz (RPS): RPM / 60
        self.minimum_allowed_motor_hz = f32::from(self.config.dyn_idle_min_rpm_d100) * 100.0 / 60.0;
        self.pid.set_setpoint(self.minimum_allowed_motor_hz);

        #[allow(clippy::cast_precision_loss)]
        let delta_t = self.task_interval_microseconds as f32 * 0.000_001;

        // Use Betaflight multipliers for compatibility with Betaflight Configurator
        let pid_gains = PidGainsf32 {
            kp: f32::from(self.config.dyn_idle_p_gain_x100) * 0.00015,
            ki: f32::from(self.config.dyn_idle_i_gain_x100) * 0.01 * delta_t,
            kd: f32::from(self.config.dyn_idle_d_gain_x100) * 0.000_000_3 / delta_t,
            ks: 0.0,
            kk: 0.0,
        };
        self.pid.set_gains(pid_gains);
        // Limit Iterm to range [0, _max_increase] to prevent integral windup.
        self.pid.set_integral_max(self.max_increase);
        self.pid.set_integral_min(0.0);

        // PT1 low pass filter step response cutoff selection (approx 20ms delay alignment)
        self.dterm_filter.set_k(800.0 * delta_t / 20.0);
    }

    #[inline]
    #[must_use]
    pub fn minimum_allowed_motor_hz(&self) -> f32 {
        self.minimum_allowed_motor_hz
    }

    #[inline]
    pub fn set_minimum_allowed_motor_hz(&mut self, minimum_allowed_motor_hz: f32) {
        self.minimum_allowed_motor_hz = minimum_allowed_motor_hz;
        self.pid.set_setpoint(self.minimum_allowed_motor_hz);
    }

    #[must_use]
    pub fn calculate_speed_increase(&mut self, slowest_motor_hz: f32, delta_t: f32) -> f32 {
        // Fast-path bypass if idle control feature is disabled
        if self.minimum_allowed_motor_hz <= 0.0 {
            return 0.0;
        }

        // Derive the localized frequency delta derivative step
        let slowest_motor_hz_delta_filtered =
            self.dterm_filter.update(slowest_motor_hz - self.pid.previous_measurement());

        // Calculate the raw PID loop response step adjustment
        let speed_increase = self.pid.update_delta(slowest_motor_hz, slowest_motor_hz_delta_filtered, delta_t);

        // Clamp the final correction to avoid overloading physical motor headroom ranges
        speed_increase.clamp(0.0, self.max_increase)
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}
    #[cfg(feature = "serde")]
    fn is_serde<T: Serialize + MaxSize + for<'a> Deserialize<'a>>() {}
    #[cfg(feature = "storage")]
    fn is_storage<T: for<'a> PostcardValue<'a>>() {}

    #[test]
    fn normal_types() {
        is_full::<DynamicIdleControllerConfig>();
        #[cfg(feature = "serde")]
        is_serde::<DynamicIdleControllerConfig>();
        #[cfg(feature = "storage")]
        is_storage::<DynamicIdleControllerConfig>();
        is_full::<DynamicIdleController>();
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]
    use super::*;
    #[allow(unused)]
    use approx::assert_abs_diff_eq;
    macro_rules! assert_near {
        ($left:expr, $right:expr) => {
            approx::assert_abs_diff_eq!($left, $right, epsilon = 4e-6);
        };
    }
    #[test]
    fn dynamic_idle_controller() {
        const TASK_INTERVAL_MICROSECONDS: u32 = 1000;
        const SLOWEST_MOTOR_HZ: f32 = 960.0 / 60.0; // 960 RPM = 16 Hz
        #[allow(clippy::cast_precision_loss)]
        const DELTA_T: f32 = TASK_INTERVAL_MICROSECONDS as f32 * 0.000_001;

        let dynamic_idle_controller_config = DynamicIdleControllerConfig::default();
        let mut dynamic_idle_controller = DynamicIdleController::new(TASK_INTERVAL_MICROSECONDS);
        dynamic_idle_controller.set_config(dynamic_idle_controller_config);

        assert_eq!(0, dynamic_idle_controller.config().dyn_idle_min_rpm_d100);

        assert_eq!(0.0, dynamic_idle_controller.calculate_speed_increase(0.0, DELTA_T));
        assert_eq!(960.0, SLOWEST_MOTOR_HZ.to_rpm());
        assert_eq!(SLOWEST_MOTOR_HZ, 960.0.to_hz());
        assert_eq!(0.0, dynamic_idle_controller.calculate_speed_increase(SLOWEST_MOTOR_HZ, DELTA_T));
    }

    #[test]
    fn dynamic_idle_controller_p_only() {
        const TASK_INTERVAL_MICROSECONDS: u32 = 1000;
        let dynamic_idle_controller_config = DynamicIdleControllerConfig {
            dyn_idle_min_rpm_d100: 12, // 12*100 = 1200 rpm
            dyn_idle_p_gain_x100: 50,  // 50/100 = 0.5
            dyn_idle_i_gain_x100: 0,
            dyn_idle_d_gain_x100: 0,
            dyn_idle_max_increase: 150,
        };
        let mut dynamic_idle_controller = DynamicIdleController::new(TASK_INTERVAL_MICROSECONDS);
        dynamic_idle_controller.set_config(dynamic_idle_controller_config);
        #[allow(clippy::cast_precision_loss)]
        let delta_t = TASK_INTERVAL_MICROSECONDS as f32 * 0.000_001;

        assert_eq!(12, dynamic_idle_controller.config().dyn_idle_min_rpm_d100);
        assert_eq!(20.0, 1200.0.to_hz());
        assert_eq!(1200.0.to_hz(), dynamic_idle_controller.minimum_allowed_motor_hz());

        // slowest motor faster than 1200 RPM, so no speed increase
        assert_eq!(0.0, dynamic_idle_controller.calculate_speed_increase(2000.0.to_hz(), delta_t));
        assert_eq!(0.0, dynamic_idle_controller.calculate_speed_increase(1200.0.to_hz(), delta_t));

        // slowest motor slower than 1200 RPM, so speed increase
        assert_eq!(0.075, dynamic_idle_controller.calculate_speed_increase(600.0.to_hz(), delta_t));
        assert_eq!(0.075, dynamic_idle_controller.calculate_speed_increase(600.0.to_hz(), delta_t));

        // half the speed difference from 1200, so half the output, since PID is P-Term only
        assert_near!(0.0375, dynamic_idle_controller.calculate_speed_increase(900.0.to_hz(), delta_t));
        assert_near!(0.0375, dynamic_idle_controller.calculate_speed_increase(900.0.to_hz(), delta_t));
    }
    #[cfg(feature = "serde")]
    #[test]
    //#[allow(clippy::field_reassign_with_default)]
    fn config() {
        use postcard::{from_bytes, to_slice};

        let config = DynamicIdleControllerConfig { dyn_idle_d_gain_x100: 119, ..Default::default() };
        let mut buf = [0u8; 64]; // Size based on your config size
        #[allow(clippy::unwrap_used)]
        let data = to_slice(&config, &mut buf).unwrap();
        assert_eq!(5, data.len());

        // Deserialize using postcard
        let config_read: DynamicIdleControllerConfig =
            from_bytes(data).unwrap_or_else(|_| DynamicIdleControllerConfig::default());
        assert_eq!(119, config_read.dyn_idle_d_gain_x100);
    }
}
