#![allow(unused)]

mod mixer_airplane;
mod mixer_bicopter;
mod mixer_wing;

mod mixer_tricopter;

mod mixer_hexacopter;
mod mixer_octocopter;
mod mixer_quadcopter;

pub use mixer_airplane::MixerAirplane;
pub use mixer_bicopter::MixerBicopter;
pub use mixer_wing::MixerWing;

pub use mixer_tricopter::MixerTricopter;

pub use mixer_hexacopter::MixerHexacopter;
pub use mixer_octocopter::MixerOctocopter;
pub use mixer_quadcopter::MixerQuadcopter;
