//! # minimal-argyll-rust
//!
//! A minimal Rust driver for **i1Display3** / **ColorMunki Display** color
//! measurement sensors, ported from the ArgyllCMS Python driver.
//!
//! Communicates directly over USB HID — no ArgyllCMS linking required.
//!
//! ## Quick start
//!
//! ```no_run
//! use minimal_argyll_rust::I1Display3;
//!
//! let sensor = I1Display3::open().unwrap();
//! let reading = sensor.measure_xyz().unwrap();
//! println!("X={:.4} Y={:.4} Z={:.4}", reading.x, reading.y, reading.z);
//! println!("CCT={:.0} K  Duv={:.4}", reading.cct(), reading.duv());
//! ```

mod i1d3;

pub use i1d3::{CalibrationData, I1Display3, SensorInfo, XyzReading};
