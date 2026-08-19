/* 
 * Argyll Color Management System
 *
 * X-Rite i1d3 related defines
 *
 * Author: Graeme W. Gill
 * Date:   7/10/2007
 *
 * Copyright 2006 - 2013, Graeme W. Gill
 * All rights reserved.
 *
 * (Based on huey.h)
 *
 * This material is licenced under the GNU GENERAL PUBLIC LICENSE Version 2 or later :-
 * see the License2.txt file for licencing details.
 */
 
//! i1Display3 / ColorMunki Display — USB HID driver.
//!
//! Direct Rust port of the Python `i1d3.py` driver. Communicates with
//! i1Display3-family sensors over USB HID without
//! requiring direct linking to ArgyllCMS.
//!
//! Full pipeline:
//! - USB HID enumerate/open/unlock (challenge-response)
//! - EEPROM parsing (black levels, spectral sensitivities, ambient filter)
//! - CIE 1931 2° standard observer interpolation
//! - RGB→XYZ calibration matrix computation (MIbLSr)
//! - Adaptive two-stage measurement (frequency → period)
//! - Black level subtraction + matrix multiply → calibrated XYZ

use anyhow::{bail, Context, Result};
use std::time::Instant;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const VENDOR_ID: u16 = 0x0765;
const PRODUCT_ID: u16 = 0x5020;
const PRODUCT_ID_ALT: u16 = 0x5021;

const CLOCK_FREQ: f64 = 12_000_000.0;
const SAT_FREQ: f64 = 250_000.0;

const PACKET_SIZE: usize = 64;
const DEFAULT_TIMEOUT_MS: i32 = 1000;
const MEASURE_TIMEOUT_MS: i32 = 120_000;

/// Spectral array length: 351 points at 1 nm spacing (380-730 nm).
pub const SPEC_N: usize = 351;

// ---------------------------------------------------------------------------
// Command codes
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
#[allow(dead_code)]
enum Cmd {
    GetInfo       = 0x0000,
    GetStatus     = 0x0001,
    GetProdName   = 0x0010,
    GetProdType   = 0x0011,
    GetFirmVer    = 0x0012,
    GetFirmDate   = 0x0013,
    GetLocked     = 0x0020,
    FreqMeasure   = 0x0100,
    PeriodMeasure = 0x0200,
    ReadIntEe     = 0x0800,
    ReadExtEe     = 0x1200,
    SetLed        = 0x2100,
    GetDiffuser   = 0x9400,
    LockChallenge = 0x9900,
    LockResponse  = 0x9A00,
    Relock        = 0x9B00,
}

// ---------------------------------------------------------------------------
// Unlock keys (from ArgyllCMS i1d3.c)
// ---------------------------------------------------------------------------

struct UnlockKey {
    name: &'static str,
    key: (u32, u32),
    variant: &'static str,
}

const UNLOCK_KEYS: &[UnlockKey] = &[
    UnlockKey { name: "i1Display3",         key: (0xE9622E9F, 0x8D63E133), variant: "standard" },
    UnlockKey { name: "Colormunki Display", key: (0xE01E6E0A, 0x257462DE), variant: "colormunki" },
    UnlockKey { name: "i1Display3",         key: (0xCAA62B2C, 0x30815B61), variant: "OEM" },
    UnlockKey { name: "i1Display3",         key: (0xA9119479, 0x5B168761), variant: "NEC SpectraSensor Pro" },
    UnlockKey { name: "i1Display3",         key: (0x160EB6AE, 0x14440E70), variant: "Quato Silver Haze 3" },
    UnlockKey { name: "i1Display3",         key: (0x291E41D7, 0x51937BDD), variant: "HP DreamColor" },
    UnlockKey { name: "i1Display3",         key: (0x1ABFAE03, 0xF25AC8E8), variant: "Wacom DC" },
    UnlockKey { name: "i1Display3",         key: (0x828C43E9, 0xCBB8A8ED), variant: "TPA-1" },
    UnlockKey { name: "i1Display3",         key: (0xE8D1A980, 0xD146F7AD), variant: "Barco" },
    UnlockKey { name: "i1Display3",         key: (0x171AE295, 0x2E5C7664), variant: "EIZO Crysta" },
    UnlockKey { name: "i1Display3",         key: (0x64D8C546, 0x4B24B4A7), variant: "ViewSonic XRi1" },
];

// ---------------------------------------------------------------------------
// CIE 1931 2° Standard Observer (5 nm, 380–780 nm, 81 points)
// ---------------------------------------------------------------------------

#[rustfmt::skip]
const CIE_X_5NM: [f64; 81] = [
    0.001368, 0.002236, 0.004243, 0.007650, 0.014310,
    0.023190, 0.043510, 0.077630, 0.134380, 0.214770,
    0.283900, 0.328500, 0.348280, 0.348060, 0.336200,
    0.318700, 0.290800, 0.251100, 0.195360, 0.142100,
    0.095640, 0.058010, 0.032010, 0.014700, 0.004900,
    0.002400, 0.009300, 0.029100, 0.063270, 0.109600,
    0.165500, 0.225750, 0.290400, 0.359700, 0.433450,
    0.512050, 0.594500, 0.678400, 0.762100, 0.842500,
    0.916300, 0.978600, 1.026300, 1.056700, 1.062200,
    1.045600, 1.002600, 0.938400, 0.854450, 0.751400,
    0.642400, 0.541900, 0.447900, 0.360800, 0.283500,
    0.218700, 0.164900, 0.121200, 0.087400, 0.063600,
    0.046770, 0.032900, 0.022700, 0.015840, 0.011359,
    0.008111, 0.005790, 0.004109, 0.002899, 0.002049,
    0.001440, 0.001000, 0.000690, 0.000476, 0.000332,
    0.000235, 0.000166, 0.000117, 0.000083, 0.000059,
    0.000042,
];

#[rustfmt::skip]
const CIE_Y_5NM: [f64; 81] = [
    0.000039, 0.000064, 0.000120, 0.000217, 0.000396,
    0.000640, 0.001210, 0.002180, 0.004000, 0.007300,
    0.011600, 0.016840, 0.023000, 0.029800, 0.038000,
    0.048000, 0.060000, 0.073900, 0.090980, 0.112600,
    0.139020, 0.169300, 0.208020, 0.258600, 0.323000,
    0.407300, 0.503000, 0.608200, 0.710000, 0.793200,
    0.862000, 0.914850, 0.954000, 0.980300, 0.994950,
    1.000000, 0.995000, 0.978600, 0.952000, 0.915400,
    0.870000, 0.816300, 0.757000, 0.694900, 0.631000,
    0.566800, 0.503000, 0.441200, 0.381000, 0.321000,
    0.265000, 0.217000, 0.175000, 0.138200, 0.107000,
    0.081600, 0.061000, 0.044580, 0.032000, 0.023200,
    0.017000, 0.011920, 0.008210, 0.005723, 0.004102,
    0.002929, 0.002091, 0.001484, 0.001047, 0.000740,
    0.000520, 0.000361, 0.000249, 0.000172, 0.000120,
    0.000085, 0.000060, 0.000042, 0.000030, 0.000021,
    0.000015,
];

#[rustfmt::skip]
const CIE_Z_5NM: [f64; 81] = [
    0.006450, 0.010550, 0.020050, 0.036210, 0.067850,
    0.110200, 0.207400, 0.371300, 0.645600, 1.039050,
    1.385600, 1.622960, 1.747060, 1.782600, 1.772110,
    1.744100, 1.669200, 1.528100, 1.287640, 1.041900,
    0.812950, 0.616200, 0.465180, 0.353300, 0.272000,
    0.212300, 0.158200, 0.111700, 0.078250, 0.057250,
    0.042160, 0.029840, 0.020300, 0.013400, 0.008750,
    0.005750, 0.003900, 0.002750, 0.002100, 0.001800,
    0.001650, 0.001400, 0.001100, 0.001000, 0.000800,
    0.000600, 0.000340, 0.000240, 0.000190, 0.000100,
    0.000050, 0.000030, 0.000020, 0.000010, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    0.000000,
];

/// Interpolate CIE 1931 2° observer from 5 nm to 1 nm over 380–730 nm.
fn interpolate_cie_1nm() -> ([f64; SPEC_N], [f64; SPEC_N], [f64; SPEC_N]) {
    let mut x = [0.0f64; SPEC_N];
    let mut y = [0.0f64; SPEC_N];
    let mut z = [0.0f64; SPEC_N];
    for i in 0..SPEC_N {
        let wl = 380.0 + i as f64;
        let idx_f = (wl - 380.0) / 5.0;
        let idx = (idx_f as usize).min(CIE_X_5NM.len() - 2);
        let frac = idx_f - idx as f64;
        x[i] = CIE_X_5NM[idx] * (1.0 - frac) + CIE_X_5NM[idx + 1] * frac;
        y[i] = CIE_Y_5NM[idx] * (1.0 - frac) + CIE_Y_5NM[idx + 1] * frac;
        z[i] = CIE_Z_5NM[idx] * (1.0 - frac) + CIE_Z_5NM[idx + 1] * frac;
    }
    (x, y, z)
}

// ---------------------------------------------------------------------------
// 3×3 matrix math
// ---------------------------------------------------------------------------

type Mat3 = [[f64; 3]; 3];

const MAT3_IDENTITY: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

fn mat3_mul_vec(m: &Mat3, v: &[f64; 3]) -> [f64; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

fn mat3_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                r[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    r
}

fn mat3_transpose(m: &Mat3) -> Mat3 {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

fn mat3_inverse(m: &Mat3) -> Option<Mat3> {
    let mut a = *m;
    let mut inv = MAT3_IDENTITY;
    for col in 0..3 {
        let mut max_row = col;
        for row in (col + 1)..3 {
            if a[row][col].abs() > a[max_row][col].abs() {
                max_row = row;
            }
        }
        a.swap(col, max_row);
        inv.swap(col, max_row);
        let pivot = a[col][col];
        if pivot.abs() < 1e-15 {
            return None;
        }
        for j in 0..3 {
            a[col][j] /= pivot;
            inv[col][j] /= pivot;
        }
        for row in 0..3 {
            if row == col {
                continue;
            }
            let factor = a[row][col];
            for j in 0..3 {
                a[row][j] -= factor * a[col][j];
                inv[row][j] -= factor * inv[col][j];
            }
        }
    }
    Some(inv)
}

// ---------------------------------------------------------------------------
// Calibration matrix computation (MIbLSr)
// ---------------------------------------------------------------------------

fn compute_calibration_matrix(
    sens: &[[f64; SPEC_N]; 3],
    cie_x: &[f64; SPEC_N],
    cie_y: &[f64; SPEC_N],
    cie_z: &[f64; SPEC_N],
) -> Result<Mat3> {
    // Use sensor sensitivities as spectral samples (MIbLSr default)
    let nsamp = 3usize;

    // Compute XYZ and sensor RGB for each of the 3 sensor channels
    let mut samp_xyz = [[0.0f64; 3]; 3];
    let mut samp_rgb = [[0.0f64; 3]; 3];

    for k in 0..nsamp {
        // XYZ: integrate sample × CIE observer
        let mut x_sum = 0.0;
        let mut y_sum = 0.0;
        let mut z_sum = 0.0;
        for i in 0..SPEC_N {
            x_sum += sens[k][i] * cie_x[i];
            y_sum += sens[k][i] * cie_y[i];
            z_sum += sens[k][i] * cie_z[i];
        }
        samp_xyz[k] = [x_sum, y_sum, z_sum];

        // Sensor RGB: integrate sample × sensor sensitivity / 0.683002
        for j in 0..3 {
            let mut sum = 0.0;
            for i in 0..SPEC_N {
                sum += sens[k][i] * sens[j][i];
            }
            samp_rgb[k][j] = sum / 0.683002;
        }
    }

    // Direct 3×3 inversion (exact solution for 3 samples)
    let rgb_mat: Mat3 = [samp_rgb[0], samp_rgb[1], samp_rgb[2]];
    let xyz_mat: Mat3 = [samp_xyz[0], samp_xyz[1], samp_xyz[2]];

    let irgb = mat3_inverse(&rgb_mat)
        .context("Sensor RGB matrix is singular — cannot compute calibration")?;

    let mat = mat3_mul(&irgb, &xyz_mat);
    Ok(mat3_transpose(&mat))
}

// ---------------------------------------------------------------------------
// Unlock challenge-response
// ---------------------------------------------------------------------------

fn create_unlock_response(key: (u32, u32), challenge: &[u8]) -> [u8; PACKET_SIZE] {
    let m: u32 = 0xFFFFFFFF;
    let (k0, k1) = key;

    let mut sc = [0u8; 8];
    for i in 0..8 {
        sc[i] = challenge[3] ^ challenge[35 + i];
    }

    let ci0 = (sc[3] as u32) << 24 | (sc[0] as u32) << 16 | (sc[4] as u32) << 8 | sc[6] as u32;
    let ci1 = (sc[1] as u32) << 24 | (sc[7] as u32) << 16 | (sc[2] as u32) << 8 | sc[5] as u32;

    let neg_k0 = k0.wrapping_neg();
    let neg_k1 = k1.wrapping_neg();

    let co0 = neg_k0.wrapping_sub(ci1);
    let co1 = neg_k1.wrapping_sub(ci0);
    let co2 = ci1.wrapping_mul(neg_k0);
    let co3 = ci0.wrapping_mul(neg_k1);

    let mut s: u32 = sc.iter().map(|&b| b as u32).sum();
    for nk in [neg_k0, neg_k1] {
        s += (nk & 0xFF) + ((nk >> 8) & 0xFF) + ((nk >> 16) & 0xFF) + ((nk >> 24) & 0xFF);
    }
    let s0 = (s & 0xFF) as u8;
    let s1 = ((s >> 8) & 0xFF) as u8;

    let sr: [u8; 16] = [
        ((co0 >> 16) as u8).wrapping_add(s0),
        ((co2 >> 8) as u8).wrapping_sub(s1),
        (co3 as u8).wrapping_add(s1),
        ((co1 >> 16) as u8).wrapping_add(s0),
        ((co2 >> 16) as u8).wrapping_sub(s1),
        ((co3 >> 16) as u8).wrapping_sub(s0),
        ((co1 >> 24) as u8).wrapping_sub(s0),
        (co0 as u8).wrapping_sub(s1),
        ((co3 >> 8) as u8).wrapping_add(s0),
        ((co2 >> 24) as u8).wrapping_sub(s1),
        ((co0 >> 8) as u8).wrapping_add(s0),
        ((co1 >> 8) as u8).wrapping_sub(s1),
        (co1 as u8).wrapping_add(s1),
        ((co3 >> 24) as u8).wrapping_add(s1),
        (co2 as u8).wrapping_add(s0),
        ((co0 >> 24) as u8).wrapping_sub(s0),
    ];

    let mut r = [0u8; PACKET_SIZE];
    let encode_key = challenge[2];
    for i in 0..16 {
        r[24 + i] = sr[i] ^ encode_key;
    }
    let _ = m; // suppress unused warning
    r
}

// ---------------------------------------------------------------------------
// Calibration data
// ---------------------------------------------------------------------------

#[derive(Clone)]
pub struct CalibrationData {
    pub serial_no: String,
    pub version_no: String,
    pub hw_revision: u8,
    pub black: [f64; 3],
    pub sens: [[f64; SPEC_N]; 3],
    pub ambi: [[f64; SPEC_N]; 3],
    pub ambi_filter: [f64; SPEC_N],
    pub emis_cal: Mat3,
    pub ambi_cal: Mat3,
    pub ccmat: Mat3,
    loaded: bool,
}

impl Default for CalibrationData {
    fn default() -> Self {
        Self {
            serial_no: String::new(),
            version_no: String::new(),
            hw_revision: 0,
            black: [0.0; 3],
            sens: [[0.0; SPEC_N]; 3],
            ambi: [[0.0; SPEC_N]; 3],
            ambi_filter: [0.0; SPEC_N],
            emis_cal: MAT3_IDENTITY,
            ambi_cal: MAT3_IDENTITY,
            ccmat: MAT3_IDENTITY,
            loaded: false,
        }
    }
}

// ---------------------------------------------------------------------------
// Measurement result
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct XyzReading {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub rgb_hz: [f64; 3],
    pub elapsed_ms: u32,
}

impl XyzReading {
    /// CIE x chromaticity.
    pub fn cx(&self) -> f64 {
        let s = self.x + self.y + self.z;
        if s > 0.0 { self.x / s } else { 0.0 }
    }

    /// CIE y chromaticity.
    pub fn cy(&self) -> f64 {
        let s = self.x + self.y + self.z;
        if s > 0.0 { self.y / s } else { 0.0 }
    }

    /// CCT (McCamy's approximation).
    pub fn cct(&self) -> f64 {
        let (xc, yc) = (self.cx(), self.cy());
        if yc == 0.0 {
            return 0.0;
        }
        let n = (xc - 0.3320) / (0.1858 - yc);
        449.0 * n * n * n + 3525.0 * n * n + 6823.3 * n + 5520.33
    }

    /// Duv (distance from Planckian locus, Ohno 2014 approximation).
    pub fn duv(&self) -> f64 {
        let (xc, yc) = (self.cx(), self.cy());
        let denom = -2.0 * xc + 12.0 * yc + 3.0;
        if denom == 0.0 {
            return 0.0;
        }
        let u = 4.0 * xc / denom;
        let v = 6.0 * yc / denom;
        let lfp = ((u - 0.292_f64).powi(2) + (v - 0.24_f64).powi(2)).sqrt();
        if lfp <= 0.0 {
            return 0.0;
        }
        let a = ((u - 0.292) / lfp).acos();
        let lbb = 0.00213 * a.powi(6) - 0.0322 * a.powi(5) + 0.192 * a.powi(4)
            - 0.572 * a.powi(3) + 0.8937 * a.powi(2) - 0.7341 * a + 0.2547;
        lfp - lbb
    }
}

// ---------------------------------------------------------------------------
// Sensor info
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, serde::Serialize)]
pub struct SensorInfo {
    pub model: String,
    pub serial: String,
    pub hw_revision: String,
    pub firmware: String,
    pub unlock_variant: String,
}

// ---------------------------------------------------------------------------
// I1Display3 driver
// ---------------------------------------------------------------------------

pub struct I1Display3 {
    device: hidapi::HidDevice,
    cal: CalibrationData,
    unlock_variant: String,
    cie_x: [f64; SPEC_N],
    cie_y: [f64; SPEC_N],
    cie_z: [f64; SPEC_N],
}

impl I1Display3 {
    fn open_from_info(api: &hidapi::HidApi, info: &hidapi::DeviceInfo) -> Result<Self> {
        let device = info.open_device(api).context(
            "Cannot open i1Display3 — check permissions. \
             On Linux, add a udev rule for VID 0x0765 or run as root."
        )?;

        // Flush any stale data
        let mut flush_buf = [0u8; PACKET_SIZE];
        let _ = device.read_timeout(&mut flush_buf, 100);

        let (cie_x, cie_y, cie_z) = interpolate_cie_1nm();

        let mut sensor = Self {
            device,
            cal: CalibrationData::default(),
            unlock_variant: String::new(),
            cie_x,
            cie_y,
            cie_z,
        };

        // Unlock
        sensor.unlock()?;

        // Read EEPROMs and compute calibration matrices
        sensor.calibrate()?;

        tracing::info!(
            "i1d3: opened {} (HW rev {}, variant: {})",
            sensor.cal.serial_no,
            (b'A' + sensor.cal.hw_revision) as char,
            sensor.unlock_variant,
        );

        Ok(sensor)
    }

    /// Open and initialize the sensor: enumerate → open → flush → unlock → calibrate.
    pub fn open() -> Result<Self> {
        let api = hidapi::HidApi::new().context("failed to initialize hidapi")?;

        // Find operational device
        let mut device_info = None;
        let mut alt_count = 0;

        for info in api.device_list() {
            if info.vendor_id() != VENDOR_ID {
                continue;
            }
            if info.product_id() == PRODUCT_ID {
                device_info = Some(info.clone());
                break;
            }
            if info.product_id() == PRODUCT_ID_ALT {
                alt_count += 1;
            }
        }

        let info = match device_info {
            Some(i) => i,
            None => {
                if alt_count > 0 {
                    bail!(
                        "Found {alt_count} device(s) in boot/DFU mode (PID 0x{PRODUCT_ID_ALT:04X}) \
                         but none with operational PID 0x{PRODUCT_ID:04X}. \
                         Try unplugging and reconnecting."
                    );
                }
                bail!(
                    "No i1Display3 found on USB. \
                     Is the sensor plugged in? \
                     Is another application holding the device?"
                );
            }
        };

        Self::open_from_info(&api, &info)
    }

    /// Open and initialize up to `max_sensors` operational i1Display3 devices.
    pub fn open_all(max_sensors: usize) -> Result<Vec<Self>> {
        let api = hidapi::HidApi::new().context("failed to initialize hidapi")?;
        let mut infos: Vec<hidapi::DeviceInfo> = api
            .device_list()
            .filter(|info| info.vendor_id() == VENDOR_ID && info.product_id() == PRODUCT_ID)
            .cloned()
            .collect();

        if infos.is_empty() {
            bail!(
                "No i1Display3 found on USB. \
                 Is the sensor plugged in? \
                 Is another application holding the device?"
            );
        }

        infos.sort_by(|a, b| {
            let ka = format!(
                "{:04x}:{:04x}:{:?}",
                a.vendor_id(),
                a.product_id(),
                a.path()
            );
            let kb = format!(
                "{:04x}:{:04x}:{:?}",
                b.vendor_id(),
                b.product_id(),
                b.path()
            );
            ka.cmp(&kb)
        });

        let limit = if max_sensors == 0 {
            infos.len()
        } else {
            max_sensors.min(infos.len())
        };

        let mut sensors = Vec::with_capacity(limit);
        for info in infos.iter().take(limit) {
            sensors.push(Self::open_from_info(&api, info)?);
        }

        Ok(sensors)
    }

    // -- Low-level HID --

    fn command(&self, cmd: Cmd, payload: &[u8], timeout_ms: i32) -> Result<[u8; PACKET_SIZE]> {
        let major = ((cmd as u16) >> 8) as u8;
        let minor = (cmd as u16 & 0xFF) as u8;

        let mut send = [0u8; PACKET_SIZE];
        send[0] = major;
        if major == 0x00 {
            send[1] = minor;
            let n = payload.len().min(PACKET_SIZE - 2);
            send[2..2 + n].copy_from_slice(&payload[..n]);
        } else {
            let n = payload.len().min(PACKET_SIZE - 1);
            send[1..1 + n].copy_from_slice(&payload[..n]);
        }

        // hidapi on Linux requires report ID 0x00 prefix
        let mut out = [0u8; 1 + PACKET_SIZE];
        out[0] = 0x00;
        out[1..].copy_from_slice(&send);
        self.device
            .write(&out)
            .context("HID write failed")?;

        let mut recv = [0u8; PACKET_SIZE];
        let n = self.device
            .read_timeout(&mut recv, timeout_ms)
            .context("HID read failed")?;
        if n == 0 {
            bail!("HID read timed out after {timeout_ms}ms — device may be unresponsive");
        }

        let status = recv[0];
        if status != 0x00 {
            bail!(
                "Command {:?} failed with status 0x{status:02X}",
                cmd
            );
        }

        // Echo check (skip for commands that don't echo)
        if cmd != Cmd::GetDiffuser {
            let echoed = recv[1];
            if echoed != major {
                bail!(
                    "Command echo mismatch: sent 0x{major:02X}, got 0x{echoed:02X}"
                );
            }
        }

        Ok(recv)
    }

    fn command_raw_send(&self, cmd: Cmd, raw: &[u8; PACKET_SIZE], timeout_ms: i32) -> Result<[u8; PACKET_SIZE]> {
        let major = ((cmd as u16) >> 8) as u8;

        let mut send = *raw;
        send[0] = major;

        let mut out = [0u8; 1 + PACKET_SIZE];
        out[0] = 0x00;
        out[1..].copy_from_slice(&send);
        self.device.write(&out).context("HID write failed")?;

        let mut recv = [0u8; PACKET_SIZE];
        let n = self.device
            .read_timeout(&mut recv, timeout_ms)
            .context("HID read failed")?;
        if n == 0 {
            bail!("HID read timed out");
        }

        Ok(recv)
    }

    // -- Unlock --

    fn get_lock_status(&self) -> Result<bool> {
        let recv = self.command(Cmd::GetLocked, &[], DEFAULT_TIMEOUT_MS)?;
        // Locked if recv[2] == 0 && recv[3] != 0
        Ok(recv[2] == 0 && recv[3] != 0)
    }

    fn unlock(&mut self) -> Result<()> {
        if !self.get_lock_status()? {
            self.unlock_variant = "(already unlocked)".to_string();
            return Ok(());
        }

        for key_info in UNLOCK_KEYS {
            let challenge = self.command(Cmd::LockChallenge, &[], DEFAULT_TIMEOUT_MS)?;
            let response = create_unlock_response(key_info.key, &challenge);
            let recv = self.command_raw_send(Cmd::LockResponse, &response, DEFAULT_TIMEOUT_MS)?;

            if recv[2] == 0x77 {
                if !self.get_lock_status()? {
                    self.unlock_variant = key_info.variant.to_string();
                    return Ok(());
                }
            }

            // Relock before trying next key
            let _ = self.command(Cmd::Relock, &[], DEFAULT_TIMEOUT_MS);
        }

        bail!("All unlock keys rejected. This device may use an unknown OEM key.");
    }

    // -- EEPROM reading --

    fn read_internal_eeprom(&self, mut address: u8, mut length: usize) -> Result<Vec<u8>> {
        let mut result = Vec::with_capacity(length);
        while length > 0 {
            let chunk = length.min(60);
            let payload = [address, chunk as u8];
            let recv = self.command(Cmd::ReadIntEe, &payload, DEFAULT_TIMEOUT_MS)?;
            result.extend_from_slice(&recv[4..4 + chunk]);
            address = address.wrapping_add(chunk as u8);
            length -= chunk;
        }
        Ok(result)
    }

    fn read_external_eeprom(&self, mut address: u16, mut length: usize) -> Result<Vec<u8>> {
        let mut result = Vec::with_capacity(length);
        while length > 0 {
            let chunk = length.min(59);
            let payload = [
                (address >> 8) as u8,
                address as u8,
                chunk as u8,
            ];
            let recv = self.command(Cmd::ReadExtEe, &payload, DEFAULT_TIMEOUT_MS)?;
            result.extend_from_slice(&recv[5..5 + chunk]);
            address += chunk as u16;
            length -= chunk;
        }
        Ok(result)
    }

    // -- EEPROM parsing --

    fn decode_internal_eeprom(&mut self, buf: &[u8]) {
        // Serial number at offset 0x10, up to 20 bytes
        let serial_end = buf[0x10..0x10 + 20]
            .iter()
            .position(|&b| b == 0 || b == 0xFF)
            .unwrap_or(20);
        self.cal.serial_no = String::from_utf8_lossy(&buf[0x10..0x10 + serial_end]).to_string();

        // Version at offset 0x2C, up to 10 bytes
        let ver_end = buf[0x2C..0x2C + 10]
            .iter()
            .position(|&b| b == 0 || b == 0xFF)
            .unwrap_or(10);
        self.cal.version_no = String::from_utf8_lossy(&buf[0x2C..0x2C + ver_end]).to_string();

        // HW revision from version string first char
        if let Some(first) = self.cal.version_no.bytes().next() {
            if first >= b'A' {
                self.cal.hw_revision = first - b'A';
            }
        }

        // Black level offsets: 3 × uint32 LE at offset 0x04
        for i in 0..3 {
            let off = 0x04 + 4 * i;
            let t1 = u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]]);
            if t1 == 0xFFFFFFFF {
                self.cal.black[i] = 0.0;
            } else {
                self.cal.black[i] = t1 as f64 / 6e6;
            }
        }
    }

    fn decode_external_eeprom(&mut self, buf: &[u8]) {
        // 3 × sensor spectral sensitivities (351 × f32 LE each)
        // Base offset 0x0026, units: Hz per W/nm → divide by 1000
        for j in 0..3 {
            let base = 0x0026 + j * SPEC_N * 4;
            for i in 0..SPEC_N {
                let off = base + i * 4;
                let val = f32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]]);
                self.cal.sens[j][i] = val as f64 / 1000.0;
            }
        }

        // Copy sensor sensitivities as ambient base
        self.cal.ambi = self.cal.sens;

        // Ambient diffuser filter spectrum (351 × f32 LE at offset 0x10BC)
        for i in 0..SPEC_N {
            let off = 0x10BC + i * 4;
            let val = f32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]]);
            self.cal.ambi_filter[i] = val as f64;
        }

        // Multiply ambient sensitivities by filter
        for j in 0..3 {
            for i in 0..SPEC_N {
                self.cal.ambi[j][i] *= self.cal.ambi_filter[i];
            }
        }
    }

    fn calibrate(&mut self) -> Result<()> {
        let int_ee = self.read_internal_eeprom(0, 256)?;
        self.decode_internal_eeprom(&int_ee);

        let ext_ee = self.read_external_eeprom(0, 6030)?;
        self.decode_external_eeprom(&ext_ee);

        // Compute emissive calibration matrix
        self.cal.emis_cal =
            compute_calibration_matrix(&self.cal.sens, &self.cie_x, &self.cie_y, &self.cie_z)?;

        // Compute ambient calibration matrix
        self.cal.ambi_cal =
            compute_calibration_matrix(&self.cal.ambi, &self.cie_x, &self.cie_y, &self.cie_z)?;

        self.cal.ccmat = MAT3_IDENTITY;
        self.cal.loaded = true;

        tracing::info!(
            "i1d3: calibrated — serial={}, black=[{:.6}, {:.6}, {:.6}]",
            self.cal.serial_no,
            self.cal.black[0],
            self.cal.black[1],
            self.cal.black[2],
        );

        Ok(())
    }

    // -- Measurements --

    fn measure_frequency(&self, integration_time: f64) -> Result<[f64; 3]> {
        let clamped = integration_time.min(20.0);
        let intclks = (clamped * CLOCK_FREQ + 0.5) as u32;
        let actual_time = intclks as f64 / CLOCK_FREQ;

        let payload = intclks.to_le_bytes();
        let recv = self.command(Cmd::FreqMeasure, &payload, MEASURE_TIMEOUT_MS)?;

        let r = u32::from_le_bytes([recv[2], recv[3], recv[4], recv[5]]) as f64 + 0.5;
        let g = u32::from_le_bytes([recv[6], recv[7], recv[8], recv[9]]) as f64 + 0.5;
        let b = u32::from_le_bytes([recv[10], recv[11], recv[12], recv[13]]) as f64 + 0.5;

        Ok([
            0.5 * r / actual_time,
            0.5 * g / actual_time,
            0.5 * b / actual_time,
        ])
    }

    fn measure_period(&self, edge_counts: [u16; 3], mask: u8) -> Result<Option<[f64; 3]>> {
        let mut payload = [0u8; 8];
        payload[0..2].copy_from_slice(&edge_counts[0].to_le_bytes());
        payload[2..4].copy_from_slice(&edge_counts[1].to_le_bytes());
        payload[4..6].copy_from_slice(&edge_counts[2].to_le_bytes());
        payload[6] = mask;

        let recv = self.command(Cmd::PeriodMeasure, &payload, MEASURE_TIMEOUT_MS);
        match recv {
            Ok(recv) => {
                let r = u32::from_le_bytes([recv[2], recv[3], recv[4], recv[5]]) as f64;
                let g = u32::from_le_bytes([recv[6], recv[7], recv[8], recv[9]]) as f64;
                let b = u32::from_le_bytes([recv[10], recv[11], recv[12], recv[13]]) as f64;

                let mut hz = [0.0f64; 3];
                let ec = [edge_counts[0] as f64, edge_counts[1] as f64, edge_counts[2] as f64];
                let clks = [r, g, b];
                for i in 0..3 {
                    if clks[i] > 0.5 {
                        hz[i] = (CLOCK_FREQ * 0.5 * ec[i]) / clks[i];
                    }
                }
                Ok(Some(hz))
            }
            Err(e) => {
                // Status 0x83 = period timeout (very dim) — return None to fall back
                let msg = format!("{e}");
                if msg.contains("0x83") {
                    Ok(None)
                } else {
                    Err(e)
                }
            }
        }
    }

    /// Check diffuser position: 0 = retracted (emissive), nonzero = deployed (ambient).
    fn get_diffuser_position(&self) -> Result<u8> {
        let recv = self.command(Cmd::GetDiffuser, &[], DEFAULT_TIMEOUT_MS)?;
        Ok(recv[1])
    }

    /// Adaptive two-stage measurement → calibrated XYZ.
    pub fn measure_xyz(&self) -> Result<XyzReading> {
        if !self.cal.loaded {
            bail!("Calibration data not loaded");
        }

        let t0 = Instant::now();
        let inttime = 0.2;

        // Stage 1: frequency measurement
        let mut rgb_hz = self.measure_frequency(inttime)?;
        let raw_edges = [
            rgb_hz[0] * inttime / 0.5,
            rgb_hz[1] * inttime / 0.5,
            rgb_hz[2] * inttime / 0.5,
        ];

        // Saturation check
        for i in 0..3 {
            if rgb_hz[i] > SAT_FREQ {
                bail!(
                    "Sensor saturated on channel {i} ({:.0} Hz > {SAT_FREQ:.0} Hz)",
                    rgb_hz[i]
                );
            }
        }

        // Stage 2: adaptive period refinement for dim channels
        let mut mask = 0u8;
        for i in 0..3 {
            if raw_edges[i] < 200.0 {
                mask |= 1 << i;
            }
        }

        if mask != 0 {
            // Pre-measurement with 2 edges
            if let Some(pre_hz) = self.measure_period([2, 2, 2], mask)? {
                let target_inttime = inttime.max(0.1);
                let mut edgec_main = [2u16; 3];
                let mut mask_main = 0u8;

                for i in 0..3 {
                    if (mask & (1 << i)) == 0 {
                        continue;
                    }
                    if pre_hz[i] > 0.001 {
                        let nedgec = (2.0 * pre_hz[i] * target_inttime + 0.5) as u16;
                        let nedgec = nedgec.max(2).min(65534);
                        let nedgec = if nedgec % 2 != 0 { nedgec + 1 } else { nedgec };
                        edgec_main[i] = nedgec;
                        mask_main |= 1 << i;
                        rgb_hz[i] = pre_hz[i]; // fallback
                    }
                }

                if mask_main != 0 {
                    if let Some(main_hz) = self.measure_period(edgec_main, mask_main)? {
                        for i in 0..3 {
                            if (mask_main & (1 << i)) != 0 && main_hz[i] > 0.001 {
                                rgb_hz[i] = main_hz[i];
                            }
                        }
                    }
                }
            }
        }

        // Black level subtraction
        for i in 0..3 {
            rgb_hz[i] -= self.cal.black[i];
            if rgb_hz[i] < 0.0 {
                rgb_hz[i] = 0.0;
            }
        }

        // Diffuser check
        let diffuser = self.get_diffuser_position()?;
        let is_ambient = diffuser != 0;

        // Matrix multiply → XYZ
        let xyz = if is_ambient {
            mat3_mul_vec(&self.cal.ambi_cal, &rgb_hz)
        } else {
            let xyz = mat3_mul_vec(&self.cal.emis_cal, &rgb_hz);
            mat3_mul_vec(&self.cal.ccmat, &xyz)
        };

        let elapsed = t0.elapsed();

        Ok(XyzReading {
            x: xyz[0],
            y: xyz[1],
            z: xyz[2],
            rgb_hz,
            elapsed_ms: elapsed.as_millis() as u32,
        })
    }

    /// Get sensor identification info.
    pub fn info(&self) -> SensorInfo {
        SensorInfo {
            model: "i1Display3".to_string(),
            serial: self.cal.serial_no.clone(),
            hw_revision: format!("{}", (b'A' + self.cal.hw_revision) as char),
            firmware: self.cal.version_no.clone(),
            unlock_variant: self.unlock_variant.clone(),
        }
    }

    /// Borrow the calibration data read from device EEPROM.
    ///
    /// # Returns
    ///
    /// Reference to the calibration data containing spectral sensitivities,
    /// ambient filter, calibration matrices, and black levels.
    pub fn calibration_data(&self) -> &CalibrationData {
        &self.cal
    }

    /// Borrow the interpolated CIE 1931 2-degree observer at 1 nm resolution.
    ///
    /// # Returns
    ///
    /// Tuple of (cie_x, cie_y, cie_z) arrays, each 351 points (380-730 nm).
    pub fn cie_observer(&self) -> (&[f64; SPEC_N], &[f64; SPEC_N], &[f64; SPEC_N]) {
        (&self.cie_x, &self.cie_y, &self.cie_z)
    }

    /// Blink the LED.
    pub fn led_blink(&self, count: u8) -> Result<()> {
        let mul2 = CLOCK_FREQ / (1u64 << 19) as f64;
        let on_time = (0.5 + 0.3 * mul2).min(255.0) as u8;
        let off_time = (0.5 + 0.3 * mul2).min(255.0) as u8;
        let payload = [1u8, off_time, on_time, count.min(0x80)];
        self.command(Cmd::SetLed, &payload, DEFAULT_TIMEOUT_MS)?;
        Ok(())
    }
}

impl Drop for I1Display3 {
    fn drop(&mut self) {
        // Try to relock before closing
        let _ = self.command(Cmd::Relock, &[], DEFAULT_TIMEOUT_MS);
    }
}
