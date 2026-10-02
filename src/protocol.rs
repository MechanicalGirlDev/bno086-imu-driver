//! Vendored host/device protocol shared with BNO086_ROS2Board.
//!
//! Upstream: `MechanicalGirlDev/BNO086_ROS2Board`'s `software/crates/protocol/src/lib.rs`
//! rev `d940d6a62f7675e1b42c772add39709df04915e9`（2026-07-22）。
//!
//! **Why vendored:** Cargo git dependencies require a repository-root `Cargo.toml`, while
//! upstream's workspace is under `software/crates/` and Cargo cannot select a subdirectory.
//! This is temporary until upstream is published to crates.io.
//!
//! **Drift detection:** On connection, the driver sends `GetInfo` and compares the returned
//! [`DeviceInfo::proto`] with [`PROTOCOL_VERSION`]. A wire-format change that increments the
//! upstream version is therefore detected even though these definitions are copied.
//!
//! Two upstream elements are intentionally omitted:
//! - The `no_std`-specific `euler()` and math helpers; consumers can perform orientation
//!   calculations with their chosen math library.
//! - The `can` module; this driver implements serial links only.
//!
//! Type definitions, especially enum variant order and struct field order, determine postcard
//! wire encoding. Preserve their upstream order exactly.

use serde::{Deserialize, Serialize};

/// Protocol version. Increment this for every breaking compatibility change.
pub const PROTOCOL_VERSION: u16 = 1;

/// USB vendor ID used for automatic port discovery.
pub const USB_VID: u16 = 0x1209;
/// USB PID of the board (see [`USB_VID`]).
pub const USB_PID: u16 = 0x0001;

/// Maximum frame length, including COBS and CRC, matching firmware buffer capacity.
pub const MAX_FRAME_LEN: usize = 128;

/// Host-to-device messages.
// The widest variant is 8 bytes; boxing it would cost an allocation to save nothing.
#[allow(variant_size_differences)]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum HostToDevice {
    /// Liveness probe; the board answers [`DeviceToHost::Pong`].
    Ping,
    /// Asks for [`DeviceInfo`].
    GetInfo,
    /// Set a report interval in microseconds; zero stops the report.
    SetReportRate {
        /// Which sensor report to (re)configure.
        report: ReportKind,
        /// Reporting period in microseconds; 0 stops the report.
        interval_us: u32,
    },
    /// Axes: bit 0 = X, bit 1 = Y, bit 2 = Z. Optionally persist the tare.
    Tare {
        /// Bit mask of the axes to tare.
        axes: u8,
        /// Whether to store the tare in non-volatile memory.
        persist: bool,
    },
    /// Stores the current sensor calibration in non-volatile memory.
    SaveCalibration,
    /// Clears the stored sensor calibration.
    ClearCalibration,
    /// Resets the BNO086 itself, leaving the MCU running.
    ResetImu,
    /// Reboots the STM32.
    RebootMcu,
    /// Clears the tare (reorientation); `ClearCalibration` does not clear it.
    ///
    /// Postcard encodes the variant index; always append new variants at the end.
    ClearTare,
}

/// Device-to-host messages.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum DeviceToHost {
    /// Answer to [`HostToDevice::Ping`].
    Pong,
    /// Answer to [`HostToDevice::GetInfo`].
    Info(DeviceInfo),
    /// One fused sensor sample.
    Sample(ImuSample),
    /// 1Hz
    Status(DeviceStatus),
    /// The board refused the last command.
    Nack {
        /// Why the command was refused.
        reason: NackReason,
    },
}

/// Board identity, answered to [`HostToDevice::GetInfo`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DeviceInfo {
    /// The board's [`PROTOCOL_VERSION`]; the driver warns when it differs from ours.
    pub proto: u16,
    /// Firmware version as `[major, minor, patch]`.
    pub fw: [u8; 3],
    /// Which physical link the board is currently talking over.
    pub mode: LinkMode,
    /// STM32 96bit UID
    pub uid: [u8; 12],
}

/// Which physical link the board is talking over.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LinkMode {
    /// USB CDC (the usual bring-up link).
    Usb,
    /// Plain UART.
    Uart,
    /// CAN.
    Can,
}

/// Why the board refused a command.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NackReason {
    /// Decode failure or unknown command.
    BadRequest,
    /// Command could not run, for example because the IMU did not respond.
    ImuError,
    /// Command is not available in the current mode.
    WrongMode,
}

/// Sensor report types that can be enabled.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReportKind {
    /// Fused orientation quaternion (SH-2 `SH2_ROTATION_VECTOR`).
    RotationVector,
    /// Angular rate.
    Gyro,
    /// Acceleration including gravity.
    Accel,
    /// Acceleration with gravity removed.
    LinAccel,
    /// Magnetic field.
    Mag,
}

/// Per-sensor accuracy status from SH-2 (0..=3, with 3 as the highest).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct AccuracyFlags {
    /// Accuracy of the rotation vector. Stays 0 while the magnetometer is uncalibrated,
    /// which is the steady state indoors and does not mean the attitude is bad.
    pub quat: u8,
    /// Accuracy of the gyro report.
    pub gyro: u8,
    /// Accuracy of the accelerometer report.
    pub accel: u8,
    /// Accuracy of the magnetometer report.
    pub mag: u8,
}

/// Sample combining reports with nearby timestamps in firmware.
///
/// All values are in the **board frame**; firmware already corrects the U4's 180-degree mounting.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct ImuSample {
    /// Microseconds elapsed since boot.
    pub t_us: u64,
    /// w, x, y, z（Rotation Vector）
    pub quat: Option<[f32; 4]>,
    /// rad/s
    pub gyro: Option<[f32; 3]>,
    /// m/s^2
    pub accel: Option<[f32; 3]>,
    /// m/s^2
    pub lin_accel: Option<[f32; 3]>,
    /// µT
    pub mag: Option<[f32; 3]>,
    /// Per-sensor accuracy reported alongside this sample.
    pub acc_status: AccuracyFlags,
}

/// Statistics sent at 1 Hz.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct DeviceStatus {
    /// Milliseconds since the board booted.
    pub uptime_ms: u32,
    /// How often the FW had to reset the BNO086.
    pub imu_resets: u16,
    /// SPI errors seen on the MCU↔BNO086 link.
    pub spi_errors: u16,
    /// Samples dropped because the receiver was too slow.
    pub tx_dropped: u16,
}
