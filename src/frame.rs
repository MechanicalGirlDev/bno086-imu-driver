//! Framing layer: `COBS(payload ‖ CRC16-CCITT(payload)) ‖ 0x00` (vendored).
//!
//! Upstream: `MechanicalGirlDev/BNO086_ROS2Board`'s `software/crates/protocol/src/frame.rs`
//! at revision `d940d6a62f7675e1b42c772add39709df04915e9`. See [`crate::protocol`] for details.
//!
//! The only upstream adaptation is the container type (`heapless::Vec<u8, MAX_FRAME_LEN>` to
//! `Vec<u8>`), avoiding a host-side `heapless` dependency. Frame-length checks are retained.
//! The COBS and CRC algorithms are unchanged; they define wire compatibility.

use crc::{CRC_16_IBM_SDLC, Crc};
use serde::{Serialize, de::DeserializeOwned};

use crate::protocol::MAX_FRAME_LEN;

/// CRC-16/IBM-SDLC (a CCITT variant using the reflected 0x1021 polynomial).
pub const CRC16: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_SDLC);

/// Why a frame could not be encoded or decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum FrameError {
    /// Insufficient buffer space or serialization failure.
    #[error("frame encode failed (payload too long or serialize error)")]
    Encode,
    /// COBS decoding failed.
    #[error("COBS decode failed")]
    Cobs,
    /// CRC mismatch.
    #[error("CRC mismatch")]
    Crc,
    /// Postcard decoding failed.
    #[error("postcard decode failed")]
    Decode,
}

/// Encodes a message as one frame, including the trailing `0x00`.
pub fn encode_frame<T: Serialize>(msg: &T) -> Result<Vec<u8>, FrameError> {
    // 1) Encode the postcard payload; 2) append CRC16 in little-endian order.
    let mut buf = [0u8; MAX_FRAME_LEN];
    let payload_len = postcard::to_slice(msg, &mut buf)
        .map_err(|_| FrameError::Encode)?
        .len();
    if payload_len + 2 > MAX_FRAME_LEN {
        return Err(FrameError::Encode);
    }
    let crc = CRC16.checksum(&buf[..payload_len]);
    buf[payload_len..payload_len + 2].copy_from_slice(&crc.to_le_bytes());

    // 3) Apply COBS and append the `0x00` terminator.
    let mut out = Vec::new();
    cobs_encode(&buf[..payload_len + 2], &mut out);
    // Upstream rejects failed pushes into heapless::Vec; the terminator must fit too.
    if out.len() + 1 > MAX_FRAME_LEN {
        return Err(FrameError::Encode);
    }
    out.push(0);
    Ok(out)
}

/// Decodes one frame without its `0x00` terminator; a present terminator is ignored.
pub fn decode_frame<T: DeserializeOwned>(frame: &[u8]) -> Result<T, FrameError> {
    let frame = match frame.split_last() {
        Some((0, rest)) => rest,
        _ => frame,
    };
    let mut buf = [0u8; MAX_FRAME_LEN];
    let len = cobs_decode(frame, &mut buf)?;
    if len < 2 {
        return Err(FrameError::Crc);
    }
    let (payload, crc_bytes) = buf[..len].split_at(len - 2);
    let expect = u16::from_le_bytes([crc_bytes[0], crc_bytes[1]]);
    if CRC16.checksum(payload) != expect {
        return Err(FrameError::Crc);
    }
    postcard::from_bytes(payload).map_err(|_| FrameError::Decode)
}

/// Accumulates frames delimited by `0x00` from a receive stream.
#[derive(Debug, Default)]
pub struct FrameAccumulator {
    buf: Vec<u8>,
    overflowed: bool,
}

impl FrameAccumulator {
    /// Creates an empty accumulator.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            buf: Vec::new(),
            overflowed: false,
        }
    }

    /// Feeds one byte and returns the COBS slice when a frame is complete.
    /// The returned slice is valid until the next push. Call [`Self::clear`] after processing.
    pub fn push(&mut self, byte: u8) -> Option<&[u8]> {
        if byte == 0 {
            let complete = !self.overflowed && !self.buf.is_empty();
            self.overflowed = false;
            if complete {
                return Some(&self.buf);
            }
            self.buf.clear();
            return None;
        }
        if self.overflowed {
            return None;
        }
        if self.buf.len() >= MAX_FRAME_LEN {
            // Frame too long; discard input until the next 0x00.
            self.buf.clear();
            self.overflowed = true;
            return None;
        }
        self.buf.push(byte);
        None
    }

    /// Clears a completed frame after processing.
    pub fn clear(&mut self) {
        self.buf.clear();
    }
}

// --- COBS implementation: upstream algorithm retained, container adapted to Vec. ---

fn cobs_encode(src: &[u8], dst: &mut Vec<u8>) {
    let mut code_idx = dst.len();
    dst.push(0);
    let mut code: u8 = 1;
    for &b in src {
        if b == 0 {
            dst[code_idx] = code;
            code_idx = dst.len();
            dst.push(0);
            code = 1;
        } else {
            dst.push(b);
            code += 1;
            if code == 0xFF {
                dst[code_idx] = code;
                code_idx = dst.len();
                dst.push(0);
                code = 1;
            }
        }
    }
    dst[code_idx] = code;
}

fn cobs_decode(src: &[u8], dst: &mut [u8]) -> Result<usize, FrameError> {
    let mut out = 0usize;
    let mut i = 0usize;
    while i < src.len() {
        let code = src[i] as usize;
        if code == 0 {
            return Err(FrameError::Cobs);
        }
        i += 1;
        for _ in 0..code - 1 {
            if i >= src.len() || out >= dst.len() {
                return Err(FrameError::Cobs);
            }
            dst[out] = src[i];
            out += 1;
            i += 1;
        }
        if code != 0xFF && i < src.len() {
            if out >= dst.len() {
                return Err(FrameError::Cobs);
            }
            dst[out] = 0;
            out += 1;
        }
    }
    Ok(out)
}

#[cfg(test)]
#[allow(clippy::expect_used)]
mod tests {
    use super::*;
    use crate::protocol::*;

    fn roundtrip<T: Serialize + DeserializeOwned + PartialEq + core::fmt::Debug>(msg: T) {
        let frame = encode_frame(&msg).expect("encode");
        // The only 0x00 in a frame is its terminator.
        assert_eq!(frame.iter().filter(|&&b| b == 0).count(), 1);
        assert_eq!(*frame.last().expect("non-empty"), 0);
        let decoded: T = decode_frame(&frame).expect("decode");
        assert_eq!(decoded, msg);
    }

    #[test]
    fn roundtrip_host_to_device() {
        roundtrip(HostToDevice::Ping);
        roundtrip(HostToDevice::GetInfo);
        roundtrip(HostToDevice::ClearTare);
    }

    #[test]
    fn roundtrip_device_to_host() {
        roundtrip(DeviceToHost::Info(DeviceInfo {
            proto: PROTOCOL_VERSION,
            fw: [0, 1, 0],
            mode: LinkMode::Usb,
            uid: [0xAA; 12],
        }));
        roundtrip(DeviceToHost::Sample(ImuSample {
            t_us: u64::MAX / 3,
            quat: Some([1.0, 0.0, 0.0, 0.0]),
            gyro: Some([0.1, -0.2, 0.3]),
            accel: None,
            lin_accel: Some([0.0, 0.0, 9.81]),
            mag: None,
            acc_status: AccuracyFlags {
                quat: 3,
                gyro: 2,
                accel: 1,
                mag: 0,
            },
        }));
        roundtrip(DeviceToHost::Nack {
            reason: NackReason::WrongMode,
        });
    }

    /// Wire-compatibility anchor: output must match upstream revision d940d6a.
    /// This fails if COBS, CRC, or postcard encoding drifts.
    #[test]
    fn wire_bytes_match_upstream() {
        // HostToDevice::Ping = postcard variant 0 -> payload [0x00]
        //   CRC-16/IBM-SDLC([0x00]) = 0xF078 -> LE [0x78, 0xF0]
        //   COBS([0x00, 0x78, 0xF0]) = [0x01, 0x03, 0x78, 0xF0] -> trailing 0x00
        assert_eq!(
            encode_frame(&HostToDevice::Ping).expect("encode"),
            vec![0x01, 0x03, 0x78, 0xF0, 0x00]
        );
    }

    #[test]
    fn crc_detects_corruption() {
        let mut frame = encode_frame(&HostToDevice::Ping).expect("encode");
        let idx = frame.len() - 2;
        frame[idx] ^= 0x5A;
        let r: Result<HostToDevice, _> = decode_frame(&frame);
        assert!(r.is_err());
    }

    #[test]
    fn accumulator_splits_frames() {
        let f1 = encode_frame(&HostToDevice::Ping).expect("encode");
        let f2 = encode_frame(&HostToDevice::GetInfo).expect("encode");
        let mut acc = FrameAccumulator::new();
        let mut decoded: Vec<HostToDevice> = Vec::new();
        for &b in f1.iter().chain(f2.iter()) {
            if let Some(frame) = acc.push(b) {
                decoded.push(decode_frame(frame).expect("decode"));
                acc.clear();
            }
        }
        assert_eq!(decoded, vec![HostToDevice::Ping, HostToDevice::GetInfo]);
    }

    #[test]
    fn accumulator_recovers_from_garbage() {
        let mut acc = FrameAccumulator::new();
        // Garbage (an overlong frame) -> 0x00 -> a valid frame.
        for _ in 0..(MAX_FRAME_LEN + 10) {
            assert!(acc.push(0xFF).is_none());
        }
        assert!(acc.push(0).is_none());
        let f = encode_frame(&HostToDevice::Ping).expect("encode");
        let mut got = None;
        for &b in f.iter() {
            if let Some(frame) = acc.push(b) {
                got = Some(decode_frame::<HostToDevice>(frame).expect("decode"));
            }
        }
        assert_eq!(got, Some(HostToDevice::Ping));
    }
}
