//! Serial transport crate for the BNO086_ROS2Board custom board (BNO086 + STM32F042C6).
//!
//! Board firmware streams Rotation Vector, Gyroscope, and Accelerometer reports at 400 Hz
//! in `COBS(postcard ‖ CRC16) ‖ 0x00` frames. The host only needs to read; no report
//! configuration command is required. This crate receives those frames:
//!
//! - [`spawn`] starts a reader thread, opens a serial port (USB CDC or UART), decodes frames,
//!   and retries after disconnection.
//! - Received samples are kept latest-wins in the original board frame and are available
//!   through the synchronous [`Bno086Handle::latest`] API. Conversion to a robot body frame
//!   and freshness checks belong to the consuming application.
//!
//! This is a transport layer intended to sit below a HAL; it has no humanoid-system or
//! reiny dependencies.

pub mod frame;
pub mod protocol;

extern crate alloc;

use alloc::sync::Arc;
use core::sync::atomic::{AtomicU64, Ordering};
use core::time::Duration;
use std::io::{Read, Write};
use std::sync::Mutex;
use std::time::Instant;

use tracing::{debug, info, warn};

pub use protocol::{DeviceInfo, ImuSample, PROTOCOL_VERSION, USB_PID, USB_VID};

/// Selects automatic port discovery by VID/PID ([`USB_VID`]/[`USB_PID`]) when used as `port`.
pub const PORT_AUTO: &str = "auto";

/// Reader thread configuration.
#[derive(Debug, Clone)]
pub struct Bno086Config {
    /// Serial port name (`"COM7"` or `"/dev/ttyACM0"`); [`PORT_AUTO`] enables discovery.
    pub port: String,
    /// Baud rate. Ignored by USB CDC; UART1 firmware defaults to 921600.
    pub baud: u32,
}

impl Default for Bno086Config {
    fn default() -> Self {
        Self {
            port: PORT_AUTO.to_string(),
            baud: 921_600,
        }
    }
}

/// One sample with its receive time and connection generation.
///
/// `connection_id` increments each time the port is reopened. Consumers can detect a new
/// connection generation and reset their orientation reference.
#[derive(Debug, Clone, Copy)]
pub struct StampedSample {
    /// Time when the frame was decoded; freshness checks use this timestamp.
    pub received: Instant,
    /// Generation number of the connection that sent this sample (starting at 1).
    pub connection_id: u64,
    /// The decoded sample, still in board-frame units.
    pub sample: ImuSample,
}

#[derive(Debug)]
struct Shared {
    latest: Mutex<Option<StampedSample>>,
    /// Generation number of the active connection (zero means disconnected).
    active_conn: AtomicU64,
    next_conn: AtomicU64,
    samples: AtomicU64,
    /// Cumulative number of frames with CRC or decoding errors.
    frame_errors: AtomicU64,
    /// Maximum sample interval within one connection, in microseconds.
    max_gap_us: AtomicU64,
}

/// Synchronous handle for observing reader thread state.
#[derive(Debug, Clone)]
pub struct Bno086Handle {
    shared: Arc<Shared>,
}

impl Bno086Handle {
    /// Latest sample with receive time and connection generation, or `None` before reception.
    pub fn latest(&self) -> Option<StampedSample> {
        *self.shared.latest.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Whether the port is open and samples are arriving.
    pub fn connected(&self) -> bool {
        self.shared.active_conn.load(Ordering::Relaxed) != 0
    }

    /// Cumulative number of received samples.
    pub fn sample_count(&self) -> u64 {
        self.shared.samples.load(Ordering::Relaxed)
    }

    /// Cumulative number of frame errors (CRC or decoding failures).
    pub fn frame_errors(&self) -> u64 {
        self.shared.frame_errors.load(Ordering::Relaxed)
    }

    /// Returns the **maximum observed sample interval** since the previous call and resets it.
    ///
    /// The reader timestamps every sample, so the maximum is independent of the caller's polling
    /// interval. Configure any consumer freshness timeout above this value. Intervals spanning
    /// reconnects are excluded because reconnect downtime is handled as a disconnection.
    pub fn take_max_gap(&self) -> Duration {
        Duration::from_micros(self.shared.max_gap_us.swap(0, Ordering::Relaxed))
    }
}

/// Starts the reader thread and returns a handle.
///
/// Failure to open the port is not returned as an error; the reader retries every
/// [`RECONNECT_INTERVAL`] to handle board removal, reinsertion, and USB port re-enumeration.
/// Inspect connection state with [`Bno086Handle::connected`].
///
/// The thread runs until process exit. There is no stop API; process shutdown lets the OS
/// release the serial port.
pub fn spawn(cfg: Bno086Config) -> Bno086Handle {
    let shared = Arc::new(Shared {
        latest: Mutex::new(None),
        active_conn: AtomicU64::new(0),
        next_conn: AtomicU64::new(0),
        samples: AtomicU64::new(0),
        frame_errors: AtomicU64::new(0),
        max_gap_us: AtomicU64::new(0),
    });
    let worker = Arc::clone(&shared);
    let desc = format!("port={}, baud={}", cfg.port, cfg.baud);
    match std::thread::Builder::new()
        .name("bno086-serial".to_string())
        .spawn(move || reader_loop(cfg, worker))
    {
        Ok(_) => info!("BNO086 reader thread started ({desc})"),
        Err(e) => warn!("failed to spawn BNO086 reader thread: {e}"),
    }
    Bno086Handle { shared }
}

/// Delay between reconnection attempts.
pub const RECONNECT_INTERVAL: Duration = Duration::from_millis(500);

/// Consecutive `Ok(0)` reads treated as a disconnect, preventing a spin in [`serve_port`].
const MAX_ZERO_READS: u32 = 16;

fn reader_loop(cfg: Bno086Config, shared: Arc<Shared>) {
    // Avoid logging every 500 ms while unavailable; report only state transitions.
    let mut reported_failure = false;
    loop {
        match open_port(&cfg) {
            Ok((name, port)) => {
                reported_failure = false;
                let conn = shared.next_conn.fetch_add(1, Ordering::Relaxed) + 1;
                info!("BNO086 connected on {name} (connection #{conn})");
                shared.active_conn.store(conn, Ordering::Relaxed);
                let reason = serve_port(port, conn, &shared);
                shared.active_conn.store(0, Ordering::Relaxed);
                warn!("BNO086 disconnected from {name}: {reason}");
            }
            Err(e) => {
                if !reported_failure {
                    warn!("BNO086 port unavailable ({e}); retrying every {RECONNECT_INTERVAL:?}");
                    reported_failure = true;
                }
            }
        }
        std::thread::sleep(RECONNECT_INTERVAL);
    }
}

/// Reads one connection until it ends and returns the disconnection reason.
// `core::io` is unstable on stable Rust, so `ErrorKind` has to come from `std` here.
#[allow(clippy::std_instead_of_core)]
///
/// Takes `Read + Write` rather than the boxed `SerialPort` it is handed in production:
/// the loop never touches a serial-port setting, and the narrower bound is what lets the
/// tests drive the frame accumulator, the zero-read guard and the timeout branch without
/// a physical board.
fn serve_port(mut port: impl Read + Write, conn: u64, shared: &Shared) -> String {
    // Request the protocol version immediately after connecting; compare it in the Info reply.
    if let Err(e) = write_msg(&mut port, &protocol::HostToDevice::GetInfo) {
        return format!("failed to send GetInfo: {e}");
    }

    let mut acc = frame::FrameAccumulator::new();
    let mut buf = [0u8; 1024];
    // No data arrives as a read timeout (20 ms), so Ok(0) is an EOF-like error. Continuing
    // indefinitely would spin at 100% CPU and prevent reconnect handling. Allow a few zero reads
    // before treating the connection as gone.
    let mut zero_reads = 0u32;
    loop {
        let n = match port.read(&mut buf) {
            Ok(0) => {
                zero_reads += 1;
                if zero_reads >= MAX_ZERO_READS {
                    return format!("read returned 0 bytes {MAX_ZERO_READS} times (port gone?)");
                }
                continue;
            }
            Ok(n) => n,
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(e) => return format!("read failed: {e}"),
        };
        zero_reads = 0;
        for &b in &buf[..n] {
            let Some(raw) = acc.push(b) else { continue };
            match frame::decode_frame::<protocol::DeviceToHost>(raw) {
                Ok(msg) => handle_msg(msg, conn, shared),
                Err(e) => {
                    let _ = shared.frame_errors.fetch_add(1, Ordering::Relaxed);
                    debug!("BNO086 frame error: {e}");
                }
            }
            acc.clear();
        }
    }
}

fn handle_msg(msg: protocol::DeviceToHost, conn: u64, shared: &Shared) {
    match msg {
        protocol::DeviceToHost::Sample(sample) => {
            let received = Instant::now();
            // Latest-wins: do not queue 400 Hz samples when control reads at 100 Hz.
            let mut slot = shared.latest.lock().unwrap_or_else(|e| e.into_inner());
            if let Some(prev) = slot.filter(|p| p.connection_id == conn) {
                let gap = received.duration_since(prev.received).as_micros() as u64;
                let _ = shared.max_gap_us.fetch_max(gap, Ordering::Relaxed);
            }
            *slot = Some(StampedSample {
                received,
                connection_id: conn,
                sample,
            });
            drop(slot);
            let _ = shared.samples.fetch_add(1, Ordering::Relaxed);
        }
        protocol::DeviceToHost::Info(info) => {
            if info.proto == PROTOCOL_VERSION {
                info!(
                    "BNO086 firmware v{}.{}.{} (proto {}, link {:?})",
                    info.fw[0], info.fw[1], info.fw[2], info.proto, info.mode
                );
            } else {
                // Detect protocol drift because these protocol definitions are vendored.
                warn!(
                    "BNO086 protocol mismatch: device speaks {} but this build expects {}; \
                     re-vendor crates/drivers/bno086-serial/src/protocol.rs from upstream",
                    info.proto, PROTOCOL_VERSION
                );
            }
        }
        // Status, Nack, and Pong are decoded and ignored; expose them if diagnosing link quality.
        other => debug!("BNO086 message ignored: {other:?}"),
    }
}

fn write_msg(port: &mut dyn Write, msg: &protocol::HostToDevice) -> Result<(), String> {
    let frame = frame::encode_frame(msg).map_err(|e| e.to_string())?;
    port.write_all(&frame).map_err(|e| e.to_string())?;
    let _ = port.flush();
    Ok(())
}

fn open_port(cfg: &Bno086Config) -> Result<(String, Box<dyn serialport::SerialPort>), String> {
    let name = if cfg.port == PORT_AUTO {
        auto_detect_port()?
    } else {
        cfg.port.clone()
    };
    let port = serialport::new(&name, cfg.baud)
        .timeout(Duration::from_millis(20))
        .open()
        .map_err(|e| format!("cannot open {name}: {e}"))?;
    Ok((name, port))
}

/// Finds a port whose VID/PID matches the board.
fn auto_detect_port() -> Result<String, String> {
    let ports = serialport::available_ports().map_err(|e| format!("cannot list ports: {e}"))?;
    ports
        .into_iter()
        .find(|p| {
            matches!(&p.port_type,
                serialport::SerialPortType::UsbPort(u) if u.vid == USB_VID && u.pid == USB_PID)
        })
        .map(|p| p.port_name)
        .ok_or_else(|| format!("no USB port with VID:PID {USB_VID:04x}:{USB_PID:04x}"))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, unused_results)]
mod tests {
    use super::*;
    use crate::protocol::{AccuracyFlags, DeviceInfo, DeviceToHost, ImuSample, LinkMode};
    use std::io;

    fn shared() -> Arc<Shared> {
        Arc::new(Shared {
            latest: Mutex::new(None),
            active_conn: AtomicU64::new(0),
            next_conn: AtomicU64::new(0),
            samples: AtomicU64::new(0),
            frame_errors: AtomicU64::new(0),
            max_gap_us: AtomicU64::new(0),
        })
    }

    fn sample(t_us: u64) -> ImuSample {
        ImuSample {
            t_us,
            quat: Some([0.0, 0.0, 0.0, 1.0]),
            gyro: Some([0.0; 3]),
            accel: Some([0.0, 0.0, 9.81]),
            lin_accel: None,
            mag: None,
            acc_status: AccuracyFlags {
                quat: 0,
                gyro: 3,
                accel: 3,
                mag: 0,
            },
        }
    }

    /// A scripted `Read + Write` stand-in for the board's serial port. Each entry is one
    /// `read` result, so a test can hand the reader exactly the sequence it wants —
    /// including the `Ok(0)` and `TimedOut` shapes `serve_port` has to tell apart.
    struct ScriptedPort {
        script: alloc::collections::VecDeque<io::Result<Vec<u8>>>,
        written: Vec<u8>,
    }

    impl ScriptedPort {
        fn new(script: Vec<io::Result<Vec<u8>>>) -> Self {
            Self {
                script: script.into_iter().collect(),
                written: Vec::new(),
            }
        }
    }

    impl Read for ScriptedPort {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            match self.script.pop_front() {
                Some(Ok(bytes)) => {
                    let n = bytes.len().min(buf.len());
                    buf[..n].copy_from_slice(&bytes[..n]);
                    Ok(n)
                }
                Some(Err(e)) => Err(e),
                // Running off the end of the script ends the connection.
                None => Err(io::Error::other("script exhausted")),
            }
        }
    }

    impl Write for ScriptedPort {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.written.extend_from_slice(buf);
            Ok(buf.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    fn timed_out() -> io::Error {
        io::Error::new(io::ErrorKind::TimedOut, "no data")
    }

    #[test]
    fn config_defaults_to_auto_detect_at_the_firmwares_uart_baud() {
        let cfg = Bno086Config::default();
        assert_eq!(cfg.port, PORT_AUTO);
        assert_eq!(cfg.baud, 921_600);
    }

    #[test]
    fn a_fresh_handle_reports_nothing_received_and_no_connection() {
        let h = Bno086Handle { shared: shared() };
        assert!(h.latest().is_none());
        assert!(!h.connected());
        assert_eq!(h.sample_count(), 0);
        assert_eq!(h.frame_errors(), 0);
        assert_eq!(h.take_max_gap(), Duration::ZERO);
    }

    #[test]
    fn a_sample_is_stored_latest_wins_and_counted() {
        let s = shared();
        handle_msg(DeviceToHost::Sample(sample(1)), 1, &s);
        handle_msg(DeviceToHost::Sample(sample(2)), 1, &s);
        let h = Bno086Handle {
            shared: Arc::clone(&s),
        };
        assert_eq!(h.sample_count(), 2);
        let latest = h.latest().unwrap();
        assert_eq!(latest.sample.t_us, 2, "400Hz samples must not queue up");
        assert_eq!(latest.connection_id, 1);
    }

    #[test]
    fn the_gap_measurement_resets_when_it_is_read() {
        let s = shared();
        handle_msg(DeviceToHost::Sample(sample(1)), 1, &s);
        handle_msg(DeviceToHost::Sample(sample(2)), 1, &s);
        let h = Bno086Handle {
            shared: Arc::clone(&s),
        };
        // Two samples in the same connection produce one measurable gap…
        let _first = h.take_max_gap();
        // …and reading it clears the running maximum, so the next window starts fresh.
        assert_eq!(h.take_max_gap(), Duration::ZERO);
    }

    #[test]
    fn a_gap_is_never_measured_across_a_reconnect() {
        // The blank between connections is a disconnect, not a dropped sample; folding it
        // into max_gap would make `stale_timeout_ms` look far too tight.
        let s = shared();
        handle_msg(DeviceToHost::Sample(sample(1)), 1, &s);
        handle_msg(DeviceToHost::Sample(sample(2)), 2, &s); // new connection
        let h = Bno086Handle {
            shared: Arc::clone(&s),
        };
        assert_eq!(h.take_max_gap(), Duration::ZERO);
        assert_eq!(h.latest().unwrap().connection_id, 2);
    }

    #[test]
    fn device_info_is_accepted_on_a_match_and_warned_about_on_a_mismatch() {
        // Both branches only log; what matters is that neither is treated as a sample.
        let s = shared();
        let info = |proto| {
            DeviceToHost::Info(DeviceInfo {
                proto,
                fw: [0, 1, 0],
                mode: LinkMode::Usb,
                uid: [0u8; 12],
            })
        };
        handle_msg(info(PROTOCOL_VERSION), 1, &s);
        handle_msg(info(PROTOCOL_VERSION + 1), 1, &s);
        assert_eq!(s.samples.load(Ordering::Relaxed), 0);
        assert!(s.latest.lock().unwrap().is_none());
    }

    #[test]
    fn serve_port_asks_for_the_device_info_before_reading() {
        let s = shared();
        let mut port = ScriptedPort::new(vec![Err(io::Error::other("gone"))]);
        let _ = serve_port(&mut port, 1, &s);
        let expected = frame::encode_frame(&protocol::HostToDevice::GetInfo).unwrap();
        assert_eq!(port.written, expected);
    }

    #[test]
    fn serve_port_decodes_framed_samples_out_of_a_split_byte_stream() {
        let s = shared();
        let frame = frame::encode_frame(&DeviceToHost::Sample(sample(42))).unwrap();
        // Split the frame across two reads — the accumulator has to bridge them.
        let (head, tail) = frame.split_at(frame.len() / 2);
        let port = ScriptedPort::new(vec![
            Ok(head.to_vec()),
            Ok(tail.to_vec()),
            Err(io::Error::other("gone")),
        ]);
        let reason = serve_port(port, 7, &s);
        assert_eq!(s.samples.load(Ordering::Relaxed), 1);
        assert_eq!(s.latest.lock().unwrap().unwrap().sample.t_us, 42);
        assert_eq!(s.latest.lock().unwrap().unwrap().connection_id, 7);
        assert!(reason.contains("read failed"), "{reason}");
    }

    #[test]
    fn a_corrupt_frame_is_counted_and_does_not_stop_the_connection() {
        let s = shared();
        let mut frame = frame::encode_frame(&DeviceToHost::Sample(sample(1))).unwrap();
        // Flip a payload byte so the CRC no longer matches.
        frame[1] ^= 0xFF;
        let good = frame::encode_frame(&DeviceToHost::Sample(sample(2))).unwrap();
        let port = ScriptedPort::new(vec![Ok(frame), Ok(good), Err(io::Error::other("gone"))]);
        let _ = serve_port(port, 1, &s);
        assert_eq!(s.frame_errors.load(Ordering::Relaxed), 1);
        assert_eq!(
            s.samples.load(Ordering::Relaxed),
            1,
            "the good frame still lands"
        );
    }

    #[test]
    fn a_read_timeout_is_idleness_not_a_disconnect() {
        let s = shared();
        let frame = frame::encode_frame(&DeviceToHost::Sample(sample(9))).unwrap();
        let port = ScriptedPort::new(vec![
            Err(timed_out()),
            Err(timed_out()),
            Ok(frame),
            Err(io::Error::other("gone")),
        ]);
        let _ = serve_port(port, 1, &s);
        assert_eq!(s.samples.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn a_run_of_zero_length_reads_is_treated_as_the_port_disappearing() {
        // Without this the loop spins at 100% CPU and `connected()` keeps lying, because
        // the reconnect branch is never reached.
        let s = shared();
        let script = (0..MAX_ZERO_READS).map(|_| Ok(Vec::new())).collect();
        let reason = serve_port(ScriptedPort::new(script), 1, &s);
        assert!(reason.contains("read returned 0 bytes"), "{reason}");
    }

    #[test]
    fn a_single_zero_read_does_not_end_the_connection() {
        let s = shared();
        let frame = frame::encode_frame(&DeviceToHost::Sample(sample(5))).unwrap();
        let port = ScriptedPort::new(vec![
            Ok(Vec::new()),
            Ok(frame),
            Err(io::Error::other("gone")),
        ]);
        let _ = serve_port(port, 1, &s);
        assert_eq!(s.samples.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn a_port_that_cannot_be_written_to_is_reported_before_any_read() {
        struct DeadWriter;
        impl Read for DeadWriter {
            fn read(&mut self, _: &mut [u8]) -> io::Result<usize> {
                unreachable!("serve_port must give up on the failed write first")
            }
        }
        impl Write for DeadWriter {
            fn write(&mut self, _: &[u8]) -> io::Result<usize> {
                Err(io::Error::other("cable pulled"))
            }
            fn flush(&mut self) -> io::Result<()> {
                Ok(())
            }
        }
        let s = shared();
        let reason = serve_port(DeadWriter, 1, &s);
        assert!(reason.contains("failed to send GetInfo"), "{reason}");
    }
}
