//! Standalone bring-up tool for checking the BNO086_ROS2Board wiring and coordinate frame.
//!
//! ```sh
//! cargo run -p bno086-serial --bin bno086            # Discover by VID/PID
//! cargo run -p bno086-serial --bin bno086 -- --port COM7
//! ```
//!
//! When the board is level and stationary, `accel ≈ [0, 0, +9.81]` means acceleration polarity
//! follows the phone IMU convention (gravity reaction is +Z). A `-Z` result indicates that the
//! consuming application may need a fixed rotation.

use core::time::Duration;

use anyhow::Result;
use clap::Parser;

#[derive(Parser, Debug)]
#[command(name = "bno086", about = "BNO086_ROS2Board serial bring-up tool")]
struct Args {
    /// Serial port name. The default, "auto", discovers the port by VID/PID.
    #[arg(long, default_value = bno086_serial::PORT_AUTO)]
    port: String,
    /// Baud rate (ignored by USB CDC; UART1 uses 921600).
    #[arg(long, default_value_t = 921_600)]
    baud: u32,
}

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()),
        )
        .init();

    let args = Args::parse();
    let handle = bno086_serial::spawn(bno086_serial::Bno086Config {
        port: args.port,
        baud: args.baud,
    });

    let mut last_count = 0u64;
    loop {
        std::thread::sleep(Duration::from_secs(1));
        let count = handle.sample_count();
        let rate = count - last_count;
        last_count = count;
        // Maximum sample gap observed during this second. Consumer freshness timeouts should be
        // comfortably larger to avoid rapid connection-state transitions.
        let gap_ms = handle.take_max_gap().as_secs_f64() * 1e3;

        match handle.latest() {
            Some(s) if handle.connected() => {
                let v = s.sample;
                println!(
                    "{rate:>4} Hz  conn#{}  t={:.3}s  quat={}  gyro={}  accel={}  acc(q/g/a)={}/{}/{}  max_gap={gap_ms:.1}ms  frame_err={}",
                    s.connection_id,
                    v.t_us as f64 / 1e6,
                    fmt4(v.quat),
                    fmt3(v.gyro),
                    fmt3(v.accel),
                    v.acc_status.quat,
                    v.acc_status.gyro,
                    v.acc_status.accel,
                    handle.frame_errors(),
                );
            }
            _ => println!(
                "   -- waiting for BNO086 (connected={})",
                handle.connected()
            ),
        }
    }
}

fn fmt3(v: Option<[f32; 3]>) -> String {
    v.map_or_else(
        || "     none        ".to_string(),
        |a| format!("[{:+6.2},{:+6.2},{:+6.2}]", a[0], a[1], a[2]),
    )
}

fn fmt4(v: Option<[f32; 4]>) -> String {
    v.map_or_else(
        || "        none           ".to_string(),
        |a| format!("[{:+5.3},{:+5.3},{:+5.3},{:+5.3}]", a[0], a[1], a[2], a[3]),
    )
}
