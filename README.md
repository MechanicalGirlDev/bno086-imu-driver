# BNO086 IMU Driver

Standalone Rust serial transport and bring-up utility for the **BNO086_ROS2Board** custom board (BNO086 + STM32F042C6). This is board-specific software, not a universal BNO086 driver.

The board firmware streams Rotation Vector, Gyroscope, and Accelerometer reports at 400 Hz in `COBS(postcard payload + CRC16) + 0x00` frames. The host reads without sending report-configuration commands. This crate reconnects after the serial device disappears and exposes the latest received sample with its timestamp and connection generation.

The sample values remain in the board frame. Coordinate conversion and freshness policy belong to the consuming application.

## Usage

```toml
[dependencies]
bno086-serial = "0.1.4"
```

Start the reader:

```rust
let handle = bno086_serial::spawn(bno086_serial::Bno086Config::default());
```

The default port setting discovers the board by USB VID/PID. Set `Bno086Config.port` to a serial port name such as `COM7` or `/dev/ttyACM0` to select it explicitly. `Bno086Handle::latest()` returns the latest sample; `connected()`, `sample_count()`, `frame_errors()`, and `take_max_gap()` expose reader status.

Install and run the standalone bring-up utility:

```sh
cargo install --path .
bno086 --help
bno086 --port COM7
```

## Reiny 0.7 integration

`bno086-serial` remains a framework-independent transport. The `bno086` CLI is
a standalone bring-up tool, not a managed Reiny module.

Put the Reiny SDK and `main.yaml` in the consuming adapter. Declare its compiled
IMU output type, open the named `Cloudy::output` and establish the adapter's
startup policy before calling `Cloudy::ready()`. Keep board-to-body conversion,
sample age and connection-generation handling in that adapter. The publisher
namespace comes from Reiny's deployment/module path, not from a serial port name.

The reader thread currently has no cancellation API and ends with its process.
Dropping `Bno086Handle` does not stop it. A host adapter must not advertise a
cooperative reader-stop guarantee that this transport does not provide.

## Wire compatibility

The host/device protocol definitions are vendored from `MechanicalGirlDev/BNO086_ROS2Board`, revision `d940d6a62f7675e1b42c772add39709df04915e9` (2026-07-22). Protocol enum variant order and structure field order are wire-significant for postcard and must be preserved. On connection, the driver requests device information and warns when the reported protocol version differs from `PROTOCOL_VERSION`.

`src/frame.rs` retains the upstream COBS and CRC16-CCITT framing algorithms. Its only upstream implementation adaptation is replacing the firmware's bounded `heapless::Vec` container with host-side `Vec`; frame-size checks remain.

## License and attribution

Licensed under Apache-2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE) for attribution and third-party notices.
