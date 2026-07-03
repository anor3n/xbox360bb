# xbox360bb

Userspace Rust library for reading Xbox 360 Big Button controller events
directly from the USB receiver on Linux.

This crate talks to the receiver with `libusb-1.0`. It does not use the kernel
driver and it does not expose `/dev/input` devices. It is intended for programs
that want to consume controller events directly.

## What it supports

- Detecting the Xbox 360 Big Button receiver (`045e:02a0`)
- Reading events for all four controllers from one receiver
- Decoding d-pad and button state
- Suppressing duplicate repeat packets
- Synthesizing release events when the receiver stops repeating a held button

## Requirements

- Linux
- `libusb-1.0`
- Permission to access the USB device

If the kernel `xbox360bb` module is loaded, the crate attempts to auto-detach
the kernel driver before claiming the interface.

## Add it to your project

If your application lives next to this crate:

```toml
[dependencies]
xbox360bb = { path = "../xbox360bb" }
```

## Quick start

```rust,no_run
use xbox360bb::Receiver;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut receiver = Receiver::open()?;

    loop {
        let event = receiver.next_event()?;
        println!("{event:?}");
    }
}
```

## API overview

### Open a receiver

```rust,no_run
use xbox360bb::Receiver;

let mut receiver = Receiver::open()?;
# Ok::<(), xbox360bb::Error>(())
```

Or with custom timeouts:

```rust,no_run
use std::time::Duration;
use xbox360bb::Receiver;

let mut receiver = Receiver::open_with_timeouts(
    Duration::from_millis(20),
    Duration::from_millis(250),
)?;
# Ok::<(), xbox360bb::Error>(())
```

### Blocking event loop

```rust,no_run
use xbox360bb::Receiver;

let mut receiver = Receiver::open()?;

loop {
    let event = receiver.next_event()?;
    println!("{:?}: {:?}", event.controller, event.state);
}
# #[allow(unreachable_code)]
# Ok::<(), xbox360bb::Error>(())
```

### Iterator-based event loop

```rust,no_run
use xbox360bb::Receiver;

let mut receiver = Receiver::open()?;

for event in receiver.events() {
    println!("{:?}", event?);
}
# #[allow(unreachable_code)]
# Ok::<(), xbox360bb::Error>(())
```

### Polling without blocking forever

```rust,no_run
use std::time::Duration;
use xbox360bb::Receiver;

let mut receiver = Receiver::open()?;

if let Some(event) = receiver.poll_event(Duration::from_millis(10))? {
    println!("{event:?}");
}
# Ok::<(), xbox360bb::Error>(())
```

### Read current state

```rust,no_run
use xbox360bb::{ControllerId, Receiver};

let receiver = Receiver::open()?;
let green = receiver.state(ControllerId::Green);
println!("{green:?}");
# Ok::<(), xbox360bb::Error>(())
```

## Event model

Each event contains:

- `controller`: which of the four controllers changed
- `state`: the full decoded state after that change
- `kind`: either `StateChanged` or `Released`

`Released` is synthesized by the crate. The physical receiver repeats held
button states, but does not always send an explicit "all buttons released"
packet. After a configurable silence timeout, the crate emits an idle state.

## Button mapping

`ControllerState` contains:

- `dpad_x`: `-1` left, `0` center, `1` right
- `dpad_y`: `-1` up, `0` center, `1` down
- `start`
- `back`
- `guide`
- `center`
- `a`
- `b`
- `x`
- `y`

## Example program

Run the included monitor:

```bash
cargo run --example monitor
```

It prints every decoded event as it arrives.
