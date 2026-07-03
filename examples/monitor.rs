use xbox360bb::{ControllerEvent, Receiver};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut receiver = Receiver::open()?;

    for event in receiver.events() {
        print_event(event?);
    }

    Ok(())
}

fn print_event(event: ControllerEvent) {
    println!(
        "{:?} {:?} x={} y={} start={} back={} guide={} center={} a={} b={} x={} y={}",
        event.controller,
        event.kind,
        event.state.dpad_x,
        event.state.dpad_y,
        event.state.start,
        event.state.back,
        event.state.guide,
        event.state.center,
        event.state.a,
        event.state.b,
        event.state.x,
        event.state.y,
    );
}
