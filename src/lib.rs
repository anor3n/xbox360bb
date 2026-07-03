//! Userspace access to Xbox 360 Big Button controllers on Linux.
//!
//! This crate talks directly to the Xbox 360 Big Button USB receiver with
//! `libusb`, so it does not depend on the kernel driver or `/dev/input`.
//!
//! The receiver multiplexes four wireless controllers behind one USB device.
//! `Receiver` exposes them as a stream of [`ControllerEvent`] values and keeps
//! enough state internally to synthesize release events when the receiver stops
//! repeating held buttons.
//!
//! # Basic usage
//!
//! ```no_run
//! use xbox360bb::Receiver;
//!
//! fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let mut receiver = Receiver::open()?;
//!
//!     loop {
//!         let event = receiver.next_event()?;
//!         println!("{event:?}");
//!     }
//! }
//! ```
//!
//! # Linux notes
//!
//! - The process must be able to access the USB device, usually through a
//!   udev rule or by running with elevated privileges.
//! - If the kernel `xbox360bb` module is loaded, `Receiver::open()` attempts to
//!   auto-detach it before claiming the USB interface.
use std::error::Error as StdError;
use std::ffi::c_int;
use std::fmt;
use std::ptr;
use std::slice;
use std::time::{Duration, Instant};

const VENDOR_ID: u16 = 0x045e;
const PRODUCT_ID: u16 = 0x02a0;
const PACKET_LEN: usize = 32;
const DEFAULT_READ_TIMEOUT: Duration = Duration::from_millis(50);
const DEFAULT_RELEASE_TIMEOUT: Duration = Duration::from_millis(250);

/// Errors returned by this crate.
#[derive(Debug)]
pub enum Error {
    DeviceNotFound,
    InvalidDeviceDescriptor,
    InvalidConfigurationDescriptor,
    InterruptEndpointNotFound,
    Usb(i32),
    TimeoutTooLarge(Duration),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DeviceNotFound => f.write_str("xbox360bb receiver not found"),
            Self::InvalidDeviceDescriptor => f.write_str("invalid USB device descriptor"),
            Self::InvalidConfigurationDescriptor => {
                f.write_str("invalid USB configuration descriptor")
            }
            Self::InterruptEndpointNotFound => {
                f.write_str("interrupt IN endpoint not found on receiver")
            }
            Self::Usb(code) => write!(f, "libusb error {} ({})", code, usb_error_name(*code)),
            Self::TimeoutTooLarge(timeout) => {
                write!(f, "timeout {:?} exceeds libusb limits", timeout)
            }
        }
    }
}

impl StdError for Error {}

/// One of the four controller slots exposed by the receiver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ControllerId {
    Green,
    Red,
    Blue,
    Yellow,
}

impl ControllerId {
    pub const ALL: [Self; 4] = [Self::Green, Self::Red, Self::Blue, Self::Yellow];

    fn from_index(index: u8) -> Option<Self> {
        match index {
            0 => Some(Self::Green),
            1 => Some(Self::Red),
            2 => Some(Self::Blue),
            3 => Some(Self::Yellow),
            _ => None,
        }
    }

    fn index(self) -> usize {
        match self {
            Self::Green => 0,
            Self::Red => 1,
            Self::Blue => 2,
            Self::Yellow => 3,
        }
    }
}

/// Snapshot of all inputs for a single controller.
///
/// `dpad_x` and `dpad_y` are normalized to `-1`, `0`, or `1`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ControllerState {
    pub dpad_x: i8,
    pub dpad_y: i8,
    pub start: bool,
    pub back: bool,
    pub guide: bool,
    pub center: bool,
    pub a: bool,
    pub b: bool,
    pub x: bool,
    pub y: bool,
}

impl ControllerState {
    /// Returns `true` when no buttons are pressed and the d-pad is centered.
    pub fn is_idle(self) -> bool {
        self == Self::default()
    }
}

/// Describes why an event was emitted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventKind {
    /// The receiver reported a new button state.
    StateChanged,
    /// The crate synthesized a release after the repeat timeout elapsed.
    Released,
}

/// A decoded event for one controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ControllerEvent {
    pub controller: ControllerId,
    pub state: ControllerState,
    pub kind: EventKind,
}

#[derive(Debug, Clone, Copy)]
struct ControllerSlot {
    state: ControllerState,
    raw_b3: u8,
    raw_b4: u8,
    last_seen: Option<Instant>,
}

impl Default for ControllerSlot {
    fn default() -> Self {
        Self {
            state: ControllerState::default(),
            raw_b3: 0,
            raw_b4: 0,
            last_seen: None,
        }
    }
}

/// Open handle to the USB receiver.
///
/// This type owns the libusb context and claimed USB interface. Drop releases
/// the interface and closes the device handle.
pub struct Receiver {
    context: *mut ffi::libusb_context,
    handle: *mut ffi::libusb_device_handle,
    interface_number: u8,
    endpoint_address: u8,
    read_timeout: Duration,
    release_timeout: Duration,
    slots: [ControllerSlot; 4],
}

impl Receiver {
    /// Opens the receiver with the default read and release timeouts.
    ///
    /// The default read timeout is 50 ms. The default synthesized release
    /// timeout is 250 ms.
    pub fn open() -> Result<Self, Error> {
        Self::open_with_timeouts(DEFAULT_READ_TIMEOUT, DEFAULT_RELEASE_TIMEOUT)
    }

    /// Opens the receiver with explicit timeouts.
    ///
    /// `read_timeout` controls how long each USB interrupt read blocks before
    /// returning to check for synthesized releases.
    ///
    /// `release_timeout` controls how long a controller may go silent after a
    /// repeated held-state report before a [`EventKind::Released`] event is
    /// emitted.
    pub fn open_with_timeouts(
        read_timeout: Duration,
        release_timeout: Duration,
    ) -> Result<Self, Error> {
        let mut context = ptr::null_mut();
        usb_call(unsafe { ffi::libusb_init_context(&mut context, ptr::null(), 0) })?;

        let open_result = open_receiver_handle(context);
        match open_result {
            Ok((handle, interface_number, endpoint_address)) => {
                let _ = unsafe { ffi::libusb_set_auto_detach_kernel_driver(handle, 1) };
                if let Err(error) = usb_call(unsafe {
                    ffi::libusb_claim_interface(handle, c_int::from(interface_number))
                }) {
                    unsafe {
                        ffi::libusb_close(handle);
                        ffi::libusb_exit(context);
                    }
                    return Err(error);
                }

                Ok(Self {
                    context,
                    handle,
                    interface_number,
                    endpoint_address,
                    read_timeout,
                    release_timeout,
                    slots: [ControllerSlot::default(); 4],
                })
            }
            Err(error) => {
                unsafe { ffi::libusb_exit(context) };
                Err(error)
            }
        }
    }

    /// Blocks until the next controller event is available.
    pub fn next_event(&mut self) -> Result<ControllerEvent, Error> {
        loop {
            if let Some(event) = self.poll_event(self.read_timeout)? {
                return Ok(event);
            }
        }
    }

    /// Polls once for an event.
    ///
    /// Returns `Ok(None)` when no packet arrived within `timeout` and no
    /// synthesized release became due.
    pub fn poll_event(&mut self, timeout: Duration) -> Result<Option<ControllerEvent>, Error> {
        let timeout_ms = duration_to_timeout_ms(timeout)?;
        let mut packet = [0u8; PACKET_LEN];
        let mut transferred = 0;

        let result = unsafe {
            ffi::libusb_interrupt_transfer(
                self.handle,
                self.endpoint_address,
                packet.as_mut_ptr(),
                PACKET_LEN as c_int,
                &mut transferred,
                timeout_ms,
            )
        };

        if result == ffi::LIBUSB_ERROR_TIMEOUT {
            return Ok(self.check_release_timeouts());
        }

        usb_call(result)?;

        if transferred < 5 {
            return Ok(self.check_release_timeouts());
        }

        if let Some(event) = self.decode_packet(&packet[..transferred as usize]) {
            return Ok(Some(event));
        }

        Ok(self.check_release_timeouts())
    }

    /// Returns the most recent known state for a controller.
    ///
    /// Before the first event for a controller, this is the idle state.
    pub fn state(&self, controller: ControllerId) -> ControllerState {
        self.slots[controller.index()].state
    }

    /// Returns an infinite iterator over events.
    ///
    /// Each call to `next()` blocks internally until an event is available or
    /// an error occurs.
    ///
    /// ```no_run
    /// use xbox360bb::Receiver;
    ///
    /// fn main() -> Result<(), Box<dyn std::error::Error>> {
    ///     let mut receiver = Receiver::open()?;
    ///
    ///     for event in receiver.events() {
    ///         println!("{:?}", event?);
    ///     }
    ///
    ///     Ok(())
    /// }
    /// ```
    pub fn events(&mut self) -> Events<'_> {
        Events { receiver: self }
    }

    fn decode_packet(&mut self, packet: &[u8]) -> Option<ControllerEvent> {
        let controller = ControllerId::from_index(packet[2])?;
        let slot = &mut self.slots[controller.index()];
        let b3 = packet[3];
        let b4 = packet[4];

        slot.last_seen = Some(Instant::now());

        if slot.raw_b3 == b3 && slot.raw_b4 == b4 {
            return None;
        }

        slot.raw_b3 = b3;
        slot.raw_b4 = b4;
        slot.state = decode_state(b3, b4);

        Some(ControllerEvent {
            controller,
            state: slot.state,
            kind: EventKind::StateChanged,
        })
    }

    fn check_release_timeouts(&mut self) -> Option<ControllerEvent> {
        let now = Instant::now();

        for controller in ControllerId::ALL {
            let slot = &mut self.slots[controller.index()];
            let expired = slot
                .last_seen
                .is_some_and(|last_seen| now.duration_since(last_seen) >= self.release_timeout);

            if expired && !slot.state.is_idle() {
                slot.state = ControllerState::default();
                slot.raw_b3 = 0;
                slot.raw_b4 = 0;
                slot.last_seen = None;

                return Some(ControllerEvent {
                    controller,
                    state: ControllerState::default(),
                    kind: EventKind::Released,
                });
            }
        }

        None
    }
}

impl Drop for Receiver {
    fn drop(&mut self) {
        unsafe {
            if !self.handle.is_null() {
                let _ =
                    ffi::libusb_release_interface(self.handle, c_int::from(self.interface_number));
                ffi::libusb_close(self.handle);
            }
            if !self.context.is_null() {
                ffi::libusb_exit(self.context);
            }
        }
    }
}

/// Iterator returned by [`Receiver::events`].
pub struct Events<'a> {
    receiver: &'a mut Receiver,
}

impl Iterator for Events<'_> {
    type Item = Result<ControllerEvent, Error>;

    fn next(&mut self) -> Option<Self::Item> {
        Some(self.receiver.next_event())
    }
}

fn decode_state(b3: u8, b4: u8) -> ControllerState {
    ControllerState {
        dpad_x: if b3 & 0x04 != 0 {
            -1
        } else if b3 & 0x08 != 0 {
            1
        } else {
            0
        },
        dpad_y: if b3 & 0x01 != 0 {
            -1
        } else if b3 & 0x02 != 0 {
            1
        } else {
            0
        },
        start: b3 & 0x10 != 0,
        back: b3 & 0x20 != 0,
        guide: b4 & 0x04 != 0,
        center: b4 & 0x08 != 0,
        a: b4 & 0x10 != 0,
        b: b4 & 0x20 != 0,
        x: b4 & 0x40 != 0,
        y: b4 & 0x80 != 0,
    }
}

fn open_receiver_handle(
    context: *mut ffi::libusb_context,
) -> Result<(*mut ffi::libusb_device_handle, u8, u8), Error> {
    let mut device_list: *const *mut ffi::libusb_device = ptr::null();
    let count = unsafe { ffi::libusb_get_device_list(context, &mut device_list) };
    if count < 0 {
        return Err(Error::Usb(count as i32));
    }

    let mut result = Err(Error::DeviceNotFound);

    unsafe {
        let devices = slice::from_raw_parts(device_list, count as usize);

        'devices: for &device in devices {
            if device.is_null() {
                continue;
            }

            let mut descriptor = ffi::libusb_device_descriptor::default();
            let descriptor_result = ffi::libusb_get_device_descriptor(device, &mut descriptor);
            if descriptor_result != 0 {
                result = Err(Error::InvalidDeviceDescriptor);
                continue;
            }

            if descriptor.idVendor != VENDOR_ID || descriptor.idProduct != PRODUCT_ID {
                continue;
            }

            let mut handle = ptr::null_mut();
            usb_call(ffi::libusb_open(device, &mut handle))?;

            match find_interrupt_endpoint(device) {
                Ok((interface_number, endpoint_address)) => {
                    result = Ok((handle, interface_number, endpoint_address));
                    break 'devices;
                }
                Err(error) => {
                    ffi::libusb_close(handle);
                    result = Err(error);
                }
            }
        }

        ffi::libusb_free_device_list(device_list, 1);
    }

    result
}

fn find_interrupt_endpoint(device: *mut ffi::libusb_device) -> Result<(u8, u8), Error> {
    let mut config = ptr::null();
    usb_call(unsafe { ffi::libusb_get_active_config_descriptor(device, &mut config) })?;

    if config.is_null() {
        return Err(Error::InvalidConfigurationDescriptor);
    }

    let result = unsafe {
        let config_ref = &*config;
        let interfaces =
            slice::from_raw_parts(config_ref.interface, config_ref.bNumInterfaces as usize);

        let mut found = None;

        for interface in interfaces {
            let altsettings =
                slice::from_raw_parts(interface.altsetting, interface.num_altsetting as usize);
            for descriptor in altsettings {
                let endpoints =
                    slice::from_raw_parts(descriptor.endpoint, descriptor.bNumEndpoints as usize);

                for endpoint in endpoints {
                    let transfer_type = endpoint.bmAttributes & ffi::LIBUSB_TRANSFER_TYPE_MASK;
                    let direction = endpoint.bEndpointAddress & ffi::LIBUSB_ENDPOINT_DIR_MASK;

                    if transfer_type == ffi::LIBUSB_TRANSFER_TYPE_INTERRUPT
                        && direction == ffi::LIBUSB_ENDPOINT_IN
                    {
                        found = Some((descriptor.bInterfaceNumber, endpoint.bEndpointAddress));
                        break;
                    }
                }

                if found.is_some() {
                    break;
                }
            }

            if found.is_some() {
                break;
            }
        }

        found.ok_or(Error::InterruptEndpointNotFound)
    };

    unsafe { ffi::libusb_free_config_descriptor(config) };
    result
}

fn usb_call(code: i32) -> Result<(), Error> {
    if code == 0 {
        Ok(())
    } else {
        Err(Error::Usb(code))
    }
}

fn duration_to_timeout_ms(duration: Duration) -> Result<u32, Error> {
    let millis = duration.as_millis();
    if millis > u32::MAX as u128 {
        return Err(Error::TimeoutTooLarge(duration));
    }
    Ok(millis as u32)
}

fn usb_error_name(code: i32) -> &'static str {
    match code {
        0 => "success",
        ffi::LIBUSB_ERROR_IO => "input/output error",
        ffi::LIBUSB_ERROR_INVALID_PARAM => "invalid parameter",
        ffi::LIBUSB_ERROR_ACCESS => "access denied",
        ffi::LIBUSB_ERROR_NO_DEVICE => "device disconnected",
        ffi::LIBUSB_ERROR_NOT_FOUND => "entity not found",
        ffi::LIBUSB_ERROR_BUSY => "resource busy",
        ffi::LIBUSB_ERROR_TIMEOUT => "timeout",
        ffi::LIBUSB_ERROR_OVERFLOW => "overflow",
        ffi::LIBUSB_ERROR_PIPE => "pipe error",
        ffi::LIBUSB_ERROR_INTERRUPTED => "system call interrupted",
        ffi::LIBUSB_ERROR_NO_MEM => "out of memory",
        ffi::LIBUSB_ERROR_NOT_SUPPORTED => "operation not supported",
        _ => "unknown error",
    }
}

#[allow(non_snake_case)]
mod ffi {
    use std::ffi::{c_int, c_uchar, c_uint, c_ushort};

    pub const LIBUSB_ENDPOINT_IN: u8 = 0x80;
    pub const LIBUSB_ENDPOINT_DIR_MASK: u8 = 0x80;
    pub const LIBUSB_TRANSFER_TYPE_MASK: u8 = 0x03;
    pub const LIBUSB_TRANSFER_TYPE_INTERRUPT: u8 = 0x03;

    pub const LIBUSB_ERROR_IO: i32 = -1;
    pub const LIBUSB_ERROR_INVALID_PARAM: i32 = -2;
    pub const LIBUSB_ERROR_ACCESS: i32 = -3;
    pub const LIBUSB_ERROR_NO_DEVICE: i32 = -4;
    pub const LIBUSB_ERROR_NOT_FOUND: i32 = -5;
    pub const LIBUSB_ERROR_BUSY: i32 = -6;
    pub const LIBUSB_ERROR_TIMEOUT: i32 = -7;
    pub const LIBUSB_ERROR_OVERFLOW: i32 = -8;
    pub const LIBUSB_ERROR_PIPE: i32 = -9;
    pub const LIBUSB_ERROR_INTERRUPTED: i32 = -10;
    pub const LIBUSB_ERROR_NO_MEM: i32 = -11;
    pub const LIBUSB_ERROR_NOT_SUPPORTED: i32 = -12;

    #[repr(C)]
    pub struct libusb_context {
        _private: [u8; 0],
    }

    #[repr(C)]
    pub struct libusb_device {
        _private: [u8; 0],
    }

    #[repr(C)]
    pub struct libusb_device_handle {
        _private: [u8; 0],
    }

    #[repr(C)]
    #[derive(Clone, Copy, Default)]
    pub struct libusb_device_descriptor {
        pub bLength: c_uchar,
        pub bDescriptorType: c_uchar,
        pub bcdUSB: c_ushort,
        pub bDeviceClass: c_uchar,
        pub bDeviceSubClass: c_uchar,
        pub bDeviceProtocol: c_uchar,
        pub bMaxPacketSize0: c_uchar,
        pub idVendor: c_ushort,
        pub idProduct: c_ushort,
        pub bcdDevice: c_ushort,
        pub iManufacturer: c_uchar,
        pub iProduct: c_uchar,
        pub iSerialNumber: c_uchar,
        pub bNumConfigurations: c_uchar,
    }

    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct libusb_endpoint_descriptor {
        pub bLength: c_uchar,
        pub bDescriptorType: c_uchar,
        pub bEndpointAddress: c_uchar,
        pub bmAttributes: c_uchar,
        pub wMaxPacketSize: c_ushort,
        pub bInterval: c_uchar,
        pub bRefresh: c_uchar,
        pub bSynchAddress: c_uchar,
        pub extra: *const c_uchar,
        pub extra_length: c_int,
    }

    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct libusb_interface_descriptor {
        pub bLength: c_uchar,
        pub bDescriptorType: c_uchar,
        pub bInterfaceNumber: c_uchar,
        pub bAlternateSetting: c_uchar,
        pub bNumEndpoints: c_uchar,
        pub bInterfaceClass: c_uchar,
        pub bInterfaceSubClass: c_uchar,
        pub bInterfaceProtocol: c_uchar,
        pub iInterface: c_uchar,
        pub endpoint: *const libusb_endpoint_descriptor,
        pub extra: *const c_uchar,
        pub extra_length: c_int,
    }

    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct libusb_interface {
        pub altsetting: *const libusb_interface_descriptor,
        pub num_altsetting: c_int,
    }

    #[repr(C)]
    pub struct libusb_config_descriptor {
        pub bLength: c_uchar,
        pub bDescriptorType: c_uchar,
        pub wTotalLength: c_ushort,
        pub bNumInterfaces: c_uchar,
        pub bConfigurationValue: c_uchar,
        pub iConfiguration: c_uchar,
        pub bmAttributes: c_uchar,
        pub MaxPower: c_uchar,
        pub interface: *const libusb_interface,
        pub extra: *const c_uchar,
        pub extra_length: c_int,
    }

    #[link(name = "usb-1.0")]
    unsafe extern "C" {
        pub fn libusb_init_context(
            ctx: *mut *mut libusb_context,
            options: *const core::ffi::c_void,
            num_options: c_int,
        ) -> c_int;
        pub fn libusb_exit(ctx: *mut libusb_context);
        pub fn libusb_get_device_list(
            ctx: *mut libusb_context,
            list: *mut *const *mut libusb_device,
        ) -> isize;
        pub fn libusb_free_device_list(list: *const *mut libusb_device, unref_devices: c_int);
        pub fn libusb_get_device_descriptor(
            dev: *mut libusb_device,
            desc: *mut libusb_device_descriptor,
        ) -> c_int;
        pub fn libusb_open(
            dev: *mut libusb_device,
            handle: *mut *mut libusb_device_handle,
        ) -> c_int;
        pub fn libusb_close(dev_handle: *mut libusb_device_handle);
        pub fn libusb_get_active_config_descriptor(
            dev: *mut libusb_device,
            config: *mut *const libusb_config_descriptor,
        ) -> c_int;
        pub fn libusb_free_config_descriptor(config: *const libusb_config_descriptor);
        pub fn libusb_claim_interface(
            dev_handle: *mut libusb_device_handle,
            interface_number: c_int,
        ) -> c_int;
        pub fn libusb_release_interface(
            dev_handle: *mut libusb_device_handle,
            interface_number: c_int,
        ) -> c_int;
        pub fn libusb_set_auto_detach_kernel_driver(
            dev_handle: *mut libusb_device_handle,
            enable: c_int,
        ) -> c_int;
        pub fn libusb_interrupt_transfer(
            dev_handle: *mut libusb_device_handle,
            endpoint: c_uchar,
            data: *mut u8,
            length: c_int,
            transferred: *mut c_int,
            timeout: c_uint,
        ) -> c_int;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_button_bits() {
        let state = decode_state(0x19, 0x5c);
        assert_eq!(state.dpad_x, 1);
        assert_eq!(state.dpad_y, -1);
        assert!(state.start);
        assert!(!state.back);
        assert!(state.guide);
        assert!(state.center);
        assert!(state.a);
        assert!(!state.b);
        assert!(state.x);
        assert!(!state.y);
    }

    #[test]
    fn release_timeout_emits_idle_state() {
        let mut receiver = Receiver {
            context: ptr::null_mut(),
            handle: ptr::null_mut(),
            interface_number: 0,
            endpoint_address: 0,
            read_timeout: DEFAULT_READ_TIMEOUT,
            release_timeout: Duration::from_millis(10),
            slots: [ControllerSlot::default(); 4],
        };

        receiver.slots[0].state = decode_state(0x10, 0x10);
        receiver.slots[0].raw_b3 = 0x10;
        receiver.slots[0].raw_b4 = 0x10;
        receiver.slots[0].last_seen = Some(Instant::now() - Duration::from_millis(20));

        let event = receiver
            .check_release_timeouts()
            .expect("expected release event");
        assert_eq!(event.controller, ControllerId::Green);
        assert_eq!(event.kind, EventKind::Released);
        assert!(event.state.is_idle());
        assert!(receiver.slots[0].state.is_idle());
    }
}
