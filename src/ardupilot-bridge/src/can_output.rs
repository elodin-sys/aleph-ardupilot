/// DroneCAN ESC command output via Linux SocketCAN.
///
/// Sends `uavcan.equipment.esc.RawCommand` (DSDL ID 1030) frames on the
/// configured CAN interface. The RawCommand message contains up to 20
/// 14-bit signed integers. For ESC throttle, values are [0, 8191] where
/// 8191 = maximum throttle.
///
/// DroneCAN frame format on CAN bus:
///   - CAN ID encodes: priority (5 bits), data type ID (16 bits),
///     service/message flag, source node ID (7 bits)
///   - For broadcast messages: bits [28:24]=priority, [23:8]=dtid, [7]=0, [6:0]=source_node_id
///   - Tail byte: start/end/toggle bits + transfer ID (5 bits)
use std::ffi::CString;

/// DroneCAN data type ID for esc.RawCommand
const ESC_RAW_COMMAND_DTID: u16 = 1030;

/// Default DroneCAN node ID for the bridge
const DEFAULT_NODE_ID: u8 = 100;

/// Maximum ESC throttle value (14-bit signed positive range)
const ESC_MAX_THROTTLE: i16 = 8191;

/// CAN_RAW protocol number
const CAN_RAW: libc::c_int = 1;

/// PF_CAN address family
const PF_CAN: libc::c_int = 29;

/// sockaddr_can structure for SocketCAN
#[repr(C)]
struct SockAddrCan {
    can_family: u16,
    can_ifindex: i32,
    _pad: [u8; 8],
}

/// CAN frame structure
#[repr(C)]
struct CanFrame {
    can_id: u32,
    can_dlc: u8,
    _pad: u8,
    _res0: u8,
    _res1: u8,
    data: [u8; 8],
}

pub struct CanOutput {
    interface: String,
    fd: Option<i32>,
    node_id: u8,
    transfer_id: u8,
}

impl CanOutput {
    pub fn new(interface: &str) -> Self {
        Self {
            interface: interface.to_string(),
            fd: None,
            node_id: DEFAULT_NODE_ID,
            transfer_id: 0,
        }
    }

    pub fn is_enabled(&self) -> bool {
        !self.interface.is_empty()
    }

    pub fn open(&mut self) -> anyhow::Result<()> {
        if !self.is_enabled() {
            return Ok(());
        }

        let fd = unsafe { libc::socket(PF_CAN, libc::SOCK_RAW, CAN_RAW) };
        if fd < 0 {
            anyhow::bail!("failed to create CAN socket: {}", std::io::Error::last_os_error());
        }

        let ifname = CString::new(self.interface.as_str())?;
        let ifindex = unsafe {
            libc::if_nametoindex(ifname.as_ptr()) as i32
        };
        if ifindex == 0 {
            unsafe { libc::close(fd); }
            anyhow::bail!("CAN interface '{}' not found", self.interface);
        }

        let addr = SockAddrCan {
            can_family: PF_CAN as u16,
            can_ifindex: ifindex,
            _pad: [0; 8],
        };

        let ret = unsafe {
            libc::bind(
                fd,
                &addr as *const SockAddrCan as *const libc::sockaddr,
                std::mem::size_of::<SockAddrCan>() as libc::socklen_t,
            )
        };
        if ret < 0 {
            unsafe { libc::close(fd); }
            anyhow::bail!("failed to bind CAN socket: {}", std::io::Error::last_os_error());
        }

        self.fd = Some(fd);
        tracing::info!("CAN output opened on {} (ifindex={})", self.interface, ifindex);
        Ok(())
    }

    /// Send ESC commands for the given motor values (normalized 0.0-1.0).
    pub fn send_esc_command(&mut self, motors: &[f64]) -> anyhow::Result<()> {
        let fd = match self.fd {
            Some(fd) => fd,
            None => return Ok(()),
        };

        let mut payload = Vec::with_capacity(motors.len() * 2);
        for &m in motors {
            let val = (m.clamp(0.0, 1.0) * ESC_MAX_THROTTLE as f64) as i16;
            let raw = val as u16;
            payload.push((raw & 0xFF) as u8);
            payload.push(((raw >> 8) & 0x3F) as u8);
        }

        // For 4 motors: 4 * 14 bits = 56 bits = 7 bytes payload
        // Plus 1 tail byte = 8 bytes total, fits in a single CAN frame
        // Pack 14-bit values: motor values are packed sequentially at bit level
        let mut packed = vec![0u8; ((motors.len() * 14) + 7) / 8];
        let mut bit_offset = 0usize;
        for &m in motors {
            let val = (m.clamp(0.0, 1.0) * ESC_MAX_THROTTLE as f64) as i16;
            let raw = (val & 0x3FFF) as u16;
            for bit in 0..14 {
                if raw & (1 << bit) != 0 {
                    let byte_idx = (bit_offset + bit) / 8;
                    let bit_idx = (bit_offset + bit) % 8;
                    if byte_idx < packed.len() {
                        packed[byte_idx] |= 1 << bit_idx;
                    }
                }
            }
            bit_offset += 14;
        }

        // DroneCAN tail byte: start_of_transfer | end_of_transfer | toggle | transfer_id
        let tail_byte = 0xC0 | (self.transfer_id & 0x1F); // single-frame: SOT=1, EOT=1, toggle=0
        self.transfer_id = self.transfer_id.wrapping_add(1);

        let dlc = packed.len() + 1; // payload + tail byte
        if dlc > 8 {
            anyhow::bail!("ESC command too large for single CAN frame ({} motors)", motors.len());
        }

        let mut data = [0u8; 8];
        data[..packed.len()].copy_from_slice(&packed);
        data[packed.len()] = tail_byte;

        // CAN ID: priority=16 (medium), DTID=1030, source_node_id
        let can_id = ((16u32 & 0x1F) << 24)
            | ((ESC_RAW_COMMAND_DTID as u32 & 0xFFFF) << 8)
            | (self.node_id as u32 & 0x7F);
        let can_id = can_id | 0x80000000; // Extended frame flag (EFF)

        let frame = CanFrame {
            can_id,
            can_dlc: dlc as u8,
            _pad: 0,
            _res0: 0,
            _res1: 0,
            data,
        };

        let ret = unsafe {
            libc::write(
                fd,
                &frame as *const CanFrame as *const libc::c_void,
                std::mem::size_of::<CanFrame>(),
            )
        };
        if ret < 0 {
            anyhow::bail!("CAN write failed: {}", std::io::Error::last_os_error());
        }

        Ok(())
    }
}

impl Drop for CanOutput {
    fn drop(&mut self) {
        if let Some(fd) = self.fd.take() {
            unsafe { libc::close(fd); }
        }
    }
}
