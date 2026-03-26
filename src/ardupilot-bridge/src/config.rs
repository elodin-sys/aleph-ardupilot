use clap::Parser;

#[derive(Parser, Debug)]
#[command(name = "ardupilot-bridge")]
#[command(about = "Bridges Elodin-DB sensor data to ArduPilot SITL and CAN ESC output")]
pub struct Config {
    /// Elodin-DB address
    #[arg(long, default_value = "127.0.0.1:2240", env = "ELODIN_DB_ADDR")]
    pub elodin_addr: String,

    /// ArduPilot SITL JSON control interface UDP port (send sensor data here,
    /// receive servo output back on the same socket)
    #[arg(long, default_value_t = 9002, env = "AP_CONTROL_PORT")]
    pub servo_port: u16,

    /// ArduPilot SITL host address
    #[arg(long, default_value = "127.0.0.1", env = "AP_HOST")]
    pub ap_host: String,

    /// Number of motor channels (typically 4 for quadcopter)
    #[arg(long, default_value_t = 4, env = "NUM_MOTORS")]
    pub num_motors: usize,

    /// HITL TCP listen port (0 = disabled)
    #[arg(long, default_value_t = 0, env = "HITL_PORT")]
    pub hitl_port: u16,

    /// SocketCAN interface for DroneCAN ESC output (empty = disabled)
    #[arg(long, default_value = "", env = "CAN_INTERFACE")]
    pub can_interface: String,
}
