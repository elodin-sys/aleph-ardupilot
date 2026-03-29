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

    /// ArduPilot home location as "lat,lon,alt,heading". Must match the
    /// --home flag passed to arducopter so the bridge can convert GPS LLA
    /// to NED position relative to the same origin.
    #[arg(long, default_value = "37.7749,-122.4194,10,270", env = "AP_HOME")]
    pub home: String,
}

/// Parsed home location for NED coordinate conversions.
#[derive(Debug, Clone, Copy)]
pub struct HomeLocation {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f64,
}

impl HomeLocation {
    pub fn parse(s: &str) -> anyhow::Result<Self> {
        let parts: Vec<&str> = s.split(',').collect();
        if parts.len() < 3 {
            anyhow::bail!("home location needs at least lat,lon,alt -- got {:?}", s);
        }
        Ok(Self {
            lat_deg: parts[0].parse()?,
            lon_deg: parts[1].parse()?,
            alt_m: parts[2].parse()?,
        })
    }
}
