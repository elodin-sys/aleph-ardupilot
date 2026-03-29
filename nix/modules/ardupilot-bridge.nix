{ config, lib, pkgs, ... }:

with lib;

let
  cfg = config.services.ardupilot-bridge;
in
{
  options.services.ardupilot-bridge = {
    enable = mkEnableOption "ArduPilot bridge (Elodin-DB sensors to ArduPilot SITL)";

    elodinAddr = mkOption {
      type = types.str;
      default = "127.0.0.1:2240";
      description = "Elodin-DB TCP address.";
    };

    controlPort = mkOption {
      type = types.int;
      default = 9002;
      description = "ArduPilot SITL JSON control interface UDP port.";
    };

    numMotors = mkOption {
      type = types.int;
      default = 4;
      description = "Number of motor channels.";
    };

    hitlPort = mkOption {
      type = types.int;
      default = 0;
      description = "HITL TCP listen port (0 = disabled).";
    };

    canInterface = mkOption {
      type = types.str;
      default = "";
      description = "SocketCAN interface for DroneCAN ESC output (empty = disabled).";
    };

    homeLocation = mkOption {
      type = types.str;
      default = config.services.arducopter.homeLocation;
      description = "Home location (lat,lon,alt,heading). Defaults to arducopter's homeLocation.";
    };
  };

  config = mkIf cfg.enable {
    systemd.services.ardupilot-bridge = {
      description = "ArduPilot Bridge - Elodin-DB to ArduPilot SITL";
      after = [ "network.target" "elodin-db-default.service" "arducopter.service" ];
      wantedBy = [ "multi-user.target" ];

      environment = {
        ELODIN_DB_ADDR = cfg.elodinAddr;
        AP_CONTROL_PORT = toString cfg.controlPort;
        NUM_MOTORS = toString cfg.numMotors;
        HITL_PORT = toString cfg.hitlPort;
        CAN_INTERFACE = cfg.canInterface;
        AP_HOME = cfg.homeLocation;
        RUST_LOG = "info";
      };

      serviceConfig = {
        ExecStart = "${pkgs.ardupilot-bridge}/bin/ardupilot-bridge";
        Restart = "on-failure";
        RestartSec = "3s";
        StandardOutput = "journal";
        StandardError = "journal";
      };
    };
  };
}
