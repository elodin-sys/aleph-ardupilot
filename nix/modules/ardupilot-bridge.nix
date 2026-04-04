{ config, lib, pkgs, ... }:

with lib;

let
  cfg = config.services.ardupilot-bridge;
in
{
  options.services.ardupilot-bridge = {
    enable = mkEnableOption "ArduPilot bridge (Elodin-DB sensors to ArduPilot ExternalAHRS)";

    elodinAddr = mkOption {
      type = types.str;
      default = "127.0.0.1:2240";
      description = "Elodin-DB TCP address.";
    };

    controlPort = mkOption {
      type = types.int;
      default = 9002;
      description = "ArduPilot servo output UDP port.";
    };

    numMotors = mkOption {
      type = types.int;
      default = 4;
      description = "Number of motor channels.";
    };

    canInterface = mkOption {
      type = types.str;
      default = "";
      description = "SocketCAN interface for DroneCAN ESC output (empty = disabled).";
    };
  };

  config = mkIf cfg.enable {
    systemd.services.ardupilot-bridge = {
      description = "ArduPilot Bridge - Elodin-DB to ArduPilot ExternalAHRS";
      after = [ "network.target" "elodin-db-default.service" "arducopter.service" ];
      wantedBy = [ "multi-user.target" ];
      stopIfChanged = false;
      restartIfChanged = false;

      environment = {
        ELODIN_DB_ADDR = cfg.elodinAddr;
        AP_CONTROL_PORT = toString cfg.controlPort;
        NUM_MOTORS = toString cfg.numMotors;
        CAN_INTERFACE = cfg.canInterface;
        AP_HOST = "127.0.0.1";
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

    system.activationScripts.restartArdupilotBridge = {
      text = ''
        if [ -d /run/systemd/system ] && \
           /run/current-system/sw/bin/systemctl is-active --quiet ardupilot-bridge.service 2>/dev/null; then
          echo "restarting ardupilot-bridge for fresh state..."
          /run/current-system/sw/bin/systemctl daemon-reload
          /run/current-system/sw/bin/systemctl restart ardupilot-bridge.service || true
        fi
      '';
    };
  };
}
