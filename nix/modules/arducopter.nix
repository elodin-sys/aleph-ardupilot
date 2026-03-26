{ config, lib, pkgs, ... }:

with lib;

let
  cfg = config.services.arducopter;
in
{
  options.services.arducopter = {
    enable = mkEnableOption "ArduCopter SITL flight controller";

    homeLocation = mkOption {
      type = types.str;
      default = "37.7749,-122.4194,10,270";
      description = "Home location as lat,lon,alt,heading.";
      example = "47.3977,8.5456,500,90";
    };

    model = mkOption {
      type = types.str;
      default = "JSON";
      description = "Simulation model backend (JSON for external sensor bridge).";
    };

    extraFlags = mkOption {
      type = types.listOf types.str;
      default = [];
      description = "Additional command-line flags for arducopter.";
      example = [ "--speedup" "1" ];
    };

    workingDirectory = mkOption {
      type = types.str;
      default = "/var/lib/arducopter";
      description = "Working directory for ArduPilot (stores eeprom.bin, logs, terrain).";
    };
  };

  config = mkIf cfg.enable {
    systemd.tmpfiles.rules = [
      "d ${cfg.workingDirectory} 0755 root root -"
    ];

    systemd.services.arducopter = {
      description = "ArduCopter SITL Flight Controller";
      after = [ "network.target" ];
      wantedBy = [ "multi-user.target" ];

      serviceConfig = {
        # --serial0 mcast: uses UDP multicast for MAVLink, avoids blocking
        # on TCP accept that prevents the SITL loop from starting.
        ExecStart = concatStringsSep " " ([
          "${pkgs.arducopter-sitl}/bin/arducopter"
          "--model" cfg.model
          "--home" cfg.homeLocation
          "--serial0" "mcast:"
        ] ++ cfg.extraFlags);

        WorkingDirectory = cfg.workingDirectory;
        Restart = "on-failure";
        RestartSec = "3s";
        StandardOutput = "journal";
        StandardError = "journal";
      };
    };
  };
}
