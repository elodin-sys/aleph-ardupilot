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

    defaultsFile = mkOption {
      type = types.nullOr types.path;
      default = null;
      description = "ArduPilot parameter defaults file (passed via --defaults).";
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
      stopIfChanged = false;
      restartIfChanged = false;

      serviceConfig = {
        ExecStartPre = "${pkgs.coreutils}/bin/rm -f ${cfg.workingDirectory}/eeprom.bin";
        ExecStart = concatStringsSep " " ([
          "${pkgs.arducopter-sitl}/bin/arducopter"
          "--model" cfg.model
          "--home" cfg.homeLocation
          "--serial0" "mcast:"
        ] ++ (lib.optionals (cfg.defaultsFile != null) [
          "--defaults" "${cfg.defaultsFile}"
        ]) ++ cfg.extraFlags);

        WorkingDirectory = cfg.workingDirectory;
        Restart = "on-failure";
        RestartSec = "3s";
        StandardOutput = "journal";
        StandardError = "journal";
      };
    };

    system.activationScripts.restartArducopter = {
      text = ''
        if [ -d /run/systemd/system ] && \
           /run/current-system/sw/bin/systemctl is-active --quiet arducopter.service 2>/dev/null; then
          echo "restarting arducopter for fresh parameter state..."
          /run/current-system/sw/bin/systemctl daemon-reload
          /run/current-system/sw/bin/systemctl restart arducopter.service || true
        fi
      '';
    };
  };
}
