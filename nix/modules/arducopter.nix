{ config, lib, pkgs, ... }:

with lib;

let
  cfg = config.services.arducopter;
in
{
  options.services.arducopter = {
    enable = mkEnableOption "ArduCopter Linux flight controller";

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

    clearEeprom = mkOption {
      type = types.bool;
      default = true;
      description = "Delete eeprom.bin on every start (true = clean param slate, false = persist calibration data).";
    };
  };

  config = mkIf cfg.enable {
    systemd.tmpfiles.rules = [
      "d ${cfg.workingDirectory} 0755 root root -"
    ];

    systemd.services.arducopter = {
      description = "ArduCopter Linux Flight Controller";
      after = [ "network.target" ];
      wantedBy = [ "multi-user.target" ];
      stopIfChanged = false;
      restartIfChanged = false;

      serviceConfig = {
        ExecStartPre = lib.mkIf cfg.clearEeprom "${pkgs.coreutils}/bin/rm -f ${cfg.workingDirectory}/ArduCopter.stg";
        ExecStart = concatStringsSep " " ([
          "${pkgs.arducopter-aleph}/bin/arducopter"
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
