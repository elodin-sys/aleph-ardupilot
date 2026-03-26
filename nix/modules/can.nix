{ config, lib, pkgs, ... }:

with lib;

let
  cfg = config.services.can-setup;
in
{
  options.services.can-setup = {
    enable = mkEnableOption "SocketCAN interface setup for Orin NX mttcan";

    interface = mkOption {
      type = types.str;
      default = "can0";
      description = "CAN interface name.";
    };

    bitrate = mkOption {
      type = types.int;
      default = 1000000;
      description = "CAN bus bitrate in bits/second.";
    };
  };

  config = mkIf cfg.enable {
    boot.kernelModules = [ "mttcan" ];

    environment.systemPackages = with pkgs; [
      can-utils
    ];

    systemd.services.can-setup = {
      description = "Configure SocketCAN ${cfg.interface} interface";
      after = [ "sys-subsystem-net-devices-${cfg.interface}.device" ];
      wants = [ "sys-subsystem-net-devices-${cfg.interface}.device" ];
      wantedBy = [ "multi-user.target" ];

      serviceConfig = {
        Type = "oneshot";
        RemainAfterExit = true;
        ExecStart = pkgs.writeShellScript "can-setup" ''
          ${pkgs.iproute2}/bin/ip link set ${cfg.interface} type can bitrate ${toString cfg.bitrate}
          ${pkgs.iproute2}/bin/ip link set ${cfg.interface} up
        '';
        ExecStop = "${pkgs.iproute2}/bin/ip link set ${cfg.interface} down";
      };
    };
  };
}
