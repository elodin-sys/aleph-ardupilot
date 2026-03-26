# Example: NixOS module with systemd service
#
# This module demonstrates how to create a configurable NixOS service
# that deploys your application as a systemd unit. This pattern shows:
#   - Declaring module options with mkOption
#   - Creating systemd service configurations
#   - Passing configuration to your application via environment variables
#   - Proper service lifecycle management
#
# Usage in your NixOS configuration:
#   services.hello-service = {
#     enable = true;
#     message = "Hello from my Aleph!";
#     interval = 30;
#   };
#
{ config, lib, pkgs, ... }:

with lib;

let
  cfg = config.services.hello-service;
in
{
  #############################################################################
  # Module Options
  #############################################################################
  options.services.hello-service = {
    enable = mkEnableOption "Hello Service example daemon";

    message = mkOption {
      type = types.str;
      default = "Hello from Aleph!";
      description = "The message to log periodically.";
      example = "Aleph is running smoothly!";
    };

    interval = mkOption {
      type = types.int;
      default = 10;
      description = "Interval in seconds between messages.";
      example = 30;
    };

    user = mkOption {
      type = types.str;
      default = "hello-service";
      description = "User account under which the service runs.";
    };

    group = mkOption {
      type = types.str;
      default = "hello-service";
      description = "Group under which the service runs.";
    };
  };

  #############################################################################
  # Module Implementation
  #############################################################################
  config = mkIf cfg.enable {
    # Create the service user and group
    users.users.${cfg.user} = mkIf (cfg.user == "hello-service") {
      isSystemUser = true;
      group = cfg.group;
      description = "Hello Service daemon user";
    };

    users.groups.${cfg.group} = mkIf (cfg.group == "hello-service") {};

    # Define the systemd service
    systemd.services.hello-service = {
      description = "Hello Service - Example Aleph Daemon";
      
      # Start after network is available (typical for most services)
      after = [ "network.target" ];
      
      # Enable the service to start on boot
      wantedBy = [ "multi-user.target" ];

      # Pass configuration via environment variables
      environment = {
        HELLO_MESSAGE = cfg.message;
        HELLO_INTERVAL = toString cfg.interval;
      };

      serviceConfig = {
        # The executable to run
        ExecStart = "${pkgs.hello-service}/bin/hello-service";
        
        # Run as the configured user
        User = cfg.user;
        Group = cfg.group;
        
        # Restart policy
        Restart = "on-failure";
        RestartSec = "5s";
        
        # Security hardening (optional but recommended)
        NoNewPrivileges = true;
        ProtectSystem = "strict";
        ProtectHome = true;
        PrivateTmp = true;
        
        # Logging - stdout/stderr go to journald automatically
        StandardOutput = "journal";
        StandardError = "journal";
      };
    };
  };
}

# Tips for systemd services:
#
# 1. View logs:
#    journalctl -u hello-service -f
#
# 2. Check status:
#    systemctl status hello-service
#
# 3. Restart after config changes:
#    systemctl restart hello-service
#
# 4. Common serviceConfig options:
#    - Type = "simple" | "forking" | "oneshot" | "notify"
#    - WorkingDirectory = "/path/to/dir"
#    - EnvironmentFile = "/path/to/env"
#    - ExecStartPre = "command to run before start"
#    - ExecStopPost = "command to run after stop"
#    - TimeoutStartSec = "30s"
#    - WatchdogSec = "60s"  (for Type=notify)

