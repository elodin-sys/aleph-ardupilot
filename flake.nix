# Aleph ArduPilot
#
# NixOS-based flight software stack for the Elodin Aleph flight computer
# running ArduCopter Linux with a Rust sensor bridge (ardupilot-bridge).
#
# Two deployment configurations are provided:
#   - default:   Full onboard stack (local Elodin-DB, sensors, bridge, ArduCopter)
#   - sim-hitl:  Sim-in-the-loop (bridge points at laptop's Elodin-DB for physics sim)
#
# Build:   nix build --accept-flake-config .#packages.aarch64-linux.toplevel --show-trace
# Deploy:  ./deploy.sh -h <aleph-ip> -u root
#          ./deploy.sh -c sim-hitl -h <aleph-ip> -u root
#
{
  nixConfig = {
    extra-substituters = ["https://elodin-nix-cache.s3.us-west-2.amazonaws.com"];
    extra-trusted-public-keys = [
      "elodin-cache-1:vvbmIQvTOjcBjIs8Ri7xlT2I3XAmeJyF5mNlWB+fIwM="
    ];
  };

  inputs = {
    aleph.url = "github:elodin-sys/elodin?rev=aea84b785479779bae30a5ea6781768f5cc29aab&dir=aleph";
    elodin.url = "github:elodin-sys/elodin?rev=aea84b785479779bae30a5ea6781768f5cc29aab";
    nixpkgs.follows = "aleph/nixpkgs";

    ardupilot-src = {
      type = "git";
      url = "https://github.com/ArduPilot/ardupilot";
      rev = "7de88b5f5ab8e9d1d330af9a641f1cd46eba13ee";
      flake = false;
      submodules = true;
    };
  };

  outputs = {
    nixpkgs,
    aleph,
    elodin,
    ardupilot-src,
    flake-utils,
    self,
    ...
  }:
  ###########################################################################
  # Ground Station DevShell (runs on the developer's machine)
  #
  # Provides Elodin Editor/CLI, Elodin-DB, Elodin Python wheel, and
  # QGroundControl (Linux only) for ground station operations.
  #
  # Usage:
  #   nix develop --accept-flake-config
  #   elodin editor <aleph-ip>:2240
  #   elodin run sim/sim-hitl/main.py
  #   qgroundcontrol   # Linux only
  ###########################################################################
  flake-utils.lib.eachDefaultSystem (system: let
    pkgs = import nixpkgs { inherit system; };
  in {
    devShells.default = pkgs.mkShell {
      name = "aleph-ardupilot-gcs";
      packages = [
        elodin.packages.${system}.elodin-cli
        elodin.packages.${system}.elodin-db
        elodin.packages.${system}.elodin-py
        pkgs.gcc.cc.lib
        pkgs.gfortran.cc.lib
        pkgs.which
      ] ++ pkgs.lib.optionals pkgs.stdenv.isLinux [
        pkgs.qgroundcontrol
      ];
      shellHook = ''
        # Prevent uv/elodin from discovering parent-directory .venvs which
        # would use the wrong Python version and cause SRE module mismatch.
        unset VIRTUAL_ENV
        export UV_PYTHON="$(which python3)"

        echo "Aleph ArduPilot ground station shell"
        echo "  elodin editor <aleph-ip>:2240    - live telemetry viewer"
        echo "  elodin run sim/sim-hitl/main.py             - HITL simulation"
      '' + pkgs.lib.optionalString pkgs.stdenv.isLinux ''
        echo "  qgroundcontrol                   - GCS for ArduPilot"
      '' + pkgs.lib.optionalString pkgs.stdenv.isDarwin ''
        echo "  QGroundControl: install from https://docs.qgroundcontrol.com (not available via nix on macOS)"
      '';
    };
  })
  ###########################################################################
  # Aleph NixOS system configuration (aarch64-linux target)
  ###########################################################################
  // rec {
    system = "aarch64-linux";

    ###########################################################################
    # Overlay
    ###########################################################################
    overlays.default = final: prev: {
      arducopter-aleph = final.callPackage ./nix/pkgs/arducopter.nix {
        src = ardupilot-src;
      };

      ardupilot-bridge = final.callPackage ./nix/pkgs/ardupilot-bridge.nix {
        src = ./src/ardupilot-bridge;
      };
    };

    ###########################################################################
    # Common NixOS Module
    #
    # Shared configuration imported by every deployment variant.
    ###########################################################################
    nixosModules.common = {config, pkgs, lib, ...}: {
      imports = with aleph.nixosModules; [
        # Aleph hardware
        jetpack hardware fs
        usb-eth wifi
        aleph-setup aleph-base aleph-dev

        # Flight software
        stm
        mekf
        elodin-db
        aleph-serial-bridge
        tegrastats-bridge

        # Project modules
        ./nix/modules/arducopter.nix
        ./nix/modules/ardupilot-bridge.nix
        ./nix/modules/can.nix
      ];

      nixpkgs.overlays = [
        aleph.overlays.jetpack
        aleph.overlays.default
        overlays.default
      ];

      system.stateVersion = "25.11";
      i18n.supportedLocales = [(config.i18n.defaultLocale + "/UTF-8")];

      services.nvpmodel = {
        enable = true;
        profileNumber = 0;
      };

      # Force non-interactive mode for nvpmodel profile switching.
      systemd.services.nvpmodel.serviceConfig.ExecStart = lib.mkForce ''
        /run/current-system/sw/bin/nvpmodel --force -f /etc/nvpmodel.conf -m 0
      '';

      # Uncomment ONE line to enable GPS-disciplined timestamping:
      services.sensor-fw.gps.model = "m10q";   # SAM-M10Q (9600 baud)
      # services.sensor-fw.gps.model = "m9n";    # NEO-M9N / M9N-5883 (38400 baud)

      services.arducopter = {
        enable = true;
        clearEeprom = false;
        defaultsFile = ./src/ardupilot-defaults.param;
        extraFlags = [ "--serial0" "udp:192.168.4.64:14550" ];
      };

      environment.systemPackages = with pkgs; [
        git vim tmux htop btop
        wget curl
        usbutils pciutils lshw
        python3
      ];

      users.users.aleph = {
        isNormalUser = true;
        openssh.authorizedKeys.keys = [
          "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIKtPjReJktl58C9GKjgl0pkUZ87XqpYKfOiSqXrhwoXq aleph-key"
        ];
        extraGroups = [ "wheel" "dialout" "video" "audio" "networkmanager" "podman" ];
        shell = "/run/current-system/sw/bin/bash";
      };

      services.openssh = {
        enable = true;
        settings = {
          PasswordAuthentication = true;
          PubkeyAuthentication = true;
          PermitRootLogin = "yes";
        };
      };

      security.sudo.wheelNeedsPassword = false;
      nix.settings.trusted-users = ["@wheel" "root" "ubuntu" "aleph"];
      networking.firewall.enable = false;
    };

    ###########################################################################
    # Default -- full onboard stack
    #
    # ArduPilot bridge talks to the local Elodin-DB and exposes an HITL port
    # for optional external physics simulation.
    ###########################################################################
    nixosModules.default = {config, pkgs, lib, ...}: {
      imports = [ nixosModules.common ];

      services.ardupilot-bridge = {
        enable = true;
      };
    };

    ###########################################################################
    # Sim-HITL -- physics simulation on laptop
    #
    # The ardupilot-bridge connects to the laptop's Elodin-DB instead of the
    # local one.  Deploy with:
    #   ./deploy.sh -c sim-hitl -h <aleph-ip> -u root
    ###########################################################################
    nixosModules.sim-hitl = {config, pkgs, lib, ...}: {
      imports = [ nixosModules.common ];

      # Bridge points at the laptop's Elodin-DB.
      services.ardupilot-bridge = {
        enable = true;
        elodinAddr = "192.168.4.64:2240";
      };
    };

    ###########################################################################
    # NixOS Configurations
    ###########################################################################
    nixosConfigurations = {
      default = nixpkgs.lib.nixosSystem {
        inherit system;
        modules = [nixosModules.default];
      };

      sim-hitl = nixpkgs.lib.nixosSystem {
        inherit system;
        modules = [nixosModules.sim-hitl];
      };
    };

    ###########################################################################
    # Package Outputs
    ###########################################################################
    packages.aarch64-linux = {
      sdimage = aleph.packages.aarch64-linux.sdimage;

      default = nixosConfigurations.default.config.system.build.toplevel;
      toplevel = nixosConfigurations.default.config.system.build.toplevel;
      sim-hitl = nixosConfigurations.sim-hitl.config.system.build.toplevel;
    };
  };
}
