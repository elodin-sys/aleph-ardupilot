# Aleph Template Project
#
# This flake demonstrates the recommended patterns for developing and deploying
# software to the Aleph flight computer using Nix and NixOS.
#
# Three example patterns are included:
#   1. Using packages from nixpkgs (example-nixpkgs)
#   2. Building packages from source (example-from-source)
#   3. Local Python application as systemd service (hello-service)
#
# Build command:
#   nix build --accept-flake-config .#packages.aarch64-linux.toplevel --show-trace
#
# Deploy command:
#   ./deploy.sh
#
{
  nixConfig = {
    extra-substituters = ["https://elodin-nix-cache.s3.us-west-2.amazonaws.com"];
    extra-trusted-public-keys = [
      "elodin-cache-1:vvbmIQvTOjcBjIs8Ri7xlT2I3XAmeJyF5mNlWB+fIwM="
    ];
  };

  inputs = {
    aleph.url = "github:elodin-sys/elodin?rev=1b2615f508c2296c49e803ae5f13b4a82713bea2&dir=aleph";
    elodin.url = "github:elodin-sys/elodin?rev=1b2615f508c2296c49e803ae5f13b4a82713bea2";
    flake-utils.follows = "aleph/flake-utils";
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
  #   elodin run examples/ardupilot-hitl/main.py
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
        echo "  elodin run sim/ardupilot-hitl/main.py      - HITL simulation"
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
    # Custom Overlay
    #
    # This overlay makes your custom packages available to the NixOS system.
    # Packages defined here can be used in environment.systemPackages or
    # referenced by modules.
    ###########################################################################
    overlays.default = final: prev: {
      # Pattern 1: Clean nixpkgs dependency
      example-nixpkgs = final.callPackage ./nix/pkgs/example-nixpkgs.nix {};

      # Pattern 2: Build from source
      example-from-source = final.callPackage ./nix/pkgs/example-from-source.nix {};

      # Pattern 3: Local Python application
      hello-service = final.callPackage ./nix/pkgs/hello-service.nix {
        src = ./src/hello-service;
      };

      # ArduPilot SITL binary (built from flake input with submodules)
      arducopter-sitl = final.callPackage ./nix/pkgs/arducopter.nix {
        src = ardupilot-src;
      };

      # ArduPilot bridge: Elodin-DB <-> ArduPilot SITL <-> CAN ESCs
      ardupilot-bridge = final.callPackage ./nix/pkgs/ardupilot-bridge.nix {
        src = ./src/ardupilot-bridge;
      };
    };

    ###########################################################################
    # NixOS Module
    #
    # This module configures your Aleph system. It imports:
    #   - Aleph hardware and base modules (required)
    #   - Your custom modules (for services you create)
    ###########################################################################
    nixosModules.default = {config, pkgs, lib, ...}: {
      imports = with aleph.nixosModules; [
        #######################################################################
        # Aleph Hardware Modules (required)
        #######################################################################
        jetpack   # Core module required for NVIDIA Jetpack/Orin support
        hardware  # Aleph-specific hardware, kernel, and device tree
        fs        # SD card image building support

        #######################################################################
        # Aleph Networking Modules (optional - enable as needed)
        #######################################################################
        usb-eth     # USB ethernet gadget for direct connection
        wifi        # WiFi support using iwd

        #######################################################################
        # Aleph Tooling Modules (recommended)
        #######################################################################
        aleph-setup   # First-boot setup wizard for WiFi and user config
        aleph-base    # Sensible default configuration for development
        aleph-dev     # Development packages (CUDA, OpenCV, git, etc.)

        #######################################################################
        # Aleph Default Flight Software Modules
        #######################################################################
        sensor-fw # full sensor firmware: streams IMU/mag/baro data to elodin-db at 1 Mbaud
        mekf # a basic attitude mekf that runs on the sensor data from the expansion board
        elodin-db # brings in elodin-db as a default service
        aleph-serial-bridge # pushes sensor data into elodin-db from the default expansion board firmware
        tegrastats-bridge # pushes telemetry form the Orin SoC into elodin-db (i.e cpu usage, temps, etc)

        #######################################################################
        # Your Custom Modules
        #######################################################################
        ./nix/modules/hello-service.nix
        ./nix/modules/arducopter.nix
        ./nix/modules/ardupilot-bridge.nix
        ./nix/modules/can.nix
      ];

      # Apply overlays (order matters!)
      # 1. jetpack: NVIDIA Jetpack packages
      # 2. aleph: Aleph-specific packages and device tree
      # 3. default: Your custom packages
      nixpkgs.overlays = [
        aleph.overlays.jetpack
        aleph.overlays.default
        overlays.default
      ];

      system.stateVersion = "25.11";
      i18n.supportedLocales = [(config.i18n.defaultLocale + "/UTF-8")];

      # Re-enable MAXN profile for full Orin NX performance.
      services.nvpmodel = {
        enable = true;
        profileNumber = 0;
      };

      # nvpmodel may require reboot when switching profiles; force non-interactive mode.
      systemd.services.nvpmodel.serviceConfig.ExecStart = lib.mkForce ''
        /run/current-system/sw/bin/nvpmodel --force -f /etc/nvpmodel.conf -m 0
      '';

      #########################################################################
      # Enable the Hello Service (Pattern 3 demonstration)
      #########################################################################
      services.hello-service = {
        enable = true;
        message = "Hello from Aleph Template Project!";
        interval = 30;
      };

      #########################################################################
      # ArduCopter SITL Flight Controller
      #########################################################################
      services.arducopter = {
        enable = true;
        model = "JSON";
        homeLocation = "37.7749,-122.4194,10,270";
        defaultsFile = ./src/ardupilot-defaults.param;
        extraFlags = [ "--serial0" "udpclient:192.168.4.182:14550" ];
      };

      #########################################################################
      # ArduPilot Bridge (Elodin-DB sensors -> ArduPilot -> CAN ESCs)
      #########################################################################
      services.ardupilot-bridge = {
        enable = true;
        hitlPort = 9100;
      };

      #########################################################################
      # System Packages
      #
      # Include packages you want available system-wide.
      # Your overlay packages are available via `pkgs.<name>`.
      #########################################################################
      environment.systemPackages = with pkgs; [
        # Pattern 1: nixpkgs wrapper package
        example-nixpkgs   # Available as 'aleph-monitor' command

        # Pattern 2: Built from source
        example-from-source  # Available as 'lazygit' command

        # The hello-service package is managed by its systemd service,
        # but you can also add it here if you want CLI access:
        # hello-service

        # Common development tools
        git
        vim
        tmux
        htop
        btop

        # Network utilities
        wget
        curl

        # Hardware debugging
        usbutils
        pciutils
        lshw

        # Python for scripting
        python3
      ];

      #########################################################################
      # User Configuration
      #
      # Configure your Aleph user account here.
      #########################################################################
      users.users.aleph = {
        isNormalUser = true;
        openssh.authorizedKeys.keys = [
          # Your SSH public key (from ssh/aleph-key.pub)
          "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIKtPjReJktl58C9GKjgl0pkUZ87XqpYKfOiSqXrhwoXq aleph-key"
        ];
        extraGroups = [
          "wheel"         # sudo access
          "dialout"       # Serial port access
          "video"         # Video device access
          "audio"         # Audio device access
          "networkmanager"
          "podman"        # Container support
        ];
        shell = "/run/current-system/sw/bin/bash";
      };

      #########################################################################
      # SSH Configuration
      #########################################################################
      services.openssh = {
        enable = true;
        settings = {
          PasswordAuthentication = true;
          PubkeyAuthentication = true;
          PermitRootLogin = "yes";
        };
      };

      #########################################################################
      # Security & Nix Settings
      #########################################################################
      security.sudo.wheelNeedsPassword = false;
      nix.settings.trusted-users = ["@wheel" "root" "ubuntu" "aleph"];
      networking.firewall.enable = false;
    };

    ###########################################################################
    # Sim-HITL NixOS Module
    #
    # Like the default module but without the local sensor stack.
    # The ardupilot-bridge connects to the laptop's Elodin-DB instead of a
    # local one.  Deploy with:
    #   ./deploy.sh -c sim-hitl -h 192.168.4.185 -u root
    ###########################################################################
    nixosModules.sim-hitl = {config, pkgs, lib, ...}: {
      imports = with aleph.nixosModules; [
        # Hardware (same as default)
        jetpack hardware fs
        usb-eth wifi
        aleph-setup aleph-base aleph-dev

        # NO sensor-fw, mekf, elodin-db, aleph-serial-bridge, tegrastats-bridge
        # Sensor data comes from the laptop simulation via Elodin-DB over TCP.

        # Custom modules
        ./nix/modules/hello-service.nix
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

      services.nvpmodel = { enable = true; profileNumber = 0; };
      systemd.services.nvpmodel.serviceConfig.ExecStart = lib.mkForce ''
        /run/current-system/sw/bin/nvpmodel --force -f /etc/nvpmodel.conf -m 0
      '';

      services.hello-service = {
        enable = true;
        message = "Aleph sim-HITL mode";
        interval = 30;
      };

      services.arducopter = {
        enable = true;
        model = "JSON";
        homeLocation = "37.7749,-122.4194,10,270";
        extraFlags = [ "--serial0" "udpclient:192.168.4.182:14550" ];
      };

      #########################################################################
      # ArduPilot Bridge -- points at the LAPTOP's Elodin-DB
      # Change this IP to your laptop's address on the shared network.
      #########################################################################
      services.ardupilot-bridge = {
        enable = true;
        elodinAddr = "192.168.4.182:2240";
        hitlPort = 9100;
      };

      environment.systemPackages = with pkgs; [
        git vim tmux htop btop wget curl
        usbutils pciutils lshw python3
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
    # NixOS Configurations
    #
    # Define different system configurations here. The 'default' configuration
    # is used by deploy.sh. You can add more for different setups.
    ###########################################################################
    nixosConfigurations = {
      default = nixpkgs.lib.nixosSystem {
        inherit system;
        modules = [nixosModules.default];
      };

      # Sim-HITL: bridge + ArduPilot on Aleph, physics sim on laptop
      sim-hitl = nixpkgs.lib.nixosSystem {
        inherit system;
        modules = [nixosModules.sim-hitl];
      };
    };

    ###########################################################################
    # Package Outputs
    #
    # These are the build targets available via `nix build`.
    ###########################################################################
    packages.aarch64-linux = {
      # SD card image for initial Aleph setup
      sdimage = aleph.packages.aarch64-linux.sdimage;

      # System toplevel - used by deploy.sh for OTA updates
      default = nixosConfigurations.default.config.system.build.toplevel;
      toplevel = nixosConfigurations.default.config.system.build.toplevel;
      sim-hitl = nixosConfigurations.sim-hitl.config.system.build.toplevel;
    };
  };
}
