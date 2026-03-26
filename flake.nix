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
    aleph.url = "github:elodin-sys/elodin?rev=80f226daf08f9fb9819adbe2e711e8eec2f83543&dir=aleph";
    flake-utils.follows = "aleph/flake-utils";
    nixpkgs.follows = "aleph/nixpkgs";
    self.submodules = true;
  };

  outputs = {
    nixpkgs,
    aleph,
    self,
    ...
  }: rec {
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
      # Wraps btop with a custom launcher script
      example-nixpkgs = final.callPackage ./nix/pkgs/example-nixpkgs.nix {};

      # Pattern 2: Build from source
      # Fetches and builds lazygit from GitHub
      example-from-source = final.callPackage ./nix/pkgs/example-from-source.nix {};

      # Pattern 3: Local Python application
      # Packages the hello-service from src/hello-service/
      hello-service = final.callPackage ./nix/pkgs/hello-service.nix {
        src = ./src/hello-service;
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
        interval = 30;  # Log every 30 seconds
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

      # Example: Add additional configurations for different use cases
      # minimal = nixpkgs.lib.nixosSystem {
      #   inherit system;
      #   modules = [nixosModules.minimal];
      # };
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
    };
  };
}
