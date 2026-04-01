# Aleph ArduPilot

ArduCopter on the [Elodin Aleph](https://github.com/elodin-sys/elodin/tree/main/aleph) flight computer with a Rust sensor bridge (Elodin-DB to ArduPilot SITL) and CAN ESC output. See [ARCHITECTURE.md](ARCHITECTURE.md) for design details.

## Prerequisites

- [Determinate Systems Nix](https://install.determinate.systems/nix)
- An Aleph flight computer with the base NixOS image flashed
- Network connectivity to the Aleph (WiFi or USB ethernet)

## SSH Setup

Copy the repo's SSH key and configure `~/.ssh/config` for streamlined access:

```bash
cp ssh/aleph-key ~/.ssh/aleph-key
chmod 600 ~/.ssh/aleph-key
```

Add to `~/.ssh/config`:

```
Host aleph
    HostName <aleph-ip>
    User aleph
    IdentityFile ~/.ssh/aleph-key
```

The first deployment to a fresh Aleph requires `root`/`root` (password auth). After that first deploy, `flake.nix` installs the matching public key for the `aleph` user, and subsequent deploys can use `-u aleph` or the SSH config above.

## Getting Started

**1. Enter the dev shell**

```bash
nix develop --accept-flake-config
```

**2. Configure for your setup**

Edit `flake.nix` -- home location, GPS model, WiFi, GCS IP, etc.

**3. Deploy to Aleph**

```bash
# Default (full onboard stack):
./deploy.sh -h <aleph-ip> -u root

# Sim-HITL (physics sim on laptop):
./deploy.sh -c sim-hitl -h <aleph-ip> -u root
```

**4. Run the simulation** (sim-hitl only)

```bash
elodin run sim/sim-hitl/main.py
```

**5. Connect Elodin Editor**

```bash
elodin editor <aleph-ip>:2240
```

Confirm sensor data and motor commands are flowing.

**6. Connect QGroundControl**

Launch QGC and connect via MAVLink (UDP, auto-detected on the GCS IP configured in `flake.nix`).

**7. Fly**

Arm and fly -- the workflow is identical whether running sim-hitl or real hardware.

## Troubleshooting

SSH into the Aleph and check service status:

```bash
ssh aleph   # or: ssh -i ./ssh/aleph-key root@<aleph-ip>

systemctl status arducopter ardupilot-bridge sensor-fw
journalctl -u ardupilot-bridge -f
```

**Serial console** (if no network): connect FTDI USB, then `screen /dev/tty.usbserial-* 115200`, login as `root`/`root`.

**WiFi setup** (on Aleph): `iwctl` then `station wlan0 connect "YourNetwork"`.

## Links

- [Elodin](https://github.com/elodin-sys/elodin) / [Docs](https://docs.elodin.systems)
- [ArduPilot](https://ardupilot.org/dev/)
- [NixOS](https://nixos.org/manual/nixos/stable/)

## License

Apache-2.0. See [LICENSE](LICENSE).
