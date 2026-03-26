{ lib
, rustPlatform
, pkg-config
, src
}:

rustPlatform.buildRustPackage {
  pname = "ardupilot-bridge";
  version = "0.1.0";

  inherit src;

  cargoLock = {
    lockFile = "${src}/Cargo.lock";
    allowBuiltinFetchGit = true;
  };

  nativeBuildInputs = [ pkg-config ];

  doCheck = false;

  meta = with lib; {
    description = "Bridge between Elodin-DB sensors and ArduPilot SITL with CAN ESC output";
    license = licenses.asl20;
    platforms = platforms.linux;
  };
}
