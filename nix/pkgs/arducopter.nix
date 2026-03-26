{ lib
, stdenv
, buildPackages
, pkg-config
, which
, gawk
, python311
, writers
, src
}:

let
  fake-git = import ./fake-git.nix { inherit writers; };

  # ArduPilot requires empy 3.x; nixpkgs ships 4.x which is incompatible.
  empy3 = buildPackages.python311.pkgs.empy.overrideAttrs (old: rec {
    version = "3.3.4";
    src = buildPackages.python311.pkgs.fetchPypi {
      pname = "empy";
      inherit version;
      hash = "sha256-c6xJeFtgFHnfTqGKfHm8EwSop8NMArlHLPEgauiPAbM=";
    };
  });

  pythonBuildEnv = buildPackages.python311.withPackages (ps: [
    ps.future
    ps.pyserial
    empy3
    ps.pexpect
    ps.setuptools
  ]);
in
stdenv.mkDerivation rec {
  pname = "arducopter-sitl";
  version = "4.6-dev";

  inherit src;

  nativeBuildInputs = [
    pkg-config
    which
    gawk
    fake-git
    pythonBuildEnv
  ];

  env.NIX_CFLAGS_COMPILE = "-Wno-error=maybe-uninitialized";

  postPatch = ''
    patchShebangs waf Tools modules/waf
  '';

  preConfigure = ''
    export PKGCONFIG="$PKG_CONFIG"
  '';

  configurePhase = ''
    runHook preConfigure
    ./waf configure --board sitl
    runHook postConfigure
  '';

  buildPhase = ''
    runHook preBuild
    ./waf copter
    runHook postBuild
  '';

  installPhase = ''
    runHook preInstall
    mkdir -p $out/bin
    cp build/sitl/bin/arducopter $out/bin/
    runHook postInstall
  '';

  separateDebugInfo = true;
  stripAllList = [ "bin" ];

  meta = with lib; {
    description = "ArduPilot Copter SITL binary for Aleph flight computer";
    homepage = "https://ardupilot.org";
    license = licenses.gpl3Plus;
    platforms = platforms.linux;
  };
}
