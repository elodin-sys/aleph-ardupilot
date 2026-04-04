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
  pname = "arducopter-aleph";
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

    # Inject Aleph ExternalAHRS backend sources.
    cp ${../../src/ardupilot-custom/AP_ExternalAHRS_Aleph.h} \
      libraries/AP_ExternalAHRS/AP_ExternalAHRS_Aleph.h
    cp ${../../src/ardupilot-custom/AP_ExternalAHRS_Aleph.cpp} \
      libraries/AP_ExternalAHRS/AP_ExternalAHRS_Aleph.cpp

    # Inject Linux hwdef for Aleph board.
    mkdir -p libraries/AP_HAL_Linux/hwdef/aleph
    cp ${../../src/ardupilot-custom/hwdef-aleph/hwdef.dat} \
      libraries/AP_HAL_Linux/hwdef/aleph/hwdef.dat

    python3 - <<'PY'
from pathlib import Path

cfg = Path("libraries/AP_ExternalAHRS/AP_ExternalAHRS_config.h")
cfg_text = cfg.read_text()
cfg_block = """#ifndef AP_EXTERNAL_AHRS_ALEPH_ENABLED
#define AP_EXTERNAL_AHRS_ALEPH_ENABLED AP_EXTERNAL_AHRS_BACKEND_DEFAULT_ENABLED
#endif
"""
if "AP_EXTERNAL_AHRS_ALEPH_ENABLED" not in cfg_text:
    cfg_text = cfg_text.rstrip() + "\n\n" + cfg_block
cfg.write_text(cfg_text)

hdr = Path("libraries/AP_ExternalAHRS/AP_ExternalAHRS.h")
hdr_text = hdr.read_text()
needle = """#if AP_EXTERNAL_AHRS_SENSAITION_ENABLED
        SensAItion = 11,
#endif
"""
replacement = """#if AP_EXTERNAL_AHRS_SENSAITION_ENABLED
        SensAItion = 11,
#endif
#if AP_EXTERNAL_AHRS_ALEPH_ENABLED
        Aleph = 12,
#endif
"""
if "Aleph = 12" not in hdr_text:
    hdr_text = hdr_text.replace(needle, replacement)
hdr.write_text(hdr_text)

cpp = Path("libraries/AP_ExternalAHRS/AP_ExternalAHRS.cpp")
cpp_text = cpp.read_text()
if '#include "AP_ExternalAHRS_Aleph.h"' not in cpp_text:
    cpp_text = cpp_text.replace(
        '#include "AP_ExternalAHRS_SensAItion.h"\n',
        '#include "AP_ExternalAHRS_SensAItion.h"\n#include "AP_ExternalAHRS_Aleph.h"\n'
    )
cpp_text = cpp_text.replace(
    "0:None,1:VectorNav,2:MicroStrain5,5:InertialLabs,6:Trimble GSOF,7:MicroStrain7,8:SBG,11:SensAItion",
    "0:None,1:VectorNav,2:MicroStrain5,5:InertialLabs,6:Trimble GSOF,7:MicroStrain7,8:SBG,11:SensAItion,12:Aleph"
)
if "case DevType::Aleph:" not in cpp_text:
    cpp_text = cpp_text.replace(
        "#if AP_EXTERNAL_AHRS_SBG_ENABLED\n    case DevType::SBG:\n        backend = NEW_NOTHROW AP_ExternalAHRS_SBG(this, state);\n        return;\n#endif // AP_EXTERNAL_AHRS_SBG_ENABLED\n",
        "#if AP_EXTERNAL_AHRS_SBG_ENABLED\n    case DevType::SBG:\n        backend = NEW_NOTHROW AP_ExternalAHRS_SBG(this, state);\n        return;\n#endif // AP_EXTERNAL_AHRS_SBG_ENABLED\n\n#if AP_EXTERNAL_AHRS_ALEPH_ENABLED\n    case DevType::Aleph:\n        backend = NEW_NOTHROW AP_ExternalAHRS_Aleph(this, state);\n        return;\n#endif // AP_EXTERNAL_AHRS_ALEPH_ENABLED\n"
    )
cpp.write_text(cpp_text)

# --- Patch INS register_accel to auto-save device ID (like SITL does) ---
ins = Path("libraries/AP_InertialSensor/AP_InertialSensor.cpp")
ins_text = ins.read_text()
ins_text = ins_text.replace(
    '#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || (CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && AP_SIM_ENABLED)\n'
    '        // assume this is the same sensor and save its ID to allow seamless\n'
    '        // transition from when we didn\'t have the IDs.\n'
    '        _accel_id_ok[_accel_count] = true;\n'
    '        _accel_id(_accel_count).save();\n'
    '#endif',
    '#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || (CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && AP_SIM_ENABLED) || AP_EXTERNAL_AHRS_ALEPH_ENABLED\n'
    '        // assume this is the same sensor and save its ID to allow seamless\n'
    '        // transition from when we didn\'t have the IDs.\n'
    '        _accel_id_ok[_accel_count] = true;\n'
    '        _accel_id(_accel_count).save();\n'
    '#endif',
)

# --- Patch INS register_gyro to auto-save device ID ---
ins_text = ins_text.replace(
    '#if CONFIG_HAL_BOARD == HAL_BOARD_SITL\n'
    '    if (!saved) {\n'
    '        // assume this is the same sensor and save its ID to allow seamless\n'
    '        // transition from when we didn\'t have the IDs.\n'
    '        _gyro_id(_gyro_count).save();\n'
    '    }\n'
    '#endif',
    '#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || AP_EXTERNAL_AHRS_ALEPH_ENABLED\n'
    '    if (!saved) {\n'
    '        // assume this is the same sensor and save its ID to allow seamless\n'
    '        // transition from when we didn\'t have the IDs.\n'
    '        _gyro_id(_gyro_count).save();\n'
    '    }\n'
    '#endif',
)
ins.write_text(ins_text)

# --- Patch compass ExternalAHRS to save dev_id on probe ---
cmps = Path("libraries/AP_Compass/AP_Compass_ExternalAHRS.cpp")
cmps_text = cmps.read_text()
cmps_text = cmps_text.replace(
    'ret->set_external(true);\n    return ret;',
    'ret->set_external(true);\n    ret->save_dev_id();\n    return ret;',
)
cmps.write_text(cmps_text)
PY
  '';

  preConfigure = ''
    export PKGCONFIG="$PKG_CONFIG"
  '';

  configurePhase = ''
    runHook preConfigure
    ./waf configure --board aleph
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
    cp build/aleph/bin/arducopter $out/bin/
    runHook postInstall
  '';

  separateDebugInfo = true;
  stripAllList = [ "bin" ];

  meta = with lib; {
    description = "ArduPilot Copter Linux binary for Aleph flight computer";
    homepage = "https://ardupilot.org";
    license = licenses.gpl3Plus;
    platforms = platforms.linux;
  };
}
