# Example: Packaging a local Python application
#
# This file demonstrates how to package a Python application from your
# local source directory. This pattern is useful for:
#   - Your own applications that live in the repo
#   - Custom tools specific to your Aleph deployment
#   - Services that will run as systemd units
#
# The source lives in src/hello-service/ and this package makes it
# available as an executable in the Nix store.
#
# Usage in overlay:
#   hello-service = final.callPackage ./nix/pkgs/hello-service.nix {
#     src = ./src/hello-service;
#   };
#
{ lib
, stdenv
, python3
, makeWrapper
, src
}:

let
  pythonEnv = python3.withPackages (ps: [
    # Add any Python dependencies your application needs here
    # ps.requests
    # ps.numpy
  ]);
in
stdenv.mkDerivation rec {
  pname = "hello-service";
  version = "1.0.0";

  inherit src;

  nativeBuildInputs = [ makeWrapper ];

  # No build step needed for a simple Python script
  dontBuild = true;

  installPhase = ''
    runHook preInstall

    # Create output directories
    mkdir -p $out/bin
    mkdir -p $out/lib/hello-service

    # Copy the Python source
    cp main.py $out/lib/hello-service/

    # Create the executable wrapper
    makeWrapper ${pythonEnv}/bin/python3 $out/bin/hello-service \
      --add-flags "$out/lib/hello-service/main.py"

    runHook postInstall
  '';

  meta = with lib; {
    description = "Example Hello World service for Aleph";
    longDescription = ''
      A minimal Python daemon that demonstrates how to create and deploy
      a systemd service on Aleph. It periodically logs a configurable
      message and handles graceful shutdown.
    '';
    license = licenses.mit;
    platforms = platforms.linux;
  };
}

# Alternative pattern using buildPythonApplication:
#
# python3.pkgs.buildPythonApplication rec {
#   pname = "hello-service";
#   version = "1.0.0";
#   src = ./../../src/hello-service;
#   format = "other";
#
#   installPhase = ''
#     mkdir -p $out/bin
#     cp main.py $out/bin/hello-service
#     chmod +x $out/bin/hello-service
#   '';
# }

