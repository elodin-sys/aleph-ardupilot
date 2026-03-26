# Example: Fetching and building a dependency from source
#
# This file demonstrates how to fetch source code from GitHub
# and build it using Nix. This pattern is useful when:
#   - A package isn't in nixpkgs
#   - You need a specific version or fork
#   - You want to apply custom patches
#
# In this example, we build `lazygit` - a simple terminal UI for git.
# It's a Go application, so we use buildGoModule.
#
# Usage in overlay:
#   example-from-source = final.callPackage ./nix/pkgs/example-from-source.nix {};
#
{ lib
, buildGoModule
, fetchFromGitHub
}:

buildGoModule rec {
  pname = "lazygit";
  version = "0.44.1";

  src = fetchFromGitHub {
    owner = "jesseduffield";
    repo = "lazygit";
    rev = "v${version}";
    hash = "sha256-BP5PMgRq8LHLuUYDrWaX1PgfT9VEhj3xeLE2aDMAPF0=";
  };

  # Set to null when the repository includes a vendor folder
  vendorHash = null;

  # Skip tests that require git repository
  doCheck = false;

  # Build flags to embed version info
  ldflags = [
    "-s"
    "-w"
    "-X main.version=${version}"
    "-X main.buildSource=nix"
  ];

  meta = with lib; {
    description = "Simple terminal UI for git commands";
    homepage = "https://github.com/jesseduffield/lazygit";
    license = licenses.mit;
    platforms = platforms.linux;
  };
}

# Other common patterns for fetching from source:
#
# 1. For Python packages:
#    python3.pkgs.buildPythonPackage rec {
#      src = fetchFromGitHub { ... };
#      ...
#    }
#
# 2. For Rust packages:
#    rustPlatform.buildRustPackage rec {
#      src = fetchFromGitHub { ... };
#      cargoHash = "sha256-...";
#      ...
#    }
#
# 3. For C/C++ with CMake:
#    stdenv.mkDerivation rec {
#      src = fetchFromGitHub { ... };
#      nativeBuildInputs = [ cmake ];
#      ...
#    }
#
# 4. For generic Makefiles:
#    stdenv.mkDerivation rec {
#      src = fetchFromGitHub { ... };
#      buildPhase = "make";
#      installPhase = "make install PREFIX=$out";
#    }
#
# Finding the hash:
#   - First, set hash = "";
#   - Run the build, it will fail and show the expected hash
#   - Or use: nix-prefetch-github <owner> <repo> --rev <version>

