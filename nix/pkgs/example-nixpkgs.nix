# Example: Using a clean dependency from nixpkgs
#
# This file demonstrates how to expose and optionally configure
# a package that already exists in nixpkgs.
#
# In this case, we're using `btop` - a modern resource monitor.
# This pattern is useful when you want to:
#   - Include a nixpkgs package in your overlay
#   - Optionally wrap it with custom configuration
#   - Make it available to your NixOS modules
#
# Usage in overlay:
#   example-nixpkgs = final.callPackage ./nix/pkgs/example-nixpkgs.nix {};
#
{ lib
, btop
, writeShellScriptBin
}:

# For simple cases, you can just re-export the package directly:
# btop

# For more advanced cases, you might want to wrap it with defaults.
# Here's an example of creating a wrapper script with custom configuration:
writeShellScriptBin "aleph-monitor" ''
  # Launch btop with some sensible defaults for Aleph
  # This demonstrates how you can wrap a nixpkgs package
  exec ${btop}/bin/btop "$@"
''

# Alternative patterns you might use:
#
# 1. Just re-export the package (simplest):
#    btop
#
# 2. Override package options:
#    btop.override { }
#
# 3. Override derivation attributes:
#    btop.overrideAttrs (old: {
#      # modify attributes here
#    })

