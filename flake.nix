{
  description = "The core algorithms of the Humanoid Path Planner framework";

  inputs = {
    nixpkgs.url = "github:gepetto/nixpkgs";
    flake-parts = {
      url = "github:hercules-ci/flake-parts";
      inputs.nixpkgs-lib.follows = "nixpkgs";
    };
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "aarch64-darwin"
        "x86_64-darwin"
      ];
      perSystem =
        { pkgs, self', ... }:
        {
          devShells.default = pkgs.mkShell { inputsFrom = [ self'.packages.default ]; };
          packages = {
            default = self'.packages.hpp-core;
            hpp-core = pkgs.hpp-core.overrideAttrs (_: {
              # TODO: remove this after next release
              patches = [];
              src = pkgs.lib.fileset.toSource {
                root = ./.;
                fileset = pkgs.lib.fileset.unions [
                  ./cmake-modules
                  ./CMakeLists.txt
                  ./doc
                  ./include
                  ./package.xml
                  ./plugins
                  ./src
                  ./tests
                ];
              };
            });
          };
        };
    };
}
