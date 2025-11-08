{
  description = "The core algorithms of the Humanoid Path Planner framework";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    flake-parts.follows = "gepetto/flake-parts";
    nixpkgs.follows = "gepetto/nixpkgs";
    nix-ros-overlay.follows = "gepetto/nix-ros-overlay";
    systems.follows = "gepetto/systems";
    treefmt-nix.follows = "gepetto/treefmt-nix";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, self, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.gepetto.flakeModule
          { gepetto-pkgs.overlays = [ self.overlays.default ]; }
        ];
        flake.overlays.default = _final: prev: {
          hpp-core = prev.hpp-core.overrideAttrs {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = lib.fileset.unions [
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
          };
        };
        perSystem =
          { pkgs, self', ... }:
          {
            packages = {
              default = self'.packages.hpp-core;
              hpp-core = pkgs.hpp-core;
            };
          };
      }
    );
}
