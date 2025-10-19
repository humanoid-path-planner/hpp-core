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
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = import inputs.systems;
      imports = [
        inputs.gepetto.flakeModule
        {
          gepetto-pkgs.overlays = [
            (final: prev: {
              hpp-constraints = prev.hpp-constraints.overrideAttrs {
                src = final.fetchFromGitHub {
                  owner = "humanoid-path-planner";
                  repo = "hpp-constraints";
                  rev = "3da47f6366e2729a84b9ebbb3199fb42c53faf8b"; # pr/246 merge commit
                  hash = "sha256-jZSki9Lbs3/nNUc556sA66x5vwdcTBKWnBrRpezAf5Q=";
                };
              };
            })
          ];
        }
      ];
      perSystem =
        {
          lib,
          pkgs,
          self',
          ...
        }:
        {
          packages = {
            default = self'.packages.hpp-core;
            hpp-core = pkgs.hpp-core.overrideAttrs {
              patches = [ ]; # TODO: remove on next release
              nativeCheckInputs = [ pkgs.ctestCheckHook ];
              disabledTests = [ "test-path-extraction" ]; # TODO
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
        };
    };
}
