{
  description = "The core algorithms of the Humanoid Path Planner framework";

  inputs = {
    nixpkgs.url = "github:nim65s/nixpkgs/gepetto";
    flake-parts = {
      url = "github:hercules-ci/flake-parts";
      inputs.nixpkgs-lib.follows = "nixpkgs";
    };
    hpp-constraints = {
      url = "github:humanoid-path-planner/hpp-constraints/release/5.1.0";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-parts.follows = "flake-parts";
    };
  };

  outputs =
    inputs@{ flake-parts, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      imports = [ ];
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "aarch64-darwin"
        "x86_64-darwin"
      ];
      perSystem =
        {
          self',
          pkgs,
          system,
          ...
        }:
        {
          packages.default = pkgs.callPackage ./. {
            hpp-constraints = inputs.hpp-constraints.packages.${system}.default;
          };
          devShells.default = pkgs.mkShell { inputsFrom = [ self'.packages.default ]; };
        };
    };
}
