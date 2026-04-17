{
  description = "The core algorithms of the Humanoid Path Planner framework";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        overrideAttrs.hpp-core = {
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
      }
    );
}
