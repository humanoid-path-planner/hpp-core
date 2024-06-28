{
  lib,
  stdenv,
  cmake,
  hpp-constraints,
  proxsuite,
}:

stdenv.mkDerivation {
  pname = "hpp-core";
  version = "5.0.0";

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

  strictDeps = true;

  nativeBuildInputs = [ cmake ];
  propagatedBuildInputs = [
    hpp-constraints
    proxsuite
  ];

  doCheck = true;

  meta = {
    description = "The core algorithms of the Humanoid Path Planner framework";
    homepage = "https://github.com/humanoid-path-planner/hpp-core";
    license = lib.licenses.bsd2;
    maintainers = [ lib.maintainers.nim65s ];
  };
}
