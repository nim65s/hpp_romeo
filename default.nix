{
  lib,
  cmake,
  hpp-corbaserver,
  pkg-config,
  python3Packages
}:

python3Packages.buildPythonPackage {
  pname = "hpp-romeo";
  version = "5.0.0";
  pyproject = false;

  src = lib.fileset.toSource {
    root = ./.;
    fileset = lib.fileset.unions [
      ./CMakeLists.txt
      ./doc
      ./launch
      ./package.xml
      ./rviz
      ./scripts
      ./src
    ];
  };

  strictDeps = true;

  nativeBuildInputs = [
    cmake
    pkg-config
  ];
  propagatedBuildInputs = [ hpp-corbaserver ];

  meta = {
    description = "Python and ros launch files for Romeo robot in hpp";
    homepage = "https://github.com/humanoid-path-planner/hpp_romeo";
    license = lib.licenses.bsd2;
    maintainers = [ lib.maintainers.nim65s ];
  };
}
