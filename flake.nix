{
  description = "Python and ros launch files for Romeo robot in hpp";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        overrideAttrs.hpp-romeo = {
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
        };
      }
    );
}
