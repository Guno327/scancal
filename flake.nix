{
  description = "scancal — flatbed scanner dimensional calibration";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      forAllSystems = nixpkgs.lib.genAttrs [ "x86_64-linux" "aarch64-linux" ];
    in
    {
      packages = forAllSystems (system:
        let
          pkgs = nixpkgs.legacyPackages.${system};
          python = pkgs.python3.withPackages (ps: [ ps.numpy ps.opencv4 ]);
        in
        rec {
          scancal = pkgs.stdenvNoCC.mkDerivation {
            pname = "scancal";
            version = "0.5.0";
            src = ./.;
            dontBuild = true;
            installPhase = ''
              install -Dm755 scancal.py $out/bin/scancal
              sed -i "1s|.*|#!${python}/bin/python3|" $out/bin/scancal
            '';
            meta = {
              description = "Flatbed scanner dimensional calibration via a 3D-printed plate";
              mainProgram = "scancal";
            };
          };
          default = scancal;
        });

      apps = forAllSystems (system: {
        default = {
          type = "app";
          program = nixpkgs.lib.getExe self.packages.${system}.default;
        };
      });
    };
}
