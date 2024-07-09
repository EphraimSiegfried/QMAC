{
  description = "A development shell for PlatformIO";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
        };
      in
      rec {
        flakedPkgs = pkgs;

        # enables use of `nix shell`
        devShell = pkgs.mkShell {
          # add things you want in your shell here
          buildInputs = with pkgs; [
            platformio
            avrdude
          ];
          shellHook = ''
            	  export PLATFORMIO_CORE_DIR=$PWD/.platformio;
            	  '';
        };
      }
    );
}
