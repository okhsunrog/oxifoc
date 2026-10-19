{
  description = "Oxifoc desktop and firmware development environment";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs =
    { nixpkgs, ... }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
      ];
    in
    {
      devShells = nixpkgs.lib.genAttrs systems (
        system:
        let
          pkgs = import nixpkgs { inherit system; };
          libraries = with pkgs; [
            dbus
            fontconfig
            libxkbcommon
            openssl
            udev
            wayland
          ];
        in
        {
          default = pkgs.mkShell {
            # rustup honors the toolchains and targets already pinned per crate.
            packages = with pkgs; [
              rustup
              clang
              cmake
              ninja
              pkg-config
            ];
            buildInputs = libraries;
            LIBCLANG_PATH = "${pkgs.libclang.lib}/lib";
            # Slint/winit load graphics libraries dynamically at runtime.
            LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath (
              libraries
              ++ (with pkgs; [
                stdenv.cc.cc.lib
                libGL
                vulkan-loader
                libx11
                libxcursor
                libxi
                libxrandr
              ])
            );
          };
        }
      );
    };
}
