{
  description = "OxiFOC Dioxus Desktop / WebKitGTK experiment";
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/fe13242281a67641c0102cdb1087f0ebe586c7df";
  inputs.nixpkgs-webkit.url = "github:NixOS/nixpkgs/512527cdaf7eab2f9e4c5e6e3935c1648d156621";
  outputs = { nixpkgs, nixpkgs-webkit, ... }:
    let
      system = "x86_64-linux";
      webkitPkgs = import nixpkgs-webkit { inherit system; };
      pkgs = import nixpkgs { inherit system; overlays = [ (_: _: { webkitgtk_4_1 = webkitPkgs.webkitgtk_4_1; }) ]; };
    in {
      devShells.${system}.default = pkgs.mkShell {
        packages = with pkgs; [ rustup pkg-config ];
        buildInputs = with pkgs; [ openssl gtk3 webkitgtk_4_1 ];
        LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath (with pkgs; [ openssl gtk3 glib gdk-pixbuf cairo dbus webkitgtk_4_1 libsoup_3 stdenv.cc.cc.lib glibc ]);
      };
    };
}
