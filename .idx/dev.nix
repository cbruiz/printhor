# To learn more about how to use Nix to configure your environment
# see: https://developers.google.com/idx/guides/customize-idx-env
{ pkgs, ... }: {
  # Which nixpkgs channel to use.
  # channel = "stable-24.11"; # or "unstable"
  channel = "unstable";
  # Use https://search.nixos.org/packages to find packages
  packages = [
    pkgs.cargo
    pkgs.rustc
    pkgs.rustfmt
    pkgs.stdenv.cc
  ];
  # Sets environment variables in the workspace
  env = {
    RUST_SRC_PATH = "${pkgs.rustPlatform.rustLibSrc}";
  };
  idx = {
    # Search for the extensions you want on https://open-vsx.org/ and use "publisher.id"
    extensions = [
        "rust-lang.rust-analyzer"
        "serayuzgur.crates"
        "vadimcn.vscode-lldb"
        "edwinhuish.better-comments-next"
        "MoBalic.jetbrains-dark-theme"
    ];
    workspace = {
      onCreate = {
        # Open editors for the following files by default, if they exist:
        default.openFiles = ["printhor/src/bin/s_plot.rs"];
        # Post create hook to init git submodules:
        post-install = "git submodule update --init --recursive";
      };
    };
    # Enable previews and customize configuration
    previews = {};
  };
}