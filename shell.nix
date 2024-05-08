{ pkgs ? import (fetchTarball "https://github.com/NixOS/nixpkgs/archive/refs/tags/23.05.tar.gz") {} }:
  let 
    my-python = pkgs.python3;
    # jinja2 is required to build Coyote. 
    python-with-my-packages = my-python.withPackages (p: with p; 
    [
      jinja2
    ]);
  in
  pkgs.mkShell {
    # nativeBuildInputs is usually what you want -- tools you need to run
    nativeBuildInputs = with pkgs.buildPackages; [ 
      cmake
      boost
      coreutils
      git
      nasm
      binutils
      usbutils
      pciutils
      pahole
      gcc
      zlib.dev
      openssl.dev
      scc

      python3
      python-with-my-packages
    ];

    shellHook = ''
      export NIXKERNEL=$(find /nix -type d -regex ".*linux-$(uname -r)-dev")
      export PYTHONPATH=${python-with-my-packages}/${python-with-my-packages.sitePackages}
    '';
}

