# Build and run PX4 1.16 SITL natively on NixOS for the local simulator.
#
#   nix-shell localsim/px4/shell.nix --run 'localsim/px4/build.sh'
#
# PX4 1.16 needs empy 3.x (nixpkgs ships 4.x) and pyros-genmsg (not in
# nixpkgs); both are pinned from PyPI here, with pyulog for the ULogs
# (tools/fit_attitude_lag.py). Python 3.12 keeps empy 3 happy.
# The same shell runs the simulator: python3 -m localsim.run --px4
{ pkgs ? import <nixpkgs> { } }:
let
  py = pkgs.python312;
  empy3 = py.pkgs.buildPythonPackage rec {
    pname = "empy";
    version = "3.3.4";
    format = "setuptools";
    src = pkgs.fetchurl {
      url = "https://files.pythonhosted.org/packages/source/e/empy/empy-${version}.tar.gz";
      sha256 = "1cq1izl6l87i5i3vj0jcqfksh10kpiwpr2m19vgpj530bdw4kb3k";
    };
    nativeBuildInputs = [ py.pkgs.setuptools ];
    doCheck = false;
  };
  genmsg = py.pkgs.buildPythonPackage rec {
    pname = "pyros-genmsg";
    version = "0.5.8";
    format = "setuptools";
    src = pkgs.fetchurl {
      url = "https://files.pythonhosted.org/packages/f8/ca/96c243af4feb684bbb0f4126e6b3d2d330cc935e6a3c31fb1d7194ef4729/pyros_genmsg-${version}.tar.gz";
      sha256 = "0y7l131lc77v0c1rhxza41cxnnxc7acfqzlqf84fdya0kiyv071w";
    };
    nativeBuildInputs = [ py.pkgs.setuptools ];
    doCheck = false;
  };
  pyulog = py.pkgs.buildPythonPackage rec {
    pname = "pyulog";
    version = "1.3.0";
    format = "pyproject";
    src = pkgs.fetchurl {
      url = "https://files.pythonhosted.org/packages/2a/20/ac8bb7a68cd60175f279bf07b85b0a5634d8f88a1e841228cf05b281ff9d/pyulog-${version}.tar.gz";
      sha256 = "162aq5pvcp8ijm9vy7w537a4fqg8k8xgb4ic1mjargfi851gzfjc";
    };
    nativeBuildInputs = [ py.pkgs.setuptools py.pkgs.setuptools-scm py.pkgs.wheel ];
    propagatedBuildInputs = [ py.pkgs.numpy ];
    doCheck = false;
  };
  python = py.withPackages (p: [
    empy3 genmsg pyulog
    p.jinja2 p.jsonschema p.kconfiglib p.pyyaml p.future p.packaging p.numpy
    p.setuptools p.toml p.six p.psutil p.pyserial p.lxml p.pymavlink
    p.pygments p.cerberus p.argcomplete p.requests p.sympy
    # The local simulator itself.
    p.scipy p.matplotlib
  ]);
in
pkgs.mkShell {
  packages = [ python pkgs.gcc pkgs.cmake pkgs.ninja pkgs.git pkgs.gnumake pkgs.which pkgs.bash ];
  # PX4's CMake picks the Python on PATH; make sure it is this one.
  shellHook = ''
    export PYTHON_EXECUTABLE=${python}/bin/python3
  '';
}
