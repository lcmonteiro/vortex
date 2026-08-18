#!/usr/bin/env bash
#
# setup.sh — prepare the local environment and build + run the vortex tests.
#
# Mirrors the workflow documented in README.md:
#   * ensures the build prerequisites are present (CMake, a C++23 compiler,
#     git, and the LAPACK/BLAS development libraries used by Blaze),
#   * configures the CMake project (Blaze and GoogleTest are fetched
#     automatically via FetchContent),
#   * builds everything including the tests,
#   * runs the test suite with ctest.
#
# Configuration via environment variables:
#   BUILD_DIR=build       Build directory.
#   BUILD_TYPE=Release    CMake build type.
#   JOBS=<nproc>          Parallel build jobs.
#   INSTALL_DEPS=auto     auto | yes | no  (system package installation).
#
# Usage:
#   ./setup.sh [--debug|--release] [--build-type=<Type>]
#
# CLI flags take precedence over BUILD_TYPE.
#
set -euo pipefail

# --- Resolve the repository root (directory containing this script) ----------
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

BUILD_DIR="${BUILD_DIR:-build}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 2)}"
INSTALL_DEPS="${INSTALL_DEPS:-auto}"

log()  { printf '\033[1;34m[setup]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[setup]\033[0m %s\n' "$*" >&2; }
err()  { printf '\033[1;31m[setup]\033[0m %s\n' "$*" >&2; }
have() { command -v "$1" >/dev/null 2>&1; }

usage() {
  cat <<'EOF'
Usage: ./setup.sh [--debug|--release] [--build-type=<Type>]

Options:
  --debug               Shortcut for --build-type=Debug
  --release             Shortcut for --build-type=Release
  --build-type=<Type>   Explicit CMake build type (Debug, Release, RelWithDebInfo, MinSizeRel)
  -h, --help            Show this help message and exit
EOF
}

parse_args() {
  while [ "$#" -gt 0 ]; do
    case "$1" in
      --debug) BUILD_TYPE="Debug" ;;
      --release) BUILD_TYPE="Release" ;;
      --build-type=*) BUILD_TYPE="${1#*=}" ;;
      -h|--help) usage; exit 0 ;;
      *) err "Unknown argument: $1"; usage; exit 2 ;;
    esac
    shift
  done
}

# --- 1. Dependencies ---------------------------------------------------------
maybe_sudo() {
  if [ "$(id -u)" -eq 0 ]; then "$@"; else sudo "$@"; fi
}

install_apt_deps() {
  local pkgs=(build-essential cmake git liblapack-dev libblas-dev)
  log "Installing dependencies via apt: ${pkgs[*]}"
  maybe_sudo apt-get update -y
  maybe_sudo apt-get install -y "${pkgs[@]}"
}

install_termux_deps() {
  local pkgs=(cmake git clang blas-openblas)
  log "Installing dependencies via pkg: ${pkgs[*]}"
  pkg install -y "${pkgs[@]}"
}

# Whether liblapack/libblas (or an equivalent like openblas) are already
# resolvable on the linker's default search paths. This is separate from
# the compiler/cmake/git check below -- those being present says nothing
# about whether the math libraries this project links against are.
have_blas_lapack() {
  if have ldconfig; then
    if ldconfig -p 2>/dev/null | grep -qE 'lib(blas|openblas)\.so' && \
       ldconfig -p 2>/dev/null | grep -qE 'lib(lapack|openblas)\.so'; then
      return 0
    fi
  fi
  # ldconfig isn't available (Termux, some minimal images) -- fall back to
  # checking common lib directories directly, including Termux's $PREFIX.
  local dir found_blas=0 found_lapack=0 name
  for dir in /usr/lib /usr/lib64 /usr/local/lib "${PREFIX:-}/lib" /lib/*-linux-gnu; do
    [ -d "$dir" ] || continue
    for name in libblas.so libopenblas.so; do
      compgen -G "$dir/${name}*" >/dev/null 2>&1 && found_blas=1
    done
    for name in liblapack.so libopenblas.so; do
      compgen -G "$dir/${name}*" >/dev/null 2>&1 && found_lapack=1
    done
  done
  [ "$found_blas" -eq 1 ] && [ "$found_lapack" -eq 1 ]
}

ensure_deps() {
  case "$INSTALL_DEPS" in
    no)
      log "Skipping dependency installation (INSTALL_DEPS=no)."
      ;;
    yes)
      if [ -n "${PREFIX:-}" ] && [[ "$PREFIX" == *com.termux* ]]; then install_termux_deps
      elif have apt-get; then install_apt_deps
      else err "No supported package manager found (apt-get or Termux's pkg); please install the prerequisites manually."; fi
      ;;
    auto)
      local have_tools=0
      have cmake && have git && { have g++ || have clang++; } && have_tools=1
      if [ "$have_tools" -eq 1 ] && have_blas_lapack; then
        log "Core build tools and BLAS/LAPACK already present."
      elif [ -n "${PREFIX:-}" ] && [[ "$PREFIX" == *com.termux* ]]; then
        install_termux_deps
      elif have apt-get; then
        install_apt_deps
      else
        warn "Missing build tools or BLAS/LAPACK, and no supported package manager available."
        warn "Please install: cmake (>=3.24), a C++23 compiler, git, and BLAS/LAPACK (see README.md)."
      fi
      ;;
    *)
      err "Invalid INSTALL_DEPS='$INSTALL_DEPS' (expected auto|yes|no)."; exit 2
      ;;
  esac
}

check_tools() {
  local missing=0
  for t in cmake git; do
    have "$t" || { err "Required tool missing: $t"; missing=1; }
  done
  if ! have g++ && ! have clang++; then
    err "No C++ compiler found (need g++ or clang++ with C++23 support)."
    missing=1
  fi
  if [ "$missing" -ne 0 ]; then
    err "Install the missing prerequisites and re-run (see README.md)."
    exit 1
  fi
}

# --- 2. Configure / build / test ---------------------------------------------
configure() {
  log "Configuring '$BUILD_TYPE' build in ./$BUILD_DIR"
  cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DVORTEX_BUILD_TESTS=ON
}

build() {
  log "Building (jobs: $JOBS)"
  cmake --build "$BUILD_DIR" -j "$JOBS"
}

run_tests() {
  log "Running tests"
  ctest --test-dir "$BUILD_DIR" --output-on-failure
}

main() {
  parse_args "$@"
  ensure_deps
  check_tools
  configure
  build
  run_tests
  log "All done — environment ready, build and tests passed."
}

main "$@"
