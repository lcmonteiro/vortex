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
#   ./setup.sh
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

ensure_deps() {
  case "$INSTALL_DEPS" in
    no)
      log "Skipping dependency installation (INSTALL_DEPS=no)."
      ;;
    yes)
      if have apt-get; then install_apt_deps
      else err "apt-get not found; please install the prerequisites manually."; fi
      ;;
    auto)
      if have cmake && have git && { have g++ || have clang++; }; then
        log "Core build tools already present."
      elif have apt-get; then
        install_apt_deps
      else
        warn "Missing build tools and no apt-get available."
        warn "Please install: cmake (>=3.24), a C++23 compiler, git, liblapack-dev, libblas-dev."
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
  ensure_deps
  check_tools
  configure
  build
  run_tests
  log "All done — environment ready, build and tests passed."
}

main "$@"
