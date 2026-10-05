#!/usr/bin/env bash
# Build OcclusaCAD on Linux or macOS.
#
#   ./build.sh                 check dependencies, configure and build (Release) into ./build
#   ./build.sh --test          ... and run the unit tests
#   ./build.sh --demo          ... and create synthetic demo cases (implant, crown, bridge)
#   ./build.sh --demo --run    ... and start OcclusaCAD DB on the demo data
#
# Run ./build.sh --help for all options. On Windows, open the folder in Visual Studio 2022 instead.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$ROOT/build"
BUILD_TYPE="Release"
JOBS=""
CLEAN=0
TEST=0
E2E=0
DEMO=0
DEMO_DIR="$HOME/OcclusaCAD-demo"
RUN=0
CHECK_ONLY=0

usage() {
    cat <<EOF
Usage: ./build.sh [options]

  --debug            Debug build (default: Release)
  --relwithdebinfo   Optimised build with debug information
  --clean            Delete the build directory first
  --build-dir DIR    Build directory (default: ./build)
  --jobs N           Parallel build jobs (default: all cores)
  --test             Run the unit tests after building
  --e2e              Also build and run the headless end-to-end tests (Linux, needs EGL/Mesa)
  --demo [DIR]       Create synthetic demo data in DIR (default: ~/OcclusaCAD-demo)
  --run              Start OcclusaCAD DB when done (on the demo data with --demo)
  --check            Only check the build dependencies
  -h, --help         Show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --debug) BUILD_TYPE="Debug" ;;
        --relwithdebinfo) BUILD_TYPE="RelWithDebInfo" ;;
        --clean) CLEAN=1 ;;
        --build-dir) BUILD_DIR="$(realpath -m "$2")"; shift ;;
        --jobs) JOBS="$2"; shift ;;
        --test) TEST=1 ;;
        --e2e) E2E=1; TEST=1 ;;
        --demo)
            DEMO=1
            if [[ $# -gt 1 && "$2" != --* ]]; then DEMO_DIR="$(realpath -m "$2")"; shift; fi ;;
        --run) RUN=1 ;;
        --check) CHECK_ONLY=1 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

if [[ -t 1 ]]; then
    BOLD=$'\e[1m'; RED=$'\e[31m'; GREEN=$'\e[32m'; YELLOW=$'\e[33m'; RESET=$'\e[0m'
else
    BOLD=""; RED=""; GREEN=""; YELLOW=""; RESET=""
fi
step() { echo "${BOLD}==> $*${RESET}"; }
ok()   { echo "  ${GREEN}ok${RESET}   $*"; }
bad()  { echo "  ${RED}missing${RESET} $*"; }
warn() { echo "  ${YELLOW}note${RESET} $*"; }

OS="$(uname -s)"
if [[ -z "$JOBS" ]]; then
    JOBS="$( (command -v nproc >/dev/null && nproc) || sysctl -n hw.ncpu 2>/dev/null || echo 4)"
fi

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

distro_id() {
    if [[ -r /etc/os-release ]]; then
        # shellcheck disable=SC1091
        . /etc/os-release
        echo "${ID:-} ${ID_LIKE:-}"
    fi
}

install_hint() {
    local ids
    ids="$(distro_id)"
    case " $ids " in
        *" arch "*|*" manjaro "*|*" endeavouros "*|*" cachyos "*)
            echo "sudo pacman -S --needed base-devel cmake ninja pkgconf wayland wayland-protocols libxkbcommon libx11 libxrandr libxinerama libxcursor libxi dbus mesa libglvnd" ;;
        *" debian "*|*" ubuntu "*)
            echo "sudo apt install build-essential cmake ninja-build pkg-config libwayland-dev libxkbcommon-dev wayland-protocols xorg-dev libdbus-1-dev libgl-dev libegl-dev" ;;
        *" fedora "*|*" rhel "*)
            echo "sudo dnf install gcc-c++ cmake ninja-build pkgconf-pkg-config wayland-devel wayland-protocols-devel libxkbcommon-devel libX11-devel libXrandr-devel libXinerama-devel libXcursor-devel libXi-devel dbus-devel mesa-libGL-devel mesa-libEGL-devel" ;;
        *" opensuse"*|*" suse "*)
            echo "sudo zypper install gcc-c++ cmake ninja pkgconf wayland-devel wayland-protocols-devel libxkbcommon-devel libX11-devel libXrandr-devel libXinerama-devel libXcursor-devel libXi-devel dbus-1-devel Mesa-libGL-devel Mesa-libEGL-devel" ;;
        *)
            echo "Install: a C++20 compiler, CMake 3.21+, pkg-config, and the development packages for Wayland, wayland-protocols, xkbcommon, X11 (Xrandr, Xinerama, Xcursor, Xi), D-Bus, OpenGL and EGL." ;;
    esac
}

version_ge() { # version_ge 3.25.1 3.21
    [[ "$(printf '%s\n%s\n' "$2" "$1" | sort -V | head -n1)" == "$2" ]]
}

check_deps() {
    local missing=0
    step "Checking build dependencies ($OS)"

    if command -v cmake >/dev/null; then
        local v
        v="$(cmake --version | head -n1 | awk '{print $3}')"
        if version_ge "$v" 3.21; then ok "cmake $v"; else bad "cmake 3.21 or newer (found $v)"; missing=1; fi
    else
        bad "cmake"; missing=1
    fi

    local cxx="${CXX:-}"
    [[ -z "$cxx" ]] && for c in c++ g++ clang++; do command -v "$c" >/dev/null && { cxx="$c"; break; }; done
    if [[ -n "$cxx" ]] && command -v "$cxx" >/dev/null; then
        local ver
        ver="$("$cxx" --version | head -n1)"
        if [[ "$ver" == *clang* || "$ver" == *Clang* ]]; then
            local major
            major="$(echo "$ver" | grep -oE '[0-9]+\.[0-9]+(\.[0-9]+)?' | head -n1 | cut -d. -f1)"
            if [[ "$OS" == Darwin || "${major:-0}" -ge 17 ]]; then ok "$ver"; else bad "clang 17+ (found $ver)"; missing=1; fi
        else
            local major
            major="$("$cxx" -dumpversion | cut -d. -f1)"
            if [[ "${major:-0}" -ge 13 ]]; then ok "$ver"; else bad "GCC 13+ for C++20 <format> (found $ver)"; missing=1; fi
        fi
    else
        bad "a C++ compiler (GCC 13+ or Clang 17+)"; missing=1
    fi

    if command -v ninja >/dev/null; then ok "ninja $(ninja --version)"; else warn "ninja not found; using make (slower)"; fi

    if [[ "$OS" == Linux ]]; then
        if ! command -v pkg-config >/dev/null; then
            bad "pkg-config"; missing=1
        else
            ok "pkg-config"
            local mods=(wayland-client wayland-cursor wayland-egl wayland-protocols xkbcommon x11 xrandr xinerama xcursor xi dbus-1 gl egl)
            for m in "${mods[@]}"; do
                if pkg-config --exists "$m"; then ok "$m"; else bad "$m (development package)"; missing=1; fi
            done
            if command -v wayland-scanner >/dev/null; then ok "wayland-scanner"; else bad "wayland-scanner"; missing=1; fi
        fi
    elif [[ "$OS" == Darwin ]]; then
        if xcode-select -p >/dev/null 2>&1; then ok "Xcode command line tools"; else bad "Xcode command line tools (xcode-select --install)"; missing=1; fi
    fi

    if [[ $missing -ne 0 ]]; then
        echo
        echo "${RED}Some dependencies are missing.${RESET}"
        if [[ "$OS" == Linux ]]; then
            echo "Install them with:"
            echo "  $(install_hint)"
        elif [[ "$OS" == Darwin ]]; then
            echo "Install them with: xcode-select --install && brew install cmake ninja"
        fi
        return 1
    fi
    return 0
}

check_deps || exit 1
[[ $CHECK_ONLY -eq 1 ]] && exit 0

# ---------------------------------------------------------------------------
# Configure and build
# ---------------------------------------------------------------------------

if [[ $CLEAN -eq 1 && -d "$BUILD_DIR" ]]; then
    step "Removing $BUILD_DIR"
    rm -rf -- "${BUILD_DIR:?}"
fi

GENERATOR=()
# Keep the generator of an existing build directory (CMake refuses to switch).
if [[ ! -f "$BUILD_DIR/CMakeCache.txt" ]] && command -v ninja >/dev/null; then
    GENERATOR=(-G Ninja)
fi

step "Configuring ($BUILD_TYPE) in $BUILD_DIR"
cmake -S "$ROOT" -B "$BUILD_DIR" "${GENERATOR[@]}" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DOCCLUSACAD_HEADLESS_TESTS="$([[ $E2E -eq 1 ]] && echo ON || echo OFF)"

step "Building with $JOBS jobs"
cmake --build "$BUILD_DIR" --parallel "$JOBS"

BIN="$BUILD_DIR/bin"

if [[ $TEST -eq 1 ]]; then
    step "Running tests"
    if [[ $E2E -eq 1 ]]; then
        ctest --test-dir "$BUILD_DIR" --output-on-failure
    else
        ctest --test-dir "$BUILD_DIR" --output-on-failure -R occlusa_tests
    fi
fi

# ---------------------------------------------------------------------------
# Demo data and launch
# ---------------------------------------------------------------------------

if [[ $DEMO -eq 1 ]]; then
    if [[ -f "$DEMO_DIR/data/occlusacad.sqlite3" ]]; then
        step "Demo data already present in $DEMO_DIR (delete it to recreate)"
    else
        step "Creating demo data in $DEMO_DIR"
        mkdir -p "$DEMO_DIR"
        "$BIN/occlusa_phantom" --out "$DEMO_DIR/phantom" --create-case "$DEMO_DIR/data"
    fi
fi

echo
echo "${GREEN}${BOLD}Build complete.${RESET} Programs are in $BIN:"
echo "  OcclusaCAD DB (case management):  $BIN/OcclusaCADDB"
echo "  OcclusaCAD (designer):            $BIN/OcclusaCAD"
echo "  Tools:                            $BIN/occlusa_phantom, $BIN/occlusa_toothlib"
echo
if [[ $DEMO -eq 1 ]]; then
    echo "Start with the demo cases (implant planning, crown on 46, bridge 35-36-37):"
    echo "  $BIN/OcclusaCADDB --data-root \"$DEMO_DIR/data\""
else
    echo "Start OcclusaCAD DB; on first run it asks for a data folder (local or on a network share):"
    echo "  $BIN/OcclusaCADDB"
    echo "Or create demo cases first: ./build.sh --demo"
fi

if [[ $RUN -eq 1 ]]; then
    step "Starting OcclusaCAD DB"
    if [[ $DEMO -eq 1 ]]; then
        exec "$BIN/OcclusaCADDB" --data-root "$DEMO_DIR/data"
    else
        exec "$BIN/OcclusaCADDB"
    fi
fi
