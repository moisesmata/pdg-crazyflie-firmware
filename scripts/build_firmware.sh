#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PLATFORM="${1:-cf2}"
case "$PLATFORM" in cf2|cf21bl|bolt) ;; *) echo 'Supported: cf2, cf21bl, bolt' >&2; exit 2;; esac
export PATH="$ROOT/.venv/bin:/opt/homebrew/bin:$PATH"
for candidate in "$ROOT"/.tools/xpack-arm-none-eabi-gcc-*/bin; do
  if [[ -d "$candidate" ]]; then export PATH="$candidate:$PATH"; break; fi
done
if [[ "$(uname -s)" == Darwin ]]; then
  for tool in /opt/homebrew/opt/coreutils/libexec/gnubin/readlink /usr/local/opt/coreutils/libexec/gnubin/readlink; do
    if [[ -x "$tool" ]]; then export PATH="$(dirname "$tool"):$PATH"; break; fi
  done
  if ! readlink -m / >/dev/null 2>&1; then
    echo 'GNU coreutils is required; run brew install coreutils gnu-sed.' >&2; exit 2
  fi
fi
MAKE_BIN="${GNU_MAKE:-$(command -v gmake || command -v make)}"
if ! command -v arm-none-eabi-gcc >/dev/null; then
  echo 'Install Arm GNU Embedded toolchain or place xPack toolchain under .tools/.' >&2; exit 2
fi
if ! "$MAKE_BIN" --version | head -1 | grep -q 'GNU Make 4'; then
  echo 'GNU Make 4 is required; on macOS: brew install make coreutils gnu-sed' >&2; exit 2
fi
if [[ ! -f "$ROOT/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h" ]]; then
  echo 'Run git submodule update --init --recursive first.' >&2; exit 2
fi
"$MAKE_BIN" -C "$ROOT/apps/pdg_controller" "${PLATFORM}_defconfig"
"$MAKE_BIN" -C "$ROOT/apps/pdg_controller" -j"${JOBS:-8}"
# Refuse to report a successful custom image if the wrong config/symbols were linked.
grep -q '^CONFIG_CONTROLLER_OOT=y$' "$ROOT/apps/pdg_controller/build/.config"
grep -q '^CONFIG_MOTORS_REQUIRE_ARMING=y$' "$ROOT/apps/pdg_controller/build/.config"
PLATFORM_SYMBOL="$(printf '%s' "$PLATFORM" | tr '[:lower:]' '[:upper:]')"
grep -q "^CONFIG_PLATFORM_${PLATFORM_SYMBOL}=y$" "$ROOT/apps/pdg_controller/build/.config"
ELF="$ROOT/apps/pdg_controller/build/$PLATFORM.elf"
test -f "$ELF"
arm-none-eabi-nm "$ELF" | grep ' T controllerOutOfTree$'
arm-none-eabi-nm "$ELF" | grep ' T pdgLimitCollective$'
mkdir -p "$ROOT/output/firmware"
for ext in bin hex elf; do
  source="$ROOT/apps/pdg_controller/build/$PLATFORM.$ext"
  if [[ -f "$source" ]]; then cp "$source" "$ROOT/output/firmware/$PLATFORM.$ext"; fi
done
arm-none-eabi-size "$ELF"
"${PYTHON:-python3}" "$ROOT/scripts/build_manifest.py" "$PLATFORM" "$ELF"
echo "Built $PLATFORM firmware. Nothing was flashed."
