#!/usr/bin/env bash
# Optional repo-local compiler download, not run by make/setup.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$(uname -s)" != Darwin || "$(uname -m)" != arm64 ]]; then
  echo 'This helper supports Apple Silicon macOS only; use a matching Arm embedded toolchain on other hosts.' >&2;exit 2
fi
DEST="$ROOT/.tools"
mkdir -p "$DEST"
NAME=xpack-arm-none-eabi-gcc-14.2.1-1.1-darwin-arm64.tar.gz
BASE=https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v14.2.1-1.1
curl -fL "$BASE/$NAME" -o "$DEST/$NAME"
curl -fL "$BASE/$NAME.sha" -o "$DEST/$NAME.sha"
EXPECTED="$(awk '{print $1}' "$DEST/$NAME.sha")"
ACTUAL="$(shasum -a 256 "$DEST/$NAME" | awk '{print $1}')"
[[ "$ACTUAL" == "$EXPECTED" ]] || { echo 'Checksum mismatch' >&2;exit 1; }
tar -xzf "$DEST/$NAME" -C "$DEST"
echo 'Compiler installed under .tools; GNU make/coreutils/sed are separate dependencies.'
