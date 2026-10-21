#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ "$(uname -s)/$(uname -m)" == Linux/x86_64 ]] || { echo 'This helper supports Linux x86_64.' >&2; exit 2; }
NAME=xpack-arm-none-eabi-gcc-14.2.1-1.1-linux-x64.tar.gz
BASE=https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v14.2.1-1.1
mkdir -p "$ROOT/.tools"
curl -fL --retry 2 "$BASE/$NAME" -o "$ROOT/.tools/$NAME"
curl -fL --retry 2 "$BASE/$NAME.sha" -o "$ROOT/.tools/$NAME.sha"
EXPECTED="$(awk '{print $1}' "$ROOT/.tools/$NAME.sha")"
ACTUAL="$(sha256sum "$ROOT/.tools/$NAME" | awk '{print $1}')"
[[ "$EXPECTED" == "$ACTUAL" ]] || { echo 'Checksum mismatch' >&2; exit 1; }
tar -xzf "$ROOT/.tools/$NAME" -C "$ROOT/.tools"
