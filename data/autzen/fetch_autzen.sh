#!/usr/bin/env bash
# Fetch Autzen Stadium evaluation data (S3, no git-lfs needed).
# Usage:
#   ./fetch_autzen.sh            # download both files
#   ./fetch_autzen.sh --list     # show URLs + expected sizes
#   ./fetch_autzen.sh --lazy     # skip files already present with correct size
set -euo pipefail
cd "$(dirname "$0")"

AUTZEN_LAZ_URL="https://s3.amazonaws.com/hobu-lidar/autzen.laz"
AUTZEN_LAZ_SIZE=56350988
COPC_URL="https://s3.amazonaws.com/hobu-lidar/autzen-classified.copc.laz"
COPC_SIZE=81123042

list() {
  echo "autzen.laz                    $AUTZEN_LAZ_SIZE bytes  $AUTZEN_LAZ_URL"
  echo "autzen-classified.copc.laz    $COPC_SIZE bytes  $COPC_URL"
}

fetch_one() {
  local url="$1" expected="$2" out="$3"
  if [[ -f "$out" ]]; then
    local have
    have=$(stat -c%s "$out")
    if [[ "$have" == "$expected" ]]; then
      echo "OK (exists, size matches): $out"
      return 0
    fi
    echo "WARN: $out exists but size $have != expected $expected, re-downloading"
  fi
  echo "GET $url -> $out"
  curl -fL --retry 3 -C - -o "$out" "$url"
  local have
  have=$(stat -c%s "$out")
  if [[ "$have" != "$expected" ]]; then
    echo "ERROR: $out size $have != expected $expected" >&2
    return 1
  fi
  echo "OK: $out ($have bytes)"
}

case "${1:---lazy}" in
  --list) list ;;
  --lazy|--all|"")
    fetch_one "$AUTZEN_LAZ_URL" "$AUTZEN_LAZ_SIZE" "autzen.laz"
    fetch_one "$COPC_URL" "$COPC_SIZE" "autzen-classified.copc.laz"
    echo "Done. See README.md for usage."
    ;;
  *) echo "usage: $0 [--list|--lazy|--all]" >&2; exit 2 ;;
esac
