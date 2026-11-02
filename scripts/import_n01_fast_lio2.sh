#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$ROOT/src/third_party/FAST_LIO"
URL="https://github.com/hku-mars/FAST_LIO.git"
PIN="cc3c9e8b1469b33b8072764d6db2348ed6dc597f"

mkdir -p "$(dirname "$DEST")"

if [[ -e "$DEST" && ! -d "$DEST/.git" ]]; then
  echo "[N01-FAST-LIO2] Refusing non-git path: $DEST" >&2
  exit 2
fi

if [[ ! -d "$DEST/.git" ]]; then
  echo "[N01-FAST-LIO2] Cloning upstream ROS2 source at pinned revision"
  git clone --recurse-submodules --branch ROS2 --single-branch "$URL" "$DEST"
else
  CURRENT="$(git -C "$DEST" rev-parse HEAD)"
  if [[ "$CURRENT" != "$PIN" ]]; then
    if [[ -n "$(git -C "$DEST" status --porcelain)" ]]; then
      echo "[N01-FAST-LIO2] Existing checkout has local changes; refusing to reset it." >&2
      echo "Move it aside or restore it manually, then rerun." >&2
      exit 2
    fi
    git -C "$DEST" fetch origin "$PIN"
    git -C "$DEST" checkout --detach "$PIN"
  fi
fi

ACTUAL="$(git -C "$DEST" rev-parse HEAD)"
if [[ "$ACTUAL" != "$PIN" ]]; then
  echo "[N01-FAST-LIO2] Wrong upstream revision: $ACTUAL" >&2
  exit 2
fi

git -C "$DEST" submodule sync --recursive
git -C "$DEST" submodule update --init --recursive

python3 "$ROOT/scripts/patch_n01_fast_lio2.py" --source "$DEST"
echo "[N01-FAST-LIO2] Source ready at $DEST"
echo "[N01-FAST-LIO2] Upstream revision: $PIN"
