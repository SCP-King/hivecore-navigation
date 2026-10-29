#!/usr/bin/env bash
# Record ONLY the N01 RViz window, not the user's full desktop.
set -Eeuo pipefail
if [[ $# -lt 1 || $# -gt 2 ]]; then echo "usage: bash scripts/record_n01_demo.sh OUTPUT.mp4 [RVIZ_PROCESS_PID]" >&2; exit 2; fi
OUT="$1"
if [[ -z "${DISPLAY:-}" ]]; then echo "DISPLAY missing: use an X11 desktop session" >&2; exit 2; fi
command -v ffmpeg >/dev/null || { echo "Missing ffmpeg" >&2; exit 2; }
command -v xdotool >/dev/null || { echo "Missing xdotool" >&2; exit 2; }
# Require X11 input's window_id support; never fall back to whole-screen capture.
ffmpeg -hide_banner -h demuxer=x11grab 2>&1 | grep -q 'window_id' || {
  echo "ffmpeg x11grab -window_id not supported. Refusing whole-screen capture." >&2
  exit 2
}
mkdir -p "$(dirname "$OUT")"
window=""
for _ in $(seq 1 40); do
  if [[ $# -eq 2 ]]; then
    window="$(xdotool search --onlyvisible --pid "$2" 2>/dev/null | tail -n 1 || true)"
  else
    window="$(xdotool search --onlyvisible --class rviz 2>/dev/null | tail -n 1 || true)"
  fi
  if [[ -n "$window" ]]; then break; fi
  sleep .5
done
if [[ -z "$window" ]]; then
  echo "RViz2 X11 window not found. Use X11 or QT_QPA_PLATFORM=xcb; no screen recorded." >&2
  exit 3
fi
echo "Recording ONLY RViz window ID $window -> $OUT"
# ffmpeg handles SIGINT/SIGTERM, finalizing its output container when stopped.
exec ffmpeg -hide_banner -loglevel warning -nostdin -y \
  -f x11grab -framerate 12 -draw_mouse 0 -window_id "$window" \
  -i "$DISPLAY" -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2" \
  -c:v libx264 -preset veryfast -crf 25 \
  -pix_fmt yuv420p -movflags +faststart "$OUT"
