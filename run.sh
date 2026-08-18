#!/bin/bash
# Convenience wrapper around the container.
#
#   ./run.sh gui     lt2      2-camera run, viewer opens in the browser automatically
#   ./run.sh native  lt2      same, but in macOS Screen Sharing instead of a browser
#   ./run.sh trackers lt2     tracker overlays only (no 3D scene)
#   ./run.sh mono    lt2      visual-only, headless  (bring-up stage)
#   ./run.sh batch   lt2      2-camera, headless, as fast as possible
#   ./run.sh check            config + calibration dry run
#   ./run.sh shell            interactive shell in the container
#
# Extra args after the sequence are passed through to rovtio_player, e.g.
#   ./run.sh gui lt2 --max-seconds 60 --viewer-hz 10
# Set NO_OPEN=1 to stop the browser from opening by itself.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA="${ROVTIO_DATA:-/Users/jhpark/VSLAM/Datasets/rovtio/extracted}"
OUT="$ROOT/out"
IMG=rovtio-rosfree
# Host-side ports. Deliberately NOT 6080/5900: a browser caches by origin, so reusing a
# port another project served noVNC on resurrects that project's stale JS. 5900 is also
# macOS's own Screen Sharing port.
WEB_PORT="${ROVTIO_WEB_PORT:-6081}"
VNC_PORT="${ROVTIO_VNC_PORT:-5901}"
VNC_URL="http://localhost:$WEB_PORT/vnc.html?autoconnect=true&resize=scale&reconnect=true"
VNC_NATIVE="vnc://localhost:$VNC_PORT"
mkdir -p "$OUT"

TTY=(); [ -t 0 ] && [ -t 1 ] && TTY=(-it)

docker_run() {
    docker run --rm ${TTY[@]+"${TTY[@]}"} --platform linux/arm64 \
        -p "$WEB_PORT":6080 -p "$VNC_PORT":5900 \
        -v "$DATA:/data:ro" -v "$OUT:/out" \
        --shm-size=1g "$IMG" "$@"
}

# Wait for noVNC to accept connections, then hand the URL to the browser. Runs in the
# background because the container itself holds the foreground.
open_when_ready() {   # $1 = port to wait for, $2 = url to hand to the OS
    [ -n "${NO_OPEN:-}" ] && return 0
    command -v open >/dev/null || return 0
    (
        for _ in $(seq 1 150); do
            if /usr/bin/nc -z localhost "$1" 2>/dev/null; then
                sleep 0.5          # let the server finish binding
                open "$2"
                exit 0
            fi
            sleep 0.2
        done
        echo "note: port $1 did not come up within 30 s -- open $2 by hand" >&2
    ) &
}

MODE="${1:-gui}"
SEQ="${2:-lt2}"
CFG=(--info /opt/rovtio/cfg/rovtio_rosfree.info)

case "$MODE" in
  check)
    docker_run --no-display /opt/rovtio/bin/rovtio_player ${CFG[@]+"${CFG[@]}"} --dry-run
    ;;
  mono)
    docker_run --no-display /opt/rovtio/bin/rovtio_player_mono ${CFG[@]+"${CFG[@]}"} \
        --seq "/data/$SEQ" --cams 1 --rate 0 --gui none \
        --tum "/out/${SEQ}_mono.txt" --diag "/out/${SEQ}_mono_diag.csv" "${@:3}"
    ;;
  batch)
    docker_run --no-display /opt/rovtio/bin/rovtio_player ${CFG[@]+"${CFG[@]}"} \
        --seq "/data/$SEQ" --rate 0 --gui none \
        --tum "/out/${SEQ}.txt" --diag "/out/${SEQ}_diag.csv" "${@:3}"
    ;;
  gui|trackers)
    G=all; [ "$MODE" = trackers ] && G=trackers
    echo "viewer: $VNC_URL"
    open_when_ready "$WEB_PORT" "$VNC_URL"
    docker_run /opt/rovtio/bin/rovtio_player ${CFG[@]+"${CFG[@]}"} \
        --seq "/data/$SEQ" --rate 1.0 --gui "$G" --viewer-hz 15 \
        --tum "/out/${SEQ}.txt" --diag "/out/${SEQ}_diag.csv" "${@:3}"
    ;;
  native)
    # macOS Screen Sharing talks to the same x11vnc the browser path uses, but gives a
    # native window and skips the browser canvas. XQuartz cannot be used instead -- see
    # README, its GLX is GL 1.4 and even the OpenCV windows need a GLX visual it lacks.
    echo "viewer: $VNC_NATIVE (macOS Screen Sharing)"
    open_when_ready "$VNC_PORT" "$VNC_NATIVE"
    docker_run /opt/rovtio/bin/rovtio_player ${CFG[@]+"${CFG[@]}"} \
        --seq "/data/$SEQ" --rate 1.0 --gui all --viewer-hz 15 \
        --tum "/out/${SEQ}.txt" --diag "/out/${SEQ}_diag.csv" "${@:3}"
    ;;
  shell)
    docker_run --no-display /bin/bash
    ;;
  *)
    sed -n '2,13p' "$0"; exit 2
    ;;
esac
