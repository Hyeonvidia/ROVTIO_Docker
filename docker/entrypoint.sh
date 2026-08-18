#!/bin/bash
# Brings up an in-container X server with software GL, then hands it to the app.
# The GUI is reached from the Mac at http://localhost:6080/vnc.html -- no XQuartz,
# no GLX over TCP, and MIT-SHM works because client and server share the container.
set -e

start_display() {
    [ -e /tmp/.X11-unix/X${DISPLAY#:} ] && return 0
    Xvfb "$DISPLAY" -screen 0 "${XVFB_GEOM:-2200x1300x24}" +extension GLX +render -noreset \
        >/var/log/xvfb.log 2>&1 &
    for _ in $(seq 1 100); do
        xdpyinfo -display "$DISPLAY" >/dev/null 2>&1 && break
        sleep 0.1
    done
    xdpyinfo -display "$DISPLAY" >/dev/null 2>&1 || { echo "Xvfb failed:"; cat /var/log/xvfb.log; exit 1; }

    x11vnc -display "$DISPLAY" -forever -shared -nopw -quiet -rfbport 5900 \
        >/var/log/x11vnc.log 2>&1 &
    websockify --web=/usr/share/novnc 6080 localhost:5900 \
        >/var/log/websockify.log 2>&1 &
    echo "viewer: http://localhost:6080/vnc.html"
}

case "${1:-}" in
    --no-display) shift ;;
    *) start_display ;;
esac

exec "$@"
