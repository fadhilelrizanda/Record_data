#!/usr/bin/env bash
# Launch the live LiDAR + camera viewer with the env + network workarounds this
# machine needs.
#
# Why the env vars (conda base env fights the system graphics stack):
#   LD_PRELOAD libstdc++  -> conda's libstdc++ is too old for system Mesa (iris_dri.so)
#   LIBGL_ALWAYS_SOFTWARE -> session is Wayland; Open3D GLFW/GLEW fails on it, so use
#   -u WAYLAND_DISPLAY       software GL through XWayland instead (CPU render, but works)
#   --camera 3            -> USB HD Pro Webcam C920 capture node is /dev/video3
#                           (video4 is its metadata node; video1/2 are the laptop cam)
#
# Why the network preflight (the #1 reason "the LiDAR isn't streaming"):
#   The Velodyne (192.168.0.201) ARP-requests the host at 192.168.0.2 and only
#   unicasts point data once that address answers. If the wired interface has no
#   192.168.0.x address, no packets ever arrive and the LiDAR panel sits on
#   "waiting for LiDAR..." forever — which is exactly what happens after a reboot,
#   since the IP is not persistent. So we (re)assign it here before launching.
set -euo pipefail
cd "$(dirname "$0")"

IFACE="enx00e04c680f63"      # USB/Ethernet adapter on this machine (see scan_lidar.sh)
HOST_IP="192.168.0.2"
HOST_CIDR="192.168.0.2/24"
LIDAR_IP="192.168.0.201"

# --- Ensure the wired link is up and on the LiDAR's subnet ------------------
if ip link show "$IFACE" >/dev/null 2>&1; then
    if ! ip -4 addr show dev "$IFACE" | grep -q "\b${HOST_IP}\b"; then
        echo "[run_live_view] $IFACE has no $HOST_IP — configuring (needs sudo)..."
        sudo ip addr replace "$HOST_CIDR" dev "$IFACE"
    fi
    sudo ip link set "$IFACE" up
    # Prefer the wired link for the sensor, not Wi-Fi (a stale default route can
    # otherwise send/expect its traffic on wlan).
    sudo ip route replace "${LIDAR_IP}/32" dev "$IFACE" src "$HOST_IP" 2>/dev/null || true
    echo "[run_live_view] $IFACE = $(ip -4 -br addr show dev "$IFACE")"
else
    echo "[run_live_view] WARNING: interface '$IFACE' not found — skipping network setup." >&2
    echo "                Run ./scan_lidar.sh to find the right interface name." >&2
fi

# --- Launch the viewer (window auto-fits the current screen size) -----------
env -u WAYLAND_DISPLAY XDG_SESSION_TYPE=x11 DISPLAY=:0 LIBGL_ALWAYS_SOFTWARE=1 \
    LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 \
    python live_view.py 0.0.0.0 2368 --camera 3 --record --interval 0.2 "$@"
