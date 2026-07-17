#!/usr/bin/env bash
# Discover a Velodyne LiDAR on a wired interface.
#
# Why sniff instead of ping: this sensor does NOT answer ICMP ping. It streams
# UDP (2368 = data, 8308 = position) and ARP-requests the host it wants to send
# to. So we passively listen on the wire and read its IP off those packets.
# This works even BEFORE you've assigned an IP to the interface.
#
# Usage:  ./scan_lidar.sh [interface] [seconds]
#   ./scan_lidar.sh                      # default iface, 8s
#   ./scan_lidar.sh enx00e04c680f63 10   # explicit iface, 10s
#
# Needs sudo (tcpdump). Run:  sudo ./scan_lidar.sh   (or it will re-ask).

IFACE="${1:-enx00e04c680f63}"
SECS="${2:-8}"
IPRE='([0-9]{1,3}\.){3}[0-9]{1,3}'   # rough IPv4 regex

# --- sanity checks ---------------------------------------------------------
if ! command -v tcpdump >/dev/null 2>&1; then
    echo "ERROR: tcpdump not found. Install it:  sudo apt install tcpdump" >&2
    exit 1
fi
if ! ip link show "$IFACE" >/dev/null 2>&1; then
    echo "ERROR: interface '$IFACE' not found. Available wired interfaces:" >&2
    ip -br link show | grep -E 'enx|eth|en[0-9]' >&2
    echo "Pass the right one:  ./scan_lidar.sh <iface>" >&2
    exit 1
fi

# Make sure the link is up so we can actually receive frames.
sudo ip link set "$IFACE" up 2>/dev/null

echo "Sniffing '$IFACE' for ${SECS}s (LiDAR UDP 2368/8308 + ARP)..."
echo "(no ping — this sensor only reveals itself via its own packets)"
echo

TMP="$(mktemp)"
trap 'rm -f "$TMP"' EXIT

# Capture UDP + ARP. -n no DNS, -t no timestamps (cleaner to parse), -l line-buffered.
sudo timeout "$SECS" tcpdump -ni "$IFACE" -t -l \
    '(udp and (port 2368 or port 8308)) or arp' > "$TMP" 2>/dev/null

if [ ! -s "$TMP" ]; then
    echo "No LiDAR traffic seen in ${SECS}s."
    echo "Check:  cable connected? sensor powered? right interface? Try a longer scan:"
    echo "  ./scan_lidar.sh $IFACE 15"
    exit 2
fi

# --- parse -----------------------------------------------------------------
# Sensor = source IP sending FROM the data/position ports.
SENSOR_IPS="$(grep -oE "${IPRE}\.(2368|8308) >" "$TMP" \
              | grep -oE "$IPRE" | sort -u)"

# Host the sensor is targeting = destination of those UDP packets.
HOST_IPS="$(grep -oE "> ${IPRE}\.(2368|8308)" "$TMP" \
            | grep -oE "$IPRE" | sort -u)"

# Ports seen from the sensor.
PORTS="$(grep -oE "${IPRE}\.(2368|8308) >" "$TMP" \
         | grep -oE '\.(2368|8308) ' | grep -oE '[0-9]+' | sort -un | paste -sd, -)"

# ARP: "who-has <hostwanted> ... tell <sensor>"  — reveals required host IP.
# NOTE: $IPRE has an inner capture group, so each ($IPRE) consumes TWO groups.
# who-has <\1...> tell <\3...>  ->  host wanted = \1, sensor = \3.
ARP_HINT="$(grep -i 'who-has' "$TMP" \
            | sed -E "s/.*who-has ($IPRE).*tell ($IPRE).*/  sensor \3 is asking for host \1/" \
            | sort -u | head -3)"

echo "================ RESULT ================"
if [ -n "$SENSOR_IPS" ]; then
    echo "LiDAR (sensor) IP : $(echo "$SENSOR_IPS" | paste -sd, - | sed 's/,/, /g')"
    echo "Data ports        : ${PORTS:-?}  (2368=points, 8308=position)"
    [ -n "$HOST_IPS" ] && echo "Sending to host   : $(echo "$HOST_IPS" | paste -sd, - | sed 's/,/, /g')"
else
    echo "Saw ARP but no UDP data stream yet. Likely sensor IP from ARP below."
fi
if [ -n "$ARP_HINT" ]; then
    echo "ARP hints:"
    echo "$ARP_HINT"
    echo "  -> set this machine to that 'host' IP so the sensor streams to it."
fi
echo "========================================"
