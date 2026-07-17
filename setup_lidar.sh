#!/usr/bin/env bash
# Configure the wired interface to talk to the LiDAR.
# This sensor (192.168.0.201) ARP-requests the host at 192.168.0.2,
# so the host must sit on the same 192.168.0.x/24 subnet.

IFACE="enx00e04c680f63"   # USB/Ethernet adapter on this machine
HOST_IP="192.168.1.2/24"
LIDAR_IP="192.168.1.201"

sudo ip addr replace "$HOST_IP" dev "$IFACE"
sudo ip link set "$IFACE" up

echo "Interface $IFACE set to $HOST_IP."
echo "Listening for data from $LIDAR_IP (Ctrl-C to stop)..."
sudo tcpdump -ni "$IFACE" -c 10 host "$LIDAR_IP"
