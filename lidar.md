sudo ip addr add 192.168.1.2/24 dev eqos_0
sudo ip link set eqos_0 up
sudo ip route replace 192.168.0.201/32 dev eqos_0 src 192.168.1.2 metric 1
