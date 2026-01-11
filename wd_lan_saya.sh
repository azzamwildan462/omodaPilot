#!/bin/bash

IFACE="enp3s0"

ping -c 3 192.168.10.100 >/dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "Network down, resetting $IFACE"
    ip link set $IFACE down
    sleep 2
    ip link set $IFACE up
    systemctl restart NetworkManager 2>/dev/null
fi
