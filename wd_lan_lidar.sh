#!/bin/bash

IFACE="enp2s0"

ping -c 3 192.168.1.202 >/dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "Network down, resetting $IFACE"
    ip link set $IFACE down
    sleep 2
    ip link set $IFACE up
    systemctl restart NetworkManager 2>/dev/null
fi
