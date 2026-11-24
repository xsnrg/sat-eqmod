#!/bin/bash
nohup indiserver -v /usr/bin/indi_eqmod_telescope > /tmp/indiserver.log 2>&1 &
sleep 2
indi_setprop "EQMod Mount.DEVICE_PORT.PORT=/dev/ttyUSB0" >/dev/null 2>&1
DRV=$(pgrep -f "^/usr/bin/indi_eqmod_telescope$" | head -1)
echo "driver pid: $DRV"
sudo -n timeout 35 strace -f -tt -e trace=ioctl,read,write -s 60 -p "$DRV" -o /tmp/sirius.trace &
sleep 2
indi_setprop "EQMod Mount.CONNECTION.CONNECT=On" >/dev/null 2>&1
echo "connect issued"
sleep 20
echo "=== :e1 exchange ==="
grep -n -A10 "write.*:e1" /tmp/sirius.trace | head -12
echo "=== connection state / driver messages ==="
indi_getprop "EQMod Mount.CONNECTION.*" "EQMod Mount.DRIVER_INFO.*" 2>/dev/null
tail -5 /tmp/indiserver.log
