#!/bin/bash
# One-time host tuning for a Raspberry Pi (3 or 5) running the racer container.
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }

IS_PI5=0; grep -q 'Raspberry Pi 5' /proc/device-tree/model 2>/dev/null && IS_PI5=1

# 1) Swap file on the SD card (dphys-swapfile on Raspberry Pi OS): 2 GB on small-RAM Pis, 1 GB on a Pi 5
SWAP=2048; [ "$IS_PI5" = 1 ] && SWAP=1024
if [ -f /etc/dphys-swapfile ]; then
  sed -i "s/^CONF_SWAPSIZE=.*/CONF_SWAPSIZE=$SWAP/; s/^#\?CONF_MAXSWAP=.*/CONF_MAXSWAP=$SWAP/" /etc/dphys-swapfile
  dphys-swapfile swapoff || true; dphys-swapfile setup; dphys-swapfile swapon
fi

# 2) Prefer RAM, swap only under real pressure (SD cards wear out)
cat > /etc/sysctl.d/99-racer.conf <<'CONF'
vm.swappiness=20
vm.vfs_cache_pressure=50
CONF
sysctl --system >/dev/null

# 3) Docker: small logs, lower overhead
mkdir -p /etc/docker
cat > /etc/docker/daemon.json <<'CONF'
{ "log-driver": "json-file", "log-opts": { "max-size": "5m", "max-file": "2" } }
CONF

# 4) GPU memory split and cgroup memory limits.
#    NOT 16: with gpu_mem=16 the Pi 3 loads the cut-down firmware, which has no camera/ISP support
#    (the CSI camera then fails with "rpicam-apps currently only supports the Raspberry Pi platforms").
#    64 is the Pi 3 default and keeps the camera working.
#    The Pi 5 ignores gpu_mem (its GPU memory is dynamic), so skip it there.
CFG=/boot/firmware/config.txt; [ -f "$CFG" ] || CFG=/boot/config.txt
if [ "$IS_PI5" = 0 ]; then
  grep -q '^gpu_mem=' "$CFG" && sed -i 's/^gpu_mem=.*/gpu_mem=64/' "$CFG" || echo 'gpu_mem=64' >> "$CFG"
fi
# Hardware PWM on GPIO18/19 for the steering servo (steady pulses), writable by group gpio
grep -q '^dtoverlay=pwm-2chan' "$CFG" || echo 'dtoverlay=pwm-2chan' >> "$CFG"
cat > /etc/udev/rules.d/99-racer-pwm.rules <<'CONF'
SUBSYSTEM=="pwm", ACTION=="add|change", RUN+="/bin/sh -c 'chgrp -R gpio /sys%p && chmod -R g=u /sys%p'"
CONF
udevadm control --reload-rules
# I2C for the OLED (/dev/i2c-1 must exist before the container starts)
grep -q '^dtparam=i2c_arm=on' "$CFG" || echo 'dtparam=i2c_arm=on' >> "$CFG"
grep -q '^i2c-dev' /etc/modules || echo 'i2c-dev' >> /etc/modules
CMD=/boot/firmware/cmdline.txt; [ -f "$CMD" ] || CMD=/boot/cmdline.txt
grep -q 'cgroup_enable=memory' "$CMD" || sed -i '1 s/$/ cgroup_enable=memory cgroup_memory=1/' "$CMD"

# 5) Services a headless robot doesn't need (Bluetooth stays ON for the Joy-Cons)
systemctl disable --now ModemManager triggerhappy 2>/dev/null || true

systemctl restart docker || true
echo "Done. Reboot to apply gpu_mem / cgroup changes."
