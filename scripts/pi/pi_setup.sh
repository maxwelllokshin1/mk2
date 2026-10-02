#!/bin/bash
# One-time host tuning for a 1 GB Raspberry Pi running the racer container.
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }

# 1) 2 GB swap file on the SD card (dphys-swapfile on Raspberry Pi OS)
if [ -f /etc/dphys-swapfile ]; then
  sed -i 's/^CONF_SWAPSIZE=.*/CONF_SWAPSIZE=2048/; s/^#\?CONF_MAXSWAP=.*/CONF_MAXSWAP=2048/' /etc/dphys-swapfile
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

# 4) Minimal GPU memory split (headless) and cgroup memory limits
CFG=/boot/firmware/config.txt; [ -f "$CFG" ] || CFG=/boot/config.txt
grep -q '^gpu_mem=' "$CFG" && sed -i 's/^gpu_mem=.*/gpu_mem=16/' "$CFG" || echo 'gpu_mem=16' >> "$CFG"
CMD=/boot/firmware/cmdline.txt; [ -f "$CMD" ] || CMD=/boot/cmdline.txt
grep -q 'cgroup_enable=memory' "$CMD" || sed -i '1 s/$/ cgroup_enable=memory cgroup_memory=1/' "$CMD"

# 5) Services a headless robot doesn't need (Bluetooth stays ON for the Joy-Cons)
systemctl disable --now ModemManager triggerhappy 2>/dev/null || true

systemctl restart docker || true
echo "Done. Reboot to apply gpu_mem / cgroup changes."
