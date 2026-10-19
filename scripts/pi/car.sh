#!/usr/bin/env bash
# car.sh: one-word commands for the racer on the Raspberry Pi.   Run ON THE PI.
#
#   scripts/pi/car.sh setup       one time: camera stream service + your group ids
#   scripts/pi/car.sh eyes        OLED + camera person tracking, in this terminal (Ctrl+C stops)
#   scripts/pi/car.sh servo       Joy-Con + servo only (no OLED / camera), in this terminal
#   scripts/pi/car.sh auto        autonomy test: LiDAR + autonomy + dashboard, no OLED / camera, in this terminal
#   scripts/pi/car.sh car         EVERYTHING, in the background (restarts on boot)
#   scripts/pi/car.sh boot        what the racer.service runs at power-on: wait for I2C / LiDAR, then 'car'
#   scripts/pi/car.sh stop        stop whatever is running
#   scripts/pi/car.sh logs        watch the background car
#   scripts/pi/car.sh status      what is running, camera service, memory
#   scripts/pi/car.sh oled-test [image]    host OLED test pattern / image (stops the car first)
#   scripts/pi/car.sh oled-anim [gif]      host OLED animation          (stops the car first)
#
# DRY=1 scripts/pi/car.sh eyes     prints the commands instead of running them.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."          # repo root, so it works from any folder

if [ "${1:-}" = boot ]; then          # right after power-on the I2C / USB devices may not exist yet
  for _ in $(seq 30); do [ -e /dev/i2c-1 ] && break; sleep 1; done
  for _ in $(seq 10); do [ -e /dev/ttyUSB0 ] && break; sleep 1; done
fi

COMPOSE=(docker compose -f docker-compose.pi.yml)
[ -e /dev/ttyUSB0 ] && COMPOSE+=(-f docker-compose.lidar.yml)   # RPLIDAR plugged in
LAUNCH=(ros2 launch pi_drive pi_drive.launch.py)
IMAGE=airacer:pi
OLED_PY="${OLED_PY:-$HOME/oled/bin/python}"

run() {                                            # DRY=1 -> just show it
  if [ "${DRY:-0}" = 1 ]; then printf '+'; printf ' %q' "$@"; echo; else "$@"; fi
}
die() { echo "ERROR: $*" >&2; exit 1; }
dry() { [ "${DRY:-0}" = 1 ]; }

stop_all() {                                       # background car AND one-off runs
  dry && { echo "+ docker stop <running $IMAGE containers>"; return; }
  local ids; ids=$(docker ps -q --filter "ancestor=$IMAGE" || true)
  if [ -n "$ids" ]; then echo "stopping running car..."; docker stop $ids >/dev/null; fi
}

preflight() {
  dry && return 0
  command -v docker >/dev/null || die "docker not found"
  docker image inspect "$IMAGE" >/dev/null 2>&1 || die "image $IMAGE not on this Pi. Send it from your Mac (see scripts/pi/README.md)"
  [ -e /dev/i2c-1 ] || die "/dev/i2c-1 missing: enable I2C (sudo raspi-config nonint do_i2c 0) and reboot"
  grep -q '^I2C_GID=' .env 2>/dev/null || die "group ids not in .env yet: run  scripts/pi/car.sh setup"
}

launch_fg() {                                      # foreground launch with extra launch args
  stop_all; preflight
  run "${COMPOSE[@]}" run --rm racer "${LAUNCH[@]}" "$@"
}

setup() {
  echo "== camera stream"
  command -v rpicam-vid >/dev/null || run sudo apt install -y rpicam-apps-lite
  run sudo cp scripts/pi/camera-stream.service /etc/systemd/system/
  run sudo systemctl daemon-reload
  run sudo systemctl enable --now camera-stream
  echo "== group ids -> .env"
  if dry; then echo "+ (would append GPIO_GID / DIALOUT_GID / INPUT_GID / I2C_GID to .env)"
  elif grep -q '^I2C_GID=' .env 2>/dev/null; then echo "already in .env"
  else
    for g in gpio:GPIO dialout:DIALOUT input:INPUT i2c:I2C; do
      echo "${g##*:}_GID=$(getent group "${g%%:*}" | cut -d: -f3)"
    done >> .env
  fi
  dry || { echo "-- .env ends with:"; tail -4 .env; }
  echo "done. Next:  scripts/pi/car.sh eyes"
}

status() {
  echo "== containers";      docker ps --filter "ancestor=$IMAGE" --format '{{.Names}}  {{.Status}}'
  echo "== camera stream";   systemctl is-active camera-stream || true
  echo "== memory";          free -h | head -2
  echo "== container usage"; docker stats --no-stream --format '{{.Name}}  mem {{.MemUsage}}  cpu {{.CPUPerc}}' $(docker ps -q --filter "ancestor=$IMAGE") 2>/dev/null || true
}

cmd="${1:-help}"; shift || true
case "$cmd" in
  setup)  setup ;;
  eyes)   launch_fg lidar:=false autonomy:=false dashboard:=false ;;
  servo)  launch_fg lidar:=false autonomy:=false dashboard:=false person:=false oled:=false ;;
  auto)   launch_fg oled:=false person:=false ;;
  car|boot) stop_all; preflight; run "${COMPOSE[@]}" up -d
          echo "car is running in the background.  scripts/pi/car.sh logs | status | stop" ;;
  stop)   stop_all; echo "stopped" ;;
  logs)   run docker logs -f --tail 50 racer ;;
  status) status ;;
  oled-test) stop_all; run "$OLED_PY" scripts/pi/oled_test.py "$@"
             echo "(the car is stopped. Restart it with:  scripts/pi/car.sh car)" ;;
  oled-anim) stop_all; run "$OLED_PY" scripts/pi/oled_animate.py "$@"
             echo "(the car is stopped. Restart it with:  scripts/pi/car.sh car)" ;;
  *)      sed -n '2,16p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' ;;
esac
