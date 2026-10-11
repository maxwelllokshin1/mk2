# Running the racer image on a 1 GB Raspberry Pi 3

## 1. Build on your Mac / PC (NOT on the Pi)
```bash
docker buildx build --platform linux/arm64 -f Dockerfile.pi -t airacer:pi --load .
docker save airacer:pi | gzip | ssh pi@<pi-ip> 'gunzip | docker load'
```
Then clone this repo on the Pi: the compose file bind-mounts `ros2_ws/src/racer_ros/pi_drive/config/pi_drive.yaml` so you can edit pins and tuning there without rebuilding.

## 2. One-time Pi setup (64-bit Raspberry Pi OS **Lite**, no desktop)
```bash
sudo bash scripts/pi/pi_setup.sh && sudo reboot
```
This adds a 2 GB swap file, caps docker logs and sets gpu_mem=64 (the camera needs it).

## 3. Joy-Cons, pins and tuning
See [pi_drive/README.md](../../ros2_ws/src/racer_ros/pi_drive/README.md) (pairing, button check, pins).

## 4. Run: `scripts/pi/car.sh` (run it ON the Pi)

| Command | What it does |
|---|---|
| `scripts/pi/car.sh setup` | one time: camera stream service + your group ids in `.env` |
| `scripts/pi/car.sh eyes` | OLED eyes + camera person tracking, output in this terminal (Ctrl+C stops) |
| `scripts/pi/car.sh servo` | Joy-Con + servo only (no OLED / camera) |
| `scripts/pi/car.sh car` | everything, in the background, restarts on boot |
| `scripts/pi/car.sh stop` / `logs` / `status` | stop it / watch it / what is running + memory |
| `scripts/pi/car.sh oled-test [image]` | host OLED test (stops the car first, because only one program can use the screen) |
| `scripts/pi/car.sh oled-anim [gif]` | host OLED animation (same) |

`DRY=1 scripts/pi/car.sh eyes` prints the commands instead of running them. If it says "permission denied",
run it as `bash scripts/pi/car.sh ...` or `chmod +x scripts/pi/car.sh`.
Dashboard (when the full car runs): `http://<pi-ip>:8080` from your Mac.
Check memory: `scripts/pi/car.sh status`.
