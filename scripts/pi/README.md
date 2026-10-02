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
This adds a 2 GB swap file, caps docker logs and frees GPU memory.

## 3. Joy-Cons, pins and tuning
See [pi_drive/README.md](../../ros2_ws/src/racer_ros/pi_drive/README.md) (pairing, button check, pins).

## 4. Run
```bash
getent group gpio dialout input    # put the numbers in GPIO_GID / DIALOUT_GID / INPUT_GID if different
docker compose -f docker-compose.pi.yml up -d
docker logs -f racer                # starts Joy-Con teleop, GPIO driver, LiDAR, autonomy, dashboard
docker exec -it racer bash
```
Dashboard: `http://<pi-ip>:8080` from your Mac. Without pins set the car runs DRY (nothing moves).
Check memory: `docker stats racer` (limit is 640 MB).
