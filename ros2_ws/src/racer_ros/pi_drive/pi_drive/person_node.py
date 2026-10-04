#!/usr/bin/env python3
"""
person_detector: camera -> YOLOv8 (ONNX, OpenCV) -> ONE person.

Publishes
    /person           geometry_msgs/PointStamped   only while someone is seen:
                        point.x, point.y = center of the chosen person, 0..1 (0,0 = image top-left)
                        point.z          = confidence
    /person_detected  std_msgs/Bool                every detection cycle

Only one person is reported: the largest box (closest), or the most confident (`select`).

`source`:
    tcp://127.0.0.1:8888   Raspberry Pi ribbon (CSI) camera: the HOST runs `rpicam-vid` and streams MJPEG
                           to that port (see scripts/pi/camera-stream.service); reconnects on its own
    /dev/video0            USB webcam
    a video file / an image file (an image is served over and over: handy for testing with no camera)
Runs at detect_hz (default 2): a Pi 3 can't do much more with YOLO, and it must not starve the
GPIO driver of CPU. The launch file also runs it at lower priority (nice).
"""
import os
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from ament_index_python.packages import get_package_share_directory

from pi_drive.person_detect import PersonDetector, center_norm, pick_one

IMAGE_EXT = ('.jpg', '.jpeg', '.png', '.bmp')
STREAM_PREFIXES = ('tcp://', 'udp://', 'http://', 'rtsp://')


class PersonDetectorNode(Node):
    def __init__(self):
        super().__init__('person_detector')

        d = self.declare_parameter
        d('source', 'tcp://127.0.0.1:8888')
        d('width', 320)
        d('height', 240)
        d('fourcc', 'MJPG')              # webcams send MJPG at far lower CPU cost than raw YUYV
        d('model_path', 'yolov8n.onnx')  # relative = the package config dir
        d('input_size', 320)             # must match the imgsz used for the ONNX export
        d('conf_threshold', 0.5)
        d('nms_threshold', 0.45)
        d('select', 'largest')           # largest | confident
        d('detect_hz', 2.0)
        d('cv_threads', 2)
        d('debug_image_path', '')        # e.g. /tmp/person.jpg: last frame with the box, once a second

        p = lambda n: self.get_parameter(n).value
        model = p('model_path')
        if not os.path.isabs(model):
            model = os.path.join(get_package_share_directory('pi_drive'), 'config', model)
        self.detector = None
        try:
            self.detector = PersonDetector(model, p('input_size'), p('conf_threshold'),
                                           p('nms_threshold'), p('cv_threads'))
            self.get_logger().info(f'model loaded: {model}')
        except Exception as exc:
            self.get_logger().error(f'model not loaded ({exc}); no detections will be published')

        self.select = p('select')
        self.debug_path = p('debug_image_path')
        self.last_debug = 0.0
        src = str(p('source'))
        self.source = int(src) if src.isdigit() else src
        self.still = None
        self.frame = None
        self.frame_id = 0
        self.seen_id = -1
        self.lock = threading.Lock()
        self.running = True

        self.person_pub = self.create_publisher(PointStamped, '/person', 10)
        self.detected_pub = self.create_publisher(Bool, '/person_detected', 10)
        threading.Thread(target=self._capture_loop, daemon=True).start()
        self.create_timer(1.0 / float(p('detect_hz')), self._detect)

    # ---- capture thread: always keeps only the newest frame (no lag from a buffer) ----
    def _open(self):
        p = lambda n: self.get_parameter(n).value
        if isinstance(self.source, str) and self.source.lower().endswith(IMAGE_EXT):
            img = cv2.imread(self.source)
            if img is None:
                raise IOError(f'cannot read image {self.source}')
            self.still = img
            return None
        if str(self.source).startswith(STREAM_PREFIXES):
            # low-latency FFmpeg options: don't buffer/probe for seconds before the first frame
            os.environ.setdefault('OPENCV_FFMPEG_CAPTURE_OPTIONS',
                                  'fflags;nobuffer|flags;low_delay|probesize;32768|analyzeduration;0')
        cap = cv2.VideoCapture(self.source, cv2.CAP_V4L2) if (
            isinstance(self.source, int) or str(self.source).startswith('/dev/')) else cv2.VideoCapture(self.source)
        if not cap.isOpened():
            raise IOError(f'cannot open camera {self.source}')
        fourcc = str(p('fourcc'))
        if len(fourcc) == 4:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(p('width')))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(p('height')))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _capture_loop(self):
        cap = None
        while self.running:
            try:
                if self.still is None and cap is None:
                    cap = self._open()
                if self.still is not None:
                    with self.lock:
                        self.frame, self.frame_id = self.still, self.frame_id + 1
                    time.sleep(0.2)
                    continue
                ok, frame = cap.read()
                if not ok:
                    raise IOError('camera read failed')
                with self.lock:
                    self.frame, self.frame_id = frame, self.frame_id + 1
            except Exception as exc:
                self.get_logger().warn(f'camera: {exc}; retrying in 2 s', throttle_duration_sec=20.0)
                if cap is not None:
                    cap.release()
                cap = None
                time.sleep(2.0)
        if cap is not None:
            cap.release()

    # ---- detection timer ----
    def _detect(self):
        if self.detector is None:
            return
        with self.lock:
            frame, fid = self.frame, self.frame_id
        if frame is None or fid == self.seen_id:
            return
        self.seen_id = fid

        t = time.monotonic()
        dets = self.detector.detect(frame)
        ms = (time.monotonic() - t) * 1000.0
        best = pick_one(dets, self.select)

        self.detected_pub.publish(Bool(data=best is not None))
        if best is not None:
            h, w = frame.shape[:2]
            x, y = center_norm(best, w, h)
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            msg.point.x, msg.point.y, msg.point.z = float(x), float(y), float(best.conf)
            self.person_pub.publish(msg)
        self.get_logger().info(
            f'inference {ms:.0f} ms, persons={len(dets)}' + (f', chosen x={x:.2f} y={y:.2f} conf={best.conf:.2f}' if best else ''),
            throttle_duration_sec=5.0)

        if self.debug_path and time.monotonic() - self.last_debug > 1.0:
            self.last_debug = time.monotonic()
            vis = frame.copy()
            if best is not None:
                cv2.rectangle(vis, (int(best.x1), int(best.y1)), (int(best.x2), int(best.y2)), (0, 255, 0), 2)
                cv2.circle(vis, (int((best.x1 + best.x2) / 2), int((best.y1 + best.y2) / 2)), 5, (0, 0, 255), -1)
            cv2.imwrite(self.debug_path, vis)

    def stop(self):
        self.running = False


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
