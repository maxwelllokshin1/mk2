#!/usr/bin/env python3
"""
expression_node: the car's personality. Picks the expression (published on /expression for the OLED eyes)
and speaks a random voice line of that expression through the MAX98357 amps. The rules are in expressions.py.

Inputs:
    /drive_cmd         Float32MultiArray [steer, throttle]  |throttle| > drive_throttle_min = driving
    /person_detected   Bool                                 from person_detector
    /lap_completed     Empty                                from lap_detector      -> EXCITED
    /excite            Empty                                Joy-Con excite button  -> EXCITED
    /surprise          Empty                                from hazard_node       -> SURPRISED (about to hit a wall)
    /collision         Empty                                from hazard_node       -> DIZZY
Output:
    /expression          String (latched)                   e.g. 'sleepy'
    /expression_closing  String                             eyelid faces: start playing the lid backwards

Playback is one line at a time. Every file goes through ffmpeg (loudness-normalised) into aplay; without ffmpeg,
.mp3 plays through mpg123. Volume is a software scale, so 0.75 = 75 %.
"""
import os
import shutil
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool, Empty, Float32MultiArray, String

from pi_drive.expressions import EXPRESSIONS, Director, ShuffleBag, Timing, eye_plan, find_voice_files


def gif_seconds(path):
    """How long a gif plays once through, or None (no PIL / unreadable)."""
    try:
        from PIL import Image
        with Image.open(path) as im:
            total = 0
            for i in range(getattr(im, 'n_frames', 1)):
                im.seek(i)
                total += im.info.get('duration', 100)
        return max(total, 100) / 1000.0
    except Exception:
        return None


class ExpressionNode(Node):
    def __init__(self):
        super().__init__('expression')
        d = self.declare_parameter
        d('expressions_dir', '')                    # '' = the package config/expressions folder
        d('volume', 0.75)                           # 0..1
        d('normalize', True)                        # loudness-normalise every line (ffmpeg loudnorm) so quiet recordings are as loud as the rest
        d('audio_device', 'plughw:CARD=sndrpihifiberry')   # ALSA device of the I2S amps ('default' = system default)
        d('drive_throttle_min', 0.05)               # |throttle| above this counts as driving
        d('drive_hold_s', 3.0)                      # stay in "driving" this long after the throttle drops
        d('person_hold_s', 3.0)                     # a person counts as present this long after the last detection
        for f in Timing.__dataclass_fields__.values():
            d(f.name, float(f.default))

        p = lambda n: self.get_parameter(n).value
        self.root = p('expressions_dir') or os.path.join(get_package_share_directory('pi_drive'), 'config', 'expressions')
        self.device = str(p('audio_device'))
        self.normalize = bool(p('normalize'))
        self.scale = int(32768 * min(max(float(p('volume')), 0.0), 1.0))   # mpg123 -f: 32768 = 100 %
        self.thr_min = float(p('drive_throttle_min'))
        self.drive_hold = float(p('drive_hold_s'))
        self.person_hold = float(p('person_hold_s'))
        lengths = {}                                   # faces that last as long as their gif (bored)
        for name, spec in EXPRESSIONS.items():
            if spec.once:
                seconds = gif_seconds(next(iter(eye_plan(self.root, name)['loop']), ''))
                if seconds:
                    lengths[name] = seconds
        self.director = Director(Timing(**{k: float(p(k)) for k in Timing.__dataclass_fields__}), lengths=lengths)
        self.bags = {name: ShuffleBag() for name in EXPRESSIONS}
        self.proc = None
        self.last_drive = self.last_person = None

        self.have_ffmpeg = shutil.which('ffmpeg') is not None and shutil.which('aplay') is not None
        if not self.have_ffmpeg:
            self.get_logger().warn('ffmpeg/aplay not found: only .mp3 (mpg123, no loudness normalising) will play '
                                   '(apt install ffmpeg alsa-utils)')
        if shutil.which('mpg123') is None and not self.have_ffmpeg:
            self.get_logger().error('no player found: no sound will play')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.expr_pub = self.create_publisher(String, '/expression', latched)
        self.create_subscription(Float32MultiArray, '/drive_cmd', self._on_cmd, 10)
        self.create_subscription(Bool, '/person_detected', self._on_person, 10)
        self.create_subscription(Empty, '/lap_completed', lambda _m: self._excite('lap'), 10)
        self.create_subscription(Empty, '/excite', lambda _m: self._excite('button'), 10)
        self.create_subscription(Empty, '/surprise', lambda _m: self._force('surprised'), 10)
        self.create_subscription(Empty, '/collision', lambda _m: self._force('dizzy'), 10)
        self.closing_pub = self.create_publisher(String, '/expression_closing', 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f'expression up: volume {self.scale / 32768:.0%}, device {self.device}, folder {self.root}')

    # ---------- inputs ----------
    def _on_cmd(self, msg):
        if len(msg.data) >= 2 and abs(msg.data[1]) > self.thr_min:
            self.last_drive = time.monotonic()

    def _on_person(self, msg):
        if msg.data:
            self.last_person = time.monotonic()

    def _excite(self, why):
        self.get_logger().info(f'excited ({why})')
        self.director.excite(time.monotonic())

    def _force(self, name):
        self.get_logger().info(f'{name} (hazard)')
        self.director.force(time.monotonic(), name)

    # ---------- sound ----------
    def _speaking(self):
        if self.proc is not None and self.proc.poll() is not None:
            self.proc = None
        return self.proc is not None

    def _stop(self):
        if self.proc is not None:
            self._signal(signal.SIGTERM)
            try:
                self.proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._signal(signal.SIGKILL)
        self.proc = None

    def _signal(self, sig):
        try:
            os.killpg(self.proc.pid, sig)          # the whole ffmpeg | aplay pipeline, not just the shell
        except OSError:
            pass

    def _command(self, path):
        if path.lower().endswith('.mp3') and not (self.normalize and self.have_ffmpeg):
            cmd = ['mpg123', '-q', '-f', str(self.scale)]
            if self.device != 'default':
                cmd += ['-a', self.device]
            return cmd + [path]
        # ffmpeg decodes, evens out the loudness and scales to a wav stream, aplay plays it
        gain = f'volume={self.scale / 32768:.3f}'
        chain = f'loudnorm=I=-12:TP=-1:LRA=7,aresample=48000,{gain}' if self.normalize else gain
        return ['sh', '-c', 'ffmpeg -v quiet -i "$1" -filter:a "$2" -f wav - | aplay -q ${3:+-D "$3"} -',
                'sh', path, chain, '' if self.device == 'default' else self.device]

    def _say(self, name, interrupt):
        files = find_voice_files(self.root, name)         # re-read each time: new voice files need no restart
        if not files:
            self.get_logger().warn(f'no voice lines (.mp3/.m4a/.wav) for {name}', throttle_duration_sec=60.0)
            return
        if self._speaking():
            if not interrupt:
                return
            self._stop()
        path = self.bags[name].next(files)
        try:
            self.proc = subprocess.Popen(self._command(path), stdin=subprocess.DEVNULL, start_new_session=True,
                                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f'{name}: {os.path.basename(path)}')
        except OSError as exc:
            self.get_logger().error(f'cannot play {path}: {exc}', throttle_duration_sec=30.0)

    # ---------- the behavior ----------
    def _tick(self):
        now = time.monotonic()
        driving = self.last_drive is not None and now - self.last_drive < self.drive_hold
        person = self.last_person is not None and now - self.last_person < self.person_hold
        for event in self.director.step(now, driving, person, self._speaking()):
            if event[0] == 'expression':
                self.expr_pub.publish(String(data=event[1]))
                self.get_logger().info(f'expression: {event[1]}')
            elif event[0] == 'closing':
                self.closing_pub.publish(String(data=event[1]))
            else:
                self._say(event[1], event[2])

    def shutdown(self):
        self._stop()


def main(args=None):
    rclpy.init(args=args)
    node = ExpressionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
