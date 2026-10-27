"""Laptop webcam -> MediaPipe Tasks Gesture Recognizer -> ROS String + live UI.

Capture and inference run outside the ROS executor. Only fresh completed results
refresh the control stream, so a blocked webcam/inference thread cannot keep a
robot moving by repeatedly publishing an old recognition result.
"""
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

from .wave import WaveDetector
from .pose_geometry import analyze, resolve


CONNECTIONS = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
               (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15),
               (15, 16), (13, 17), (0, 17), (17, 18), (18, 19), (19, 20)]
CONTROL_CLASSES = {'Pointing_Up', 'Victory', 'Thumb_Up', 'Open_Palm', 'Closed_Fist'}


class GestureRecognitionNode(Node):
    def __init__(self):
        super().__init__('gesture_recognition_node')
        for key, value in {'camera_index': 0, 'width': 640, 'height': 480,
                           'confidence': 0.65, 'stop_confidence': 0.50,
                           'freshness_timeout': 0.30, 'show_window': True,
                           'window_x': 20, 'window_y': 60,
                           'model': str(Path.home() / '.cache/turtlebot4_gesture_hri/gesture_recognizer.task')}.items():
            self.declare_parameter(key, value)
        model = Path(self.value('model')).expanduser()
        if not model.is_file():
            raise FileNotFoundError(f'{model}: run tools/download_model.py first')
        import mediapipe as mp
        self.mp = mp
        options = mp.tasks.vision.GestureRecognizerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(model),
                                             delegate=mp.tasks.BaseOptions.Delegate.CPU),
            running_mode=mp.tasks.vision.RunningMode.VIDEO, num_hands=2,
            min_hand_detection_confidence=0.5, min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5)
        self.recognizer = mp.tasks.vision.GestureRecognizer.create_from_options(options)
        self.capture = cv2.VideoCapture(self.value('camera_index'), cv2.CAP_V4L2)
        if not self.capture.isOpened():
            self.recognizer.close()
            raise RuntimeError('Cannot open webcam; check camera_index and device access')
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.value('width'))
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.value('height'))
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.capture.set(cv2.CAP_PROP_FPS, 30)
        cv2.setNumThreads(1)
        self.publisher = self.create_publisher(String, '/detected_gesture',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.debug = self.create_publisher(String, '/gesture_debug', 1)
        self.behavior = {'mode': 'IDLE', 'selected': None, 'reason': 'Waiting for controller'}
        self.state_sub = self.create_subscription(String, '/behavior_state', self.on_state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.latest = self.result = None
        self.failed = False
        self.wave = WaveDetector()
        self.last_timestamp = -1
        self.last_debug = 0.0
        self.last_draw = -math.inf
        self.display = None
        self.positioned_window = False
        self.title = 'BAI 6 - Webcam gestures / TurtleBot 4 (NO keyboard teleoperation)'
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.inference_thread = threading.Thread(target=self.infer_loop, daemon=True)
        self.capture_thread.start(); self.inference_thread.start()
        self.timer = self.create_timer(0.05, self.publish_result)
        self.get_logger().info('Webcam ready. Select Wave/Pointing_Up/Victory; then Thumb_Up. Open_Palm stops.')

    def value(self, name):
        return self.get_parameter(name).value

    def on_state(self, msg):
        try:
            self.behavior = json.loads(msg.data)
        except ValueError:
            pass

    def capture_loop(self):
        while not self.stop_event.is_set():
            ok, frame = self.capture.read()
            stamp = time.monotonic()
            if not ok:
                self.stop_event.wait(0.05)
                continue
            with self.lock:
                self.latest = (stamp, cv2.flip(frame, 1))  # Mirror only the displayed recognition feed.

    def infer_loop(self):
        try:
            while not self.stop_event.is_set():
                with self.lock:
                    sample, self.latest = self.latest, None
                if sample is None:
                    self.stop_event.wait(0.005)
                    continue
                captured, frame = sample
                if time.monotonic() - captured > self.value('freshness_timeout'):
                    continue
                timestamp = max(self.last_timestamp + 1, int(captured * 1000))
                self.last_timestamp = timestamp
                image = self.mp.Image(image_format=self.mp.ImageFormat.SRGB,
                                      data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                result = self.recognizer.recognize_for_video(image, timestamp)
                label, score, published = 'No_Hand', 0.0, 'No_Hand'
                evidence={'pose_source':'no_hand','pose_label':None,'finger_states':[]}
                candidates=[(i,analyze(points,frame.shape[0]/frame.shape[1]))
                            for i,points in enumerate(result.hand_landmarks)]
                evidence['hand_rejection_reasons']=[p.reason for _,p in candidates if not p.visible]
                candidates=[(i,p) for i,p in candidates if p.visible]
                if len(candidates) == 1:
                    index,pose=candidates[0]
                    label, score = 'Unknown', 0.0
                    if len(result.gestures)>index and result.gestures[index]:
                        best = result.gestures[index][0]
                        label, score = best.category_name, float(best.score)
                    published,source=resolve(pose,label,score,self.value('confidence'),self.value('stop_confidence'))
                    evidence={'pose_source':source,'pose_label':published,'finger_states':list(pose.fingers)}
                    wrist = result.hand_landmarks[index][0]
                    wave = self.wave.update(captured, wrist.x, wrist.y, published if published!='Hand_Present' else 'None',
                                            published=='Open_Palm',
                                            self.behavior.get('mode') != 'RUNNING')
                    if wave:
                        published = 'Wave'
                    h, w = frame.shape[:2]
                    points = [(int(p.x * w), int(p.y * h)) for p in result.hand_landmarks[index]]
                    for a, b in CONNECTIONS:
                        cv2.line(frame, points[a], points[b], (60, 215, 80), 2)
                    for point in points:
                        cv2.circle(frame, point, 3, (10, 210, 255), -1)
                else:
                    if not candidates:
                        self.wave.missing(captured)
                    else:
                        self.wave.reset()
                    if len(candidates) > 1:
                        label = 'Two_Hands (control disabled)'
                    elif result.hand_landmarks:
                        label = 'Hand cropped / too small - show whole hand'
                with self.lock:
                    self.result = (captured, time.monotonic(), published, label, score, frame,
                                   len(result.hand_landmarks),evidence)
        except Exception as exc:
            self.failed = True
            self.get_logger().error(f'Inference stopped: {exc}')

    def publish_result(self):
        now = time.monotonic()
        with self.lock:
            sample = self.result
        published, label, score = 'No_Hand', 'No fresh result', 0.0
        age = inference_ms = None
        hand_count = 0
        evidence={}
        if sample is not None:
            captured, finished, command, label, score, frame, hand_count,evidence = sample
            age = now - captured
            inference_ms = (finished - captured) * 1000
            if now - captured <= self.value('freshness_timeout') and not self.failed:
                published = command
            else:
                label = 'STALE CAMERA / INFERENCE - robot must stop'
            self.display = frame.copy()
        self.publisher.publish(String(data=published))
        if now - self.last_debug > 0.2:
            self.debug.publish(String(data=json.dumps({'gesture': published, 'raw_label': label,
                'confidence': score, 'fresh': sample is not None and age <= self.value('freshness_timeout') and not self.failed,
                'frame_age_s': age, 'capture_to_result_ms': inference_ms, 'hand_count': hand_count,
                **evidence,
                'source': 'live webcam / MediaPipe AI categories + AI landmark joint rules; Wave is temporal motion'})))
            self.last_debug = now
        if self.value('show_window') and now - self.last_draw >= 0.10:
            self.last_draw = now
            canvas = np.zeros((670, 800, 3), dtype=np.uint8)
            if self.display is not None:
                canvas[:480, :640] = cv2.resize(self.display, (640, 480))
            if published in ('No_Hand', 'Hand_Present'):
                # Display guidance only: inference still uses the original frame.
                cv2.rectangle(canvas, (360,90), (625,410), (255,180,60), 2)
                cv2.putText(canvas, 'DAT CA BAN TAY O DAY', (361,78),
                            cv2.FONT_HERSHEY_SIMPLEX,.45,(255,210,100),1)
            mode = self.behavior.get('mode', 'IDLE')
            choice = {'shuttle': 'TIEN-LUI 4 CHU KY', 'circle': 'DI MOT VONG TRON',
                      'greet': 'XOAY CHAO'}.get(self.behavior.get('selected'), '')
            reason = str(self.behavior.get('reason', ''))
            color = (80, 210, 255)
            if mode == 'SELECTED':
                heading = f'DA CHON: {choice}'
                action = 'BUOC 2: GIO NGON CAI (Thumb Up) DE CHAY'
            elif mode == 'RUNNING':
                color = (80, 245, 100)
                heading = f'DANG CHAY: {choice}'
                action = f"GIU NGON CAI | Doan {self.behavior.get('segment', 0)+1}/{self.behavior.get('segments_total', 0)}"
                if self.behavior.get('selected') == 'shuttle':
                    cycle = self.behavior.get('shuttle_cycle', 1)
                    phase = 'TIEN 0.50 m' if self.behavior.get('shuttle_phase') == 'forward' else 'LUI VE DIEM DAU'
                    heading = f'TIEN-LUI: LAN {cycle}/4 - {phase}'
                    action = f"Da xong {self.behavior.get('shuttle_cycles_completed', 0)}/4 lan | GIU NGON CAI"
            elif mode == 'COMPLETED':
                color = (80, 245, 100)
                heading, action = 'DA HOAN THANH', 'Chon lai 1 ngon / 2 ngon, roi gio ngon cai'
                if self.behavior.get('selected') == 'shuttle':heading='DA XONG DU 4 LAN TIEN-LUI'
            elif mode in ('FAULT', 'STOPPED'):
                color = (80, 120, 255)
                heading = ('DA DUNG: MAT TIN HIEU TAY' if 'signal lost' in reason else
                           'DA DUNG: XOE BAN TAY' if 'Open_Palm' in reason else 'DA DUNG / CHUA SAN SANG')
                action = 'CHON LAI 1 NGON / 2 NGON, ROI GIO NGON CAI'
                if published == 'No_Hand':
                    heading, action = 'CHUA THAY DU BAN TAY', 'GIO TAY NGANG VAI, THAY DU CO TAY VA CAC NGON'
            else:
                heading = ('CHUA THAY TAY TRONG CAMERA' if published == 'No_Hand' else
                           'THAY TAY, CHUA CHON HANH VI' if published == 'Hand_Present' else 'DANG CHO CHON HANH VI')
                action = 'BUOC 1: VAY TAY / 1 NGON TRO / 2 NGON CHU V'
            for i, (pose, meaning) in enumerate([('VAY TAY <->', 'CHON CHAO'), ('1 NGON', 'TIEN-LUI x4'),
                                                ('2 NGON', 'VONG TRON'), ('NGON CAI', 'XAC NHAN'),
                                                ('XOE TAY', 'DUNG NGAY'), ('NAM TAY', 'HUY HANH VI')]):
                y = 30 + i * 73
                cv2.putText(canvas, pose, (650,y), cv2.FONT_HERSHEY_SIMPLEX,.42,(80,210,255),1)
                cv2.putText(canvas, meaning, (650,y+24), cv2.FONT_HERSHEY_SIMPLEX,.39,(220,220,220),1)
            cv2.rectangle(canvas, (0, 480), (799, 548), (28, 38, 48), -1)
            for text, y, scale, thickness in [(heading, 508, .68, 2), (action, 537, .57, 1)]:
                cv2.putText(canvas, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                            scale, color, thickness)
            lines = [f'Model: {label} p={score:.2f} | Detect: {published}',
                     f'Robot: {mode} | {reason}',
                     '1 ngon / 2 ngon CHI CHON. Ngon cai MOI CHAY.',
                     'KHI CHAY: GIU NGON CAI | XOE TAY: DUNG | NAM TAY: HUY']
            for i, text in enumerate(lines):
                cv2.putText(canvas, text, (10, 572 + i * 27), cv2.FONT_HERSHEY_SIMPLEX,
                            .47, color, 1)
            cv2.imshow(self.title, canvas)
            if not self.positioned_window:
                cv2.moveWindow(self.title, self.value('window_x'), self.value('window_y'))
                self.positioned_window = True
            cv2.waitKey(1)  # No keyboard-to-robot mappings.
            if cv2.getWindowProperty(self.title, cv2.WND_PROP_VISIBLE) < 1:
                self.stop_event.set()

    def close(self):
        self.publisher.publish(String(data='Open_Palm'))
        self.stop_event.set()
        self.capture_thread.join(timeout=1)
        self.inference_thread.join(timeout=2)
        if not self.capture_thread.is_alive():
            self.capture.release()
        if not self.inference_thread.is_alive():
            self.recognizer.close()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    node = None
    try:
        node = GestureRecognitionNode()
        while rclpy.ok() and not node.stop_event.is_set() and not node.failed:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            if rclpy.ok():
                node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
