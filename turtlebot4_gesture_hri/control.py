"""Deterministic, ROS-independent selection/confirmation and motion controller.

All time arguments are monotonic WALL seconds. Motion progress uses odometry,
so a slow/paused simulator cannot silently finish a timed motion sequence.
"""
from dataclasses import dataclass
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clip(value, limit):
    return max(-limit, min(limit, value))


@dataclass
class Settings:
    stable_seconds: float = 0.30
    stable_samples: int = 3
    signal_timeout: float = 0.65
    selection_signal_timeout: float = 1.8
    cancel_selection_seconds: float = 0.25
    sensor_timeout: float = 0.8
    selection_timeout: float = 12.0
    behavior_timeout: float = 180.0
    progress_timeout: float = 12.0
    linear_speed: float = 0.28
    angular_speed: float = 1.0
    # A fixed pair of endpoints prevents accumulated drift over four cycles.
    # prepare_arena enables long reversing only on the Gazebo driver.
    shuttle_distance: float = 0.50
    circle_radius: float = 0.75
    circle_speed: float = 0.26
    obstacle_distance: float = 0.42
    rotation_clearance: float = 0.26

    def validate(self):
        for key, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if not isinstance(self.stable_samples, int) or self.stable_samples < 2:
            raise ValueError('stable_samples must be an integer >= 2')
        if self.linear_speed > 0.30 or self.angular_speed > 1.2 or self.circle_speed > 0.28:
            raise ValueError('Demo speed limits exceeded')
        if self.circle_radius < 0.25 or self.obstacle_distance < 0.30:
            raise ValueError('Demo clearance/radius too small')
        if self.shuttle_distance > 0.8 or self.circle_radius > 1.0:
            raise ValueError('Keep demo motion inside the open arena')


class Controller:
    CHOICES = {'Wave': 'greet', 'Pointing_Up': 'shuttle', 'Victory': 'circle'}
    PRESENT = set(CHOICES) | {'Thumb_Up', 'Open_Palm', 'Closed_Fist', 'Hand_Present'}

    def __init__(self, settings=None):
        self.cfg = settings or Settings()
        self.cfg.validate()
        self.mode = 'IDLE'
        self.selected = None
        self.reason = 'Choose a gesture, then confirm with Thumb_Up'
        self.armed = False
        self.last_signal = -math.inf
        self.last_odom = self.last_scan = self.last_dock = self.last_hazard = -math.inf
        self.docked = True
        self.hazard = False
        self.pose = None
        self.path_length = 0.0
        self.candidate = self.last_emitted = None
        self.candidate_since = 0.0
        self.candidate_samples = 0
        self.selected_at = self.started = 0.0
        self.program = []
        self.segment = 0
        self.anchor = None
        self.shuttle_origin = None
        self.settle_until = 0.0
        self.progress_mark = None
        self.progress_at = 0.0
        self.events = []

    def event(self, now, reason):
        self.reason = reason
        self.events.append({'wall_monotonic': now, 'mode': self.mode,
                            'selected': self.selected, 'reason': reason,
                            'segment': self.segment, 'pose': self.pose})

    def stop(self, now, reason, clear=False, fault=False):
        self.program = []
        self.anchor = None
        self.shuttle_origin = None
        self.armed = False
        # A fault must not leave the previous selection edge latched. Showing
        # that same choice again should work; Thumb_Up alone still cannot run.
        self.candidate = self.last_emitted = None
        self.candidate_samples = 0
        if clear:
            self.selected = None
        self.mode = 'FAULT' if fault else ('IDLE' if clear else 'STOPPED')
        self.event(now, reason)

    def on_pose(self, x, y, yaw, now):
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return
        if self.pose is None:
            self.pose = (x, y, yaw)
        else:
            px, py, unwrapped = self.pose
            distance = math.hypot(x - px, y - py)
            if distance > 0.5:
                self.stop(now, 'Odometry jumped; select again', clear=True, fault=True)
            self.path_length += distance
            self.pose = (x, y, unwrapped + wrap(yaw - wrap(unwrapped)))
        self.last_odom = now

    def on_scan(self, front, rear, nearest, now):
        if any(math.isnan(v) or v < 0 for v in (front, rear, nearest)):
            return
        self.scan = (front, rear, nearest)
        self.last_scan = now

    def ready(self, now):
        return (self.pose is not None and not self.docked and not self.hazard
                and all(now - t <= self.cfg.sensor_timeout for t in
                        (self.last_odom, self.last_scan, self.last_dock, self.last_hazard)))

    def on_gesture(self, gesture, now):
        if gesture not in self.PRESENT:
            self.candidate = None
            self.candidate_samples = 0
            return
        self.last_signal = now
        if gesture == 'Open_Palm':
            # After waving, a stationary open hand is the natural handover to
            # Thumb_Up. The robot is already stopped in SELECTED; do not erase
            # that greeting choice. A palm during ANY running motion stops it.
            if self.mode == 'RUNNING' or (self.armed and self.selected != 'greet'):
                self.stop(now, 'Open_Palm: stopped; select and confirm again')
            self.candidate = self.last_emitted = gesture
            self.candidate_samples = 0
            return
        if gesture == 'Closed_Fist':
            # A natural Pointing_Up -> Thumb_Up transition can briefly resemble
            # a fist. While stationary, require a short deliberate hold to
            # clear selection. While moving, cancel immediately as before.
            if self.mode == 'SELECTED':
                if self.candidate != gesture:
                    self.candidate, self.candidate_since, self.candidate_samples = gesture, now, 1
                    return
                self.candidate_samples += 1
                if (now - self.candidate_since < self.cfg.cancel_selection_seconds
                        or self.candidate_samples < self.cfg.stable_samples):
                    return
            if self.mode != 'IDLE' or self.selected is not None:
                self.stop(now, 'Closed_Fist: canceled and cleared selection', clear=True)
            self.candidate = self.last_emitted = gesture
            self.candidate_samples = 0
            return
        if self.candidate != gesture:
            self.candidate, self.candidate_since, self.candidate_samples = gesture, now, 1
            return
        self.candidate_samples += 1
        if (gesture == self.last_emitted or now - self.candidate_since < self.cfg.stable_seconds
                or self.candidate_samples < self.cfg.stable_samples):
            return
        self.last_emitted = gesture
        if gesture in self.CHOICES:
            if self.mode == 'RUNNING':
                if self.CHOICES[gesture] == self.selected:
                    return  # Holding the current choice is just a heartbeat.
                self.stop(now, 'Changed behavior: stop and require a fresh confirmation')
                self.segment = 0
                self.last_emitted = gesture
            self.selected = self.CHOICES[gesture]
            self.segment = 0
            self.selected_at = now
            self.armed = True
            self.mode = 'SELECTED'
            self.event(now, f'Selected {self.selected}; show Thumb_Up to start')
        elif gesture == 'Thumb_Up' and self.mode == 'SELECTED' and self.armed:
            if now - self.selected_at > self.cfg.selection_timeout:
                self.stop(now, 'Selection expired', clear=True)
            elif not self.ready(now):
                self.stop(now, 'Not ready: require fresh odom/scan/dock/hazard and undocked robot',
                          clear=True, fault=True)
            else:
                self.start(now)

    def start(self, now):
        self.program = {'greet': [('turn', math.pi / 4), ('turn', -math.pi / 2),
                                  ('turn', math.pi / 4)],
                        'shuttle': [('shuttle', self.cfg.shuttle_distance),
                                    ('shuttle', 0.0)] * 4,
                        'circle': [('circle', 2 * math.pi)]}[self.selected]
        self.shuttle_origin = self.pose if self.selected == 'shuttle' else None
        self.mode = 'RUNNING'
        self.armed = False
        self.started = now
        self.segment = 0
        self.anchor = None
        self.settle_until = now
        self.event(now, f'Confirmed: executing {self.selected}')

    def tick(self, now):
        # No motion is possible in SELECTED. Allow a brief handover gap here;
        # the RUNNING watchdog remains the strict 0.65 s motion stop rule.
        if self.mode == 'SELECTED' and now - self.last_signal > self.cfg.selection_signal_timeout:
            self.stop(now, 'Control signal lost before confirmation', clear=True, fault=True)
        if self.mode == 'SELECTED' and now - self.selected_at > self.cfg.selection_timeout:
            self.stop(now, 'Selection expired', clear=True)
        if self.mode != 'RUNNING':
            return 0.0, 0.0
        if now - self.last_signal > self.cfg.signal_timeout:
            self.stop(now, 'Control signal lost; select again', clear=True, fault=True)
        elif not self.ready(now):
            self.stop(now, 'Sensor/dock/hazard interlock; select again', clear=True, fault=True)
        elif now - self.started > self.cfg.behavior_timeout:
            self.stop(now, 'Behavior wall timeout', clear=True, fault=True)
        if self.mode != 'RUNNING' or now < self.settle_until:
            return 0.0, 0.0
        if self.segment == len(self.program):
            self.mode = 'COMPLETED'
            self.program = []
            self.event(now, 'Completed; choose a gesture again for another run')
            return 0.0, 0.0
        x, y, yaw = self.pose
        if self.anchor is None:
            self.anchor = (x, y, yaw, self.path_length)
            self.progress_mark = (x, y, yaw)
            self.progress_at = now
        ax, ay, heading, length = self.anchor
        px, py, pa = self.progress_mark
        if math.hypot(x - px, y - py) > 0.02 or abs(yaw - pa) > 0.08:
            self.progress_mark, self.progress_at = (x, y, yaw), now
        if now - self.progress_at > self.cfg.progress_timeout:
            self.stop(now, 'No odometry progress', clear=True, fault=True)
            return 0.0, 0.0
        kind, target = self.program[self.segment]
        if kind == 'shuttle':
            ax, ay, heading = self.shuttle_origin
            traveled = (x - ax) * math.cos(heading) + (y - ay) * math.sin(heading)
            error = target - traveled
            done = abs(error) < 0.015
            v, w = clip(3.5 * error, self.cfg.linear_speed), clip(1.8 * wrap(heading - yaw), 0.4)
        elif kind == 'turn':
            error = target - (yaw - heading)
            done = abs(error) < 0.035
            v, w = 0.0, clip(3.2 * error, self.cfg.angular_speed)
        else:
            remaining = target - (yaw - heading)
            done = remaining < 0.045
            w = min(self.cfg.circle_speed / self.cfg.circle_radius, max(0.04, remaining))
            v = self.cfg.circle_radius * w
            if done and self.path_length - length < 0.7 * 2 * math.pi * self.cfg.circle_radius:
                self.stop(now, 'Circle rotation without enough translation', clear=True, fault=True)
                return 0.0, 0.0
        if done:
            self.segment += 1
            self.anchor = None
            self.settle_until = now + 0.2
            self.event(now, f'Finished motion segment {self.segment}')
            return 0.0, 0.0
        front, rear, nearest = self.scan
        blocked = ((v > 0 and front < self.cfg.obstacle_distance)
                   or (v < 0 and rear < self.cfg.obstacle_distance)
                   or (abs(w) > 0.02 and nearest < self.cfg.rotation_clearance))
        if blocked:
            self.stop(now, 'LiDAR obstacle interlock; select again', clear=True, fault=True)
            return 0.0, 0.0
        return v, w

    def status(self, now):
        return {'mode': self.mode, 'selected': self.selected, 'reason': self.reason,
                'segment': self.segment, 'segments_total': len(self.program),
                'shuttle_cycle': min(4, self.segment // 2 + 1) if self.selected == 'shuttle' else 0,
                'shuttle_cycles_completed': min(4, self.segment // 2) if self.selected == 'shuttle' else 0,
                'shuttle_phase': ('forward' if self.segment % 2 == 0 else 'return') if self.selected == 'shuttle' else '',
                'ready': self.ready(now), 'signal_age': min(999.0, now - self.last_signal)}
