"""Temporal Wave detector built on a confident MediaPipe Open_Palm and wrist x.

Wave is NOT a built-in canned MediaPipe category. Coordinates are normalized
image coordinates, and times are monotonic capture timestamps.
"""
from collections import deque


class WaveDetector:
    def __init__(self, window=2.4, amplitude=0.12, excursion=0.025, reversals=2):
        self.window, self.amplitude, self.excursion = window, amplitude, excursion
        self.required = reversals
        self.samples = deque()
        self.latched_until = 0.0
        self.last_confident = -float('inf')

    def reset(self):
        self.samples.clear()
        self.latched_until = 0.0
        self.last_confident = -float('inf')

    def missing(self, now):
        """Retain only a short occlusion history; never emit a control label."""
        if now - self.last_confident > .25:
            self.reset()

    def update(self, now, x=None, y=None, label='', confident=False, enabled=True):
        if not enabled or x is None or y is None:
            self.reset()
            return False
        if label != 'Open_Palm' or not confident:
            # A single uncertain AI frame should not erase the wrist trajectory.
            # Other control poses and a missing hand still reset immediately.
            if label in ('Open_Palm', 'None', 'Unknown') and self.samples and now-self.last_confident <= .25:
                return now < self.latched_until
            self.reset()
            return False
        if self.samples and (now - self.samples[-1][0] > 0.50
                             or abs(x - self.samples[-1][1]) > 0.35):
            self.reset()
        self.last_confident = now
        self.samples.append((now, x, y))
        while self.samples and now - self.samples[0][0] > self.window:
            self.samples.popleft()
        if now < self.latched_until:
            return True
        if len(self.samples) < 6:
            return False
        xs = [s[1] for s in self.samples]
        ys = [s[2] for s in self.samples]
        if max(xs) - min(xs) < self.amplitude or max(ys) - min(ys) > 0.30:
            return False
        pivot, direction, reversals = xs[0], 0, 0
        for value in xs[1:]:
            delta = value - pivot
            if abs(delta) >= self.excursion:
                sign = 1 if delta > 0 else -1
                if direction and sign != direction:
                    reversals += 1
                direction, pivot = sign, value
        if reversals >= self.required:
            # Hold the selection while the same open palm remains visible.
            # A different pose/no hand resets it. RUNNING always disables Wave,
            # so a palm shown during robot motion is still an immediate STOP.
            self.latched_until = float('inf')
            self.samples.clear()
            self.samples.append((now, x, y))
            return True
        return False
