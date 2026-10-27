#!/usr/bin/env python3
"""Download the official, versioned MediaPipe Gesture Recognizer task bundle."""
import argparse
import hashlib
from pathlib import Path
from urllib.request import urlretrieve

URL = 'https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
                        default=Path.home()/'.cache/turtlebot4_gesture_hri/gesture_recognizer.task')
    args = parser.parse_args();args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix('.download')
    urlretrieve(URL, temporary)
    if temporary.stat().st_size < 100000:
        raise RuntimeError('Model download unexpectedly small')
    temporary.replace(args.output)
    print(args.output)
    print('SHA256:', hashlib.sha256(args.output.read_bytes()).hexdigest())


if __name__ == '__main__':main()
