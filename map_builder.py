#!/usr/bin/env python3
"""
map_builder.py
- Connects to the ESP32-CAM MJPEG stream and captures keyframes to build a simple visual map.
- Saves images and ORB descriptors to disk for later localization.

Usage:
  python map_builder.py --stream http://<ESP_IP>:81/stream --out mapdir --interval 2.5

Press 'c' to capture a keyframe manually, or the script will capture at regular intervals.
"""
import cv2
import numpy as np
import os
import argparse
import pickle
import time

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--stream", required=True, help="MJPEG stream URL (e.g. http://192.168.1.10:81/stream)")
    p.add_argument("--out", default="map", help="output directory for map keyframes")
    p.add_argument("--interval", type=float, default=2.5, help="automatic capture interval (s)")
    return p.parse_args()

def ensure_dir(d):
    if not os.path.exists(d):
        os.makedirs(d)

def main():
    args = parse_args()
    ensure_dir(args.out)
    cap = cv2.VideoCapture(args.stream)
    if not cap.isOpened():
        print("Failed to open stream:", args.stream)
        return

    orb = cv2.ORB_create(nfeatures=2000)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    frame_count = 0
    last_capture = time.time()
    keyframes = []  # list of (img_filename, kp, des)

    print("Press 'c' to capture keyframe, 'q' to quit")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame read failed. Retrying")
            time.sleep(0.5)
            continue
        display = frame.copy()
        cv2.putText(display, f"Frames: {frame_count}", (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        cv2.imshow("stream", display)
        key = cv2.waitKey(1) & 0xFF
        now = time.time()
        capture_now = False
        if key == ord('q'):
            break
        if key == ord('c'):
            capture_now = True
        elif now - last_capture >= args.interval:
            capture_now = True
            last_capture = now

        if capture_now:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            kp, des = orb.detectAndCompute(gray, None)
            fname = os.path.join(args.out, f"kf_{frame_count:04d}.jpg")
            cv2.imwrite(fname, frame)
            # Save keypoint locations and descriptors
            kps = [(int(k.pt[0]), int(k.pt[1])) for k in kp]
            meta = {"kps": kps, "des": des}
            with open(os.path.join(args.out, f"kf_{frame_count:04d}.pkl"), "wb") as f:
                pickle.dump(meta, f)
            print(f"Saved keyframe {fname} kp:{len(kp)}")
            keyframes.append(fname)
            frame_count += 1

    cap.release()
    cv2.destroyAllWindows()
    print("Map building complete. Keyframes saved to:", args.out)

if __name__ == "__main__":
    main()
