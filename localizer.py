#!/usr/bin/env python3
"""
localizer.py
- Loads a map directory (images + descriptor pickle files) created by map_builder.py
- Reads frames from the ESP32-CAM stream and matches ORB features against map descriptors
- Displays best-matching keyframe and the matched keypoint visualization
- Prints the best-match keyframe id and match count

Usage:
  python localizer.py --stream http://<ESP_IP>:81/stream --map map
"""
import cv2
import numpy as np
import argparse
import os
import pickle
from collections import namedtuple

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--stream", required=True)
    p.add_argument("--map", required=True)
    p.add_argument("--min_matches", type=int, default=12)
    return p.parse_args()

Keyframe = namedtuple("Keyframe", ["img", "kps", "des", "name"])

def load_map(mapdir):
    kfs = []
    files = sorted([f for f in os.listdir(mapdir) if f.endswith(".pkl")])
    for pkl in files:
        base = pkl[:-4]
        imgpath = os.path.join(mapdir, base + ".jpg")
        if not os.path.exists(imgpath):
            continue
        with open(os.path.join(mapdir, pkl), "rb") as f:
            meta = pickle.load(f)
        img = cv2.imread(imgpath)
        kps = [cv2.KeyPoint(x=float(pt[0]), y=float(pt[1]), _size=1) for pt in meta["kps"]]
        des = meta["des"]
        kfs.append(Keyframe(img=img, kps=kps, des=des, name=base))
    print(f"Loaded {len(kfs)} keyframes")
    return kfs

def main():
    args = parse_args()
    kfs = load_map(args.map)
    if not kfs:
        print("Map is empty. Run map_builder.py first.")
        return

    cap = cv2.VideoCapture(args.stream)
    orb = cv2.ORB_create(nfeatures=2000)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Stream read failed. retrying...")
            import time; time.sleep(0.5)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kp, des = orb.detectAndCompute(gray, None)
        if des is None:
            cv2.imshow("localizer", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
            continue

        best_count = 0
        best_match_img = None
        best_matches = None
        best_kp = None
        best_kf = None

        for kf in kfs:
            if kf.des is None:
                continue
            matches = bf.match(des, kf.des)
            matches = sorted(matches, key=lambda x: x.distance)
            # take only good matches (distance threshold)
            good = [m for m in matches if m.distance < 60]
            if len(good) > best_count:
                best_count = len(good)
                best_matches = good
                best_kf = kf
                best_kp = kp

        if best_count >= args.min_matches and best_kf is not None:
            # Draw matches
            kf_img = best_kf.img
            # convert keypoints for drawing
            img_matches = cv2.drawMatches(frame, best_kp, kf_img, best_kf.kps, best_matches[:50], None, flags=2)
            cv2.putText(img_matches, f"Best: {best_kf.name} matches: {best_count}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
            cv2.imshow("localizer", img_matches)
            print(f"Match -> {best_kf.name} : {best_count}")
        else:
            cv2.putText(frame, "No good match", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
            cv2.imshow("localizer", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
