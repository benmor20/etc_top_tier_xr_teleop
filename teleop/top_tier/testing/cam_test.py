#!/usr/bin/env python3
"""
Display two webcam feeds side by side with OpenCV (Ubuntu / V4L2),
with keyboard-controlled recording to a separate file per camera.

Usage:
    python3 dual_webcam.py              # auto-detect the first two working cameras
    python3 dual_webcam.py 0 2          # use specific device indices (/dev/video0, /dev/video2)
    python3 dual_webcam.py 0 2 --separate   # show each feed in its own window

Keys:  r = start recording,   x = stop recording,
       s = save a snapshot of both frames,   q / Esc = quit

Recordings are saved as cam<idx>_<timestamp>.mp4 (one file per camera) and
contain only the raw camera frames: no labels, FPS counter, or REC indicator.
"""
import sys
import time
import cv2
import numpy as np

WIDTH, HEIGHT, FPS = 640, 480, 30


def open_cam(index):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    # MJPG greatly reduces USB bandwidth; without it two cams on the same
    # USB controller often fail with "No space left on device" / timeouts.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # keep latency low
    ok, _ = cap.read()
    if not ok:
        cap.release()
        return None
    return cap


def autodetect(n=2, max_index=10):
    """Each UVC webcam usually creates two /dev/video nodes (video + metadata),
    so the real cameras are often 0 and 2. Probe until we find n that work."""
    found = []
    for i in range(max_index):
        cap = open_cam(i)
        if cap is not None:
            print(f"Found camera at /dev/video{i}")
            found.append((i, cap))
            if len(found) == n:
                break
    return found


def label(frame, text):
    cv2.putText(frame, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(frame, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (255, 255, 255), 1, cv2.LINE_AA)


def start_recording(cams, frames, fps):
    """Open one VideoWriter per camera. Returns {idx: writer}."""
    ts = time.strftime("%Y%m%d_%H%M%S")
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    # Use the measured loop rate so playback speed matches real time.
    rec_fps = float(min(max(fps, 5.0), FPS)) if fps > 0 else float(FPS)
    writers = {}
    for (idx, _), f in zip(cams, frames):
        if f is None:
            continue
        h, w = f.shape[:2]
        path = f"cam{idx}_{ts}.mp4"
        writer = cv2.VideoWriter(path, fourcc, rec_fps, (w, h))
        if writer.isOpened():
            writers[idx] = writer
            print(f"Recording /dev/video{idx} -> {path} ({rec_fps:.0f} fps)")
        else:
            print(f"Could not open writer for {path}")
    return writers


def stop_recording(writers):
    for w in writers.values():
        w.release()
    print("Recording stopped")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    separate = "--separate" in sys.argv

    if len(args) >= 2:
        cams = []
        for a in args[:2]:
            idx = int(a)
            cap = open_cam(idx)
            if cap is None:
                sys.exit(f"Could not open /dev/video{idx}")
            cams.append((idx, cap))
    else:
        cams = autodetect()
        if len(cams) < 2:
            for _, c in cams:
                c.release()
            sys.exit(f"Only found {len(cams)} working camera(s). "
                     "Check `v4l2-ctl --list-devices` and pass indices explicitly.")

    blank = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    prev = time.time()
    fps = 0.0
    writers = {}  # empty = not recording

    try:
        while True:
            raw = []      # untouched camera frames (None if read failed)
            for idx, cap in cams:
                ok, f = cap.read()
                raw.append(f if ok else None)

            now = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(now - prev, 1e-6))
            prev = now

            # Write the raw frames BEFORE any text is drawn.
            if writers:
                for (idx, _), f in zip(cams, raw):
                    if f is not None and idx in writers:
                        writers[idx].write(f)

            # Build display copies so the overlays never touch the recording.
            frames = []
            for (idx, _), f in zip(cams, raw):
                if f is None:
                    d = blank.copy()
                    label(d, f"/dev/video{idx}: no frame")
                else:
                    d = f.copy()
                label(d, f"/dev/video{idx}  {fps:.0f} fps")
                if writers:
                    cv2.circle(d, (d.shape[1] - 25, 22), 8, (0, 0, 255), -1)
                    cv2.putText(d, "REC", (d.shape[1] - 80, 28),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 0, 255), 2, cv2.LINE_AA)
                frames.append(d)

            if separate:
                for (idx, _), f in zip(cams, frames):
                    cv2.imshow(f"Camera {idx}", f)
            else:
                # Match heights so hconcat works even if cameras differ in resolution
                h = min(f.shape[0] for f in frames)
                resized = [cv2.resize(f, (int(f.shape[1] * h / f.shape[0]), h))
                           for f in frames]
                cv2.imshow("Dual Webcam", cv2.hconcat(resized))

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord("r") and not writers:
                writers = start_recording(cams, raw, fps)
            elif key == ord("x") and writers:
                stop_recording(writers)
                writers = {}
            elif key == ord("s"):
                ts = time.strftime("%Y%m%d_%H%M%S")
                for (idx, _), f in zip(cams, raw):
                    if f is not None:
                        cv2.imwrite(f"cam{idx}_{ts}.png", f)
                print(f"Saved snapshots ({ts})")
    finally:
        if writers:
            stop_recording(writers)
        for _, cap in cams:
            cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()