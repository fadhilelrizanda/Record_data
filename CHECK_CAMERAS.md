# Checking Available Cameras (Linux)

Quick reference for listing and inspecting cameras on this machine.

## Current devices on this machine

| Device | Name | Notes |
|--------|------|-------|
| `/dev/video0` | **DroidCam** (`v4l2loopback`) | *Virtual* camera — streams video from a phone over the network, not a physical webcam. Only appears while the DroidCam client/app is connected. |
| `/dev/v4l-touch0` | Synaptics RMI4 Touch Sensor | Not a camera (touchpad exposed via V4L). |

Usable camera index: **`0`**. No physical/built-in webcam is currently enumerated. A plugged-in USB webcam would show up as `/dev/video1` (or higher).

## Commands

### List devices (cleanest)
```bash
v4l2-ctl --list-devices
```

### Raw device nodes
```bash
ls -l /dev/video*
```

### Supported resolutions / formats for a specific camera
```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
```

### With ffmpeg
```bash
ffmpeg -f v4l2 -list_formats all -i /dev/video0
```

## From Python (OpenCV)
```python
import cv2

for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"Camera index {i} available")
        cap.release()
```

## Notes
- `v4l2-ctl` comes from the `v4l-utils` package.
- DroidCam is a network virtual camera; it only enumerates as `/dev/video0` while the phone is connected.
- Rerun `v4l2-ctl --list-devices` after plugging in new hardware to confirm the new index.
