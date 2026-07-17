import cv2
import time
import queue
import numpy as np
from typing import Any


def camera_reader(src: Any, out_q: "queue.Queue", stop_flag: dict, read_fps: int = 30,
                  use_dummy_on_fail: bool = True, dummy_resolution: tuple = (1920, 1080),
                  with_timestamp: bool = False):
    """
    Read frames from `src` and push the latest frame into out_q.
    If camera fails and use_dummy_on_fail=True, generates blank frames instead.

    Args:
        src: Camera source (int for device ID, str for video file/stream)
        out_q: Queue to put frames into
        stop_flag: Dict with stop_flag["stop"] = True to terminate
        read_fps: Target FPS for reading
        use_dummy_on_fail: If True, generate blank frames when camera unavailable
        dummy_resolution: (width, height) for dummy frames
        with_timestamp: If True, push (capture_time, frame) tuples so the consumer
            can time-match each frame to a LiDAR sweep. If False, push bare frames.
    """
    cap = cv2.VideoCapture(src)
    camera_available = cap.isOpened()

    if not camera_available:
        print(f"[camera_stream] Cannot open source: {src}")
        if not use_dummy_on_fail:
            print(f"[camera_stream] Exiting (use_dummy_on_fail=False)")
            return
        print(f"[camera_stream] Using dummy blank frames at {dummy_resolution}")
    else:
        # Get actual camera resolution
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        dummy_resolution = (actual_width, actual_height)
        print(f"[camera_stream] Camera opened: {actual_width}x{actual_height}")

    frame_interval = 1.0 / max(1, read_fps)
    frame_count = 0

    def _push(frame):
        # Capture the time as close as possible to when the frame was read.
        item = (time.time(), frame) if with_timestamp else frame
        try:
            out_q.put_nowait(item)
        except queue.Full:
            # Drop the oldest frame and enqueue the newest one.
            try:
                _ = out_q.get_nowait()
                out_q.put_nowait(item)
            except (queue.Empty, queue.Full):
                pass

    try:
        while not stop_flag.get("stop", False):
            if camera_available:
                # Try to read from real camera
                ret, frame = cap.read()
                if not ret:
                    print(f"[camera_stream] Failed to read frame, switching to dummy mode")
                    camera_available = False
                    if not use_dummy_on_fail:
                        break
                    # Fall through to dummy frame generation
                else:
                    frame_count += 1
                    _push(frame)
                    time.sleep(frame_interval)
                    continue

            if not camera_available and use_dummy_on_fail:
                frame = create_blank_frame(dummy_resolution, frame_count)
                frame_count += 1
                _push(frame)

            time.sleep(frame_interval)

    finally:
        if cap.isOpened():
            cap.release()
        print(f"[camera_stream] Stopped (total frames: {frame_count})")


def create_blank_frame(resolution: tuple, frame_number: int = 0,
                       add_info: bool = True) -> np.ndarray:
    """
    Create a blank black frame with optional information overlay.

    Args:
        resolution: (width, height) tuple
        frame_number: Current frame number for display
        add_info: Whether to add text information

    Returns:
        Blank frame as numpy array (BGR)
    """
    width, height = resolution
    frame = np.zeros((height, width, 3), dtype=np.uint8)

    if add_info:
        info_lines = [
            "Camera: OFFLINE (Dummy Mode)",
            f"Resolution: {width}x{height}",
            f"Frame: {frame_number}",
        ]

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        font_thickness = 2
        text_color = (80, 80, 80)  # Dark gray

        y_offset = height // 2 - 50

        for text in info_lines:
            (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, font_thickness)
            x_pos = (width - text_width) // 2
            cv2.putText(frame, text, (x_pos, y_offset),
                        font, font_scale, text_color, font_thickness)
            y_offset += 40

        timestamp = time.strftime("%H:%M:%S")
        cv2.putText(frame, timestamp, (20, height - 20),
                    font, 0.5, (60, 60, 60), 1)

    return frame


def create_static_blank_frame(resolution: tuple) -> np.ndarray:
    """Create a completely blank black frame without any text."""
    width, height = resolution
    return np.zeros((height, width, 3), dtype=np.uint8)
