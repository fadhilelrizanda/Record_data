#!/usr/bin/env python3
"""
Live visualization of Velodyne LiDAR + camera in a SINGLE window.

- Receives Velodyne UDP packets, decodes complete scans, and renders the point
  cloud OFFSCREEN with Open3D (colored by intensity).
- Grabs the live camera feed and composites it next to the point cloud, so both
  sensors share one OpenCV window (no separate Open3D window).

Why offscreen rendering: on Wayland sessions the legacy Open3D `Visualizer`
fails to open a GL window (GLFW/GLEW init fails). `OffscreenRenderer` uses
Filament + headless EGL, which works, and we show the result via OpenCV.

Network setup (run once, see lidar.md — adapt the interface name to your machine):
    sudo ip addr add 192.168.0.2/24 dev <iface>
    sudo ip link set <iface> up
    sudo ip route replace 192.168.0.201/32 dev <iface> src 192.168.0.2 metric 1

Then:
    python3 live_view.py 0.0.0.0 2368 --camera 0

No hardware handy? Exercise the whole pipeline with synthetic data:
    python3 live_view.py --simulate --record --duration 60

Press 'q' or ESC in the window (or close it) to quit.
"""
import os
import sys

# --- Make system Mesa/EGL loadable, then re-exec once. ----------------------
# Two environment problems block Open3D's offscreen renderer under conda:
#   1. The DRI drivers live in /usr/lib/x86_64-linux-gnu/dri, not the default
#      /usr/lib/dri the loader searches.
#   2. conda ships an older libstdc++ than system Mesa/LLVM needs (missing
#      GLIBCXX_3.4.30), so the driver fails to load.
# LD_PRELOAD must be set before the process starts, so we set the env and
# re-exec ourselves exactly once (guarded by _LIVE_VIEW_REEXEC).
if os.environ.get("_LIVE_VIEW_REEXEC") != "1":
    _fixes = {"_LIVE_VIEW_REEXEC": "1"}
    _stdcxx = "/usr/lib/x86_64-linux-gnu/libstdc++.so.6"
    if os.path.exists(_stdcxx):
        _pre = os.environ.get("LD_PRELOAD", "")
        _fixes["LD_PRELOAD"] = _stdcxx + ((":" + _pre) if _pre else "")
    _dri = "/usr/lib/x86_64-linux-gnu/dri"
    if os.path.isdir(_dri):
        _fixes["LIBGL_DRIVERS_PATH"] = _dri
    os.environ.setdefault("EGL_PLATFORM", "surfaceless")
    if any(k not in os.environ or os.environ[k] != v for k, v in _fixes.items()):
        os.environ.update(_fixes)
        os.execv(sys.executable, [sys.executable] + sys.argv)

import csv
import time
import socket
import threading
import argparse
import logging
import subprocess
from collections import deque

import numpy as np
import cv2
import open3d as o3d

import velodyne_decoder as vd

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

# Column layout of the decoded (N, 8) array (from vd.PointField):
#   0:x 1:y 2:z 3:intensity 4:time 5:column 6:ring 7:return_type
COL_X, COL_Y, COL_Z, COL_INTENSITY = 0, 1, 2, 3


def colorize(points: np.ndarray) -> np.ndarray:
    """Map intensity (fallback: height) to an RGB color array in [0, 1]."""
    if points.shape[1] > COL_INTENSITY:
        v = points[:, COL_INTENSITY].astype(np.float32)
    else:
        v = points[:, COL_Z].astype(np.float32)
    # Robust normalization so a few hot returns don't wash everything out.
    lo, hi = np.percentile(v, 2), np.percentile(v, 98)
    if hi <= lo:
        hi = lo + 1.0
    norm = np.clip((v - lo) / (hi - lo), 0.0, 1.0)
    bgr = cv2.applyColorMap((norm * 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_JET)
    rgb = bgr.reshape(-1, 3)[:, ::-1].astype(np.float64) / 255.0
    return rgb


def nearest_frame(cam_items, t):
    """Pick the (ts, frame) pair closest in time to LiDAR stamp `t`.
    Returns (frame, dt) where dt = cam_ts - t (seconds), or (None, None)."""
    if not cam_items:
        return None, None
    cam_ts, frame = min(cam_items, key=lambda p: abs(p[0] - t))
    return frame, (cam_ts - t)


def detect_screen_size(default=(1280, 720)):
    """Best-effort current screen resolution as (width, height).

    We run under XWayland/X11 (DISPLAY set by run_live_view.sh), so query the
    server with xrandr (current mode is the line containing '*'), falling back to
    xdpyinfo, then a sane default. Used to size the window to the actual screen."""
    try:
        out = subprocess.run(["xrandr"], capture_output=True, text=True, timeout=2).stdout
        for line in out.splitlines():
            if "*" in line:  # e.g. "   1360x768     59.80*+"
                w, h = line.split()[0].split("x")
                return int(w), int(h)
    except Exception:
        pass
    try:
        out = subprocess.run(["xdpyinfo"], capture_output=True, text=True, timeout=2).stdout
        for line in out.splitlines():
            if "dimensions:" in line:  # e.g. "  dimensions:    1366x768 pixels"
                w, h = line.split()[1].split("x")
                return int(w), int(h)
    except Exception:
        pass
    return default


def placeholder_panel(w: int, h: int, text: str) -> np.ndarray:
    """A dark panel with a centered message, used before a sensor has data."""
    panel = np.full((h, w, 3), 30, dtype=np.uint8)
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
    cv2.putText(panel, text, ((w - tw) // 2, (h + th) // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (180, 180, 180), 2, cv2.LINE_AA)
    return panel


def label(img: np.ndarray, text: str) -> np.ndarray:
    """Draw a small caption in the top-left corner (in place)."""
    cv2.putText(img, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (255, 255, 255), 2, cv2.LINE_AA)
    return img


def compose(lidar_bgr, cam_bgr, height: int) -> np.ndarray:
    """Stack the LiDAR render and camera frame side-by-side at a common height."""
    def fit(img):
        h, w = img.shape[:2]
        return cv2.resize(img, (int(w * height / h), height), interpolation=cv2.INTER_AREA)
    return cv2.hconcat([fit(lidar_bgr), fit(cam_bgr)])


# --- Open3D offscreen rendering --------------------------------------------
def make_renderer(w, h, point_size):
    """Create an OffscreenRenderer with a dark background."""
    rnd = o3d.visualization.rendering.OffscreenRenderer(w, h)
    rnd.scene.set_background([0.05, 0.05, 0.05, 1.0])
    mat = o3d.visualization.rendering.MaterialRecord()
    mat.shader = "defaultUnlit"
    mat.point_size = float(point_size)
    return rnd, mat


def setup_view(rnd, xyz, zoom=1.0):
    """Aim the camera at the cloud from an elevated, slightly-behind angle.

    `zoom` magnifies the view (2-3 = closer/bigger) by pulling the eye in toward
    the look-at center along the same direction, so the framing angle is kept."""
    center = xyz.mean(axis=0)
    extent = float(np.linalg.norm(xyz.max(axis=0) - xyz.min(axis=0)))
    if extent < 1.0:
        extent = 20.0
    offset = np.array([0.0, -0.6 * extent, 0.4 * extent]) / max(zoom, 0.1)
    eye = center + offset
    rnd.setup_camera(60.0, center.tolist(), eye.tolist(), [0.0, 0.0, 1.0])


def render_lidar(rnd, mat, xyz, colors, init_view=False, zoom=1.0) -> np.ndarray:
    """Replace the point geometry and render the scene to a BGR uint8 image.

    The camera (init_view) MUST be set after the cloud is added: setup_camera
    derives the near/far clip planes from the current scene bounds, so aiming it
    while only the tiny axes exist would clip away the real points."""
    if rnd.scene.has_geometry("pcd"):
        rnd.scene.remove_geometry("pcd")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    rnd.scene.add_geometry("pcd", pcd, mat)
    if init_view:
        setup_view(rnd, xyz, zoom)
    img = np.asarray(rnd.render_to_image())  # RGB uint8
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


# --- Sensor threads ---------------------------------------------------------
def lidar_receiver(ip, port, cfg, holder, stop_flag):
    """Background thread: receive UDP packets, decode full scans into holder."""
    try:
        decoder = vd.StreamDecoder(cfg)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((ip, port))
        s.settimeout(1.0)
        logging.info("Listening for LiDAR on %s:%d", ip, port)
    except Exception:
        logging.exception("Failed to bind LiDAR socket")
        stop_flag["stop"] = True
        return

    packets = 0
    while not stop_flag["stop"]:
        try:
            data, _ = s.recvfrom(vd.PACKET_SIZE * 2)
        except socket.timeout:
            continue
        except OSError:
            break
        packets += 1
        try:
            result = decoder.decode(time.time(), data)
        except Exception:
            logging.exception("decode failed, skipping packet")
            continue
        if result is None:
            continue  # scan not complete yet
        _stamp, pts = result
        arr = np.asarray(pts)
        if arr.ndim == 2 and arr.shape[0] > 0 and arr.shape[1] >= 3:
            with holder["lock"]:
                holder["points"] = arr
                holder["seq"] += 1
                holder["stamp"] = time.time()
    s.close()
    logging.info("LiDAR receiver stopped (%d packets seen)", packets)


def camera_thread(src, holder, stop_flag, read_fps=30):
    """Background thread: push timestamped frames into holder["buffer"] so each
    LiDAR sweep can be paired with the camera frame CLOSEST in time."""
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        logging.warning("Cannot open camera source %s — camera panel disabled", src)
        return
    # Keep only the freshest frame in the driver queue so cap.read() never returns
    # a stale, buffered frame — this is the main source of hidden camera latency on
    # USB webcams. Combined with no sleep (read() blocks ~1/fps on its own), every
    # frame is stamped as close as possible to its true capture time.
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if read_fps:
        cap.set(cv2.CAP_PROP_FPS, read_fps)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    logging.info("Camera opened (source=%s, %dx%d, %.0f fps, buffersize=1)", src, w, h, fps)
    try:
        while not stop_flag["stop"]:
            ok, frame = cap.read()  # blocks until the next frame; returns the newest
            if ok and frame is not None:
                # Stamp right after read(); same clock as lidar_holder["stamp"].
                stamp = time.time()
                with holder["lock"]:
                    holder["buffer"].append((stamp, frame))
            else:
                time.sleep(0.005)  # transient read failure — back off briefly
    finally:
        cap.release()
        logging.info("Camera stopped")


# --- Simulation (no hardware) ----------------------------------------------
def sim_lidar(holder, stop_flag):
    """Synthetic 10 Hz scans: an animated ground ripple over a 30 m disk."""
    t0 = time.time()
    logging.info("SIMULATED LiDAR running (10 Hz synthetic scans)")
    while not stop_flag["stop"]:
        n = 25000
        ang = np.random.uniform(0, 2 * np.pi, n)
        rad = np.random.uniform(1.0, 30.0, n)
        x = rad * np.cos(ang)
        y = rad * np.sin(ang)
        z = 2.0 * np.sin(rad * 0.3 + (time.time() - t0) * 1.5) * np.exp(-rad * 0.04)
        inten = (rad / 30.0 * 255.0).astype(np.float32)
        arr = np.stack([x, y, z, inten], axis=1).astype(np.float32)
        with holder["lock"]:
            holder["points"] = arr
            holder["seq"] += 1
            holder["stamp"] = time.time()
        time.sleep(0.1)


def sim_camera(holder, stop_flag):
    """Synthetic 30 fps frames with a moving marker and frame counter."""
    i = 0
    logging.info("SIMULATED camera running (30 fps synthetic frames)")
    while not stop_flag["stop"]:
        i += 1
        f = np.full((480, 640, 3), 40, dtype=np.uint8)
        cv2.putText(f, f"SIM CAM  frame {i}", (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)
        cx = 40 + (i * 6) % 560
        cv2.circle(f, (cx, 280), 30, (0, 128, 255), -1)
        with holder["lock"]:
            holder["buffer"].append((time.time(), f))
        time.sleep(1 / 30)


def main():
    parser = argparse.ArgumentParser(description="Live LiDAR + camera in one window")
    parser.add_argument("host_ip", nargs="?", default="0.0.0.0", help="IP to bind for LiDAR UDP (default 0.0.0.0)")
    parser.add_argument("port", nargs="?", type=int, default=2368, help="UDP port (default 2368)")
    parser.add_argument("--camera", "-c", default="4", help="camera source (int index or path; default 0)")
    parser.add_argument("--model", default="VLP32C", help="Velodyne model (default VLP32C)")
    parser.add_argument("--record", action="store_true", help="also save synchronized lidar+camera frames to disk")
    parser.add_argument("--out", "-o", default="./recordings", help="output directory for --record")
    parser.add_argument("--interval", type=float, default=0.0, help="min seconds between saved frames (0 = every scan)")
    parser.add_argument("--max-sync-dt", type=float, default=0.05,
                        help="warn when the matched camera frame is staler than this many seconds (default 0.05)")
    parser.add_argument("--duration", type=float, default=0.0, help="auto-stop after N seconds (0 = run until quit)")
    parser.add_argument("--point-size", type=float, default=2.0, help="rendered point size (default 2.0)")
    parser.add_argument("--zoom", type=float, default=2.5, help="LiDAR view zoom factor (1 = fit, 2-3 = closer; default 2.5)")
    parser.add_argument("--simulate", action="store_true", help="generate synthetic LiDAR + camera (no hardware needed)")
    parser.add_argument("--no-display", action="store_true", help="don't open a window (record/headless only)")
    parser.add_argument("--window-width", type=int, default=0, help="window width in px (0 = auto-fit current screen)")
    parser.add_argument("--window-height", type=int, default=0, help="window height in px (0 = auto-fit current screen)")
    args = parser.parse_args()

    if args.record:
        os.makedirs(args.out, exist_ok=True)
        logging.info("Recording enabled -> %s (interval=%.3fs, max_sync_dt=%.3fs)",
                     args.out, args.interval, args.max_sync_dt)

    cam_src = int(args.camera) if str(args.camera).isdigit() else args.camera
    cfg = vd.Config(model=getattr(vd.Model, args.model))

    stop_flag = {"stop": False}
    lidar_holder = {"points": None, "seq": 0, "stamp": 0.0, "lock": threading.Lock()}
    # Buffer of recent (capture_time, frame) so each LiDAR sweep is paired with the
    # camera frame CLOSEST in time, not just "the latest" (~3 s history at 30 fps).
    cam_holder = {"buffer": deque(maxlen=90), "lock": threading.Lock()}

    if args.simulate:
        threading.Thread(target=sim_lidar, args=(lidar_holder, stop_flag), daemon=True).start()
        threading.Thread(target=sim_camera, args=(cam_holder, stop_flag), daemon=True).start()
    else:
        threading.Thread(target=lidar_receiver, args=(args.host_ip, args.port, cfg, lidar_holder, stop_flag), daemon=True).start()
        threading.Thread(target=camera_thread, args=(cam_src, cam_holder, stop_flag), daemon=True).start()

    # Offscreen LiDAR renderer; we composite its frames with the camera into one
    # OpenCV window instead of opening a separate Open3D window.
    LIDAR_W, LIDAR_H = 960, 720
    rnd, mat = make_renderer(LIDAR_W, LIDAR_H, args.point_size)
    view_window = "LiDAR + camera (live)"

    # Size the window to the CURRENT screen so the side-by-side composite (which
    # is ~2x as wide as it is tall) doesn't overflow the display. We render each
    # panel at panel_h, then let a resizable, aspect-preserving window scale the
    # whole composite down to fit the available screen area.
    display_ok = not args.no_display
    win_w, win_h = args.window_width, args.window_height
    if win_w <= 0 or win_h <= 0:
        scr_w, scr_h = detect_screen_size()
        win_w = max(640, int(scr_w * 0.95))   # leave a margin for the WM decorations
        win_h = max(360, int(scr_h * 0.90))
    panel_h = min(LIDAR_H, win_h)  # render height of each panel before scaling
    if display_ok:
        cv2.namedWindow(view_window, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(view_window, win_w, win_h)
        logging.info("Window sized to %dx%d (panels rendered at h=%d)", win_w, win_h, panel_h)

    lidar_bgr = placeholder_panel(LIDAR_W, LIDAR_H, "waiting for LiDAR...")
    last_seq = -1
    view_initialized = False

    saved_count = 0
    last_saved_ts = None
    manifest_file = None
    manifest_writer = None

    start_t = time.time()
    logging.info("Visualizing. Press 'q'/ESC in the window to quit (or Ctrl-C).")
    try:
        while not stop_flag["stop"]:
            # --- LiDAR: re-render only when a new scan arrived ---
            with lidar_holder["lock"]:
                arr = lidar_holder["points"]
                seq = lidar_holder["seq"]
                stamp = lidar_holder["stamp"]
            if arr is not None and seq != last_seq:
                xyz = arr[:, :3].astype(np.float64)
                do_init = (not view_initialized) and len(xyz) > 0
                lidar_bgr = render_lidar(rnd, mat, xyz, colorize(arr), init_view=do_init, zoom=args.zoom)
                if do_init:
                    view_initialized = True
                last_seq = seq

                # --- optional recording (one save per new scan) ---
                if args.record and (args.interval <= 0 or last_saved_ts is None
                                    or stamp >= last_saved_ts + args.interval):
                    # Pair this sweep with the camera frame CLOSEST in time.
                    with cam_holder["lock"]:
                        cam_items = list(cam_holder["buffer"])
                    rec_frame, sync_dt = nearest_frame(cam_items, stamp)

                    # Every dataset sample needs BOTH sensors. If the camera has no
                    # frame yet (e.g. still warming up), skip this sweep rather than
                    # write a LiDAR-only orphan the fusion model can't use.
                    if rec_frame is not None:
                        # Lazily open the manifest once (documents the dataset).
                        if manifest_writer is None:
                            manifest_file = open(os.path.join(args.out, "manifest.csv"), "w", newline="")
                            manifest_writer = csv.writer(manifest_file)
                            manifest_writer.writerow(["frame_idx", "lidar_stamp", "cam_lidar_dt_ms",
                                                      "num_points", "lidar_npy", "lidar_ply", "camera_jpg"])

                        ts_str = f"{int(stamp * 1e6):020d}"
                        dt_ms = sync_dt * 1e3
                        if abs(sync_dt) > args.max_sync_dt:
                            logging.warning("Camera-LiDAR sync %.1fms exceeds %.0fms (overlay will shift "
                                            "if the scene is moving)", dt_ms, args.max_sync_dt * 1e3)

                        cam_name = f"camera_{ts_str}.jpg"
                        cv2.imwrite(os.path.join(args.out, cam_name), rec_frame)
                        np.save(os.path.join(args.out, f"lidar_{ts_str}.npy"), arr)
                        try:
                            pcd_save = o3d.geometry.PointCloud()
                            pcd_save.points = o3d.utility.Vector3dVector(xyz)
                            o3d.io.write_point_cloud(os.path.join(args.out, f"lidar_{ts_str}.ply"),
                                                     pcd_save, write_ascii=False)
                        except Exception:
                            logging.debug("PLY write failed (non-fatal)", exc_info=True)

                        manifest_writer.writerow([saved_count, f"{stamp:.6f}", f"{dt_ms:.2f}",
                                                  len(xyz), f"lidar_{ts_str}.npy", f"lidar_{ts_str}.ply",
                                                  cam_name])
                        saved_count += 1
                        last_saved_ts = stamp
                        if saved_count % 10 == 1:
                            logging.info("Saved #%d (ts=%s, %d points, cam-lidar dt=%.1fms)",
                                         saved_count, ts_str, len(xyz), dt_ms)
                    else:
                        logging.debug("No camera frame yet — skipping LiDAR-only sweep at ts=%.6f", stamp)

            # --- Camera: always show the freshest frame ---
            with cam_holder["lock"]:
                frame = cam_holder["buffer"][-1][1] if cam_holder["buffer"] else None
            cam_bgr = frame.copy() if frame is not None else \
                placeholder_panel(LIDAR_W, LIDAR_H, "waiting for camera...")

            # --- Display the combined window ---
            if display_ok:
                combined = compose(label(lidar_bgr.copy(), "LiDAR"),
                                   label(cam_bgr, "camera"), panel_h)
                try:
                    cv2.imshow(view_window, combined)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord('q')):
                        break
                    if cv2.getWindowProperty(view_window, cv2.WND_PROP_VISIBLE) < 1:
                        break  # window closed via title-bar [x]
                except cv2.error as e:
                    logging.warning("Display unavailable (%s) — continuing headless", e)
                    display_ok = False
            else:
                time.sleep(0.005)  # headless: avoid a busy spin

            if args.duration and (time.time() - start_t) >= args.duration:
                logging.info("Reached --duration %.1fs, stopping.", args.duration)
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop_flag["stop"] = True
        if manifest_file is not None:
            manifest_file.close()
        if display_ok:
            cv2.destroyAllWindows()
        time.sleep(0.3)
        if args.record:
            logging.info("Done. Saved %d frame(s) to %s (see manifest.csv)", saved_count, args.out)
        else:
            logging.info("Done.")


if __name__ == "__main__":
    main()
