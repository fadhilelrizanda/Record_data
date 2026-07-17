#!/usr/bin/env python3
import sys, os, time, socket, traceback, threading, queue
from collections import deque
import numpy as np
import cv2
import open3d as o3d
import logging

# ensure velodyne_decoder path like main.py
p = "/home/beliau/fe-dev/orin_inference_base/Lidar_AI_Solution/CUDA-BEVFusion/build"
if p not in sys.path:
    sys.path.insert(0, p)
os.environ.setdefault("PYTHONPATH", p + (":" + os.environ["PYTHONPATH"] if "PYTHONPATH" in os.environ else ""))

import velodyne_decoder as vd
from utils.camera_stream import camera_reader

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

def read_live_data(ip, port, config, as_pcl_structs=False):
    try:
        decoder = vd.StreamDecoder(config)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind((ip, port))
        s.settimeout(1.0)  # avoid blocking forever
        logging.info("Listening for LiDAR on %s:%d", ip, port)
    except Exception as e:
        logging.exception("Failed to initialize LiDAR decoder")
        raise

    try:
        while True:
            try:
                data, address = s.recvfrom(vd.PACKET_SIZE * 2)
                recv_stamp = time.time()
                decoded = decoder.decode(recv_stamp, data, as_pcl_structs)
                yield decoded
            except socket.timeout:
                continue
            except Exception:
                logging.exception("Error decoding LiDAR packet, skipping")
                continue
    except KeyboardInterrupt:
        logging.info("Stopping LiDAR reader (KeyboardInterrupt)")
    finally:
        s.close()

def ensure_dir(path):
    os.makedirs(path, exist_ok=True)

def run_recorder(ip, port, out_dir, camera_source=0, max_queue=8, use_dummy_on_fail=True, dummy_resolution=(1920,1080),
                 start_offset: float = 0.0, duration: float = 0.0, interval: float = 0.0):
    """
    start_offset: seconds after the first LiDAR timestamp to start saving (>=0)
    duration: seconds to save for after start (<=0 means no end)
    interval: minimum seconds between saved frames (<=0 means save every frame)
    """
    ensure_dir(out_dir)
    cfg = vd.Config(model=vd.Model.VLP32C)

    camera_q: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=max_queue)
    stop_flag = {"stop": False}

    # start camera reader thread (timestamped frames so we can time-match to LiDAR)
    threading.Thread(
        target=camera_reader,
        args=(camera_source, camera_q, stop_flag, 30),
        kwargs={'use_dummy_on_fail': use_dummy_on_fail, 'dummy_resolution': dummy_resolution,
                'with_timestamp': True},
        daemon=True,
    ).start()
    logging.info("camera_reader started (source=%s)", str(camera_source))
    logging.info("Save parameters: start_offset=%.3f, duration=%.3f, interval=%.3f", start_offset, duration, interval)

    # Recent (capture_time, frame) history, so each LiDAR sweep can be paired with
    # the camera frame CLOSEST in time rather than just "the latest".
    cam_buffer = deque(maxlen=60)
    # Warn when the best camera match is staler than this (s). On a moving vehicle
    # this offset times the speed is the projection shift you see.
    MAX_SYNC_DT = 0.05
    saved_count = 0

    first_stamp = None
    start_ts_abs = None
    end_ts_abs = None
    last_saved_ts = None

    try:
        for Data in read_live_data(ip, port, cfg):
            if stop_flag["stop"]:
                break
            if Data is None:
                continue

            # Normalize decoded data -> (stamp, points)
            stamp = None
            points = None
            try:
                if isinstance(Data, tuple) or isinstance(Data, list):
                    stamp, points = Data[0], Data[1]
                elif isinstance(Data, dict):
                    # support dict-shaped decoders
                    stamp = Data.get("stamp") or Data.get("time") or Data.get("timestamp")
                    points = Data.get("points") or Data.get("points_xyz") or Data.get("pc")
                else:
                    logging.warning("Unknown LiDAR data format: %s", type(Data))
                    continue
            except Exception:
                logging.exception("Failed to unpack LiDAR data")
                continue

            if stamp is None or points is None:
                logging.warning("LiDAR data missing stamp or points, skipping")
                continue

            # initialize start/end absolute times based on first received stamp
            try:
                stamp_f = float(stamp)
            except Exception:
                stamp_f = time.time()
            if first_stamp is None:
                first_stamp = stamp_f
                start_ts_abs = first_stamp + max(0.0, float(start_offset or 0.0))
                if duration and duration > 0.0:
                    end_ts_abs = start_ts_abs + float(duration)
                logging.info("First LiDAR stamp=%.6f. Will start saving at %.6f", first_stamp, start_ts_abs)
                if end_ts_abs:
                    logging.info("Will stop saving at %.6f", end_ts_abs)

            # drain camera queue into the timestamped history buffer
            try:
                while True:
                    item = camera_q.get_nowait()
                    if isinstance(item, tuple):
                        cam_buffer.append(item)               # (capture_time, frame)
                    else:
                        cam_buffer.append((time.time(), item))  # fallback: bare frame
            except queue.Empty:
                pass
            except Exception:
                logging.exception("Error reading camera queue")

            # check interval and start/end windows
            if start_ts_abs is not None and stamp_f < start_ts_abs:
                # not yet in saving window
                continue
            if end_ts_abs is not None and stamp_f > end_ts_abs:
                logging.info("Reached end of requested duration (stamp=%.6f > end=%.6f). Stopping.", stamp_f, end_ts_abs)
                break
            if interval and interval > 0.0 and last_saved_ts is not None and stamp_f < (last_saved_ts + interval):
                # skip save due to interval throttle
                continue

            # safe timestamp for filenames (microsecond integer)
            ts_us = int(stamp_f * 1e6)
            ts_str = f"{ts_us:020d}"
            img_fname = os.path.join(out_dir, f"camera_{ts_str}.jpg")
            lidar_fname = os.path.join(out_dir, f"lidar_{ts_str}.npy")
            pcd_fname = os.path.join(out_dir, f"lidar_{ts_str}.ply")

            # pick the camera frame closest in time to this LiDAR sweep
            best_frame = None
            sync_dt = None
            if cam_buffer:
                t_cam, best_frame = min(cam_buffer, key=lambda tf: abs(tf[0] - stamp_f))
                sync_dt = t_cam - stamp_f

            try:
                # save camera image if available
                if best_frame is not None:
                    cv2.imwrite(img_fname, best_frame)
                    if sync_dt is not None and abs(sync_dt) > MAX_SYNC_DT:
                        logging.warning("Camera-LiDAR sync offset %.3fs exceeds %.3fs "
                                        "(box overlay will shift if the vehicle is moving)",
                                        sync_dt, MAX_SYNC_DT)
                else:
                    placeholder = np.zeros((dummy_resolution[1], dummy_resolution[0], 3), dtype=np.uint8)
                    cv2.putText(placeholder, "NO_CAMERA", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 2)
                    cv2.imwrite(img_fname, placeholder)

                # save raw points as numpy array
                if points is not None:
                    np.save(lidar_fname, np.asarray(points))
                    # also save a PLY for convenience if points have XYZ
                    try:
                        arr = np.asarray(points)
                        if arr.ndim == 2 and arr.shape[1] >= 3:
                            xyz = arr[:, :3].astype(np.float32)
                            pcd = o3d.geometry.PointCloud()
                            pcd.points = o3d.utility.Vector3dVector(xyz)
                            o3d.io.write_point_cloud(pcd_fname, pcd, write_ascii=False)
                    except Exception:
                        logging.debug("PLY write failed (non-fatal)", exc_info=True)
                saved_count += 1
                last_saved_ts = stamp_f
                dt_ms = (sync_dt * 1e3) if sync_dt is not None else float('nan')
                logging.info("Saved #%d: %s , %s (stamp=%.6f, cam-lidar dt=%.1fms)",
                             saved_count, img_fname, lidar_fname, stamp_f, dt_ms)
            except Exception:
                logging.exception("Failed to save data for ts=%s", ts_str)

    except KeyboardInterrupt:
        logging.info("Interrupted by user")
    except Exception:
        logging.exception("Recorder error")
    finally:
        stop_flag["stop"] = True
        logging.info("Done. Saved %d pairs to %s", saved_count, out_dir)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Record LiDAR + Camera synchronized by LiDAR timestamps")
    parser.add_argument("host_ip", help="host IP to bind for LiDAR UDP")
    parser.add_argument("port", type=int, help="port to bind for LiDAR UDP")
    parser.add_argument("--out", "-o", default="./recordings", help="output directory")
    parser.add_argument("--camera", "-c", default="0", help="camera source (int for device or path)")
    parser.add_argument("--start-offset", type=float, default=0.0, help="seconds after first LiDAR stamp to start saving")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds to save for after start (0 = no limit)")
    parser.add_argument("--interval", type=float, default=0.0, help="minimum seconds between saved frames (0 = save every frame)")
    args = parser.parse_args()

    cam_src = int(args.camera) if args.camera.isdigit() else args.camera
    run_recorder(args.host_ip, args.port, args.out, camera_source=cam_src,
                 start_offset=args.start_offset, duration=args.duration, interval=args.interval)
