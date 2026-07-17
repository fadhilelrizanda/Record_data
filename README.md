# LiDAR + Camera Synchronized Recording & Live Preview Pipeline

This repository contains a robust, real-time pipeline for capturing, visualizing, and recording synchronized data from a Velodyne LiDAR (specifically configured for **VLP32C**) and a camera source. It is designed to work in performance-critical configurations, including hardware environments running under Conda/Wayland/X11.

---

## 🚀 Key Features

*   **LiDAR-Time Synchronization**: Points are timestamped by the LiDAR sweeps, and each sweep is paired with the camera frame closest in time using an automated time-matched buffer history (`deque`). This minimizes projection shifts on moving platforms.
*   **Offscreen Filament Rendering**: Renders high-performance, intensity-colored 3D point clouds headless using Open3D's EGL/Filament `OffscreenRenderer`, sidestepping Wayland/XWayland display driver bugs.
*   **Unified Preview Window**: Composites the live camera stream and the rendered point cloud side-by-side inside a single OpenCV window.
*   **Flexible Dataset Exporters**: Saves synchronized sensor frames as raw points (`.npy`), point clouds (`.ply`), and images (`.jpg`), while documenting the session metadata inside a centralized `manifest.csv`.
*   **Simulation Support**: Run the entire rendering, previewing, and recording pipeline using synthetic LiDAR sweeps and camera streams without needing physical hardware.
*   **Network Diagnostics**: Included bash utilities to sniff UDP/ARP networks (`scan_lidar.sh`) and configure interface routing (`setup_lidar.sh`).

---

## 📁 Repository Structure

*   [`main.py`](file:///home/fadhil/program/Record_data/main.py): Core CLI recorder. Manages the socket decoder thread, the camera thread queue, time-synchronization logic, and file writer.
*   [`live_view.py`](file:///home/fadhil/program/Record_data/live_view.py): Dual-sensor visualizer. Coordinates EGL-based offscreen Open3D point cloud rendering and composite preview drawing. Supports live recording and simulation modes.
*   [`utils/camera_stream.py`](file:///home/fadhil/program/Record_data/utils/camera_stream.py): Thread-safe camera reader. Supports dynamic frame-rate limiting, device-to-file fallback, and automatic blank/dummy frame generation if the camera goes offline.
*   [`run_live_view.sh`](file:///home/fadhil/program/Record_data/run_live_view.sh): Helper wrapper script configured to resolve graphics stack clashes (Mesa vs. Conda) and set the interface routing before launching `live_view.py`.
*   [`scan_lidar.sh`](file:///home/fadhil/program/Record_data/scan_lidar.sh): Network sniffer to dynamically capture UDP/ARP frames to find a Velodyne sensor on the local link.
*   [`setup_lidar.sh`](file:///home/fadhil/program/Record_data/setup_lidar.sh): Static interface address config utility.
*   [`CHECK_CAMERAS.md`](file:///home/fadhil/program/Record_data/CHECK_CAMERAS.md): Diagnostic reference for enumerating video capturing devices on Linux.
*   [`lidar.md`](file:///home/fadhil/program/Record_data/lidar.md): Routing quick-reference rules.

---

## 🛠️ Installation & Requirements

Ensure you have the required system and Python dependencies installed:

### 1. System Dependencies (Debian/Ubuntu)
```bash
sudo apt update
sudo apt install tcpdump v4l-utils libgl1-mesa-glx
```

### 2. Python Packages
Install the required packages in your active environment:
```bash
pip install numpy opencv-python open3d velodyne_decoder
```

---

## 🌐 Network Setup (Velodyne LiDAR)

The Velodyne sensor (default IP `192.168.0.201` or `192.168.1.201`) operates by broadcasting/unicasting UDP packets. Crucially, it sends an ARP request for the target host IP (usually `192.168.0.2` or `192.168.1.2`) and will only stream points once the host interface answers.

### Discovering the Sensor
If you do not know the sensor's IP or which network interface it is connected to, run:
```bash
sudo ./scan_lidar.sh <interface_name> [seconds]
```
This utility uses `tcpdump` to sniff the wire for UDP (ports `2368`/`8308`) and ARP traffic. It prints out the sensor's IP, target host IP, and configuration tips.

### Configuring the Network Interface
Bind your local interface to the subnet expected by the LiDAR:
```bash
# Example setup for a 192.168.1.x subnet
sudo ip addr replace 192.168.1.2/24 dev <interface_name>
sudo ip link set <interface_name> up
```
Or utilize the [`setup_lidar.sh`](file:///home/fadhil/program/Record_data/setup_lidar.sh) script after customizing the interface variable.

---

## 💻 Usage

### 1. Running in Simulation (No Hardware Needed)
Test the entire pipeline using synthetic data streams:
```bash
python3 live_view.py --simulate --record --duration 60 --out ./recordings_sim
```
*   Press `q` or `ESC` inside the viewer window to quit early.

### 2. Launching Live Visualization & Recording
Run the system-optimized wrapper to bypass display server conflicts (recommending for Wayland & Conda environments):
```bash
./run_live_view.sh
```
Or launch the visualizer directly:
```bash
python3 live_view.py <host_ip> <port> --camera <camera_idx> --record
```

### 3. Recording Headless (CLI Only)
To record data in the background without launching the visualization window:
```bash
python3 main.py 0.0.0.0 2368 --camera 0 --out ./recordings --duration 120
```

### Command Line Arguments for `live_view.py`:
*   `host_ip` (positional): IP address to bind (default: `0.0.0.0`).
*   `port` (positional): Port to bind for UDP packets (default: `2368`).
*   `-c`, `--camera`: Camera index or video path (default: `4`).
*   `--record`: Save synced frames to disk.
*   `-o`, `--out`: Output directory for recordings (default: `./recordings`).
*   `--simulate`: Run using synthetic inputs.
*   `--duration`: Autostop after `N` seconds.
*   `--interval`: Minimum time (in seconds) between saved frames.
*   `--zoom`: Open3D camera zoom level (default: `2.5`).
*   `--no-display`: Suppress OpenCV visualization (headless).

---

## 📊 Recorded Dataset Output Format

Recorded data is exported into the `--out` folder in pairs synchronized on the LiDAR timestamp:

```
recordings/
├── manifest.csv
├── camera_00000178430520201021.jpg
├── lidar_00000178430520201021.npy
├── lidar_00000178430520201021.ply
...
```

*   **`manifest.csv`**: Contains indices, exact timestamps, LiDAR-Camera delta offsets (`dt_ms`), point counts, and relative filenames for every captured frame.
*   **`.jpg`**: The camera image at the matching timestamp.
*   **`.npy`**: Raw point cloud as a NumPy array (shape `N x 4`, containing `X, Y, Z, Intensity` or `N x 8` depending on decoder settings).
*   **`.ply`**: Point cloud exported in PLY format for quick inspection in standard 3D viewers (MeshLab, CloudCompare).

---

## 🎨 Wayland / Conda Workarounds

Conda base environments often bundle an older `libstdc++` which conflicts with system Mesa drivers (`iris_dri.so`), crashing hardware-accelerated Open3D windows. Additionally, Open3D's visualizer fails to open direct OpenGL windows under Wayland. 

This repository fixes both issues inside [`live_view.py`](file:///home/fadhil/program/Record_data/live_view.py) and [`run_live_view.sh`](file:///home/fadhil/program/Record_data/run_live_view.sh) by:
1.  Setting `EGL_PLATFORM=surfaceless` to force headless render.
2.  Preloading the system libstdc++ (`LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6`).
3.  Rendering Open3D point clouds offscreen to a buffer, and drawing the composite via OpenCV on XWayland.
