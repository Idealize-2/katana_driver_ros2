# Katana 400 ROS 2 Docker Environment

This directory contains the Docker configuration for running the **Neuronics Katana 400 6M180** robot arm ROS 2 driver stack inside a containerized environment based on **ROS 2 Jazzy** and **Ubuntu 24.04 (Noble)**.

---

## 📋 Features

- **Pre-configured Environment**: Fully builds ROS 2 Jazzy with MoveIt 2, `ros2_control`, `xacro`, and the `kni` SDK library.
- **X11 Forwarding Support**: Run RViz 2 and graphical tools natively on the host display.
- **Host Networking**: Low-latency direct socket access to the physical Katana arm over TCP/IP or direct device access (`/dev`).
- **`just` Integration**: Convenient shorthand commands using `just` (or standard `docker compose`).

---

## 🛠️ Prerequisites

Before running the project, ensure the host machine has:

1. **Docker Engine & Docker Compose**:
   - Install: [Docker Installation Guide](https://docs.docker.com/engine/install/)
   - Check version:
     ```bash
     docker compose version
     ```
2. **X11 Server Access** (for GUI / RViz 2):
   - Check if `xhost` is installed:
     ```bash
     xhost +local:root
     ```
3. **`just` Command Runner** *(Optional but recommended)*:
   - Install on Ubuntu/Debian:
     ```bash
     sudo apt update && sudo apt install -y just
     ```

---

## 🚀 How to Run the Project (Step-by-Step)

Follow these exact steps from your terminal to launch the driver stack.

### Step 1: Navigate to the Docker Directory

Open your terminal and change directory to `docker/`:

```bash
cd ~/Documents/internA4robotics/katans_ws/src/katana_driver_ros2/docker
```

---

### Step 2: Build the Docker Image

Build the ROS 2 Jazzy container image containing all dependencies and compiled workspace packages.

**Option A — Using `just` (Recommended):**
```bash
just build
```

**Option B — Using standard `docker compose`:**
```bash
docker compose build
```

---

### Step 3: Allow GUI / RViz Access

Grant root access to your host display so RViz 2 can open inside the container:

```bash
xhost +local:root
```
*(Note: If using `just up` in Step 4, this step is executed automatically for you).*

---

### Step 4: Run the Project

Ensure your computer is connected to the Katana arm network (default arm IP is `192.168.1.1`).

#### Method 1: Run Full System in Foreground (Hardware Driver + MoveIt + RViz)

**Using `just`:**
```bash
just up
```

**Using Docker Compose:**
```bash
docker compose up
```

*This will initialize `ros2_control`, connect to the Katana 400 arm via TCP, start `move_group`, and open the RViz 2 visualization window on your screen.*

---

#### Method 2: Run Full System in Background (Detached Mode)

If you want the container to run silently in the background:

**Using `just`:**
```bash
just up-d
```

**Using Docker Compose:**
```bash
docker compose up -d
```

---

### Step 5: Interact with the Running System

#### A. Open an Interactive Shell inside Container
To run ROS 2 CLI commands (e.g., `ros2 topic list`, `ros2 node list`, `ros2 control list_controllers`):

**Using `just`:**
```bash
just shell
```

**Using Docker command:**
```bash
docker exec -it katana_driver_container bash -c "source /opt/ros/jazzy/setup.bash && source /katana_ws/install/setup.bash && exec bash"
```

---

#### B. Control Arm via Keyboard Teleoperation
In a second terminal window (while the main container is running), launch keyboard jog controls:

**Using `just`:**
```bash
just teleop
```

**Using Docker command:**
```bash
docker exec -it katana_driver_container /docker-entrypoint.sh ros2 run katana_teleop katana_teleop_key
```

**Keyboard Controls:**
- `1` to `5`: Select arm joint (Motor 1 - Motor 5)
- `w` / `s` or `a` / `d`: Jog joint angle +/-
- `h`: Move arm to home position
- `g` / `c`: Open / Close gripper
- `e`: Enable motor power
- `p`: Print current joint encoder positions
- `q`: Quit teleop mode

---

### Step 6: Stop the Project

To stop and remove running containers:

**Using `just`:**
```bash
just down
```

**Using Docker Compose:**
```bash
docker compose down
```

---

## 🧩 Running Individual Components (Modular Execution)

If you wish to run individual sub-systems separately across different terminals:

### 1. Hardware Driver Only
Launches `ros2_control_node` and connects to physical robot:
```bash
# Using just:
just hardware

# Using Docker Compose:
docker compose run --rm katana_system ros2 launch katana400_moveit_config real_hardware.launch.py connection_type:=tcp ip_address:=192.168.1.1
```

### 2. MoveIt `move_group` Only
Launches MoveIt motion planning pipeline:
```bash
# Using just:
just move-group

# Using Docker Compose:
docker compose run --rm katana_system ros2 launch katana400_moveit_config move_group.launch.py
```

### 3. RViz 2 Visualization Only
Launches RViz 2 GUI with MoveIt motion planning panel:
```bash
# Using just:
just rviz

# Using Docker Compose:
docker compose run --rm katana_system ros2 launch katana400_moveit_config moveit_rviz.launch.py
```

---

## ⚙️ Customization & Connection Settings

### Custom Robot IP Address
If your Katana arm has a different IP address (e.g., `192.168.1.50`), specify the parameter:

```bash
docker compose run --rm katana_system ros2 launch katana400_moveit_config full_system.launch.py connection_type:=tcp ip_address:=192.168.1.50
```

### Serial / USB Connection
If connected via a serial cable (e.g., `/dev/ttyUSB0`), pass `connection_type:=serial`:

```bash
docker compose run --rm katana_system ros2 launch katana400_moveit_config full_system.launch.py connection_type:=serial device:=/dev/ttyUSB0
```

---

## 📁 File Structure Overview

- [`Dockerfile`](file:///home/kay/Documents/internA4robotics/katans_ws/src/katana_driver_ros2/docker/Dockerfile): Defines ROS 2 Jazzy image build steps, dependency installation (`rosdep`), and `colcon` workspace compilation.
- [`docker-compose.yml`](file:///home/kay/Documents/internA4robotics/katans_ws/src/katana_driver_ros2/docker/docker-compose.yml): Configures container networking (`host`), X11 display volume, and hardware device access.
- [`docker-entrypoint.sh`](file:///home/kay/Documents/internA4robotics/katans_ws/src/katana_driver_ros2/docker/docker-entrypoint.sh): Entrypoint script sourcing ROS 2 base and workspace environments automatically.
- [`justfile`](file:///home/kay/Documents/internA4robotics/katans_ws/src/katana_driver_ros2/docker/justfile): Command runner recipes for easy management.

---

## 💡 Common Troubleshooting

### Issue 1: RViz fails to open (`Error: Can't open display: :0`)
**Fix:** Grant X11 permissions on the host system:
```bash
xhost +local:root
```

### Issue 2: Network timeout connecting to arm (`192.168.1.1`)
1. Verify host IP is set to `192.168.1.x` subnet.
2. Test connection inside container shell:
   ```bash
   just shell
   ping 192.168.1.1
   ```
