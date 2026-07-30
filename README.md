# katana_driver_ros2

> **Scope:** This repository is configured and tested for the **Neuronics Katana 400 6M180** arm only.
> Other Katana variants (300, 450) have partial code in the tree but are not supported by this setup.

ROS 2 port of the `katana_driver` stack — hardware driver, `ros2_control` integration, URDF description, and MoveIt 2 motion planning for the Katana 400 6M180 robot arm.

---

## Requirements

| Item | Version |
|------|---------|
| ROS 2 | Jazzy |
| MoveIt 2 | 2.12.x |
| OS | Ubuntu 24.04 |
| Hardware | Neuronics Katana 400 6M180 |
| Connection | Serial (USB-to-serial)(not tested) **or** TCP/Ethernet(tested) |

---

## Installation

### 1. Install ROS 2 Jazzy

Follow the official guide: https://docs.ros.org/en/jazzy/Installation.html

Add to `~/.bashrc` or `~/.zshrc`:
```bash
source /opt/ros/jazzy/setup.bash   # or setup.zsh
```

### 2. Install MoveIt 2

```bash
sudo apt install ros-jazzy-moveit
```

### 3. Install additional dependencies

```bash
sudo apt install \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    python3-colcon-common-extensions
```

### 4. Clone and build the workspace

```bash
git clone https://github.com/Idealize-2/katana_driver_ros2.git kanata_ws/src
cd kanata_ws

# Install rosdep dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source the workspace
source install/setup.bash   # or setup.zsh
```

> **Tip:** Add `alias sp='source install/setup.zsh'` to your shell config so you can re-source quickly after rebuilding.

---

## Package Overview

| Package | Purpose |
|---------|---------|
| `katana_description` | URDF/xacro meshes and robot description for Katana 400 6M180 |
| `katana_driver` | `ros2_control` hardware interface plugin (`KatanaHardwareInterface`) |
| `katana400_moveit_config` | MoveIt 2 config: SRDF, kinematics, OMPL, launch files |
| `kni` | Neuronics KNI 4.3.0 SDK wrapper (shared library) |
| `katana_test` | Manual tester node (`ros2control_tester`) with keyboard control |
| `katana_msgs` | Custom message/service types |

---

## Running

### Option A — Everything in one command (recommended)

Brings up the hardware driver, MoveIt move_group, and RViz together:

```bash
# Serial connection (default), run calibration
ros2 launch katana400_moveit_config full_system.launch.py

# TCP connection, run calibration
ros2 launch katana400_moveit_config full_system.launch.py \
    connection_type:=tcp ip_address:=192.168.1.1

# TCP connection, skip calibration (arm already homed this power cycle)
ros2 launch katana400_moveit_config full_system.launch.py \
    connection_type:=tcp ip_address:=192.168.1.1 calibrate_on_startup:=false
```

`full_system.launch.py` arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `connection_type` | `serial` | `serial` or `tcp` |
| `ip_address` | `192.168.1.1` | Arm IP address (TCP mode) |
| `tcp_port` | `5566` | KNI TCP port (TCP mode) |
| `calibrate_on_startup` | `true` | Run homing sequence on first activate |

---

### Option B — Three separate terminals

Useful for debugging individual components.

**Terminal 1 — Hardware driver + ros2_control:**
```bash
# Serial
ros2 launch katana400_moveit_config real_hardware.launch.py \
    connection_type:=serial serial_port:=0

# TCP
ros2 launch katana400_moveit_config real_hardware.launch.py \
    connection_type:=tcp ip_address:=192.168.1.1

# TCP, skip calibration
ros2 launch katana400_moveit_config real_hardware.launch.py \
    connection_type:=tcp ip_address:=192.168.1.1 calibrate_on_startup:=false
```

`real_hardware.launch.py` full argument list:

| Argument | Default | Description |
|----------|---------|-------------|
| `connection_type` | `serial` | `serial` or `tcp` |
| `ip_address` | `192.168.1.1` | Arm IP (TCP mode) |
| `tcp_port` | `5566` | KNI port (TCP mode) |
| `serial_port` | `0` | Index N for `/dev/ttyS<N>` (serial mode) |
| `serial_baud` | `57600` | Baud rate (serial mode) |
| `config_file` | *(bundled katana6M180.cfg)* | Absolute path to KNI `.cfg` |
| `calibrate_on_startup` | `true` | Run homing sequence on first activate |

**Terminal 2 — MoveIt move_group:**
```bash
ros2 launch katana400_moveit_config move_group.launch.py
```

**Terminal 3 — RViz with MoveIt plugin:**
```bash
ros2 launch katana400_moveit_config moveit_rviz.launch.py
```

---

### Manual keyboard tester

After the hardware driver is running, use the tester node for direct joint control:

```bash
ros2 run katana_teleop katana_teleop_key
```

Key bindings:

| Key | Action |
|-----|--------|
| `1`–`5` | Select arm joint to jog (1=pan, 2–5=lift/wrist) |
| `W` / `S` | Jog selected joint + / - step |
| `A` / `D` | Jog joint 1 (pan) + / - step |
| `H` | Home (all joints → 0 rad) |
| `G` / `C` | Open / Close gripper |
| `E` | Enable motors |
| `P` | Print joint states |
| `+` / `-` | Double / Halve step size |
| `Q` | Quit |

---

## Calibration

The Katana 400 uses **incremental encoders** — encoder values are consistent within a power cycle but reset on power-off. The `calibrate_on_startup` option runs the full KNI homing sequence (motors drive to mechanical stops) to establish a known reference position.

- Set `calibrate_on_startup:=true` on first launch after powering the arm.
- Set `calibrate_on_startup:=false` on subsequent launches within the same power cycle to avoid re-homing.

> **Note on offsets:** When the arm finishes this physical KNI calibration sequence, it rests in a specific physical "zero" position. This physical position does **not** match the `0.0` (start/home) pose defined in the URDF file. To resolve this discrepancy, the `KatanaHardwareInterface` uses joint offsets configured in `katana_400_6m180_with_controlbox.ros2_control.xacro` to transparently map the KNI encoder space to the URDF joint space.

---

## Checking controller status

```bash
# List active controllers
ros2 control list_controllers

# Check parameters loaded into move_group
ros2 param get /move_group moveit_controller_manager
ros2 param get /move_group moveit_simple_controller_manager.controller_names
```

---

## Known Issues / Troubleshooting

See the [`problem/`](../../problem/) folder at the workspace root for detailed write-ups of known issues and their fixes, including:

- [`moveit_0_controllers_in_list.md`](../../problem/moveit_0_controllers_in_list.md) — MoveIt execution fails with "Returned 0 controllers in list"
