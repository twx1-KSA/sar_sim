# Autonomous Multi-UAV Search and Rescue Simulation

Four simulated PX4 X500 drones search a 250 m-radius mountain area for missing people,
detect them with thermal cameras, confirm them with a dedicated drone, and replace a failed
search drone automatically.

Built with PX4 SITL, Gazebo Harmonic, ROS 2 Humble and Python, on Ubuntu 22.04.

| Drone | Role | Start position (world x east, y north) |
|---|---|---|
| 0 | Search drone, west half | terrain, (-55, -20) |
| 1 | Search drone, east half | terrain, (62, 22) |
| 2 | Backup: takes over a failed search drone's remaining area | blue pad, (3, -13) |
| 3 | Confirmation: flies to each detection and confirms it | orange pad, (4, 12) |

**What works (tested in simulation):**
- Terrain-following lawnmower search at 12 m above ground.
- Thermal person detection that reports the *person's* GPS position (0.3–2.4 m error in tests).
- Automatic confirmation flights with an RGB close-up.
- Backup takeover after a search drone fails (failsafe landing or lost link).

---

## Contents

1. [Install on a new PC (step by step)](#1-install-on-a-new-pc-step-by-step)
2. [Run the simulation](#2-run-the-simulation)
3. [How it works](#3-how-it-works)
4. [Repository layout](#4-repository-layout)
5. [Troubleshooting](#5-troubleshooting)
6. [Known limitations](#6-known-limitations)
7. [Credits](#7-credits)

---

## 1. Install on a new PC (step by step)

Do the steps in order. Each step ends with a **Check**: don't continue until the check passes.
Total time is about 2–3 hours, mostly downloads and the first PX4 build.

This guide uses **Windows + VMware Workstation** with an **Ubuntu 22.04** virtual machine.
ROS 2 Humble requires Ubuntu 22.04.

> **Performance in a VM.** A VMware VM can't use the PC's GPU for Gazebo's camera sensors, so
> they are rendered on the CPU. The more CPU cores and RAM you give the VM, the faster the
> simulation runs. `start_drones.sh` detects the VM and switches to software rendering by
> itself. In tests it was much faster than VMware's 3D driver for the cameras.

### Step 0a: Create the virtual machine

1. Install **VMware Workstation Pro** on Windows (free for personal use) from
   https://www.vmware.com/products/desktop-hypervisor/workstation-and-fusion
2. Download the **Ubuntu 22.04 LTS Desktop** ISO (`ubuntu-22.04.x-desktop-amd64.iso`) from
   https://releases.ubuntu.com/22.04/
3. In VMware: **File → New Virtual Machine → Typical**. Select the Ubuntu ISO and follow the
   wizard. Name the VM, for example `SAR-Ubuntu22`.
4. Before the first start, open **Edit virtual machine settings** and set:

| Setting | Value |
|---|---|
| **Processors** | 1 processor, **as many cores as possible, leaving 2 for Windows** (for example 10 of 12, or 14 of 16) |
| **Memory** | **at least 16 GB**, about half to two-thirds of the PC's RAM (4 drones need about 8 GB) |
| **Hard disk** | **100 GB**, "Store virtual disk as a single file" |
| **Display** | "Accelerate 3D graphics" **on**, graphics memory as high as allowed (makes the Gazebo window smoother) |
| **Network** | NAT (default) |

**Check:** the VM settings show your core count and memory.

### Step 0b: Install Ubuntu in the VM

1. Start the VM and choose **Install Ubuntu**.
2. Choose **Normal installation** and tick **Download updates while installing**.
3. Choose **Erase disk and install Ubuntu**. This only erases the VM's virtual disk, not Windows.
4. Create your user. With username `twx`, the paths in this guide match exactly; any username
   works, because the scripts use `$HOME`.
5. Restart when asked, then log in.

**Check:** in a terminal (`Ctrl+Alt+T`), `lsb_release -ds` prints `Ubuntu 22.04.x LTS`.

### Step 0c: VMware tools, updates, and the Xorg desktop session

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y open-vm-tools open-vm-tools-desktop
sudo reboot
```

`open-vm-tools-desktop` gives copy/paste between Windows and the VM and resizes the screen
with the window.

Then use the **Xorg** session, because the Gazebo window has problems on Wayland:

1. Log out.
2. On the login screen, click your name, then the **gear icon** at the bottom right.
3. Choose **Ubuntu on Xorg**, and log in.

**Check:** `echo $XDG_SESSION_TYPE` prints `x11`, and `nproc` prints the number of cores you
gave the VM.

### Step 1: Basic tools

```bash
sudo apt update
sudo apt install -y git curl wget python3-pip
```

**Check:** `git --version` prints a version.

### Step 2: PX4-Autopilot (also installs Gazebo Harmonic)

This project was developed and tested with PX4 commit `c4b8f107ae` (2026-11-15). Use the
same commit so the simulation behaves the same way.

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd ~/PX4-Autopilot
git checkout c4b8f107ae
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh
```

`ubuntu.sh` installs the build tools, PX4's Python packages and **Gazebo Harmonic**. It asks
for your password and takes 10–30 minutes. **Reboot when it finishes:**

```bash
sudo reboot
```

**Check:** `gz sim --versions` prints a version starting with `8.`.

### Step 3: Build PX4 and test one drone

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

The first build takes 5–15 minutes. A Gazebo window opens with a drone on a grey ground plane,
and the terminal shows a `pxh>` prompt.

If the Gazebo window stays black, flickers, or Gazebo crashes (common with VMware's 3D driver),
press `Ctrl+C` and start it with software rendering instead:

```bash
LIBGL_ALWAYS_SOFTWARE=1 make px4_sitl gz_x500
```

**Check:** type `commander takeoff` at the `pxh>` prompt. The drone takes off in Gazebo.
Then press `Ctrl+C` to stop.

### Step 4: ROS 2 Humble

These are the official steps from the ROS 2 Humble documentation
(https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). If a command fails,
follow that page instead.

```bash
# Locale with UTF-8
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Enable the Ubuntu "universe" repository
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

# Add the ROS 2 package source
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install ROS 2 Humble
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop

# Load ROS 2 automatically in every new terminal
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Check:** in a **new** terminal, `ros2 --help` prints the ROS 2 command list.

### Step 5: ROS 2 ↔ Gazebo Harmonic bridge

ROS 2 Humble's default bridge is built for an older Gazebo. The Harmonic version comes from
the Gazebo package source that `ubuntu.sh` added in Step 2:

```bash
sudo apt install -y ros-humble-ros-gzharmonic
```

**Check:** `ros2 pkg list | grep ros_gz_bridge` prints `ros_gz_bridge`.

### Step 6: Python packages for the project

```bash
sudo apt install -y python3-numpy python3-pil
pip3 install --user pymavlink
```

**Check:**

```bash
python3 -c "import pymavlink, numpy, PIL; print('python OK, numpy', numpy.__version__)"
```

It should print `python OK`. NumPy must be **1.x** (Ubuntu 22.04 ships 1.21). If it prints
2.x, see [Troubleshooting](#5-troubleshooting).

### Step 7: Get this project and link it into PX4

The repository is public, so cloning over HTTPS needs no SSH key or GitHub login:

```bash
cd ~
git clone https://github.com/twx1-KSA/sar_sim.git sar_sim
cd ~/sar_sim
git config user.name "TWX"
git config user.email "260037218+twx1-KSA@users.noreply.github.com"
./install.sh
```

`install.sh` links the SAR world and models into `~/PX4-Autopilot/Tools/simulation/gz`, where
PX4 and Gazebo look for them. It uses links, not copies, so edits in `~/sar_sim` take effect
immediately. If PX4 is not in `~/PX4-Autopilot`, run `PX4_DIR=/path/to/PX4-Autopilot ./install.sh`.

Optional: `./install.sh --extras` also installs the earlier `x500_360_cam` experiment
(airframe 4022). After that, run `make px4_sitl` again in `~/PX4-Autopilot`.

**Check:** `ls -l ~/PX4-Autopilot/Tools/simulation/gz/worlds/sar_mountain.sdf` shows an arrow
(`->`) pointing to `~/sar_sim/gz/worlds/sar_mountain.sdf`.

**Optional: push changes from this PC.** Cloning and pulling work without a login, but pushing
needs an SSH key on your GitHub account:

```bash
ssh-keygen -t ed25519 -C "TWX sar_sim $(hostname)" -f ~/.ssh/id_ed25519 -N ""
cat ~/.ssh/id_ed25519.pub
```

Open https://github.com/settings/ssh/new, enter a title (for example `New PC`), paste the line
printed by `cat`, and click **Add SSH key**. Then switch the clone to SSH:

```bash
cd ~/sar_sim
git remote set-url origin git@github.com:twx1-KSA/sar_sim.git
```

**Check:** `ssh -T git@github.com` answers `Hi twx1-KSA! You've successfully authenticated`.
Type `yes` if it asks whether to trust github.com.

### Step 8: First SAR test, one drone

```bash
GUI=1 ~/sar_sim/start_drones.sh 0
```

A Gazebo window opens with the mountain terrain. The script prints a line like
`drone 0: [-54.99 -20.00 29.89] [-0.005 0.036 -0.000]`: the drone stands on the
terrain, nearly level.

In a **second terminal**:

```bash
ros2 launch ~/sar_sim/ros2/sar_bringup.launch.py drones:=0
```

In a **third terminal**:

```bash
ros2 topic list | grep drone0
```

**Check:** you see `/drone0/front_camera/image_raw`, `/drone0/down_camera/image_raw` and
`/drone0/thermal_camera/image_raw`. To look at them: `ros2 run rqt_image_view rqt_image_view`.

Stop everything:

```bash
~/sar_sim/stop_all.sh
```

### Step 9 (optional): QGroundControl

Download `QGroundControl-x86_64.AppImage` from https://docs.qgroundcontrol.com
(Download and Install, Ubuntu Linux), then:

```bash
sudo usermod -aG dialout $USER     # log out and back in after this
chmod +x ~/Downloads/QGroundControl-x86_64.AppImage
~/Downloads/QGroundControl-x86_64.AppImage
```

QGroundControl connects to the simulated drones automatically on UDP port 14550.

The installation is complete. Continue with [Run the simulation](#2-run-the-simulation).

---

## 2. Run the simulation

Every command below runs in its own terminal. Wait for each one to finish starting before
running the next.

### Full scenario: 4 drones

```bash
# Terminal 1: Gazebo + 4 PX4 drones (add GUI=1 in front to see the Gazebo window)
~/sar_sim/start_drones.sh

# Terminal 2: camera bridge + one thermal detector per drone
ros2 launch ~/sar_sim/ros2/sar_bringup.launch.py drones:=0,1,2,3

# Terminal 3: coordinator (confirmation drone + backup drone + fleet monitoring)
cd ~/sar_sim/ros2 && python3 sar_coordinator.py

# Terminal 4: search drone 0, west half
cd ~/sar_sim/ros2 && python3 sar_mission.py --drone 0 --region west

# Terminal 5: search drone 1, east half
cd ~/sar_sim/ros2 && python3 sar_mission.py --drone 1 --region east
```

What you should see:

1. Drones 0 and 1 take off and fly north-south sweeps over their halves.
2. When a search drone flies near a hidden person, Terminal 2 prints `NEW detection D0-001 ...`.
3. Terminal 3 prints `T1 UNCONFIRMED`, then launches drone 3.
4. Drone 3 flies there and hovers. Terminal 3 prints `T1 CONFIRMED` and saves a close-up.
5. The search drones finish and land on their pads.

A full run took about 30–45 minutes in a 4-core VM; it gets faster with more CPU cores in the VM.

Watch the results live:

```bash
ros2 topic echo --full-length /sar/detections   # every detection
ros2 topic echo --full-length /sar/targets      # target status: unconfirmed / confirming / confirmed / not_found
ros2 topic echo --full-length /sar/fleet        # search started/finished, drone failed, backup takeover
```

Stop everything: `~/sar_sim/stop_all.sh`

### Test the backup drone

While the full scenario runs, simulate a failure of search drone 0 by forcing an emergency
landing (as a PX4 failsafe would):

```bash
~/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander --instance 0 mode auto:land
```

Within a few seconds Terminal 3 prints `FLEET search_drone_failed` and `FLEET takeover_started`.
Drone 2 then takes off and finishes drone 0's remaining sweeps.

Do **not** simulate a failure by killing a PX4 process. The drone's motors keep spinning, it
tumbles, and Gazebo crashes.

### Smaller runs

```bash
~/sar_sim/start_drones.sh 0                     # one drone only
ros2 launch ~/sar_sim/ros2/sar_bringup.launch.py drones:=0
cd ~/sar_sim/ros2 && python3 sar_mission.py --drone 0 --region west

python3 sar_mission.py --plan-only              # plan the search and save a map, without flying
python3 sar_mission.py --help                   # all options (height, spacing, speed, ...)
```

### Output files (not stored in git)

| Folder | Contents |
|---|---|
| `~/sar_sim/missions/` | search plan maps (`plan_*.png`), waypoints (`plan_*.csv`), flight logs (`flight_*.csv`) |
| `~/sar_sim/detections/` | thermal image for each detection, RGB close-up for each confirmed target |
| `~/sar_sim/logs/` | PX4 console log per drone |

---

## 3. How it works

```
Gazebo Harmonic (world, physics, cameras)
   │ camera images                       ▲ motor commands
   ▼                                     │
ros_gz_bridge ──► ROS 2 topics       PX4 SITL x4 (flight control)
                     │                   ▲ missions, mode, arm (MAVLink)
                     ▼                   │
           thermal_detector x4 ──► /sar/detections ──► sar_coordinator ──► /sar/targets, /sar/fleet
                     ▲ drone position/attitude (MAVLink)       │
                     └─────────────── PX4 ◄───────────────────┘
           sar_mission.py (one per search drone) ──► PX4 mission upload
```

**World.** `gz/worlds/sar_mountain.sdf` holds a 500 × 500 m heightmap terrain up to 90 m high,
GPS origin 24.7136 N, 46.6753 E, 600 m AMSL. It contains two hidden people (warm thermal
models) and landing platforms for drones 2 and 3. The world lists all Gazebo systems PX4
needs itself, because declaring any plugin in a world makes Gazebo ignore PX4's `server.config`.

**Drone.** `gz/models/x500_dual_cam` is the PX4 X500 with three cameras:
- front RGB: 1280×960,
- downward RGB: 1280×960,
- downward thermal: 320×240, 0.01 K per count.

**Search (`ros2/sar_mission.py`).**
- Plans north-south sweep lines 22 m apart over one half of the circle.
- Sets every waypoint to terrain height plus 12 m, using the terrain heightmap.
- Shapes the altitude profile so the drone never has to climb or descend faster than 80% of
  PX4's limits.
- Uploads the plan as a normal PX4 mission over its own MAVLink link and logs clearance.

**Detection (`ros2/thermal_detector.py`).**
1. Finds pixels above 300 K and groups them into blobs.
2. Casts a ray from the drone through each blob, using camera geometry and drone attitude, and
   intersects it with the terrain to get the person's GPS position.
3. Reports a detection once the person is seen in 3 frames.

**Coordinator (`ros2/sar_coordinator.py`).**
- **Targets:** merges sightings within 10 m into one target (T1, T2, ...).
- **Confirmation:** flies drone 3 to each target at 25 m above ground, descends to 10 m and
  hovers 20 s. The target is confirmed when drone 3's own detector sees it, and an RGB
  close-up is saved.
- **Backup:** watches drones 0 and 1. If one stops sending heartbeats for 5 s of simulation
  time, leaves mission mode (failsafe) or disarms early, drone 2 resumes its plan from the last
  waypoint it reached.

**MAVLink ports per drone N** (PX4 instance N, system ID N+1):

| Port | Used by |
|---|---|
| 14550 | QGroundControl (all drones) |
| 14540+N | thermal detector N |
| 14600+N / 14610+N | `sar_mission.py`, or the coordinator for drones 2 and 3 |
| 14620+N / 14630+N | coordinator fleet monitor for search drones |

---

## 4. Repository layout

```
sar_sim/
├── README.md
├── install.sh              link world + models into PX4 (run once per PX4 checkout)
├── start_drones.sh         start Gazebo + PX4 drones on their pads
├── stop_all.sh             stop everything
├── gz/
│   ├── worlds/sar_mountain.sdf
│   └── models/
│       ├── sar_terrain/        heightmap terrain
│       ├── sar_person/         missing-person model with body heat
│       ├── x500_dual_cam/      drone with three cameras
│       ├── mono_cam_front/  mono_cam_down/  thermal_cam_down/
├── ros2/
│   ├── sar_bringup.launch.py   camera bridge + detectors
│   ├── camera_bridge.launch.py
│   ├── thermal_detector.py
│   ├── sar_mission.py
│   ├── sar_coordinator.py
│   └── sar_geo.py              shared GPS / terrain helpers
└── extras/                 earlier x500_360_cam experiment (airframe 4022), not used by SAR
```

---

## 5. Troubleshooting

| Problem | Fix |
|---|---|
| `Unable to find uri[model://sar_terrain]` | Run `./install.sh`. When running `gz sim` by hand, first `source ~/PX4-Autopilot/build/px4_sitl_default/gz_env.sh`. |
| A drone is tilted or upside down on its pad (`Preflight Fail: Attitude failure`) | Only spawn drones at the positions in `start_drones.sh`. Other spots on the terrain can tip the drone over. |
| `ImportError: numpy.core.multiarray failed to import`, or cv2 errors | A pip-installed NumPy 2.x is hiding Ubuntu's NumPy 1.x. Run `pip3 install --user "numpy<2"`. |
| `ModuleNotFoundError: pymavlink` | `pip3 install --user pymavlink`. Don't run the scripts with `python3 -I`. |
| Camera topics exist but no images arrive | Image topics are lazy: they only render while something subscribes. For the 1280×960 RGB topics use reliable QoS (best-effort drops large frames). |
| Simulation slower than real time | Give the VM more CPU cores and RAM (VM settings, with the VM shut down). Run fewer drones (`start_drones.sh 0 1`). Close other programs on Windows. To compare, try VMware's 3D driver once with `SOFTWARE_GL=0 ~/sar_sim/start_drones.sh` and keep whichever is faster. |
| Gazebo window black, flickering or crashing in the VM | Use the Xorg session (Step 0c). Open the window with software rendering: `env -u WAYLAND_DISPLAY LIBGL_ALWAYS_SOFTWARE=1 gz sim -g` (attaches to an already running simulation), or start the drones with `GUI=1 ~/sar_sim/start_drones.sh`. |
| Copy/paste between Windows and the VM doesn't work | `sudo apt install -y open-vm-tools-desktop` and reboot the VM. |
| `gz sim` keeps running after `Ctrl+C` | `~/sar_sim/stop_all.sh` (it force-stops stuck processes). |
| Arming refused: `No connection to the GCS` | Normal without QGroundControl. The SAR scripts identify themselves as a GCS. |

---

## 6. Known limitations

- **Thermal-only detection:** anything warmer than 27 °C counts as a person. There's no RGB or
  YOLO check yet.
- **Slow flying:** the drones average about 3 m/s instead of the planned 5 m/s, because PX4 slows
  at every terrain-following waypoint. After peaks they stay high, because PX4's descent limit
  is 1.5 m/s.
- **Separation by route only:** drones are kept apart by their routes and transit heights.
  There is no active collision avoidance.
- **No battery:** battery is not modeled. PX4's default return-to-home height has not been
  checked against the highest peaks.
- **No QGroundControl alerts:** detections and targets go to ROS 2 topics only. QGroundControl
  does not show them yet.

---

## 7. Credits

- Missing-person mesh and textures: "Rescue Randy" by Open Robotics (Ian Chen, Cole Biesemeyer),
  from Gazebo Fuel: https://app.gazebosim.org/OpenRobotics/fuel/models/Rescue%20Randy.
  Check that page for its license before publishing this repository publicly.
- Drone, sensors and simulation integration: PX4-Autopilot (https://px4.io), BSD-3-Clause.
