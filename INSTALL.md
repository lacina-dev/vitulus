# Vitulus — installing the stack on a robot

How to get from an empty computer to a booting Vitulus robot. Everything here
is derived from the repositories themselves; per-package details live in each
package's `README.md`.

Target: Ubuntu 20.04, ROS Noetic, Python 3, Docker. The robot user is
`vitulus` (several nodes and units hard-code `/home/vitulus`, e.g. `navi_man`
mounts `/home/vitulus/.ros` into its containers — keep that user name); the
workspace is `~/catkin_ws`.

Chapters 0–7 give a booting robot. Chapters 8–11 are optional parts (on-board
agent, field kit, firmware, developer tooling), chapter 12 is the positive
check before the first drive, chapter 13 the known traps. A one-page tick
list: [`docs/NEW_ROBOT_CHECKLIST.md`](docs/NEW_ROBOT_CHECKLIST.md).

This document describes the state a new robot must be in. Secrets (tokens,
passwords, keys, names, addresses) are never written into a repository or
into this document; the text only says where each one lives and how to obtain
it.

## 0. Prerequisites

1. **Ubuntu 20.04 (amd64)**, user `vitulus` with `sudo`.
2. **ROS Noetic** from the ROS apt repository (the official "Ubuntu install of
   ROS Noetic" steps), then:
   ```bash
   sudo apt install ros-noetic-desktop-full python3-rosdep python3-catkin-tools \
        python3-pip git curl ntpdate libjsoncpp-dev
   sudo rosdep init && rosdep update
   sudo apt install ros-noetic-realsense2-camera ros-noetic-rosbridge-server ros-noetic-rosapi \
        ros-noetic-tf2-web-republisher ros-noetic-rosserial-python ros-noetic-imu-complementary-filter \
        ros-noetic-pointcloud-to-laserscan ros-noetic-rtabmap-odom ros-noetic-swri-transform-util \
        ros-noetic-mapviz ros-noetic-controller-manager ros-noetic-robot-state-publisher \
        ros-noetic-joint-state-publisher ros-noetic-xacro ros-noetic-topic-tools ros-noetic-pcl-ros \
        ros-noetic-libg2o python3-opencv python3-scipy python3-dbus
   ```
   The `package.xml` files do not declare everything, so `rosdep` alone is not
   enough; after cloning (§2) run it for the rest:
   `rosdep install --from-paths ~/catkin_ws/src --ignore-src -r -y`.
3. **Python modules** the nodes import and apt does not bring:
   ```bash
   pip3 install shapely pyproj pyserial flask docker moteus python-networkmanager adafruit-circuitpython-bno08x
   ```
   (known-good: shapely 2.0.1, pyproj 3.5.0, Flask 2.0.1, docker 7.1.0,
   moteus 0.3.54). Check:
   `python3 -c "import numpy, scipy, cv2, yaml, shapely, pyproj, serial, flask, docker, moteus, NetworkManager, dbus, PIL, matplotlib, adafruit_bno08x"`
   prints nothing.
4. **Docker Engine with the compose plugin** from Docker's apt repository
   (`docker-ce`, `docker-compose-plugin`). Check: `docker compose version`
   answers and, after §4 (groups), `docker ps` works without `sudo`.
5. **GitHub login before cloning.** `vitulus_dock` and `vitulus_local_costmap`
   (required) and `vitulus_agent`, the field kit and the mower firmware
   (optional) are private: ask the owner for an invitation first. Then
   ```bash
   sudo apt install gh          # from GitHub's apt repository
   gh auth login                # HTTPS, the invited account
   gh auth setup-git            # lets "git clone https://…" use that login
   ```
   Check: `gh auth status` is logged in and
   `git ls-remote https://github.com/lacina-dev/vitulus_dock.git HEAD` prints
   a commit (an anonymous clone fails with "repository not found"). The field
   kit's README clones over SSH (`git@github.com:`); either add an SSH key to
   the account or clone it over HTTPS as well.
6. **Only when the chapter is used:** the `claude` CLI for §8 / §11
   (Anthropic's native installer, `curl -fsSL https://claude.ai/install.sh | bash`,
   puts it into `~/.local/bin/claude`; check `claude --version`), PlatformIO
   for §10 (the official `get-platformio.py` installer; check
   `~/.platformio/penv/bin/pio --version`), Tailscale for remote access (§4).

## 1. Workspace layout

```
~/catkin_ws/
├── src/
│   ├── vitulus/            # ALL Vitulus packages, one git repo per package
│   │   ├── vitulus/        #   this repo: boot script, main launch, sensors
│   │   ├── vitulus_base/
│   │   ├── vitulus_navi/
│   │   └── ...
│   ├── vitulus_depth_obstacles/  # obstacle clouds from the D435 (required)
│   ├── rtabmap_docker/     # launch files mounted into the rtabmap container
│   ├── docker-mapproxy/    # aerial tile server (optional)
│   └── other/              # third-party ROS packages built from source
└── devel/                  # created by the build
```

`vitulus.startup` and `nodes/main` locate the workspace from their own
location, launch files use `$(find <pkg>)`. Keep the `src/vitulus/<pkg>`
nesting, some packages still look for their sibling packages next to
themselves.

## 2. Repositories

All under `https://github.com/lacina-dev/<name>.git`.

Required — included (directly or indirectly) by `launch/vitulus_start.launch`,
clone into `src/vitulus/`:

| Repo | Role |
|---|---|
| `vitulus` | boot script, `main` node, main launch, RPLidar / D435 / Nextion launch |
| `vitulus_msgs` | shared messages |
| `vitulus_description` | URDF / robot_state_publisher |
| `vitulus_base` | motor base control |
| `vitulus_ups` | power supply / battery |
| `vitulus_imu` | IMU |
| `vitulus_gnss` | RTK GNSS + heading |
| `vitulus_mower` | mower unit |
| `vitulus_ds4` | DualShock 4 teleop |
| `vitulus_nmcli` | WiFi management |
| `vitulus_navi` | navigation manager, localization, move_base_flex config |
| `vitulus_local_costmap` (private) | local costmap (included by `vitulus_navi/launch/navi_man.launch`) |
| `vitulus_planner` | zones, coverage paths, mowing programs, calendar |
| `vitulus_mapping` | Mapping v3 (`mapping_manager` is started by `vitulus_ui.launch`) |
| `vitulus_dock` (private) | docking / undocking |
| `vitulus_safety` | safety node |
| `vitulus_rosbag` | rosbag recorder |
| `vitulus_ui` | web UI on `:7779` |
| `weather_alert` | rain radar alert |

Clone into `src/`: `rtabmap_docker` (required), `vitulus_depth_obstacles`
(required), `docker-mapproxy` (optional).

`vitulus_depth_obstacles` is required because `vitulus_navi/config/navi_manager.yaml`
ships `obstacles_engine: depth_obstacles`: `navi_man` then starts
`roslaunch vitulus_depth_obstacles depth_obstacles.launch` instead of the
`RTABMAP_OBSTACLES` container; without the package the navigation has no
obstacle clouds. Check after the first start: `rosnode info /depth_obstacles`
answers and its PID stays the same over a minute.

Optional, not needed to boot: `vitulus_teleop`, and three repositories the
field kit does not deploy (§9):

| Repo | Clone as | What it is | Set up in |
|---|---|---|---|
| `vitulus_agent` (private) | `src/vitulus/vitulus_agent` | on-board assistant, package name `vitulus_claude` | §8 |
| `precip-mosaic` | `src/vitulus/vitulus_precip` | precipitation mosaic service (Docker) | §3 |
| `vitulus_traversability` | `src/vitulus/vitulus_traversability` | local terrain map from the D435 clouds, shown in the UI only | below |

`vitulus_traversability` only has to be in the workspace and built:
`vitulus.startup` finds it with `rospack find` and exports
`VITULUS_TRAVERSABILITY_NODE=true`, which switches on
`launch/includes/traversability.launch` (at `nice 19`; the navigation does not
use it). Off by hand: `VITULUS_TRAVERSABILITY_NODE=false` in `~/.bashrc` (§4).
Check: `rosnode list | grep traversability` shows the node.

Do not clone `vitulus_webui` (the old UI, replaced by `vitulus_ui`; never run
both).

```bash
mkdir -p ~/catkin_ws/src/vitulus && cd ~/catkin_ws/src/vitulus
for r in vitulus vitulus_msgs vitulus_description vitulus_base vitulus_ups \
         vitulus_imu vitulus_gnss vitulus_mower vitulus_ds4 vitulus_nmcli \
         vitulus_navi vitulus_local_costmap vitulus_planner vitulus_mapping \
         vitulus_dock vitulus_safety vitulus_rosbag vitulus_ui weather_alert; do
  git clone https://github.com/lacina-dev/$r.git
done
# optional, not deployed by the field kit:
git clone https://github.com/lacina-dev/vitulus_traversability.git
git clone https://github.com/lacina-dev/precip-mosaic.git vitulus_precip
git clone https://github.com/lacina-dev/vitulus_agent.git          # private
cd ~/catkin_ws/src
git clone https://github.com/lacina-dev/rtabmap_docker.git
git clone https://github.com/lacina-dev/vitulus_depth_obstacles.git
git clone https://github.com/lacina-dev/docker-mapproxy.git        # optional
```

### Third-party packages built from source (`src/other/`)

| Directory | Source | Branch | Needed for |
|---|---|---|---|
| `ros-system-monitor` | `lacina-dev/ros-system-monitor` | `vitulus` | `system_monitor` |
| `interactive_marker_proxy` | `lacina-dev/interactive_marker_proxy` | `develop` | UI markers |
| `navigation` | `ros-planning/navigation` | `noetic-devel` | `costmap_2d`, `navfn`, `global_planner`, `move_base`, `map_server` |
| `move_base_flex` | `naturerobots/move_base_flex` | `master` | navigation server |
| `robot_localization` | `cra-ros-pkg/robot_localization` | `noetic-devel` | EKF |
| `teb_local_planner` | `rst-tu-dortmund/teb_local_planner` | `noetic-devel` | local planner (uses apt `ros-noetic-libg2o`) |
| `costmap_converter` | `rst-tu-dortmund/costmap_converter` | `master` | teb |
| `grid_map` | `ANYbotics/grid_map` | `master` (1.7.9) | `grid_map_msgs` for `vitulus_mapping`; core / ros for `vitulus_traversability` |
| `rplidar_ros` | `Slamtec/rplidar_ros` | `master` | lidar |
| `ublox` | `KumarRobotics/ublox` | `master` | GNSS |
| `ds4_driver` | `naoki-mizuno/ds4_driver` | `noetic-devel` | DualShock 4 |
| `ds4drv` | `naoki-mizuno/ds4drv` | `devel` | Python library for `ds4_driver`; not a catkin package |
| `laser_filters` | `ros-perception/laser_filters` | `noetic-devel` | scan filter |
| `web_video_server` | `RobotWebTools/web_video_server` | pull request 130 (commit `e96e56b`) | camera stream in the UI |

```bash
mkdir -p ~/catkin_ws/src/other && cd ~/catkin_ws/src/other
c() { git clone --branch "$2" "https://github.com/$1.git"; }
c lacina-dev/ros-system-monitor vitulus;      c lacina-dev/interactive_marker_proxy develop
c ros-planning/navigation noetic-devel;       c naturerobots/move_base_flex master
c cra-ros-pkg/robot_localization noetic-devel; c rst-tu-dortmund/teb_local_planner noetic-devel
c rst-tu-dortmund/costmap_converter master;   c ANYbotics/grid_map master
c Slamtec/rplidar_ros master;                 c KumarRobotics/ublox master
c naoki-mizuno/ds4_driver noetic-devel;       c naoki-mizuno/ds4drv devel
c ros-perception/laser_filters noetic-devel
git clone https://github.com/RobotWebTools/web_video_server.git && git -C web_video_server fetch origin pull/130/head:pr/130 && git -C web_video_server checkout pr/130   # e96e56b
pip3 install --user ./ds4drv      # see ds4_driver/README.md (udev rules too)
```

`--branch` takes the current tip; if a tip does not build, check out the
known-good commit: `move_base_flex` `367269c3c2427dfee29426e5f9f136731fb3b049`,
`robot_localization` `9ef26a57fafd9b97c135a7e07f52cf0168e94178`, `navigation`
`9ad64419`, `grid_map` `0b889ca`, `teb_local_planner` `3b91dcc`,
`costmap_converter` `e8c1d2c`, `rplidar_ros` `bf4fbfb`, `ublox` `4f107f3`,
`ds4_driver` `fe6d779`, `ds4drv` `da19463`, `laser_filters` `53e0a00`,
`ros-system-monitor` `b6c998c`, `interactive_marker_proxy` `455fb75`. No local
patches are required.

`web_video_server` runs at `e96e56b` (0.2.2 plus the output-resize fix of pull
request 130) — the only state built and run on a Vitulus robot. Upstream
merged the same fix into `ros1` (`047857a`) together with six later commits;
`ros1` is untested here, use it only after checking the camera stream in the
UI.

## 3. Docker pieces

**rtabmap** (required). `vitulus_navi/nodes/navi_man` starts containers from the
image `vitulus/rtabmap_ros:noetic-latest` (host network, the `rtabmap_docker`
directory mounted at `/rtabmap_docker`, `~/.ros` at `/tmp/.ros`). Build the
image once:

```bash
cd ~/catkin_ws/src/rtabmap_docker && docker compose build
```

The `RTABMAP_*` containers are created and removed by `navi_man` — never start
them by hand.

**MapProxy** (optional). Serves the aerial tiles for the UI's aerial layer on
`:8082`; without it only that layer stays empty. Its tile cache is trimmed by a
weekly user timer from the same repo.

```bash
cd ~/catkin_ws/src/docker-mapproxy
docker compose -f docker-compose-build.yml up -d     # restart: always
mkdir -p ~/.config/systemd/user
ln -s ~/catkin_ws/src/docker-mapproxy/scripts/systemd/mapproxy-cache-cleanup.{service,timer} ~/.config/systemd/user/
systemctl --user daemon-reload && systemctl --user enable --now mapproxy-cache-cleanup.timer
```

**Precipitation mosaic** (optional; `src/vitulus/vitulus_precip`). A container
that serves the radar time line for the UI layer *Precipitation (mosaic)* on
`:8097`, with its own Python 3.11 (nothing touches ROS Noetic's Python). Build
and start (~3 min, image ≈ 250 MB, want ≥ 1 GB free on `/`):

```bash
cd ~/catkin_ws/src/vitulus/vitulus_precip
cp config/precip.local.example.yaml config/precip.local.yaml   # git-ignored
chmod 600 config/precip.local.yaml
nano config/precip.local.yaml                                  # the garden's lat/lon (same point as weather_alert)
docker compose -f deploy/docker-compose.yml up -d --build
```

* It starts by itself after a boot: the compose file sets
  `restart: unless-stopped`, the Docker daemon brings it back (no systemd unit).
* Limits from the compose file: `mem_limit: 400m` (measured use 160–330 MiB),
  half a CPU, `nice 10`, cache volume capped at 200 MB, logs 2 × 5 MB.
* Check: `docker ps --filter name=vitulus-precip --format '{{.Names}} {{.Status}}'`
  → `Up … (healthy)` after 1–2 min; `curl -s localhost:8097/health` lists every
  source as `ok`; `docker inspect -f '{{.HostConfig.RestartPolicy.Name}} {{.HostConfig.Memory}}' vitulus-precip`
  → `unless-stopped 419430400`.
* The MapProxy layer (recreating the MapProxy container once so it sees
  `host.docker.internal`): `vitulus_precip/README.md`, *Deployment on the robot*.

## 4. Host configuration

Machine-level files live in no package; `setup/` in this repo holds examples.

* **Device symlinks.** The nodes open `/dev/rplidar`, `/dev/nextion`,
  `/dev/ups`, `/dev/mower`, `/dev/gnss`, `/dev/gnss_heading` and `/dev/imu`.
  The only source of the rules is `setup/99-vitulus.rules.example`: most rules
  are bound to the physical USB port, so the `KERNELS==` values must be changed
  to your own USB topology (the file explains how to find them).
  ```bash
  sudo cp setup/99-vitulus.rules.example /etc/udev/rules.d/99-vitulus.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```
  It covers: the symlinks `rplidar` (by USB id), `mower`, `ups`, `nextion`,
  `gnss`, `imu` (`ttyUSB*` by port), `gnss_heading` and `fdcanusb` (`ttyACM*`
  by port), `openups`; open permissions for ST devices (`0483:*`); and `ID_MM_DEVICE_IGNORE` so
  ModemManager does not probe the serial devices. Check:
  `ls -l /dev/rplidar /dev/nextion /dev/ups /dev/mower /dev/gnss /dev/gnss_heading /dev/imu`
  shows seven symlinks, each to a different device. Two devices swapped = two
  wrong `KERNELS==` values.
* **User groups.** `dialout` (serial ports), `docker`, `plugdev`, and `adm`
  (the agent reads the system journal, §8):
  `sudo usermod -aG dialout,docker,plugdev,adm $USER`, then log in again.
  Check: `id -nG` lists them.
* **ROS network.** `vitulus.startup` exports `ROS_IP` / `ROS_MASTER_URI` with
  the address `10.254.254.254`, a netplan bridge `br0` over the wired port:
  `setup/02-vitulus-network.yaml.example` (adapt the interface name, install
  as `/etc/netplan/02-vitulus-network.yaml`, `sudo netplan try`).
  Alternatively set `VITULUS_ROS_IP` to an address the host does have (next
  item). Check: `ip -br addr show br0` shows the address (the bridge is
  `optional`, so it exists with no cable).
* **`~/.bashrc`.** `vitulus.service` runs
  `bash --login -c 'source ~/.bashrc; …/vitulus.startup'`. That shell is not
  interactive, so the stock Ubuntu `~/.bashrc` returns at its
  `case $- in *i*) ;; *) return;; esac` lines: **only what stands above that
  check reaches the robot stack**.
  * Above the check — the robot's switches, one `export VITULUS_…=…` per line
    (`VITULUS_ROS_IP`, `VITULUS_WS`, `VITULUS_TIMESYNC_KEEP`,
    `VITULUS_TRAVERSABILITY_NODE`, and the switches the package READMEs
    describe). A new robot needs none.
  * At the end — the interactive ROS environment, the same values the stack
    uses, so `rostopic` etc. talk to the robot's master:
    ```bash
    export ROS_IP=10.254.254.254                      # the bridge address, or your VITULUS_ROS_IP
    export ROS_MASTER_URI=http://${ROS_IP}:11311
    source /opt/ros/noetic/setup.bash
    source ~/catkin_ws/devel/setup.bash               # devel, never install/ (§13)
    alias cm='cd ~/catkin_ws && catkin build'
    ```
  * **No `LD_PRELOAD`** anywhere (`~/.bashrc`, `~/.profile`,
    `/etc/environment`, `/etc/ld.so.preload`): a preloaded jemalloc in the
    profile crashed headless Chrome in the UI tests, and the stack uses no
    preloaded allocator.

  Check: `env -i HOME=$HOME USER=$USER bash --login -c 'source ~/.bashrc; env' | grep -E '^(VITULUS_|LD_PRELOAD)'`
  prints exactly the switches above the check and no `LD_PRELOAD` (`env -i`
  matters: a terminal or long-running session opened before a change keeps the
  old environment and hands it to everything started from it); in a new
  terminal `rosnode list` answers.
* **Ubuntu crash reporting off.** `whoopsie` uploads crash reports and
  `apport` writes memory dumps of crashed processes (ROS nodes included) — a
  privacy leak and CPU/disk load at the worst moment. Ubuntu enables both by
  default.
  ```bash
  sudo systemctl disable --now whoopsie apport
  sudo systemctl stop apport   # disable --now leaves the SysV apport service active until a reboot
  sudo rm -f /var/crash/*      # old dumps
  ```
  Check: `systemctl is-enabled whoopsie apport` → `disabled` ×2,
  `systemctl is-active whoopsie apport` → `inactive` ×2, and
  `cat /proc/sys/kernel/core_pattern` → `core` — it must not start with
  `|/usr/share/apport/apport`; if it still does, reboot and check again
  (`enabled=0` in `/etc/default/apport` keeps it off for good).
* **Time zone, swap, lingering.** `sudo timedatectl set-timezone <Region/City>`
  (the calendar of mowing programs and the night window use local time; check
  `timedatectl | grep "Time zone"`). Keep a swap file (2 GB with 16 GB RAM;
  `swapon --show`): the stack plus the agent and a build do not fit into RAM
  alone. `sudo loginctl enable-linger $USER` lets every `systemctl --user`
  unit of this document start at boot without a login (check
  `loginctl show-user $USER -p Linger` → `Linger=yes`).
* **Nothing else.** No kernel (`sysctl`) settings and no user crontab are
  needed; everything periodic is a systemd timer (§6).
* **sudoers.** Two passwordless rules for the robot user:
  * time sync — `sudo visudo -f /etc/sudoers.d/vitulus-time`:
    ```
    vitulus ALL=(ALL) NOPASSWD: /usr/bin/systemctl start systemd-timesyncd.service, /usr/bin/systemctl status systemd-timesyncd.service, /usr/bin/systemctl stop systemd-timesyncd.service, /usr/sbin/ntpdate
    ```
    At every start `vitulus.startup` runs, with internet (`ping 8.8.8.8`), up
    to five `ntpdate pool.ntp.org` attempts and then starts
    `systemd-timesyncd.service`, which **stays on**: without it the clock
    drifts ~0.09 s/h against GPS time and the localization pairs GPS samples
    badly. `VITULUS_TIMESYNC_KEEP=0` (above the check in `~/.bashrc`) stops
    the service after the initial sync instead. Without the sudoers line both
    `sudo` calls fail and the clock is never set. Check:
    `timedatectl | grep -E "synchronized|NTP service"` → `yes` / `active`.
  * UI "Restart robot" button (optional) — install
    `vitulus_ui/setup/vitulus-ui-sudoers` as `/etc/sudoers.d/vitulus-ui`, see
    `vitulus_ui/README.md`.
* **weather_alert location.** The build copies
  `weather_alert/config/config.local.example.yaml` to the git-ignored
  `config/config.local.yaml`. Set your coordinates there; until you do, the
  node runs with the placeholder location (Prague) and says so in its log.
* **NetworkManager** (optional): `scripts/org.freedesktop.NetworkManager.pkla`
  (install into `/etc/polkit-1/localauthority/50-local.d/`) lets the user
  manage WiFi without a polkit prompt. The WiFi node never reads or publishes
  the WiFi password — check
  `rostopic echo -n 1 /nmcli/conn_detail | grep psk` → `psk: ''`.
* **Dock permit owners (required before the first undock).** The robot leaves
  the dock only inside a permit window that a **human** opens (UI lock button
  or chat); agent and program names are always refused. Who counts as an owner
  is one local file, never in git:
  ```bash
  mkdir -p ~/.vitulus
  printf '%s\n' '<owner name>' > ~/.vitulus/dock_permit_owners   # one name per line, '#' comments allowed
  ```
  The name is the one the UI / chat sends as `by` (compared lower-case).
  Without a readable, non-empty file `dock_permit` **refuses every unlock**
  (result code `owners_missing`, or `owners` when the file cannot be read);
  locking always works for anybody. Check: `rostopic echo -n 1 /dock_permit/state`
  answers, an unlock from the UI is accepted and `/dock_permit/result` shows
  no `owners_missing`. Details: `vitulus_dock/README.md`,
  `vitulus_dock/config/dock_permit.yaml`.

### Network and remote access

* **Home network.** A robot behind a home router with NAT sees every client
  under the router's address, so its logs cannot tell devices apart. This is a
  property of the network; do not build access rules on client addresses.
* **Tailscale** for access from outside: install from the vendor's apt
  repository, `sudo tailscale up` and authorize in the browser with the
  owner's account. Check: `systemctl is-enabled tailscaled` → `enabled`,
  `tailscale status` lists the robot; the UI is `http://<tailscale-name>:7779/`.
* **SSH keys.** `~/.ssh/authorized_keys` (mode 600), one line per key.
  Recommended on a new robot: a key that only fetches data (logs, bags, a
  health check) should not get a shell — restrict it on its line:
  ```
  restrict,command="/home/vitulus/bin/<the one allowed script>" ssh-ed25519 AAAA… <purpose>
  ```
  Check: `awk '{print ($1 ~ /^(restrict|command=)/ ? "restricted" : "FULL SHELL"), $NF}' ~/.ssh/authorized_keys`
  — know what every `FULL SHELL` line is for.

## 5. Build

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin config --no-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON   # once per workspace
catkin build
source devel/setup.bash
```

* **`catkin build` only** (catkin_tools). `catkin_make` builds into the same
  `build/` and `devel/` with a different layout and breaks the workspace (the
  cure is deleting `build/ devel/ logs/` and a full rebuild). Where a package
  README or the error text of `vitulus.startup` says `catkin_make`, read
  `catkin build`.
* **No install space** (`--no-install`): the stack runs from `devel/`. With an
  install space every build leaves copies of the nodes in `install/`, and a
  build from a shell that sourced `install/setup.bash` bakes `install` in
  front of `devel` — after the next restart the stack runs stale copies.
  Check after every build:
  `grep -n "CMAKE_PREFIX_PATH = r" devel/_setup_util.py` → `devel;/opt/ros/noetic`,
  and `catkin config | grep "Install Space"` → `[unused]`.
* **Release.** The profile (`.catkin_tools/profiles/default/config.yaml`, not
  in git) carries `-DCMAKE_BUILD_TYPE=Release`. Without it every package that
  does not set a build type itself (`costmap_2d`, `mbf_*`, `navfn`,
  `vitulus_local_costmap`, `vitulus_base`, … about 100 packages) is compiled
  without `-O` and the navigation takes about five times more CPU. Every
  `catkin build <pkg>`, also the field kit's automatic builds, uses the active
  profile. Check: `catkin config | grep "Additional CMake Args"` shows both
  `-D` arguments and
  `grep -c -- " -O" build/costmap_2d/compile_commands.json` is not 0
  (`costmap_2d` comes from `src/other/navigation`, §2).
* `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` writes
  `build/<pkg>/compile_commands.json` (used by the check above and by clangd);
  it costs nothing at run time.
* A full build takes ~30 min (`catkin build -p1 -j2` when memory is short); a
  change of the profile's CMake args needs `catkin build --force-cmake`.
* Debugging a crash: `catkin build <pkg> --no-deps --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`
  (same speed, ~13x larger binaries; the next plain `catkin build <pkg>` is
  Release again). `Release` defines `NDEBUG`, like the ROS binary packages.

## 6. systemd services

```bash
chmod +x ~/catkin_ws/src/vitulus/vitulus/launch/vitulus.startup
sudo cp ~/catkin_ws/src/vitulus/vitulus/scripts/vitulus.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now vitulus.service
journalctl -u vitulus.service -f
```

The unit is installed **directly** from this repo (a copy, or a symlink to
`…/vitulus/scripts/vitulus.service`), never through another package. Check:
`readlink -f /etc/systemd/system/vitulus.service` prints either that path or
`/etc/systemd/system/vitulus.service` itself.

Boot chain: `vitulus.service` → `launch/vitulus.startup` (time sync, ROS
environment, workspace detection; `VITULUS_WS` overrides the detection) →
`nodes/main` (starts `roscore` if needed) → `launch/vitulus_start.launch`.
The service restarts the whole chain 5 s after it exits; publishing `fuse` on
`/restart_core` makes `main` exit. Then open `http://<robot>:7779/`.

Reading the log later: always bounded, `journalctl -u vitulus.service --since -10min`
(the journal grows to several GB). Node logs are in `~/.ros/log/latest/`.

### ROS log cleanup (user timer)

ROS writes every run into `~/.ros/log` and never deletes anything.
`scripts/vitulus_rosclean` keeps it bounded: it never touches `latest`, the
current run, open files or anything younger than 60 min, deletes entries older
than 7 days and then the oldest ones above 500 MB. Daily and 10 min after boot:

```bash
mkdir -p ~/.config/systemd/user
cp ~/catkin_ws/src/vitulus/vitulus/scripts/vitulus-rosclean.{service,timer} ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now vitulus-rosclean.timer
~/catkin_ws/src/vitulus/vitulus/scripts/vitulus_rosclean --dry-run   # preview
```

### Lidar object detection (user service)

`/safety/lidar_objects` is published by a node that runs **outside** the stack
launch, as a user service, so it survives stack restarts (it exits with code 3
when the ROS master goes away and systemd starts it again):

```bash
cp ~/catkin_ws/src/vitulus/vitulus_safety/deploy/vitulus-lidar-objects.service ~/.config/systemd/user/
systemctl --user daemon-reload && systemctl --user enable --now vitulus-lidar-objects
```

Check: `systemctl --user is-active vitulus-lidar-objects` → `active`,
`rostopic info /safety/lidar_objects` lists one publisher.

### All units and where they come from

| Unit | Source | Chapter |
|---|---|---|
| `vitulus.service` (system) | `vitulus/scripts/` | above |
| `vitulus-rosclean.timer` | `vitulus/scripts/` | above |
| `vitulus-lidar-objects.service` | `vitulus_safety/deploy/` | above |
| `mapproxy-cache-cleanup.timer` | `docker-mapproxy/scripts/systemd/` | §3 |
| `vitulus-agent-{core,web,v2,v2-bridge,supervisor}.service` | `vitulus_agent/deploy/` | §8 |
| `vitulus-field-watch.timer`, `vitulus-field-signal.service`, `vitulus-dash.service` | written by `vitulus-field-watch install` | §9 |
| `reditel-nastenka.service`, `reditel-rozbor.timer` | `claude-skills` | §11 |

Check: `systemctl is-enabled vitulus.service docker.service systemd-timesyncd.service`
→ `enabled` ×3; `systemctl --user list-timers --all` gives every timer a NEXT time.

## 7. First boot on an empty robot

All robot data lives outside the source tree and is created as you go:

| Path | Content |
|---|---|
| `~/.ros/dock_detector_map.pickle` | lidar map of the dock surroundings |
| `~/.vitulus/dock/` | `dock_program.pkl`, `undock_program.pkl` |
| `~/.vitulus/mapping_v3/<site>/` | site: datum, rasters, waypoints, zones, programs |
| `~/.vitulus/saves/`, `~/.vitulus/running/` | navigation / planner working data |

On a new robot none of this exists: there is no active map, no dock map and no
dock / undock program. Create them in this order (teleop with the on-screen
controls, keyboard or a DualShock 4):

1. **Dock map.** Put the robot in the dock. UI → *Dock* tab → *Dock detector
   map* → **New map**, drive around the dock area (a scan is added every
   0.3 m travelled), **Save map**.
2. **Dock and undock programs.** *Dock* tab → *Dock program* / *Undock
   program*: tick the stages to use (map / lidar map / intensity) and **Save**
   each. **Execute** runs the program — the robot drives.
3. **First site map.** *Map* tab → enter a site name → **Start recording**,
   drive the garden (insertion is gated on RTK FIXED in the default *RTK*
   mode), **Save & finish**. The saved map becomes the active map.
4. **Dock / undock points.** *Dock* tab → *Dock / undock points (current
   map)* → **Update** with the robot standing at the respective place.
5. **Zones and programs.** Draw zones in the map editor, then build mowing
   programs from them in the *Programs* drawer.

Progress and errors of all of the above are published on `/nextion/log_info`
and shown in the UI status bar (and on the Nextion display), e.g.
`No DOCK map available` when docking is requested without any map.

## 8. On-board agent (`vitulus_agent`, optional)

The assistant behind the chat in the UI. It is not needed to boot or mow and no
robot function may depend on it. Catkin package name `vitulus_claude`; the
field kit does not deploy it (§9). Operator manual:
`vitulus_agent/docs/AGENT_OPERATION.md`.

### Services

Four `systemd --user` services make the agent; they share code, so they must
run the same version:

| Unit | What | Starts |
|---|---|---|
| `vitulus-agent-core` | deterministic loop, queue, missions | `~/.vitulus/agent/run_core_exec.sh` |
| `vitulus-agent-web` | chat + bridge API on `:8088` (proxied by the UI) | `~/.vitulus/agent/run_web_exec.sh` |
| `vitulus-agent-v2` | v2 core: findings, summaries, self-repair of its own code | `~/.vitulus/agent/v2/run_node.sh` |
| `vitulus-agent-v2-bridge` | chat bridge between the panel and v2 | `nodes/agent_v2_bridge` directly |

Beside them `vitulus-agent-supervisor` (independent drive watchdog; a safety
layer, the agent never restarts it). The unit sources are in the repo,
`vitulus_agent/deploy/`; `~/.config/systemd/user/` holds plain copies (check
with `cmp`). `deploy/run_supervisor.sh` hard-codes the bridge address for
`ROS_IP` — change it when you use another `VITULUS_ROS_IP`.

```bash
A=~/catkin_ws/src/vitulus/vitulus_agent
cd ~/catkin_ws && catkin build vitulus_claude
mkdir -p ~/.vitulus/agent/v2 ~/.config/systemd/user

# launchers: the v2 one is in the repo; the other two are not, create them
# (systemd does not read ~/.bashrc, so each sources ROS itself)
cp $A/deploy/run_node.sh ~/.vitulus/agent/v2/run_node.sh
cat > ~/.vitulus/agent/run_core_exec.sh <<'SH'
#!/bin/bash
source /home/vitulus/catkin_ws/devel/setup.bash
cd /home/vitulus/catkin_ws/src/vitulus/vitulus_agent
exec python3 nodes/agent_core
SH
cat > ~/.vitulus/agent/run_web_exec.sh <<'SH'
#!/bin/bash
source /home/vitulus/catkin_ws/devel/setup.bash
cd /home/vitulus/catkin_ws/src/vitulus/vitulus_agent
exec python3 tools/agent_web --port 8088
SH
chmod +x ~/.vitulus/agent/run_core_exec.sh ~/.vitulus/agent/run_web_exec.sh ~/.vitulus/agent/v2/run_node.sh

cp $A/deploy/vitulus-agent-{core,web,v2,v2-bridge,supervisor}.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now vitulus-agent-core vitulus-agent-web vitulus-agent-v2 \
                              vitulus-agent-v2-bridge vitulus-agent-supervisor
```

`run_node.sh` starts v2 with `--dry` (it reports, it does not act on the
robot); going live is the owner's decision, described at the top of that file.

Check: `systemctl --user is-active vitulus-agent-core vitulus-agent-web vitulus-agent-v2 vitulus-agent-v2-bridge`
→ `active` ×4; `curl -s http://localhost:8088/api/health` → `{"ok": true, …}`
(proves only the web server); end to end:
`python3 $A/tools/agent_task "ahoj"` → a reply within 180 s.

### After every code change: restart all four

A deploy that restarts one service leaves the others on the old code (the web
then serves an old API and the UI gets HTTP 400). After `git pull` in
`vitulus_agent`, with the robot docked and no mission running:

```bash
systemctl --user restart vitulus-agent-web vitulus-agent-v2 vitulus-agent-v2-bridge
systemctl --user restart vitulus-agent-core          # the core last
```

The core also checks once a minute which commit each unit runs and restarts a
stale one itself when the robot is docked and idle. Check: in
`~/.vitulus/agent/code_state.json` `procs.<unit>.sha` is the same for every
unit and equal to `git -C $A rev-parse HEAD`.

### Model login

The agent calls the `claude` CLI on the Claude **subscription**. An
interactive `/login` expires and a service cannot renew it, so the services
use a one-year token:

1. In a **real terminal** on the robot (SSH with a TTY is fine; started from a
   script or another tool it hangs) run `claude setup-token`.
2. Open the printed link in a browser and authorize with **the account the
   agent is to run under**. The command prints the token.
3. Store it as **one line**, no quotes, no line break inside the value
   (terminals wrap long pastes):
   ```bash
   touch ~/.vitulus/agent/secrets.env && chmod 600 ~/.vitulus/agent/secrets.env
   nano ~/.vitulus/agent/secrets.env        # CLAUDE_CODE_OAUTH_TOKEN=<token>
   ```
   The file lives outside every repository. Keys of the fallback providers
   (below) go into the same file as further `KEY=value` lines.
4. All four services must read it with
   `EnvironmentFile=-/home/vitulus/.vitulus/agent/secrets.env` (the leading
   `-` keeps a missing file from blocking the start). The repo units of `-v2`
   and `-v2-bridge` carry the line; for `-core` and `-web` add a drop-in:
   ```bash
   for u in vitulus-agent-core vitulus-agent-web; do
     mkdir -p ~/.config/systemd/user/$u.service.d
     printf '[Service]\nEnvironmentFile=-%%h/.vitulus/agent/secrets.env\n' > ~/.config/systemd/user/$u.service.d/secrets.conf
   done
   systemctl --user daemon-reload      # then restart all four (above)
   ```
   A changed `secrets.env` or unit reaches a service only after
   `daemon-reload` and a restart.

Check (none of these prints the token):

```bash
stat -c '%a' ~/.vitulus/agent/secrets.env                          # 600
grep -c '^CLAUDE_CODE_OAUTH_TOKEN=' ~/.vitulus/agent/secrets.env   # 1
grep -vc -e '^#' -e '^$' -e '^[A-Z_]*=' ~/.vitulus/agent/secrets.env   # 0 = no wrapped line
for u in vitulus-agent-core vitulus-agent-web vitulus-agent-v2 vitulus-agent-v2-bridge; do
  p=$(systemctl --user show -p MainPID --value $u)
  echo "$u $(tr '\0' '\n' < /proc/$p/environ | grep -c '^CLAUDE_CODE_OAUTH_TOKEN=')"   # 1 for each
done
( set -a; . ~/.vitulus/agent/secrets.env; claude auth status | grep authMethod )   # "authMethod": "oauth_token"
```

### Models: the owner's policy and the configuration

* **Order of model providers:** Claude (subscription, the one-year token
  above) → ChatGPT (`codex`) → DeepSeek (its key in `secrets.env`). The two
  fallbacks are for work **off the robot** only, when the Claude limit is used
  up. **No paid Claude credits.**
* The judge of a piece of work must not be the model that did it; it may be
  another Claude model (opus ↔ sonnet).
* Hermes (the `hermes` CLI on ChatGPT) is the previous, optional executor of
  background jobs and is being replaced by `claude -p`; its installation is
  not part of this document. `backends.yaml` still names it as the chat
  engine; when it is absent or slow the core answers from its Claude fallback.
* `vitulus_agent/config/agent/backends.yaml` — who answers the chat
  (`brain`), the back ends (`claude_code`, `codex`) and the role → model table
  (triage, chat, planner, reflector, checker …). In git.
* `~/.vitulus/agent/v2/models.yaml` — local overrides of the v2 model
  assignment; `{}` = none. `models.yaml.last_good` is the last working
  overlay, saved by the node before each write.
* `vitulus_agent/config/agent/http_guard.yaml` — not about models: the
  freshness check of action requests from the web (a one-time ticket valid
  5 s, so a request delayed by a network drop is not executed late).
* `~/.vitulus/agent/execution.yaml` — what the agent may execute (the owner's
  consent, `enabled` / `dry_run`). `~/.vitulus/agent/` also holds the
  databases and journals; none of it is in git, back it up with the robot.

## 9. Field kit (`vitulus-field`)

The support kit that delivers releases to robots: a watcher pulls the hub,
deploys the released package versions when the robot is idle (pull,
`catkin build` with the active profile, restart) and proposes the next step
of the fix loop. It is installed **next to** the workspace, never inside it:

```bash
git clone https://github.com/lacina-dev/vitulus-field.git ~/vitulus-field   # private; login as in §0
~/vitulus-field/tools/vitulus-doctor --init     # writes ~/.config/vitulus-support/config.yaml; edit handle, robot, language, allowed
~/vitulus-field/tools/vitulus-doctor --select-workspace     # a human confirms the workspace
~/vitulus-field/tools/vitulus-field-watch install --dry-run # shows the units
~/vitulus-field/tools/vitulus-field-watch install --with-dash --access lan
```

The kit needs a `gh` login with a fine-grained token limited to that repo
(Issues RW, Contents RW, Metadata R). `install` writes the user units itself:
`vitulus-field-watch.service` + `.timer`, `vitulus-field-signal.service` (wake
pings), `vitulus-field-onfailure@.service` and, with `--with-dash`,
`vitulus-dash.service` (dashboard on `:8790`, HTTPS). The timer fires 2 min
after boot and then every 10 min; while the wake-ping signal is healthy the
kit stretches it to 30 min with a drop-in (`vitulus-field-watch.timer.d/`),
because the pings wake the watcher at once. The kit updates itself. Full
procedure incl. the handshake: `~/vitulus-field/README.md` and
`docs/FIELD_WATCH.md`.

Check: `~/vitulus-field/tools/vitulus-field-watch doctor` is clean,
`systemctl --user list-timers vitulus-field-watch.timer` shows a NEXT time,
`systemctl --user is-active vitulus-field-signal` → `active`.

### What the kit does NOT deploy

`~/vitulus-field/releases/manifest.yaml` lists the deployed repositories.
`vitulus_agent`, `vitulus_precip` and `vitulus_traversability` are named under
`excluded:`, and `vitulus_depth_obstacles` is not in the manifest at all. A
merge into their default branch changes nothing on a robot until somebody does
this by hand (robot docked, not mowing):

| Repo | After `git -C <repo> pull --ff-only` |
|---|---|
| `vitulus_agent` | restart all four agent services, core last (§8) |
| `vitulus_precip` | `docker compose -f deploy/docker-compose.yml up -d --build` (§3) |
| `vitulus_traversability` | `catkin build vitulus_traversability`, then `sudo systemctl restart vitulus.service` |
| `vitulus_depth_obstacles` | `catkin build vitulus_depth_obstacles`, then restart the stack (not `navi_man` alone, §13) |

Check for a C++ package that the running node uses the new library:
`grep <pkg> /proc/<pid>/maps` shows the file from `devel/` and the node started
after the build.

## 10. Mower board firmware

The mower unit (`/dev/mower`, rosserial) is an STM32F411 "BlackPill" with the
firmware from the private repo `lacina-dev/STMmowerROSControll` (built with
PlatformIO, §0).

* **Version that must be on the board:** `4.1.0-isrfix` (constant
  `FW_VERSION_BASE` in `src/main.cpp`, branch `main`). Older firmware freezes
  when its millisecond clock wraps and does not stop the blade when the link
  to ROS is lost.
* **Flashing** needs hands on the board and an **ST-Link** on the SWD pins
  (SWDIO / SWCLK / GND / 3V3) — the only working way; it cannot be done through
  the normal serial link, so not remotely. USB DFU needs a bootloader change
  and is not supported yet. The procedure is in the firmware repo's
  `README.md`; never flash its `wraptest` environment on a robot (bench only).
* Do not use `set_power` as a "reset": on a board that is on it resets the MCU.
* Check: after every (re)connection the firmware logs one line to `/rosout`:
  `rostopic echo /rosout | grep FW_VERSION` →
  `FW_VERSION 4.1.0-isrfix git=<sha> reset=<cause>`; `rostopic hz /mower/status`
  → 2 Hz. A reset cause `WATCHDOG` is a fault to report.

Firmware of the other boards is not covered by this document.

## 11. Developer / AI tooling (optional)

Only for a robot that is also a development machine; nothing in the stack
needs it.

* **Claude Code CLI** (§0). People log in with `claude` and `/login`; that
  login expires, so anything unattended uses the token of §8. `~/.local/bin`
  must be on the `PATH` (Ubuntu's `~/.profile` adds it when the directory
  exists).
* **Skills of the "director" workflow** — repo `lacina-dev/claude-skills`:
  ```bash
  git clone https://github.com/lacina-dev/claude-skills ~/claude-skills
  ~/claude-skills/install.sh
  ```
  `install.sh` is idempotent: it symlinks every `skills/*` into
  `~/.claude/skills/` and the command-line tools into `~/.local/bin`
  (`pred-jizdou` — the pre-drive check of §12; `tezky`, `ros`, `pid-uzlu`,
  `mer-proces`, `uzel-docasne`, `po-slouceni`, `nastenka`, `rozbor`, … — one
  line each in the repo's `README.md`, usage in `--help`). Update with
  `git -C ~/claude-skills pull`; run `install.sh` again only when a skill or
  tool was added. Check: `ls -l ~/.claude/skills` shows symlinks into
  `~/claude-skills`, `command -v pred-jizdou` answers.
* **Board service** (`reditel-nastenka`, web on `:8099`, data in `~/.reditel`)
  and the **daily review timer** (`reditel-rozbor.timer`, 04:30):
  ```bash
  ln -sfn ~/claude-skills/skills/reditel-nastenka/reditel-nastenka.service ~/.config/systemd/user/
  cp ~/claude-skills/skills/reditel/systemd/reditel-rozbor.{service,timer} ~/.config/systemd/user/
  systemctl --user daemon-reload
  systemctl --user enable --now reditel-nastenka reditel-rozbor.timer
  ```
  Check: `curl -s -o /dev/null -w '%{http_code}\n' http://localhost:8099/` →
  `200`; `systemctl --user list-timers reditel-rozbor.timer` shows the next
  04:30.

## 12. First drive check

The positive check that the robot may drive. Robot in the dock, stack running
for at least two minutes.

1. **Everything publishes** (in a new terminal, §4):
   ```bash
   systemctl is-active vitulus.service                 # active
   rostopic hz /scan                                   # ~10 Hz
   rostopic hz /mower/status                           # 2 Hz
   rostopic echo -n 1 /dock_manager/is_in_dock         # data: True
   rostopic echo -n 1 /mower_smach/busy                # data: False
   rostopic echo -n 1 /mower/link                      # state ok
   rostopic echo -n 1 /gnss_watchdog/status            # state OK, both receivers fresh
   rostopic echo -n 1 /bno085/health                   # ok
   rostopic echo -n 1 /nav_tf/loc_status               # gps_clock_offset_s near 0 (tenths of a second at most)
   rostopic echo -n 1 /dock_permit/state               # answers (locked until the owner unlocks)
   rostopic info /safety/lidar_objects                 # one publisher
   rosnode info /depth_obstacles                       # answers once navigation runs
   ```
2. **Or in one command**, with the tooling of §11: `pred-jizdou` (read-only,
   a few seconds). Its first line is the verdict — `MŮŽE JET` (may drive),
   `MŮŽE JET, ale: …` (may, with reservations that only spoil measurements) or
   `NEMŮŽE: <reason>`; exit code 0 = may. Below it one line per check, each
   `ok`, `info` or `NE`: stack, in the dock, not mowing, mower board, GNSS,
   RTK (required outside the dock only), IMU, clock offset against GPS, dock
   permit (information only), agent STOP, running missions, machine load,
   free disk. Every `NE` must be resolved first.
3. **Nothing stray:** `ps -eo cmd | grep "[c]atkin_ws/install/lib"` prints
   nothing; `ps -eo pid,ppid,lstart,cmd | grep "[r]oslaunch"` shows only
   processes started with the stack (§13).
4. **First exit, blade off.** Somebody stands at the robot with the UI open
   (STOP within reach). The owner unlocks the dock in the UI; *Dock* tab →
   *Undock program* → **Execute**. Do not start a mowing program — the blade
   stays off. Outside, wait for RTK FIXED (`pred-jizdou` now reports
   `RTK: oba RTK`), drive a few metres by teleop and compare the robot's
   position on the map with reality. Then *Dock program* → **Execute**; the
   robot must end in the dock (`/dock_manager/is_in_dock` True) and charge.
   Lock the dock again.

Only after this passes build zones and run the first mowing program.

## 13. Known traps

Each of these has already cost hours.

* Described where they belong: `catkin_make` and the install-first workspace
  (§5; symptom: after a restart no `/scan` and move_base_flex not connecting),
  `LD_PRELOAD` and the position of switches in `~/.bashrc` (§4), restarting
  only one agent service (§8), unbounded `journalctl` (§6).
* **A hand-started node survives a stack restart.** A `roslaunch` typed in a
  terminal keeps running when `vitulus.service` restarts; its node name then
  collides with the stack's node and kicks it off the master, and a node
  without `respawn` does not come back. Start temporary nodes with
  `uzel-docasne` (§11) or end them before the restart.
* **Orphan `depth_obstacles` after a `navi_man` restart.** `navi_man` starts
  `roslaunch vitulus_depth_obstacles …` as a child; killing `navi_man` leaves
  it running with parent 1, the new `navi_man` starts a second one and the two
  `/depth_obstacles` nodes throw each other off the master about twice a
  second — no obstacles and no error message. After a `navi_man` restart:
  `ps -eo pid,ppid,args | grep "[r]oslaunch"`, end the one with `ppid` 1
  (`kill -INT <pid>`), then `rosnode info /depth_obstacles` must show a stable
  PID. Restarting the whole stack avoids it.
* **`pkill -f` / `pgrep -f` inside a compound shell command** matches the
  shell itself and silently skips the rest; put kill logic in a script file or
  use `pid-uzlu` (§11).
* **Unit linked through a deleted package.** Older installations may have
  `/etc/systemd/system/vitulus.service` symlinked through the removed
  `vitulus_bringup` package; re-install it directly (§6).
* **Python nodes run through small relay stubs in `devel/`** that point into
  `src/`: an edit in `src/` is live after a node restart; never copy a file
  over the stub.
