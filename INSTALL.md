# Vitulus — installing the stack on a robot

How to get from an empty computer to a booting Vitulus robot. Everything here
is derived from the repositories themselves; per-package details live in each
package's `README.md`.

Target: Ubuntu 20.04, ROS Noetic, Python 3, Docker. The robot user on the
reference robot is `vitulus`; the standard workspace is `~/catkin_ws`.

## 1. Workspace layout

```
~/catkin_ws/
├── src/
│   ├── vitulus/            # ALL Vitulus packages, one git repo per package
│   │   ├── vitulus/        #   this repo: boot script, main launch, sensors
│   │   ├── vitulus_base/
│   │   ├── vitulus_navi/
│   │   └── ...
│   ├── rtabmap_docker/     # launch files mounted into the rtabmap container
│   ├── docker-mapproxy/    # aerial tile server (optional)
│   └── other/              # third-party ROS packages built from source
└── devel/                  # created by the build
```

`vitulus.startup` and `nodes/main` locate the workspace from their own
location, launch files use `$(find <pkg>)`. The `src/vitulus/<pkg>` nesting is
what the reference robot uses; keep it, some packages still look for their
sibling packages next to themselves.

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
| `vitulus_local_costmap` | local costmap (included by `vitulus_navi/launch/navi_man.launch`) |
| `vitulus_planner` | zones, coverage paths, mowing programs, calendar |
| `vitulus_mapping` | Mapping v3 (`mapping_manager` is started by `vitulus_ui.launch`) |
| `vitulus_dock` | docking / undocking |
| `vitulus_safety` | safety node |
| `vitulus_rosbag` | rosbag recorder |
| `vitulus_ui` | web UI on `:7779` |
| `weather_alert` | rain radar alert |

Clone into `src/`: `rtabmap_docker` (required), `docker-mapproxy` (optional).

**Repositories that need an invitation.** `vitulus_dock` and
`vitulus_local_costmap` are private on GitHub (as of 2026-09-20; an anonymous
clone fails with "repository not found"). Both are required by the boot launch,
so ask the owner for access before you start. `vitulus_agent` is private too,
but optional.

Optional, not needed to boot: `vitulus_teleop`, `vitulus_webui` (old UI,
replaced by `vitulus_ui`; do not run both), `vitulus_agent` (on-board
assistant, package name `vitulus_claude`; private).

Third-party packages the launch files use and that are built from source in
`src/other/` on the reference robot:

| Package | Source (branch on the reference robot) |
|---|---|
| `system_monitor` | `lacina-dev/ros-system-monitor` (`vitulus`) |
| `interactive_marker_proxy` | `lacina-dev/interactive_marker_proxy` (`develop`) |
| `move_base_flex` | `naturerobots/move_base_flex` (`master`) |
| `robot_localization` | `cra-ros-pkg/robot_localization` (`noetic-devel`) |
| `teb_local_planner`, `costmap_converter` | `rst-tu-dortmund/...` |
| `rplidar_ros` | `Slamtec/rplidar_ros` |
| `ublox` | `KumarRobotics/ublox` |
| `ds4_driver` | `naoki-mizuno/ds4_driver` (`noetic-devel`) |
| `laser_filters` | `ros-perception/laser_filters` (`noetic-devel`) |
| `web_video_server` | `RobotWebTools/web_video_server` |

`move_base_flex` (`master`) and `robot_localization` (`noetic-devel`) are used
as they come from upstream. Known-good versions, i.e. the commits the reference
robot runs: `move_base_flex` `367269c3c2427dfee29426e5f9f136731fb3b049`,
`robot_localization` `9ef26a57fafd9b97c135a7e07f52cf0168e94178`. The reference
robot carries two cosmetic local edits on top of them (default planner names in
the `move_base_legacy_relay.py` script, which no Vitulus launch file starts, and
one `robot_localization` log line demoted from INFO to DEBUG); they are NOT
required.

From apt (`ros-noetic-*`): `realsense2_camera`, `rosbridge_server`, `rosapi`,
`tf2_web_republisher`, `rosserial_python`, `imu_complementary_filter`,
`pointcloud_to_laserscan`, `rtabmap_odom`, `swri_transform_util`, `mapviz`,
`controller_manager`, `robot_state_publisher`, `joint_state_publisher`,
`xacro`, `topic_tools`, `pcl_ros`.

Python modules imported by the nodes: `numpy`, `scipy`, `cv2`, `yaml`,
`shapely`, `pyproj`, `serial` (pyserial), `flask`, `docker`, `moteus`,
`NetworkManager` (python-networkmanager), `dbus`, `PIL`, `matplotlib`,
`adafruit_bno08x`.

The `package.xml` files do not declare all of these, so `rosdep` alone is not
enough.

```bash
mkdir -p ~/catkin_ws/src/vitulus && cd ~/catkin_ws/src/vitulus
for r in vitulus vitulus_msgs vitulus_description vitulus_base vitulus_ups \
         vitulus_imu vitulus_gnss vitulus_mower vitulus_ds4 vitulus_nmcli \
         vitulus_navi vitulus_local_costmap vitulus_planner vitulus_mapping \
         vitulus_dock vitulus_safety vitulus_rosbag vitulus_ui weather_alert; do
  git clone https://github.com/lacina-dev/$r.git
done
cd ~/catkin_ws/src
git clone https://github.com/lacina-dev/rtabmap_docker.git
git clone https://github.com/lacina-dev/docker-mapproxy.git   # optional
```

## 3. Docker pieces

**rtabmap** (required). `vitulus_navi/nodes/navi_man` starts containers from the
image `vitulus/rtabmap_ros:noetic-latest` (host network, the `rtabmap_docker`
directory mounted at `/rtabmap_docker`, `~/.ros` at `/tmp/.ros`). Build the
image once:

```bash
cd ~/catkin_ws/src/rtabmap_docker && docker compose build
```

The robot user must be allowed to use Docker (`docker` group).

**MapProxy** (optional). Serves the aerial tiles for the UI's aerial layer on
`:8082`. Without it the UI works, only the aerial layer stays empty.

```bash
cd ~/catkin_ws/src/docker-mapproxy
docker compose -f docker-compose-build.yml up -d     # restart: always
```

## 4. Host configuration

Machine-level files live in no package; `setup/` in this repo holds examples
copied from the reference robot.

* **Device symlinks.** The nodes open `/dev/rplidar`, `/dev/nextion`,
  `/dev/ups`, `/dev/mower`, `/dev/gnss`, `/dev/gnss_heading` and `/dev/imu`.
  Start from `setup/99-vitulus.rules.example`: most rules are bound to the
  physical USB port, so the `KERNELS==` values must be changed to your own
  USB topology (the file explains how to find them).
  ```bash
  sudo cp setup/99-vitulus.rules.example /etc/udev/rules.d/99-vitulus.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```
* **ROS network.** `vitulus.startup` exports `ROS_IP` / `ROS_MASTER_URI` with
  the address `10.254.254.254`. On the reference robot that is a netplan
  bridge `br0` over the wired port: `setup/02-vitulus-network.yaml.example`
  (adapt the interface name, install as
  `/etc/netplan/02-vitulus-network.yaml`, `sudo netplan try`). Alternatively
  set `VITULUS_ROS_IP` to an address the host does have (e.g. in `~/.bashrc`,
  which `vitulus.service` sources).
* **sudoers.** Two passwordless rules for the robot user:
  * time sync — `vitulus.startup` runs `ntpdate` and stops
    `systemd-timesyncd`. `sudo visudo -f /etc/sudoers.d/vitulus-time`:
    ```
    vitulus ALL=(ALL) NOPASSWD: /usr/bin/systemctl start systemd-timesyncd.service, /usr/bin/systemctl status systemd-timesyncd.service, /usr/bin/systemctl stop systemd-timesyncd.service, /usr/sbin/ntpdate
    ```
    (`vitulus.startup` also tries `sudo apt-get install ntpdate` when it is
    missing — install it beforehand: `sudo apt install ntpdate`.)
  * UI "Restart robot" button (optional) — install
    `vitulus_ui/setup/vitulus-ui-sudoers` as `/etc/sudoers.d/vitulus-ui`, see
    `vitulus_ui/README.md`.
* **weather_alert location.** The build copies
  `weather_alert/config/config.local.example.yaml` to the git-ignored
  `config/config.local.yaml`. Set your coordinates there; until you do, the
  node runs with the placeholder location (Prague) and says so in its log.
* **NetworkManager** (optional): `scripts/org.freedesktop.NetworkManager.pkla`
  lets the `vitulus` user manage WiFi without a polkit prompt.

## 5. Build

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin build            # catkin_tools, as on the reference robot (package READMEs use catkin_make)
source devel/setup.bash
```

`vitulus.startup` needs `<workspace>/devel/setup.bash`.

## 6. systemd service

```bash
chmod +x ~/catkin_ws/src/vitulus/vitulus/launch/vitulus.startup
sudo cp ~/catkin_ws/src/vitulus/vitulus/scripts/vitulus.service /etc/systemd/system/
# edit User= / paths in the unit if your user or workspace differ
sudo systemctl daemon-reload
sudo systemctl enable --now vitulus.service
journalctl -u vitulus.service -f
```

Boot chain: `vitulus.service` → `launch/vitulus.startup` (time sync, ROS
environment, workspace detection; `VITULUS_WS` overrides the detection) →
`nodes/main` (starts `roscore` if needed) → `launch/vitulus_start.launch`.
The service restarts the whole chain 5 s after it exits; publishing `fuse` on
`/restart_core` makes `main` exit.

Then open `http://<robot>:7779/`.

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
`No DOCK map available` when docking is requested without any map. Node logs:
`journalctl -u vitulus.service`.
