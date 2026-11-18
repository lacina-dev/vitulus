# New robot — checklist

One page to tick off. Every line links to the chapter of [`INSTALL.md`](../INSTALL.md)
that has the exact command, the reason and the check. No secrets belong here.

## Required (the robot boots and drives)

- [ ] Ubuntu 20.04, ROS Noetic, apt and pip packages, Docker with compose — [§0](../INSTALL.md#0-prerequisites)
- [ ] GitHub login works for the private repos (`vitulus_dock`, `vitulus_local_costmap`) — [§0](../INSTALL.md#0-prerequisites)
- [ ] Vitulus repos cloned in the `src/vitulus/<pkg>` layout; `rtabmap_docker` and `vitulus_depth_obstacles` in `src/` — [§1](../INSTALL.md#1-workspace-layout), [§2](../INSTALL.md#2-repositories)
- [ ] Third-party repos in `src/other` on the listed branches (incl. `navigation`, `grid_map`) — [§2](../INSTALL.md#third-party-packages-built-from-source-srcother)
- [ ] rtabmap image built — [§3](../INSTALL.md#3-docker-pieces)
- [ ] udev rules adapted to this robot's USB ports; seven `/dev/*` symlinks exist — [§4](../INSTALL.md#4-host-configuration)
- [ ] User in `dialout`, `docker`, `plugdev`, `adm` — [§4](../INSTALL.md#4-host-configuration)
- [ ] ROS address: netplan bridge or `VITULUS_ROS_IP` — [§4](../INSTALL.md#4-host-configuration)
- [ ] `~/.bashrc`: switches above the interactivity check, ROS environment at the end, no `LD_PRELOAD` — [§4](../INSTALL.md#4-host-configuration)
- [ ] sudoers `vitulus-time` (and `vitulus-ui`); clock synchronized — [§4](../INSTALL.md#4-host-configuration)
- [ ] `whoopsie` and `apport` masked, `core_pattern` is `core`, `/var/crash` empty; journal capped at 1 GB — [§4](../INSTALL.md#4-host-configuration)
- [ ] Time zone set, swap present, `Linger=yes` — [§4](../INSTALL.md#4-host-configuration)
- [ ] `weather_alert` location set — [§4](../INSTALL.md#4-host-configuration)
- [ ] `~/.vitulus/dock_permit_owners` written (without it every unlock is refused) — [§4](../INSTALL.md#4-host-configuration)
- [ ] Tailscale joined — [§4](../INSTALL.md#network-and-remote-access)
- [ ] catkin profile: no install space, `Release`, `CMAKE_EXPORT_COMPILE_COMMANDS`; built with `catkin build` — [§5](../INSTALL.md#5-build)
- [ ] `devel/_setup_util.py` has `devel;/opt/ros/noetic` — [§5](../INSTALL.md#5-build)
- [ ] `vitulus.service` symlinked directly to `vitulus/scripts/vitulus.service` and enabled; UI answers on `:7779` — [§6](../INSTALL.md#6-systemd-services)
- [ ] `vitulus-rosclean.timer` and `vitulus-lidar-objects.service` enabled — [§6](../INSTALL.md#6-systemd-services)
- [ ] Mower board reports `FW_VERSION 4.1.0-isrfix` — [§10](../INSTALL.md#10-mower-board-firmware)
- [ ] Dock map, dock/undock programs, first site map, dock points — [§7](../INSTALL.md#7-first-boot-on-an-empty-robot)
- [ ] **First drive check passed** (all topics publish / `pred-jizdou` says the robot may drive; first exit with the blade off) — [§12](../INSTALL.md#12-first-drive-check)

## Optional

- [ ] SSH keys reviewed (read-only keys restricted – recommended) — [§4](../INSTALL.md#network-and-remote-access)
- [ ] MapProxy + cache cleanup timer — [§3](../INSTALL.md#3-docker-pieces)
- [ ] Precipitation service `vitulus-precip` healthy, `unless-stopped`, 400 MiB cap — [§3](../INSTALL.md#3-docker-pieces)
- [ ] `vitulus_traversability` built (switches itself on) — [§2](../INSTALL.md#2-repositories)
- [ ] Agent: four services active on the same commit — [§8](../INSTALL.md#8-on-board-agent-vitulus_agent-optional)
- [ ] Agent: `secrets.env` (600, one line per key) loaded by all four services; `authMethod` is `oauth_token` — [§8](../INSTALL.md#model-login)
- [ ] Agent: `VITULUS_ROSIO_TAP=1` drop-in on all four, `models.yaml` from the example, two standing rain rules active, automatic approval as the owner decided (mower left out) — [§8](../INSTALL.md#service-variables-web-guard-lawn-care-standing-rules)
- [ ] Field kit watcher installed; you know which repos it does not deploy — [§9](../INSTALL.md#9-field-kit-vitulus-field)
- [ ] Developer tooling: skills, board on `:8099`, review timer — [§11](../INSTALL.md#11-developer--ai-tooling-optional)

## Keep at hand

- [ ] [§13 Known traps](../INSTALL.md#13-known-traps)
