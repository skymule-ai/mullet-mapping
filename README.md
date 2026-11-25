# mullet-mapping

Maps for a Mullet drone, and localization inside them. `map_localizer` matches FAST-LIO2's scans
against a map built offline and publishes where the drone's odometry frame sits in that map
(`map → camera_init`). It runs on the Orin beside mullet-localization and never writes to PX4.

## Quick start

* What it does and why: [docs/design.md](docs/design.md)
* What it reads and writes: [Interfaces](#interfaces)
* Build and test: [Build](#build)
* Run on a drone: [Run](#run)
* Where the work stands: [Status](#status)

## Essential documentation

* Design, loop, gate and states: [docs/design.md](docs/design.md)
* Flow from mapping walk to PX4 (A/B decision): [Mullet Map Localization Flow](https://claude.ai/artifact/KAzZcNzyHfuSfNPSEkoHuY)
* Why loop closure and PGO stay offline: mullet-wiki `docs/concepts/slam-and-mapping.md`
* The mapping project this belongs to: mullet-wiki `docs/projects/onboard-mapping.md`
* FAST-LIO2 runbook (start still, never restart in flight): mullet-localization `README.md`
* FAST-LIO2 internals and outputs: mullet-lio `docs/fast-lio.md`
* Where `map.pcd` comes from: `field/tools/pgo_map.py` (offline, laptop)

## Interfaces

Every name is under the drone's namespace `/$DRONE_ID` (`uav_N`, from `/etc/mullet/drone.env`).

| Dir | Name | Type | Notes |
| --- | --- | --- | --- |
| reads | `/map/map.pcd` | PCD file | `~/maps/<site>/` on the Orin, mounted read-only |
| reads | `fast_lio/cloud_registered_body` | `sensor_msgs/PointCloud2` | deskewed scan, body (MID-360 IMU) frame, 10 Hz |
| reads | `fast_lio/odometry` | `nav_msgs/Odometry` | `camera_init → body`, same stamps as the scan |
| writes | `tf`: `map → camera_init` | `tf2_msgs/TFMessage` | matched at 1 Hz, re-sent at 10 Hz |
| writes | `map_localizer/status` | `diagnostic_msgs/DiagnosticStatus` | state, fitness, RMSE, step, age of last match |

Frames: `map → camera_init` (this node) `→ body` (FAST-LIO2) `→ base_link` (ev_bridge extrinsic).

## Layout

```text
src/map_localizer/        ament_cmake package: include/, src/, test/ (gtest)
config/map_localizer.yaml Parameters with starting values
Dockerfile                FROM mullet-base, small_gicp pinned and built from source
compose.yaml              The map_localizer service; reads /etc/mullet/drone.env
.env.example              Image tag and which map to load
scripts/smoke.sh          Sensor-free smoke test of the image
docs/design.md            The spec this code implements
.github/workflows/        ARM64 image build and publishing
```

Everything is scaffolding until [Status](#status) says otherwise: files have their structure and
`TODO(step N)` markers, not their logic.

## Build

```bash
cp .env.example .env
docker compose build     # also runs the unit tests (colcon test); a failure fails the build
scripts/smoke.sh
```

The image is arm64 (CUDA base), like every Mullet image. On an x86 laptop build against an amd64
stand-in base (`--build-arg BASE_IMAGE=...`, see mullet-localization `CLAUDE.md`).

## Run

```bash
cp .env.example .env     # MAPPING_TAG (immutable on aircraft), MAP_SITE
docker compose pull && docker compose up -d
docker compose logs -f map_localizer
```

Before starting it:

* mullet-localization runs the `fast-lio` profile, and FAST-LIO2 is publishing.
* `~/maps/<MAP_SITE>/map.pcd` is on the Orin (scp + md5, one folder per site).
* The drone stands on the mapping walk's start spot, or an initial pose is given (`docs/design.md`).

Never restart it during a flight. Ask before running anything on a drone: experiments run there
in parallel.

## Rules

* Never publishes `/fmu/in/*`. PX4 gets FAST-LIO2 odometry only, through ev_bridge (Option A).
* Never changes FAST-LIO2, ev_bridge or anything in another repository.
* Maps are read-only here. They are built offline, never updated in flight.
* Dependencies go in this image, not in `mullet-base`.
* Everything it publishes is under `/$DRONE_ID`, TF included (`/$DRONE_ID/tf`).

## Status

1. [x] Reference: this README, `docs/design.md`
2. [ ] Skeleton: finish package, Dockerfile, compose, CI so it builds in the image and the node starts
3. [ ] Map: load `map.pcd`, downsample, build the target (covariances, KD-tree)
4. [ ] Submap: stack ~1 s of scans in `camera_init` with their time-matched odometry
5. [ ] Align: GICP from the predicted `map → body`
6. [ ] Gate: overlap, RMSE, step size, degeneracy
7. [ ] States: INIT (yaw search), TRACKING, HOLDING, LOST
8. [ ] Publish: TF at 10 Hz, status
9. [ ] Replay: a recorded bag against its own map, offline
10. [ ] Orin: CPU and memory next to FAST-LIO2
11. [ ] Phase 2: hand flight, steps < 10 cm, gate never trips in the mapped area

Later: `map_server` (OctoMap), static ESDF and `clearance`, nvblox, Option B (map-frame EV).

## Related repositories

* mullet-localization: sensors, FAST-LIO2, ev_bridge, recording; the drone's config
* mullet-lio: the FAST-LIO2 image
* mullet-core: offboard supervisor; converts map-frame targets in phase 3 (Option A)
* mullet-setup: `/etc/mullet/drone.env`, the drone's identity
* mullet-wiki: architecture and concepts
* ergodic_mppi: the planner that will plan in `map`
