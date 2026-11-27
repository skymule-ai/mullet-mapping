Status
======

Each step matches the ``TODO(step N)`` markers in the code
(``grep -rn "TODO(step"``). Tick it when its markers are gone and it is tested.


Build-out
---------

1.  [x] Reference: README.txt, Documentation/, scaffolding
2.  [x] Skeleton: package, Dockerfile, compose, CI build in the image; the node
    starts
3.  [x] Map: load ``map.pcd``, downsample, build the target (covariances,
    KD-tree)
4.  [x] Submap: stack ~1 s of scans, each with the odometry of its stamp, in the
    newest scan's body frame
5.  [x] Align: GICP from the predicted ``map -> body``
6.  [x] Gate: overlap, RMSE, step size, degeneracy
7.  [x] States: INIT (yaw search), TRACKING, HOLDING, LOST
8.  [x] Publish: TF at 10 Hz, status
9.  [ ] Replay: a recorded bag against its own map, offline
10. [ ] Orin: CPU and memory next to FAST-LIO2
11. [ ] Phase 2: hand flight, steps < 10 cm, gate never trips in the mapped area


Image Hardening
---------------

Common practice in production image pipelines, in order of value here:

* [ ] Hadolint: lint the Dockerfile in CI
* [ ] Trivy: scan the image in CI, fail on critical CVEs
* [ ] OCI labels: ``org.opencontainers.image.source`` and ``.revision`` (once the
  GitHub repository exists)
* [ ] ccache: BuildKit cache mount for the colcon build

Only if the whole Mullet stack adopts them: base image pinned by digest,
multi-stage build (build tools out of the runtime image).


Later
-----

* map_server (OctoMap)
* Static ESDF and clearance
* nvblox, once obstacles move
* Option B (map-frame EV), if phase 2 allows it
