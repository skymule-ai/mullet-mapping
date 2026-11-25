Status
======

Each step matches the ``TODO(step N)`` markers in the code
(``grep -rn "TODO(step"``). Tick it when its markers are gone and it is tested.


Build-out
---------

1.  [x] Reference: README.md, Documentation/, scaffolding
2.  [ ] Skeleton: package, Dockerfile, compose, CI build in the image; the node
    starts
3.  [ ] Map: load ``map.pcd``, downsample, build the target (covariances,
    KD-tree)
4.  [ ] Submap: stack ~1 s of scans in ``camera_init`` with their time-matched
    odometry
5.  [ ] Align: GICP from the predicted ``map -> body``
6.  [ ] Gate: overlap, RMSE, step size, degeneracy
7.  [ ] States: INIT (yaw search), TRACKING, HOLDING, LOST
8.  [ ] Publish: TF at 10 Hz, status
9.  [ ] Replay: a recorded bag against its own map, offline
10. [ ] Orin: CPU and memory next to FAST-LIO2
11. [ ] Phase 2: hand flight, steps < 10 cm, gate never trips in the mapped area


Later
-----

* map_server (OctoMap)
* Static ESDF and clearance
* nvblox, once obstacles move
* Option B (map-frame EV), if phase 2 allows it
