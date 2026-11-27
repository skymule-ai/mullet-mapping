map_localizer design
====================

The spec ``src/map_localizer`` implements. Change the code and this file
together.


Purpose
-------

FAST-LIO2 gives a smooth, fast pose (``camera_init -> body``) that drifts about
0.5 % of the distance travelled. A map built offline (``map.pcd``, loop-closed by
``field/tools/pgo_map.py``) does not drift. map_localizer measures where the
drifting ``camera_init`` frame sits in that map and publishes it as
``map -> camera_init``::

    map -> body = (map -> camera_init) * (camera_init -> body)
                   this node, 1 Hz        FAST-LIO2, 10 Hz

Local (steady flight) and global (where am I in the map) stay separate. PX4
flies on FAST-LIO2 alone. The correction is only for the mission, and where it
is applied is decided in phase 2:

* Option A, default: mullet_core converts map-frame targets into odom with the
  latest correction. PX4 never sees it.
* Option B: ev_bridge sends ``map -> base_link``, the correction slewed. Chosen
  only if phase 2 shows steps of a few cm and no false matches.

The node is the same either way.


Inputs and Outputs
------------------

See interfaces.rst for names and types. Why each one:

* ``map.pcd``: the reference. Read once at start, never modified.
* ``cloud_registered_body``: the deskewed scan in body, so no smear from the
  100 ms sweep. It is FAST-LIO2's undistorted scan before its voxel filter, with
  every 3rd point kept (``point_filter_num: 3`` in mullet-localization
  ``config/fast-lio/mid360.yaml``).
* ``odometry``: ``camera_init -> body`` at the scan's stamp, to place each scan.
* TF ``map -> camera_init``: the correction. A frame has one parent, so never
  ``map -> body``.
* ``map_localizer/status``: for preflight, Paddock and the bag.


The Loop, at 1 Hz
-----------------

1. Submap. Keep the last ~1 s of scans (about 10), each with the odometry of
   the same stamp. Transform each into the newest scan's body frame,
   ``T_odom_body(newest)^-1 * T_odom_body(scan)``, so the match gives
   ``T_map_body`` at that stamp directly. One MID-360 scan is sparse; ten give
   a stable match.
2. Downsample the submap to ``submap_voxel`` (0.1-0.2 m).
3. Predict ``T_map_body = T_map_odom(last good) * T_odom_body(now)``. Off only
   by the drift since the last match: a few cm.
4. Align. GICP (small_gicp), submap to map, from the prediction, in one pass
   at ``max_corr_dist``: the prediction is a few cm off. Overlap and RMSE come
   from a nearest-neighbour pass at the result (GICP's own error is weighted by
   the covariances, not in meters).
5. Gate. Accept only if all hold (limits inclusive; a NaN fails every check):

   * converged: GICP converged within its iterations;
   * overlap: at least ``min_overlap`` (0.5) of the points within
     ``max_corr_dist`` (0.5 m) of the map;
   * RMSE of those points at most ``max_rmse`` (0.2 m);
   * step from the previous correction at most ``max_step_m`` (0.3 m) and
     ``max_step_deg`` (3 deg): ``previous^-1 * candidate``, both
     ``T_map_odom``, so the body's own motion is not in it;
   * not degenerate: the smallest eigenvalue of the translation block of the
     GICP Hessian at least ``min_hessian_eig`` (a corridor leaves one axis
     free). 0 turns the check off until it is set from a replayed bag.

6. Update on pass: ``T_map_odom = T_map_body(matched) * T_odom_body^-1``, both at
   the submap's reference stamp. On fail: keep the last good value, count it.
7. Publish ``T_map_odom`` on TF at ``tf_rate`` (10 Hz) from a separate timer, so
   lookups never go stale between matches. Publish status on every match.


States
------

========  ============================  ==========================================
State     Entered when                  Behaviour
========  ============================  ==========================================
INIT      start                         wide search around the initial pose:
                                        ``init_yaw_steps`` (12) yaw hypotheses,
                                        best one wins. Must pass before arming.
TRACKING  a match passes the gate       normal loop
HOLDING   a match fails                 keep the last good correction; the error
                                        grows at the drift rate only
LOST      no pass for                   reported; the mission layer decides
          ``lost_after_s`` (5 s), or    (hold, land). This node never guesses.
          ``max_rejected_steps`` (3)
          large steps in a row
========  ============================  ==========================================

From HOLDING, one pass returns to TRACKING. From LOST, only a pass under the
INIT search does.

Initial pose: identity by default (takeoff from the mapping walk's start).
Otherwise ``init_pose`` (x, y, z, yaw in ``map``) as a parameter.


Not in Scope
------------

* Loop closure and PGO: offline, ``pgo_map.py``.
* Map updates: the map is read-only.
* Writing to PX4: Option A is mullet_core, Option B is ev_bridge.
* Obstacles: map_server, the static ESDF and clearance, later in this
  repository.


Parameters
----------

All in ``config/map_localizer.yaml``, with the starting values above. Tune from
replayed bags, never during a flight.


Open Points
-----------

* small_gicp on aarch64: built from source in the Dockerfile; check it builds on
  the CI's ARM runner.
* ``map.pcd`` comes from ``pgo_map.py``'s export, which does not exist yet.
  Until then, build a test map from an October bag (merge ``cloud_registered``
  with the PGO poses).
* Replay (status.rst step 9) needs ``cloud_registered_body``. record-lite keeps
  the raw ``/livox/*``, not that topic: replay FAST-LIO2 on the raw scans to
  regenerate it.
* Status message: ``DiagnosticStatus`` for now. A Mullet message in
  mullet-interfaces only if Paddock or the planner need typed fields.
