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

The node is always in one of four states. The state says whether the map ->
camera_init correction it publishes can be trusted.

========  ==========================================================
State     Meaning
========  ==========================================================
INIT      Started, no match has passed yet. No correction is
          published. The drone must not arm.
TRACKING  The last match passed the gate. The correction is fresh.
HOLDING   The last match failed. The last good correction is still
          used: its error grows only at FAST-LIO2's drift rate.
LOST      Matches failed for too long. The correction is reported as
          untrusted; the mission layer decides (hold, land). This
          node never picks a pose it is not sure of.
========  ==========================================================

Transitions, once per match (``match_rate``, 1 Hz)::

    INIT ──search passes──> TRACKING <──pass── HOLDING
                               │                  │
                               └──fail──> HOLDING ┘
                                                  │ fail, and a LOST condition
                                                  v
    LOST ──search passes──> TRACKING            LOST

* **Pass**: the match passes every check in "Gate", the step included.
* **Fail**: anything else. A tick with no new scan since the last match also
  counts as a fail, so a stopped FAST-LIO2 ends in LOST instead of re-matching
  the same old submap.
* **LOST conditions**, checked on every fail in TRACKING or HOLDING:

  * no pass for more than ``lost_after_s`` (5 s), timed with the node clock
    (bag time when replaying with ``use_sim_time``);
  * ``max_rejected_steps`` (3) large steps in a row. A large step is a match
    whose quality is fine but whose step is over ``max_step_m`` or
    ``max_step_deg``: the map keeps saying the drone is elsewhere. A poor
    quality match breaks the run (resets the count); it is covered by the
    time limit instead.

* From TRACKING or HOLDING, one pass returns to TRACKING. From INIT and LOST,
  only the search below leaves; a normal match never does.


The Search (INIT and LOST)
~~~~~~~~~~~~~~~~~~~~~~~~~~

Run on every tick while in INIT or LOST:

1. Predict the body pose in the map from the current correction and odometry.
   In INIT the correction is ``init_pose`` (x, y, z, yaw of camera_init in
   ``map``; identity by default, takeoff from the mapping walk's start). In
   LOST it is the last good correction.
2. Make ``init_yaw_steps`` (12) seeds: that pose, rotated about the body's own
   position in steps of 360 / ``init_yaw_steps`` degrees.
3. Align the submap from each seed, coarse to fine: at each of
   ``search_corr_dists`` (5, 1.5 m), then at ``max_corr_dist``, each stage from
   the last one's result. One stage at 0.5 m only converges from within about
   5 deg and 1 m; the stages widen that to about 20 deg and 5 m, more than the
   15 deg a seed can be from the truth. The gate judges the final stage.
   Keep those that pass the gate's quality
   checks; the step is not checked, since the search expects a jump.
4. The one with the highest overlap wins, unless another one passing is more
   than a step (``max_step_m``, ``max_step_deg``) away from it. Then the
   answer is ambiguous (a corridor, a symmetric room): stay, warn, try again
   next tick. Seeds that converge to the same pose agree and are fine.
5. A winner becomes the correction; the state is TRACKING.

A search needs a scan newer than the last match or search: after a gap the
old scans are still stacked, and matching them would only confirm where the
drone was before it.

Cost: up to ``init_yaw_steps`` x 3 alignments per tick in INIT and LOST (measured in
step 10).


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
