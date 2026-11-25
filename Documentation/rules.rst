Rules
=====

* Never publish ``/fmu/in/*``. PX4 gets FAST-LIO2 odometry only, through
  ev_bridge (Option A).
* Never change FAST-LIO2, ev_bridge or anything in another repository.
* Maps are read-only here. They are built offline, never updated in flight.
* Dependencies go in this image, not in mullet-base.
* Everything published is under ``/$DRONE_ID``, TF included (``/$DRONE_ID/tf``).
* ``Documentation/design.rst`` and the code change together.
