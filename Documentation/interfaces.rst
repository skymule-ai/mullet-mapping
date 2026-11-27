Interfaces
==========

Every name is under the drone's namespace ``/$DRONE_ID`` (``uav_N``, from
``/etc/mullet/drone.env``).


Reads
-----

* ``/map/map.pcd``: PCD file. ``~/maps/<site>/`` on the Orin, mounted read-only.
* ``fast_lio/cloud_registered_body``: ``sensor_msgs/PointCloud2``. Deskewed
  scan in the body (MID-360 IMU) frame, 10 Hz.
* ``fast_lio/odometry``: ``nav_msgs/Odometry``. ``camera_init -> body``, same
  stamps as the scans.


Writes
------

* ``tf``: ``map -> camera_init`` as ``tf2_msgs/TFMessage``. Matched at 1 Hz,
  re-sent at 10 Hz. Nothing before the first passing match.
* ``map_localizer/status``: ``diagnostic_msgs/DiagnosticStatus``. Level OK
  (TRACKING), WARN (INIT, HOLDING), ERROR (LOST). Message: state and reason.
  Values: ``overlap``, ``rmse``, ``min_eig``, ``iterations``, ``step_meters``,
  ``step_degrees``, ``age_s`` (seconds since the last passing match, ``never``
  before the first) and ``state``. Published on every match tick.


Frames
------

::

    map -> camera_init -> body -> base_link
        this node     FAST-LIO2  ev_bridge extrinsic

``camera_init`` and ``body`` are FAST-LIO2's fixed frame names. ``map`` is the
``camera_init`` of the mapping walk, after PGO.
