Running on a Drone
==================

Ask before running anything on a drone: experiments run there in parallel.


Before Starting
---------------

* mullet-localization runs the ``fast-lio`` profile, and FAST-LIO2 publishes.
* ``~/maps/<MAP_SITE>/map.pcd`` is on the Orin (scp + md5, one folder per site).
* The drone stands on the mapping walk's start spot, or ``init_pose`` is set in
  ``config/map_localizer.yaml``.


Start
-----

::

    cp .env.example .env      # MAPPING_TAG (immutable on aircraft), MAP_SITE
    docker compose pull && docker compose up -d
    docker compose logs -f map_localizer

The ROS domain and ``DRONE_ID`` come from ``/etc/mullet/drone.env``.


Check
-----

* ``/$DRONE_ID/map_localizer/status`` reaches TRACKING on the ground, before
  arming.
* In Foxglove, fixed frame ``map``: the scan lies on the map's walls.


Never
-----

* Restart it during a flight.
* Change ``config/`` or the map during a flight. Both are mounted: edit, then
  ``docker compose restart map_localizer`` on the ground.
