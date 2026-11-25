mullet-mapping
==============

Maps for a Mullet drone, and localization inside them. map_localizer matches
FAST-LIO2's scans against a map built offline and publishes where the drone's
odometry frame sits in that map. It runs on the Orin and never writes to PX4.

Quick Start
-----------

* Build the image: Documentation/build.rst
* Run it on a drone: Documentation/running.rst
* See what is left to do: Documentation/status.rst

Essential Documentation
-----------------------

All contributors should be familiar with:

* Design: Documentation/design.rst
* Interfaces: Documentation/interfaces.rst
* Rules: Documentation/rules.rst

The full index is Documentation/index.rst.


How It Fits
===========

Scans and odometry come from FAST-LIO2 in mullet-localization. The map comes
from the offline mapping walk. The correction goes to TF, never to PX4:

    fast_lio/cloud_registered_body --+
    fast_lio/odometry ---------------+--> map_localizer --> tf: map -> camera_init
    map.pcd -------------------------+                  --> map_localizer/status

* Frames and topics: Documentation/interfaces.rst
* Flow diagram: https://claude.ai/artifact/KAzZcNzyHfuSfNPSEkoHuY


Working on It
=============

One step at a time, from Documentation/status.rst:

* Find the work: grep -rn "TODO(step N)"
* Write the unit test first, if the step has pure math
* Build and test: docker compose build (runs the unit tests), then scripts/smoke.sh
* Done when its TODOs are gone and its check passes; tick it in status.rst
* Design changes go in Documentation/design.rst, in the same commit


Related Repositories
====================

* Sensors, FAST-LIO2, ev_bridge, recording: https://github.com/skymule-ai/mullet-localization
* FAST-LIO2 image: https://github.com/skymule-ai/mullet-lio
* Offboard supervisor: https://github.com/skymule-ai/mullet-core
* Drone identity (/etc/mullet/drone.env): https://github.com/skymule-ai/mullet-setup
* SLAM and mapping: https://github.com/skymule-ai/mullet-wiki/blob/main/docs/concepts/slam-and-mapping.md
* Planner: https://github.com/skymule-ai/ergodic_mppi
