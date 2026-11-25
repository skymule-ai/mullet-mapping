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


Who Are You?
============

Find your role below:

* C++ Developer: writing and testing map_localizer
* Drone Operator: running it in the field
* Integrator: using the map correction in mullet_core, ev_bridge or a planner


For Specific Users
==================

C++ Developer
-------------

Start here:

* Design: Documentation/design.rst
* Status and next step: Documentation/status.rst
* Build and test: Documentation/build.rst
* Code: src/map_localizer/include/map_localizer/map_localizer.hpp

Drone Operator
--------------

Run and check it on a drone:

* Running: Documentation/running.rst
* Status topic: Documentation/interfaces.rst
* Rules: Documentation/rules.rst

Integrator
----------

Use map -> camera_init elsewhere in the stack:

* Interfaces and frames: Documentation/interfaces.rst
* Option A and Option B: Documentation/design.rst
* Flow diagram: https://claude.ai/artifact/KAzZcNzyHfuSfNPSEkoHuY


Related Repositories
====================

* Sensors, FAST-LIO2, ev_bridge, recording: https://github.com/skymule-ai/mullet-localization
* FAST-LIO2 image: https://github.com/skymule-ai/mullet-lio
* Offboard supervisor: https://github.com/skymule-ai/mullet-core
* Drone identity (/etc/mullet/drone.env): https://github.com/skymule-ai/mullet-setup
* SLAM and mapping: https://github.com/skymule-ai/mullet-wiki/blob/main/docs/concepts/slam-and-mapping.md
* Planner: https://github.com/skymule-ai/ergodic_mppi
