mullet-mapping documentation
============================

* design.rst: what map_localizer does and why; loop, gate, states, parameters
* interfaces.rst: topics, files and frames it reads and writes
* build.rst: repository layout, image build, unit and smoke tests
* running.rst: running it on a drone, preconditions, checks
* rules.rst: what this repository never does
* status.rst: the build-out, step by step, and what comes later

.. toctree::
   :hidden:

   design
   interfaces
   build
   running
   rules
   status

Background, outside this repository:

* Why loop closure and PGO stay offline: mullet-wiki docs/concepts/slam-and-mapping.md
* The mapping project: mullet-wiki docs/projects/onboard-mapping.md
* FAST-LIO2 runbook (start still, never restart in flight): mullet-localization README.md
* FAST-LIO2 internals and outputs: mullet-lio docs/fast-lio.md
* Where map.pcd comes from: field/tools/pgo_map.py (offline, laptop)
