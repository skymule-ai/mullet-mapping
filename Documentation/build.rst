Building and Testing
====================


Layout
------

::

    src/map_localizer/        ament_cmake package: include/, src/, test/ (gtest)
    config/map_localizer.yaml parameters with starting values
    Dockerfile                FROM mullet-base, small_gicp pinned and built from source
    compose.yaml              the map_localizer service; reads /etc/mullet/drone.env
    .env.example              image tag and which map to load
    scripts/smoke.sh          sensor-free smoke test of the image
    Documentation/            this documentation
    .github/workflows/        ARM64 image build and publishing

Until status.rst says otherwise, files are scaffolding: they have their
structure and ``TODO(step N)`` markers, not their logic.


Build
-----

::

    cp .env.example .env
    docker compose build      # also runs the unit tests; a failure fails the build
    scripts/smoke.sh

The image is arm64 (CUDA base), like every Mullet image. On an x86 laptop,
build against an amd64 stand-in base (``--build-arg BASE_IMAGE=...``, see
mullet-localization ``CLAUDE.md``).


Tests
-----

* Unit tests: ``src/map_localizer/test/``, gtest, pure math only (no ROS, no
  files). Write each test before its function.
* Smoke test: ``scripts/smoke.sh``. The image starts, the node is there, it
  loads a map and waits for scans. Not a test of localization quality.
* Replay: a recorded bag against its own map (status.rst step 9).


Releases
--------

CI builds for ARM64 on every push and publishes branch, ``sha-<commit>`` and
``vX.Y.Z`` tags to ``ghcr.io/skymule-ai/mullet-mapping``. No release tag before
a replay passes and the node has run on the Orin.
