#!/usr/bin/env bash
# Sensor-free smoke test of the image: it starts, the node is there, it loads a map and waits for
# scans. Not a test of localization quality (that is the replay, Documentation/status.rst step 9).
set -euo pipefail

image="${MAPPING_IMAGE:-ghcr.io/skymule-ai/mullet-mapping:local}"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
container_name="mullet-mapping-smoke-${RANDOM}-$$"

cleanup() {
  docker rm --force "${container_name}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

docker image inspect "${image}" >/dev/null

# 1. the package and executable exist
docker run --rm "${image}" ros2 pkg executables map_localizer | grep -q map_localizer_node

# TODO(step 3): 2. it loads a map. Write a tiny map.pcd (e.g. the walls of a box room) to a temp dir,
#   mount it at /map, run the node in its own ROS_DOMAIN_ID (never meet a real drone), and grep the
#   log for the point count it reports after downsampling.

# TODO(step 7): 3. with no scans it stays alive in INIT for a few seconds and says it is waiting,
#   then `docker inspect` shows the container still running.

echo "smoke: ${image} OK"
