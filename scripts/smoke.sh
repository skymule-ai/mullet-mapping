#!/usr/bin/env bash
# Sensor-free smoke test of the image: it starts, the node is there, it loads a map and waits for
# scans. Not a test of localization quality (that is the replay, Documentation/status.rst step 9).
set -euo pipefail

image="${MAPPING_IMAGE:-ghcr.io/skymule-ai/mullet-mapping:local}"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
container_name="mullet-mapping-smoke-${RANDOM}-$$"

map_dir="$(mktemp -d)"

cleanup() {
  docker rm --force "${container_name}" >/dev/null 2>&1 || true
  rm -rf "${map_dir}"
}
trap cleanup EXIT

docker image inspect "${image}" >/dev/null

# 1. the package and executable exist
docker run --rm "${image}" ros2 pkg executables map_localizer | grep -q map_localizer_node

# 2. it loads a map: the walls and floor of a 4 x 4 x 2.5 m room, 5 cm apart, plus one NaN point
#    the loader must drop. No network, so it can never meet a real drone.
awk 'BEGIN {
  n = 0
  for (a = 0; a <= 80; a++) for (z = 0; z <= 50; z++) {
    p[n++] = a * 0.05 " 0 " z * 0.05; p[n++] = a * 0.05 " 4 " z * 0.05
    p[n++] = "0 " a * 0.05 " " z * 0.05; p[n++] = "4 " a * 0.05 " " z * 0.05
  }
  for (a = 0; a <= 80; a++) for (b = 0; b <= 80; b++) p[n++] = a * 0.05 " " b * 0.05 " 0"
  p[n++] = "nan nan nan"
  printf "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
  printf "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n", n, n
  for (i = 0; i < n; i++) print p[i]
}' > "${map_dir}/map.pcd"

node_opts=(--network none -v "${repo_root}/config:/config:ro" -v "${map_dir}:/map:ro")

docker run -d --name "${container_name}" "${node_opts[@]}" "${image}" >/dev/null
loaded=""
for _ in $(seq 60); do  # up to 30 s, for a slow CI runner
  loaded="$(docker logs "${container_name}" 2>&1 | grep -o 'downsampled to [0-9]* points' || true)"
  [[ -n "${loaded}" ]] && break
  sleep 0.5
done
if [[ -z "${loaded}" || "${loaded}" == "downsampled to 0 points" ]]; then
  docker logs "${container_name}" >&2
  echo "smoke: no map loaded" >&2
  exit 1
fi
echo "smoke: map ${loaded}"
docker rm --force "${container_name}" >/dev/null

# 3. a missing map stops the node with an error, instead of running with no map
if out="$(docker run --rm "${node_opts[@]}" "${image}" ros2 run map_localizer map_localizer_node --ros-args \
    --params-file /config/map_localizer.yaml -p map_path:=/map/missing.pcd 2>&1)"; then
  echo "smoke: the node ran with a missing map" >&2
  exit 1
fi
grep -q "Couldn't read map file" <<<"${out}" || { echo "${out}" >&2; exit 1; }

# 4. with no scans it stays alive in INIT, says so on the status (WARN), and sends no TF
docker run -d --name "${container_name}" "${node_opts[@]}" "${image}" >/dev/null
status="$(docker exec "${container_name}" bash -c 'source /opt/mullet_ws/install/setup.bash &&
  timeout 20 ros2 topic echo --once /map_localizer/status' 2>&1 || true)"
if ! grep -qF 'level: "\x01"' <<<"${status}" ||
  ! grep -q "message: 'INIT" <<<"${status}"; then
  echo "${status}" >&2; docker logs "${container_name}" >&2
  echo "smoke: no WARN INIT status" >&2
  exit 1
fi
if docker exec "${container_name}" bash -c 'source /opt/mullet_ws/install/setup.bash &&
    timeout 5 ros2 topic echo --once /tf' >/dev/null 2>&1; then
  echo "smoke: TF sent before the first pass" >&2
  exit 1
fi
[[ "$(docker inspect -f '{{.State.Running}}' "${container_name}")" == true ]] \
  || { docker logs "${container_name}" >&2; echo "smoke: the node died in INIT" >&2; exit 1; }
echo "smoke: INIT, status WARN, no TF"

echo "smoke: ${image} OK"
