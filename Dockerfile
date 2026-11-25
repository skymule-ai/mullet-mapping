ARG BASE_IMAGE=ghcr.io/skymule-ai/mullet-base:v0.6.0
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

LABEL org.opencontainers.image.title="mullet-mapping"
LABEL org.opencontainers.image.description="Localization in a prior map (map_localizer) for the Mullet Jetson platform"

# Pin SMALL_GICP_REF to a full commit SHA for reproducible production images.
ARG SMALL_GICP_REPOSITORY=https://github.com/koide3/small_gicp.git
ARG SMALL_GICP_REF=

RUN apt-get update && apt-get install -y --no-install-recommends \
  libeigen3-dev libpcl-dev \
  ros-${ROS_DISTRO}-pcl-conversions ros-${ROS_DISTRO}-tf2-eigen \
  && rm -rf /var/lib/apt/lists/*

# TODO(step 2): build small_gicp from source and install it where CMake finds it.
#   git clone "${SMALL_GICP_REPOSITORY}" /tmp/small_gicp && git -C /tmp/small_gicp checkout "${SMALL_GICP_REF}"
#   cmake -S /tmp/small_gicp -B /tmp/small_gicp/build -DCMAKE_BUILD_TYPE=Release (+ the OpenMP option)
#   cmake --build ... -j"$(nproc)" && cmake --install ... && rm -rf /tmp/small_gicp
# Fail the build if SMALL_GICP_REF is empty.

COPY src /opt/mullet_ws/src

# Builds, then runs the unit tests: a failing test fails the image build.
RUN source /usr/local/share/mullet-ros/setup.bash \
  && cd /opt/mullet_ws \
  && colcon build --merge-install --packages-up-to map_localizer \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && colcon test --merge-install --packages-select map_localizer \
  && colcon test-result --verbose \
  && rm -rf build log

# TODO(step 2): an entrypoint that sources /usr/local/share/mullet-ros/setup.bash and
# /opt/mullet_ws/install/setup.bash, then `exec "$@"` (see mullet-lio images/fast-lio/entrypoint.sh).
# ENTRYPOINT ["/usr/local/bin/mullet-mapping-entrypoint"]
# CMD ["ros2", "run", "map_localizer", "map_localizer_node", "--ros-args", "--params-file", "/config/map_localizer.yaml"]
