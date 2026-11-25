ARG BASE_IMAGE=ghcr.io/skymule-ai/mullet-base:v0.6.0
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

LABEL org.opencontainers.image.title="mullet-mapping"
LABEL org.opencontainers.image.description="Localization in a prior map (map_localizer) for the Mullet Jetson platform"

# Pin SMALL_GICP_REF to a full commit SHA for reproducible production images.
ARG SMALL_GICP_REPOSITORY=https://github.com/koide3/small_gicp.git
ARG SMALL_GICP_REF=fd29d8cf94cf05ed7ad21c81c27b65963110adb5

RUN apt-get update && apt-get install -y --no-install-recommends \
  libeigen3-dev libpcl-dev \
  ros-${ROS_DISTRO}-pcl-conversions ros-${ROS_DISTRO}-tf2-eigen \
  && rm -rf /var/lib/apt/lists/*

RUN test -n "${SMALL_GICP_REF}" \
  && git clone "${SMALL_GICP_REPOSITORY}" /tmp/small_gicp \
  && git -C /tmp/small_gicp checkout "${SMALL_GICP_REF}" \
  && cmake -S /tmp/small_gicp -B /tmp/small_gicp/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_HELPER=ON \
  -DBUILD_WITH_OPENMP=ON \
  -DBUILD_TESTS=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF \
  && cmake --build /tmp/small_gicp/build -j"$(nproc)" \
  && cmake --install /tmp/small_gicp/build \
  && ldconfig \
  && rm -rf /tmp/small_gicp

COPY src /opt/mullet_ws/src

# Builds, then runs the unit tests: a failing test fails the image build.
RUN source /usr/local/share/mullet-ros/setup.bash \
  && cd /opt/mullet_ws \
  && colcon build --merge-install --packages-up-to map_localizer \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && colcon test --merge-install --packages-select map_localizer \
  && colcon test-result --verbose \
  && rm -rf build log

# Sources the ROS and workspace setup files, then runs CMD.
COPY images/mapping/entrypoint.sh /usr/local/bin/mullet-mapping-entrypoint

RUN chmod 0755 /usr/local/bin/mullet-mapping-entrypoint

ENTRYPOINT ["/usr/local/bin/mullet-mapping-entrypoint"]

CMD ["ros2", "run", "map_localizer", "map_localizer_node", "--ros-args", "--params-file", "/config/map_localizer.yaml"]