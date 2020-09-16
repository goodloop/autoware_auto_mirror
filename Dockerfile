# syntax=docker/dockerfile:experimental

# Use experimental buildkit for faster builds
# https://github.com/moby/buildkit/blob/master/frontend/dockerfile/docs/experimental.md
# Use `--progress=plain` to use plane stdout for docker build
#
# Example build command:
# export DOCKER_BUILDKIT=1
# export FROM_IMAGE="ros:foxy"
# export OVERLAY_MIXINS="release ccache"
# docker build -t autoware/auto:foxy \
#   --build-arg FROM_IMAGE \
#   --build-arg OVERLAY_MIXINS \
#   -f Dockerfile ./

ARG FROM_IMAGE=ros:foxy
ARG UNDERLAY_WS=/opt/underlay_ws
ARG OVERLAY_WS=/opt/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone underlay source
ARG UNDERLAY_WS
WORKDIR $UNDERLAY_WS/src
COPY ./tools/ade_image/underlay.repos ../
RUN vcs import ./ < ../underlay.repos && \
    touch cra-ros-pkg/robot_localization/COLCON_IGNORE && \
    find ./ -name ".git" | xargs rm -rf

# copy overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY ./src ./AutowareAuto/src

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder
ARG DEBIAN_FRONTEND=noninteractive

# edit apt for caching
RUN cp /etc/apt/apt.conf.d/docker-clean /etc/apt/ && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' \
      > /etc/apt/apt.conf.d/docker-clean

# install CI dependencies
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    apt-get update && apt-get install -q -y \
      clang \
      clang-tidy \
      ccache \
      lcov \
      libpcap-dev \
      python3-distro \
    && rosdep update

# install underlay dependencies
ARG UNDERLAY_WS
WORKDIR $UNDERLAY_WS
COPY --from=cacher /tmp/$UNDERLAY_WS/src ./src
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build underlay source
COPY --from=cacher $UNDERLAY_WS/src ./src
ARG UNDERLAY_MIXINS="release ccache"
RUN --mount=type=cache,target=/root/.ccache \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --mixin $UNDERLAY_MIXINS \
      --symlink-install

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    . $UNDERLAY_WS/install/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release ccache"
RUN --mount=type=cache,target=/root/.ccache \
    . $UNDERLAY_WS/install/setup.sh && \
    colcon build \
      --mixin $OVERLAY_MIXINS \
      --symlink-install

# restore apt for docker
RUN mv /etc/apt/docker-clean /etc/apt/apt.conf.d/ && \
    rm -rf /var/lib/apt/lists/

# source overlay from entrypoint
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place \
      's|^source .*|source "$OVERLAY_WS/install/setup.bash"|' \
      /ros_entrypoint.sh

# test overlay build
ARG RUN_TESTS
ARG FAIL_ON_TEST_FAILURE=Ture
RUN if [ -n "$RUN_TESTS" ]; then \
        . $OVERLAY_WS/install/setup.sh && \
        colcon test \
        && colcon test-result \
          || ([ -z "$FAIL_ON_TEST_FAILURE" ] || exit 1) \
    fi
