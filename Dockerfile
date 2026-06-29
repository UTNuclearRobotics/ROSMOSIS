# ROSMOSIS experiment image: ROS 2 Humble + CUDA 12.4 (for nbv_cpp's TSDF/NBV
# kernels, built for sm_86 / RTX A6000) on Ubuntu 22.04.
#
# Self-assembling bootstrap model: clone ROSMOSIS once and checkout
# experiment-docker to get this Dockerfile, then every `docker build` re-clones
# the outer ROSMOSIS repo (src/ + the vendored pydubins zip + ros_entrypoint.sh)
# at experiment-docker, plus the four standalone dependency repos at their pinned
# branches -- all via a BuildKit SSH mount. Nothing is COPY'd from the host, so
# the build always reflects the latest *pushed* state, never a stale/dirty server
# working tree. (Changing the Dockerfile itself still needs a `git pull` of the
# bootstrap clone; src/param changes are picked up by the re-clone alone.)
#
#   eval "$(ssh-agent -s)" && ssh-add ~/.ssh/id_ed25519
#   docker build --ssh default --build-arg CACHEBUST=$(date +%s) -t rosmosis:test0 .
#
# CACHEBUST is referenced in the clone step below, so changing it (the $(date)
# gives a fresh value every run) forces a re-clone to pick up new pushes. Leave
# it out and Docker reuses the cached clone layer -> stale source.
#
# Pinned branches:
#   ROSMOSIS             -> experiment-docker
#   nbv_cpp              -> feature/alpha-server (CMAKE_CUDA_ARCHITECTURES 75 86 89)
#   perception_open3d    -> ros2 (provides open3d_conversions, needed by nbv_cpp)
#   sample_nbv_behaviors -> rosmosis
#   nrg_behaviors        -> main

FROM nvidia/cuda:12.4.1-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8

# ---- ROS 2 Humble + git/ssh (apt) ----
# git + openssh-client are needed for the in-build clones (the CUDA base image
# ships neither).
RUN apt-get update && apt-get install -y --no-install-recommends \
        locales curl gnupg2 lsb-release \
    && locale-gen en_US.UTF-8 \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rosbag2-storage-mcap \
        python3-rosdep \
        python3-colcon-common-extensions \
        build-essential \
        cmake \
        python3-dev \
        python3-pip \
        unzip \
        git \
        openssh-client \
    && rosdep init \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ---- pip-only deps rosdep cannot resolve ----
# python3-open3d is not a real apt package (rosdep "resolves" the key but apt
# can't install it) -- the Python side needs the pip wheel. The C++ side
# (nbv_cpp, open3d_conversions) is unrelated and handled below via the real
# libopen3d-dev apt package, through rosdep.
#
# numpy<2 is pinned explicitly and FIRST: an unpinned `pip install open3d`
# pulls numpy 2.x transitively, which breaks every other Python node in the
# workspace at runtime -- ROS's apt-provided tf_transformations/transforms3d
# uses a numpy 1.x-only API (np.maximum_sctype, removed in 2.0), and open3d's
# own transitive deps (sklearn, pandas, bottleneck) are pulled in as numpy
# 1.x-ABI wheels that segfault/AttributeError under numpy 2.x ("_ARRAY_API not
# found"). Pinning here forces pip to resolve open3d against 1.x instead.
RUN pip install --no-cache-dir "numpy<2" \
    && pip install --no-cache-dir open3d

# ---- clone workspace (outer ROSMOSIS + the four nested repos) ----
# Cloned fresh from remote so a param push is picked up without staging repos on
# the host. Private repos -> credentials come from the BuildKit SSH mount; the
# key is never written into a layer. $CACHEBUST is referenced (echo) so bumping
# it actually busts this layer -- otherwise Docker would reuse the stale clone.
WORKDIR /workspace
ARG CACHEBUST=0
RUN --mount=type=ssh \
        echo "cachebust=$CACHEBUST" \
    && mkdir -p -m 0700 ~/.ssh \
    && ssh-keyscan github.com >> ~/.ssh/known_hosts 2>/dev/null \
    && git clone --depth 1 -b experiment-docker \
        git@github.com:UTNuclearRobotics/ROSMOSIS.git /workspace \
    && cd /workspace/src \
    && git clone --depth 1 -b feature/alpha-server \
        git@github.com:UTNuclearRobotics/nbv_cpp.git \
    && git clone --depth 1 -b ros2 \
        git@github.com:alexnavtt/perception_open3d.git \
    && git clone --depth 1 -b rosmosis \
        git@github.com:UTNuclearRobotics/sample_nbv_behaviors.git \
    && git clone --depth 1 -b main \
        git@github.com:UTNuclearRobotics/nrg_behaviors.git \
    && rm -rf /workspace/.git /workspace/src/*/.git

# ---- pydubins (patched fork, vendored zip pulled in by the clone above) ----
RUN cd /workspace && unzip -q pydubins-master.zip \
    && pip install --no-cache-dir ./pydubins-master \
    && rm -rf pydubins-master pydubins-master.zip

# ---- ROS dependency resolution ----
# libopen3d-dev resolves to a real apt package (jammy/universe) and is picked up
# here for nbv_cpp's find_package(Open3D REQUIRED). Only pydubins and
# python3-open3d are skipped (handled by pip above).
#
# apt-get update is required: the ROS layer above ends with
# `rm -rf /var/lib/apt/lists/*`, so the index is empty here. rosdep shells out to
# `apt-get install` but does NOT refresh the index itself -- without this update,
# packages like ros-humble-pcl-ros (pulled in by nrg_behaviors /
# sample_nbv_behaviors) fail with "Unable to locate package".
RUN apt-get update && rosdep update && rosdep install --from-paths src --ignore-src -y \
        --rosdistro humble \
        --skip-keys "pydubins python3-open3d" \
    && rm -rf /var/lib/apt/lists/*

# ---- build (Release required for nbv_cpp's TSDF/ray-cast performance) ----
# Docker's default RUN shell is /bin/sh (dash on Ubuntu); ROS's setup.bash uses
# bash-only syntax and fails under dash with "Bad substitution". Switch to bash
# for the source-and-build step.
SHELL ["/bin/bash", "-c"]
RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---- entrypoint (sources underlay + overlay, then exec's the CMD) ----
RUN cp /workspace/ros_entrypoint.sh / && chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
