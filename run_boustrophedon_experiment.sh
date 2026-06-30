#!/usr/bin/env bash
#
# Boustrophedon baseline on env_50x50_cluster_seabed: a single CPU-only run
# (no --gpus -- the lawnmower pattern has no NBV/TSDF CUDA), recording a bag
# named boustrophedon_50x50_cluster_seabed_<timestamp>.
#
# Mirrors run_CINBV_experiment.sh (same env, data dir, naming) so it can run
# alongside that sweep if needed: it uses ROS_DOMAIN_ID 26, one past the sweep's
# 21-25, so the DDS graphs don't cross-talk.
#
# Log: data/<prefix>_<stamp>.log  (also streamed to the terminal via tee)
#
set -euo pipefail

# ---- config ----
IMAGE="${IMAGE:?set IMAGE to your built image tag, e.g. IMAGE=rosmosis:test0}"
ENVIRONMENT="${ENVIRONMENT:-env_50x50_cluster_seabed}"
DATA_DIR="${DATA_DIR:-$HOME/ROSMOSIS/data}"
DOMAIN_ID="${DOMAIN_ID:-26}"         # one past the NBV sweep's 21-25

# ---- preflight ----
command -v docker >/dev/null || { echo "docker not found" >&2; exit 1; }
mkdir -p "$DATA_DIR"
LOG_DIR="$DATA_DIR/run_logs"
mkdir -p "$LOG_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"

PREFIX="boustrophedon_50x50_cluster_seabed"
LOGF="${LOG_DIR}/${PREFIX}_${STAMP}.log"

echo "Image:        $IMAGE"
echo "Environment:  $ENVIRONMENT"
echo "Data dir:     $DATA_DIR"
echo "ROS_DOMAIN:   $DOMAIN_ID  (CPU-only, no GPU)"
echo "Bag prefix:   $PREFIX"
echo "Log:          $LOGF"
echo

# single CPU run, foreground; tee shows it live and saves the log.
docker run --rm \
    -e ROS_DOMAIN_ID="${DOMAIN_ID}" \
    -v "${DATA_DIR}:/workspace/data" \
    "${IMAGE}" \
    ros2 launch baseline_mission baseline_mission_launch.py \
        start_rviz:=false debug_gui:=false record:=true \
        environment:="${ENVIRONMENT}" \
        bag_prefix:="${PREFIX}" \
    2>&1 | tee "${LOGF}"

echo
echo "Boustrophedon run complete. Bag + log under ${DATA_DIR}"
