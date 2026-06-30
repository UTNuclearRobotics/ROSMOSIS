#!/usr/bin/env bash
#
# CI-NBV alpha sweep on env_50x50_cluster_seabed: one NBV mission per alpha in
# {0, 0.25, 0.5, 0.75, 1.0}, all launched at once, each recording a bag named
# nbv_cone_alpha<val>_50x50_cluster_seabed_<timestamp>.
#
# GPUs (3x A6000): missions are pinned one-per-GPU, round-robined. With 5 on 3,
# GPU 0 and 1 each carry 2 (they time-share); GPU 2 carries 1. Each gets a
# distinct ROS_DOMAIN_ID so the DDS graphs don't cross-talk.
#
# Per-run logs: data/run_logs/<prefix>_<stamp>.log
#
set -euo pipefail

# ---- config ----
IMAGE="${IMAGE:?set IMAGE to your built image tag, e.g. IMAGE=rosmosis:test0}"
ENVIRONMENT="${ENVIRONMENT:-env_50x50_cluster_seabed}"
DATA_DIR="${DATA_DIR:-$HOME/ROSMOSIS/data}"
NUM_GPUS=3                           # A6000 count
DOMAIN_BASE="${DOMAIN_BASE:-21}"     # first ROS_DOMAIN_ID (>=20)

ALPHAS=(0 0.25 0.5 0.75 1.0)

# ---- preflight ----
command -v docker >/dev/null || { echo "docker not found" >&2; exit 1; }
mkdir -p "$DATA_DIR"
LOG_DIR="$DATA_DIR/run_logs"
mkdir -p "$LOG_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"

echo "Image:        $IMAGE"
echo "Environment:  $ENVIRONMENT"
echo "Data dir:     $DATA_DIR"
echo "GPUs:         $NUM_GPUS  (5 missions launched at once)"
echo "Alphas:       ${ALPHAS[*]}"
echo "Batch stamp:  $STAMP"
echo

# launch one mission, detached.
launch_run() {
    local alpha="$1" gpu="$2" domain="$3"
    local prefix="nbv_cone_alpha${alpha}_50x50_cluster_seabed"
    local logf="${LOG_DIR}/${prefix}_${STAMP}.log"

    echo ">> alpha=${alpha}  GPU=${gpu}  ROS_DOMAIN_ID=${domain}"
    echo "   bag prefix: ${prefix}"
    echo "   log:        ${logf}"

    docker run --rm \
        --gpus "\"device=${gpu}\"" \
        -e ROS_DOMAIN_ID="${domain}" \
        -v "${DATA_DIR}:/workspace/data" \
        "${IMAGE}" \
        ros2 launch demo_behaviors demo_mission_launch.py \
            start_rviz:=false debug_gui:=false record:=true \
            environment:="${ENVIRONMENT}" \
            alpha:="${alpha}" \
            bag_prefix:="${prefix}" \
        >"${logf}" 2>&1 &
}

# ---- launch all at once, round-robin across GPUs ----
i=0
for alpha in "${ALPHAS[@]}"; do
    gpu=$(( i % NUM_GPUS ))
    domain=$(( DOMAIN_BASE + i ))
    launch_run "${alpha}" "${gpu}" "${domain}"
    i=$(( i + 1 ))
done

echo
echo "-- all ${#ALPHAS[@]} missions launched; waiting for completion --"
wait
echo
echo "All ${#ALPHAS[@]} runs complete. Bags + logs under ${DATA_DIR}"
