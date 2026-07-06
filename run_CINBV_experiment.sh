#!/usr/bin/env bash
#
# CI-NBV alpha sweep on env_1000x1000_cluster_seabed: one NBV mission per alpha in
# {0, 0.25, 0.5, 0.75, 1.0}, recording a bag named
# nbv_<sampler>_alpha<val>_fullscale_<timestamp>.
#
# SAMPLER (=cone|helix) is REQUIRED -- it labels the bag. The actual viewpoint
# sampler is baked into the image's BT (nbv_on_target.xml), so SAMPLER must match
# the IMAGE you pass. Example:
#   IMAGE=rosmosis:cone  SAMPLER=cone  ./run_CINBV_experiment.sh
#   IMAGE=rosmosis:helix SAMPLER=helix ./run_CINBV_experiment.sh
#
# Runs in BATCHES of MAX_PARALLEL (default 3), one mission per GPU, with a
# startup STAGGER between launches. This is the "1-per-GPU" mode: no two missions
# ever share a card (MAX_PARALLEL must be <= NUM_GPUS), and the stagger spaces out
# node startup to avoid the concurrent-launch race that crashed the vehicle sim.
#   Batch 1: alpha 0, 0.25, 0.5  -> GPU 0, 1, 2
#   Batch 2: alpha 0.75, 1.0     -> GPU 0, 1
# Set MAX_PARALLEL=1 for fully sequential, or =5 to force all-at-once (shares GPUs).
#
# Per-run logs: data/run_logs/<prefix>_<stamp>.log
#
set -euo pipefail

# ---- config ----
IMAGE="${IMAGE:?set IMAGE to your built image tag, e.g. IMAGE=rosmosis:test0}"
SAMPLER="${SAMPLER:?set SAMPLER=cone or SAMPLER=helix (bag label; MUST match the sampler baked into IMAGE)}"
ENVIRONMENT="${ENVIRONMENT:-env_1000x1000_cluster_seabed}"
DATA_DIR="${DATA_DIR:-$HOME/ROSMOSIS/data}"
NUM_GPUS=3                           # A6000 count
MAX_PARALLEL="${MAX_PARALLEL:-3}"    # concurrent missions; must be <= NUM_GPUS (1 per GPU)
STAGGER="${STAGGER:-15}"             # seconds between launches within a batch
DOMAIN_BASE="${DOMAIN_BASE:-21}"     # first ROS_DOMAIN_ID (>=20)

# ---- per-mission resource caps (128-core host) ----
CORES_PER="${CORES_PER:-32}"         # dedicated cores per mission; universal bound (caps TBB raycast + OMP)
OMP_THREADS="${OMP_THREADS:-10}"     # OpenMP pool for Open3D TSDF integrate/mesh-extract (GPU scoring is separate)
MEM_LIMIT="${MEM_LIMIT:-16g}"        # RAM ceiling; mission uses ~9g, this is just a runaway backstop

# ---- live progress ----
M_TARGETS="${M_TARGETS:-10}"         # targets in the scene (box_count); denominator for the mapped bar
PROGRESS_REFRESH="${PROGRESS_REFRESH:-30}"  # seconds between progress prints while a batch runs

ALPHAS=(0 0.25 0.5 0.75 1.0)

# ---- preflight ----
command -v docker >/dev/null || { echo "docker not found" >&2; exit 1; }
[[ "$SAMPLER" == "cone" || "$SAMPLER" == "helix" ]] || { echo "SAMPLER must be 'cone' or 'helix', got '$SAMPLER'" >&2; exit 1; }
(( MAX_PARALLEL <= NUM_GPUS )) || { echo "MAX_PARALLEL ($MAX_PARALLEL) must be <= NUM_GPUS ($NUM_GPUS) to keep one mission per GPU" >&2; exit 1; }
(( MAX_PARALLEL * CORES_PER <= 120 )) || { echo "MAX_PARALLEL*CORES_PER ($((MAX_PARALLEL*CORES_PER))) exceeds 120; leave >=8 cores for the host" >&2; exit 1; }
mkdir -p "$DATA_DIR"
LOG_DIR="$DATA_DIR/run_logs"
mkdir -p "$LOG_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"

echo "Image:        $IMAGE"
echo "Sampler:      $SAMPLER   (bag label -- must match the BT baked into the image)"
echo "Environment:  $ENVIRONMENT"
echo "Data dir:     $DATA_DIR"
echo "GPUs:         $NUM_GPUS   Max parallel: $MAX_PARALLEL   Stagger: ${STAGGER}s"
echo "Per mission:  ${CORES_PER} cores, OMP=${OMP_THREADS}, mem=${MEM_LIMIT}"
echo "Alphas:       ${ALPHAS[*]}"
echo "Batch stamp:  $STAMP"
echo

# in-flight batch tracking (PID + log per running mission), reset per batch.
BATCH_PIDS=(); BATCH_LOGS=()

# launch one mission, detached. Pinned to a dedicated core block (slot = gpu
# index within the batch) so concurrent missions never contend for cores.
launch_run() {
    local alpha="$1" gpu="$2" domain="$3"
    local prefix="nbv_${SAMPLER}_alpha${alpha}_fullscale"
    local logf="${LOG_DIR}/${prefix}_${STAMP}.log"
    local cpu_lo=$(( gpu * CORES_PER ))
    local cpu_hi=$(( cpu_lo + CORES_PER - 1 ))

    echo ">> alpha=${alpha}  GPU=${gpu}  cores=${cpu_lo}-${cpu_hi}  ROS_DOMAIN_ID=${domain}"
    echo "   bag prefix: ${prefix}"
    echo "   log:        ${logf}"

    docker run --rm \
        --gpus "\"device=${gpu}\"" \
        --cpuset-cpus="${cpu_lo}-${cpu_hi}" \
        -e OMP_NUM_THREADS="${OMP_THREADS}" \
        --memory="${MEM_LIMIT}" \
        -e ROS_DOMAIN_ID="${domain}" \
        -v "${DATA_DIR}:/workspace/data" \
        "${IMAGE}" \
        ros2 launch demo_behaviors demo_mission_launch.py \
            start_rviz:=false debug_gui:=false record:=true \
            environment:="${ENVIRONMENT}" \
            alpha:="${alpha}" \
            bag_prefix:="${prefix}" \
        >"${logf}" 2>&1 &
    BATCH_PIDS+=("$!"); BATCH_LOGS+=("${logf}")
}

# 10-wide progress bar: bar <done> <total>
bar() {
    local done="$1" total="$2" w=10 f i out=""
    (( total > 0 )) || total=1
    f=$(( done * w / total ))
    if (( f > w )); then f=w; fi
    for ((i=0; i<w; i++)); do if (( i < f )); then out+="#"; else out+="-"; fi; done
    printf "[%s]" "$out"
}

# one compact status line for a mission log: mapped bar + detected + phase.
# mapped = distinct target_N.ply saved (the goal); detected = ids ever queued.
# NB: `local X=$(...)` is deliberate -- it masks the grep exit status so an empty
# match (nothing mapped yet) can't trip `set -e` and kill the sweep.
mission_progress() {
    local log="$1"
    local label=$(basename "$log" | grep -oE "alpha[0-9.]+" || echo run)
    local mapped=$(grep -oE "target_[0-9]+\.ply" "$log" 2>/dev/null | grep -oE "[0-9]+" | sort -u | wc -l)
    local detected=$( { grep -oE "ids=\[[^]]*\]" "$log"; grep -oE "target_[0-9]+\.ply" "$log"; } 2>/dev/null \
                      | grep -oE "[0-9]+" | sort -u | wc -l )
    local phase=$(grep -E "Saved model|Saturation reached|Conclude policy RUNNING|Set views RUNNING|Next search pose" "$log" 2>/dev/null \
                  | tail -1 | sed -E 's/.*\]: //; s/ *$//')
    printf "   %-10s %s mapped %d/%d | detected %d/%d | %s\n" \
           "$label" "$(bar "$mapped" "$M_TARGETS")" "$mapped" "$M_TARGETS" "$detected" "$M_TARGETS" "${phase:-navigating}"
}

# wait for the in-flight batch, printing progress every PROGRESS_REFRESH sec.
monitored_wait() {
    (( ${#BATCH_PIDS[@]} )) || return 0
    while :; do
        local alive=0 pid
        for pid in "${BATCH_PIDS[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then alive=1; break; fi
        done
        echo "-- progress $(date +%H:%M:%S) --"
        for log in "${BATCH_LOGS[@]}"; do mission_progress "$log"; done
        (( alive )) || break
        sleep "$PROGRESS_REFRESH"
    done
    wait "${BATCH_PIDS[@]}" 2>/dev/null || true
    BATCH_PIDS=(); BATCH_LOGS=()
}

# ---- run in batches of MAX_PARALLEL, one GPU per mission, staggered starts ----
n=${#ALPHAS[@]}
i=0
for alpha in "${ALPHAS[@]}"; do
    gpu=$(( i % MAX_PARALLEL ))       # distinct within a batch since MAX_PARALLEL <= NUM_GPUS
    domain=$(( DOMAIN_BASE + i ))
    launch_run "${alpha}" "${gpu}" "${domain}"
    i=$(( i + 1 ))

    if (( i % MAX_PARALLEL == 0 )); then
        echo "-- batch full (${MAX_PARALLEL} in flight); monitoring until it finishes --"
        monitored_wait
        echo
    elif (( i < n )); then
        sleep "${STAGGER}"            # space out startups to avoid the launch race
    fi
done

monitored_wait                       # drain any trailing partial batch
echo
echo "All ${n} runs complete. Bags + logs under ${DATA_DIR}"
