#!/usr/bin/env bash
# Run isaacdrones on Georgia Tech PACE ICE. See docs/PACE.md.
#
#   scripts/pace.sh setup               pull the image, install uv, sync the venv
#   scripts/pace.sh node                shell on a CPU node, enough for setup
#   scripts/pace.sh gpu [CARD]          shell on a GPU node
#   scripts/pace.sh batch [CARD] CMD... queue CMD on a GPU node; log in scratch
#   scripts/pace.sh run CMD...          run CMD inside the container, on this node
#   scripts/pace.sh shell               shell inside the container, on this node
#   scripts/pace.sh cards               the card types and what is free right now
#   scripts/pace.sh stream [CARD] [ARGS] isaacdrones in the GUI, streamed to your computer
#   scripts/pace.sh space               quota, and the size of data and runs
#   scripts/pace.sh clean runs|data|build   delete one of them, after asking
#
# CARD is blackwell (default), l40s, a40 or h100. See card_request below.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Everything big lives here: the build, and the user's data/ and runs/.
ROOT="${ISAACDRONES_SCRATCH:-$HOME/scratch/isaacdrones}"
IMAGE_URI="docker://nvcr.io/nvidia/isaac-sim:6.2.0"

CARDS="blackwell l40s a40 h100"

# ICE GPU types: card name -> partition and Slurm feature, from sinfo on
# 2026-11-14. The partition must be named. Otherwise ICE adds partitions with
# no matching node and the job stays at BadConstraints. An h100 has no RT
# cores, so Isaac Sim is unsupported on it, though our tests pass there.
card_request() {
    EXCLUDE=
    case "$1" in
    blackwell) PARTITION=ice-bw-gpu FEATURE=gpu-rtxpro-blackwell ;;
    l40s) PARTITION=ice-gpu FEATURE=gpu-l40s ;;
    a40) PARTITION=ice-gpu FEATURE=gpu-a40 ;;
    h100)
        PARTITION=coe-gpu FEATURE=gpu-h100
        # One card on each of these nodes crashed Isaac Sim at startup on
        # every run on 2026-11-14 (ERROR_DEVICE_LOST), and Slurm returns the
        # same card. PACE may repair them, so retry without this line
        # occasionally. See "H100s and RT cores" in docs/PACE.md.
        EXCLUDE=atl1-1-03-010-10-0,atl1-1-03-010-25-0
        ;;
    *) echo "Unknown card '$1'. Choose from: $CARDS" >&2; exit 1 ;;
    esac
}

is_card() {
    case " $CARDS " in *" $1 "*) return 0 ;; *) return 1 ;; esac
}

make_dirs() {
    mkdir -p "$ROOT"
    # ~/scratch is a symlink. The build uses the resolved path; the path as
    # typed is kept so that it also works inside the container.
    [ -n "${ROOT_TYPED:-}" ] || ROOT_TYPED="$ROOT"
    ROOT="$(cd "$ROOT" && pwd -P)"
    IMAGE="$ROOT/images/isaac-sim-6.2.0.sif"
    mkdir -p "$ROOT"/{images,apptainer-cache,apptainer-tmp,uv/bin,uv/cache,uv/python,home,logs,data,runs}
    export APPTAINER_CACHEDIR="$ROOT/apptainer-cache"
    export APPTAINER_TMPDIR="${TMPDIR:-$ROOT/apptainer-tmp}"
}

# Run a command inside the image, with the repo at /workspace/isaacdrones and
# scratch at its usual path. --cleanenv drops the host environment, so the
# container's variables are passed as APPTAINERENV_.
in_container() {
    make_dirs
    [ -f "$IMAGE" ] || { echo "No image at $IMAGE. Run: scripts/pace.sh setup" >&2; exit 1; }

    local nv=""
    if command -v nvidia-smi >/dev/null; then
        nv="--nv"
        export APPTAINERENV_CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
        # --nv omits the driver's NGX library, which the renderer needs for
        # DLSS denoising. Without it camera images are grainy.
        local ngx
        ngx="$(ls /usr/lib64/libnvidia-ngx.so.*.* 2>/dev/null | head -1 || true)"
        if [ -n "$ngx" ]; then
            nv="$nv --bind $ngx:/usr/local/lib/libnvidia-ngx.so.1"
        fi
    fi

    export APPTAINERENV_PREPEND_PATH="$ROOT/uv/bin"
    export APPTAINERENV_UV_CACHE_DIR="$ROOT/uv/cache"
    export APPTAINERENV_UV_PYTHON_INSTALL_DIR="$ROOT/uv/python"
    export APPTAINERENV_UV_PROJECT_ENVIRONMENT="$ROOT/venv"
    export APPTAINERENV_UV_HTTP_TIMEOUT=600
    export APPTAINERENV_ACCEPT_EULA=Y
    export APPTAINERENV_PRIVACY_CONSENT=Y
    export APPTAINERENV_OMNI_KIT_ACCEPT_EULA=YES
    # Hub is Omniverse's asset cache. The image points Isaac Sim at a Hub
    # that cannot start here, costing ten seconds of retries per launch, so
    # it is off. To enable: OMNICLIENT_HUB_MODE=shared scripts/pace.sh run ...
    export APPTAINERENV_OMNICLIENT_HUB_MODE="${OMNICLIENT_HUB_MODE:-disabled}"

    apptainer exec $nv --cleanenv \
        --home "$ROOT/home:/home/isaacdrones" \
        --bind "$ROOT" \
        --bind "$ROOT:$ROOT_TYPED" \
        --bind "$REPO:/workspace/isaacdrones" \
        --pwd /workspace/isaacdrones \
        "$IMAGE" "$@"
}

setup() {
    make_dirs
    if [ ! -f "$IMAGE" ]; then
        apptainer pull "$IMAGE" "$IMAGE_URI"
    fi
    if [ ! -x "$ROOT/uv/bin/uv" ]; then
        curl -LsSf https://astral.sh/uv/install.sh |
            env UV_INSTALL_DIR="$ROOT/uv/bin" UV_NO_MODIFY_PATH=1 sh
    fi
    in_container uv sync --extra sim
    echo "Setup done. Everything is under $ROOT"
}

case "${1:-}" in
node)
    exec srun --cpus-per-task=8 --mem=32G --time=02:00:00 --pty bash
    ;;
gpu)
    card_request "${2:-blackwell}"
    exec srun --partition="$PARTITION" --constraint="$FEATURE" --gres=gpu:1 \
        ${EXCLUDE:+--exclude="$EXCLUDE"} --cpus-per-task=8 --mem=64G --time=04:00:00 --pty bash
    ;;
batch)
    # A batch job waits in the queue without a terminal.
    shift
    card=blackwell
    if [ $# -gt 0 ] && is_card "$1"; then
        card="$1"
        shift
    fi
    [ $# -gt 0 ] || { echo "Nothing to run. Usage: scripts/pace.sh batch [CARD] CMD..." >&2; exit 1; }
    card_request "$card"
    make_dirs
    # Reject a mistyped card or command before it waits in the queue.
    if ! PATH="$ROOT/uv/bin:$ROOT/venv/bin:$PATH" command -v "$1" >/dev/null && [ ! -x "$REPO/$1" ]; then
        echo "'$1' is not a card ($CARDS) or a command. Nothing submitted." >&2
        exit 1
    fi
    job="$(sbatch --parsable --job-name="isaacdrones-$card" --gres=gpu:1 \
        --partition="$PARTITION" --constraint="$FEATURE" \
        ${EXCLUDE:+--exclude="$EXCLUDE"} --cpus-per-task=8 --mem=64G --time=04:00:00 \
        --chdir="$REPO" --output="$ROOT/logs/%j.log" \
        --wrap="scripts/pace.sh run $(printf '%q ' "$@")")"
    job="${job%%;*}" # --parsable can print "id;cluster"
    echo "Submitted job $job on $card. Check the queue with: squeue --me"
    echo "tail -n +1 -f $ROOT/logs/$job.log"
    ;;
stream)
    # isaacdrones with Isaac Sim's GUI on a GPU node, streamed over WebRTC. The
    # node also runs a relay that carries the video through an SSH tunnel
    # (docs/PACE.md, "The GUI on your own computer"). ARGS go to isaacdrones:
    # scripts/pace.sh stream --scene falling-cube
    shift
    card=blackwell
    if [ $# -gt 0 ] && is_card "$1"; then
        card="$1"
        shift
    fi
    if [ "$card" = h100 ]; then
        echo "An H100 has no video encoder, so it cannot stream. Use an RTX card." >&2
        exit 1
    fi
    card_request "$card"
    make_dirs
    job="$(sbatch --parsable --job-name="isaacdrones-stream" --gres=gpu:1 \
        --partition="$PARTITION" --constraint="$FEATURE" \
        --cpus-per-task=8 --mem=64G --time=02:00:00 \
        --chdir="$REPO" --output="$ROOT/logs/%j.log" \
        --wrap="python3 scripts/stream_relay.py node & scripts/pace.sh run uv run isaacdrones --stream $(printf '%q ' "$@")")"
    job="${job%%;*}"
    log="$ROOT/logs/$job.log"
    echo "Submitted job $job on $card. It runs for up to two hours."
    echo
    echo "Keep this open. It is waiting for the stream to be ready, and will then"
    echo "print the one command to run on your own computer to see it. That takes"
    echo "however long a card takes to come free, plus three to six minutes for"
    echo "Isaac Sim to load. Nothing is being installed in that time: the full GUI"
    echo "is a few hundred extensions, read from network storage. See docs/PACE.md."
    echo
    echo "Ctrl-C here only stops this waiting. The job carries on, and you would"
    echo "then have to find the node yourself with: squeue --me"
    echo
    # Wait, so the command to run at home can name the node.
    started=""
    while ! grep -q "isaacdrones: stream ready" "$log" 2>/dev/null; do
        state="$(squeue -h -j "$job" -o %T 2>/dev/null || true)"
        case "$state" in
        "") echo; echo "The job ended before the stream was ready. See $log" >&2; exit 1 ;;
        RUNNING)
            [ -n "$started" ] || started=$SECONDS
            status="Isaac Sim is loading, $(((SECONDS - started) / 60)) min so far"
            ;;
        *) status="waiting in the queue for a free card, $((SECONDS / 60)) min so far" ;;
        esac
        printf '\r  %s   ' "$status"
        sleep 10
    done
    node="$(squeue -h -j "$job" -o %N)"
    echo
    echo "Ready on $node. On your own computer, in the isaacdrones folder, run:"
    echo
    echo "  python3 scripts/stream_relay.py connect $USER $node"
    echo
    echo "and follow what it says. When you are done, stop the job: scancel $job"
    ;;
cards)
    # Node counts by state, from Slurm. An idle or mixed node may have a free
    # card; an allocated node has none.
    for card in $CARDS; do
        card_request "$card"
        states="$(sinfo -h -N -p "$PARTITION" -o "%N %T %f" | grep -w "$FEATURE" |
            awk '{print $2}' | sort | uniq -c | awk '{printf "%s %s  ", $1, $2}')"
        printf '%-10s %-11s %-21s %s\n' "$card" "$PARTITION" "$FEATURE" "$states"
    done
    ;;
space)
    make_dirs
    pace-quota 2>/dev/null | grep -E "Usage|Home|Scratch" || true
    du -sh "$ROOT/data" "$ROOT/runs"
    echo "files: data $(find "$ROOT/data" -type f | wc -l), runs $(find "$ROOT/runs" -type f | wc -l)"
    ;;
clean)
    make_dirs
    case "${2:-}" in
    runs | data) targets=("$ROOT/$2") ;;
    # Everything setup makes. data/ and runs/ are left alone.
    build) targets=("$ROOT"/{images,apptainer-cache,apptainer-tmp,uv,venv,home,logs}) ;;
    *) echo "Usage: scripts/pace.sh clean runs|data|build" >&2; exit 1 ;;
    esac
    printf '%s\n' "${targets[@]}"
    read -r -p "Delete these? This cannot be undone. [y/N] " answer
    [ "$answer" = y ] || { echo "Nothing deleted."; exit 0; }
    rm -rf "${targets[@]}"
    [ "$2" = build ] && echo "Deleted. Rebuild with: scripts/pace.sh setup" || mkdir -p "$ROOT/$2"
    ;;
setup)
    setup
    ;;
run)
    shift
    in_container "$@"
    ;;
shell)
    in_container bash
    ;;
*)
    sed -n '2,15p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
