#!/usr/bin/env bash
#
# Build the ROSMOSIS Docker image from the bootstrap clone on the server.
# Always forces a fresh re-clone (CACHEBUST) so the latest pushed commits
# are picked up.
#
# Usage: TAG=cone ./build_image.sh
#        TAG=helix ./build_image.sh
#
set -euo pipefail

TAG="${TAG:?set TAG to your image tag, e.g. TAG=cone ./build_image.sh}"

eval "$(ssh-agent -s)" && ssh-add ~/.ssh/id_ed25519
docker build --ssh default --build-arg CACHEBUST=$(date +%s) -t "rosmosis:${TAG}" .
