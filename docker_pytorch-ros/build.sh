#!/usr/bin/env bash

# Get dependent parameters
source "$(dirname "$(readlink -f "${0}")")/get_param.sh"

# Build docker images
docker build -t "${docker_hub_user}"/"${IMAGE}" \
    --build-arg USER="${user}" \
    --build-arg UID="${uid}" \
    --build-arg GROUP="${group}" \
    --build-arg GID="${gid}" \
    --build-arg HARDWARE="${hardware}" \
    --build-arg ENTRYPOINT_FILE="${ENTRYPOINT_FILE}" \
    -f "${FILE_DIR}"/"${DOCKERFILE_NAME}" "${FILE_DIR}"

#     --progress=plain \
#     --no-cache \

