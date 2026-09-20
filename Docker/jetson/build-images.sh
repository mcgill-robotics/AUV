#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ISAAC_ROS_DIR="${ISAAC_ROS_DIR:-${ROOT_DIR}/.isaac-ros-cli}"
ISAAC_BASE_IMAGE="${ISAAC_BASE_IMAGE:-mcgillrobotics/isaac-ros-base:r39.2}"
PERSONAL_BASE_IMAGE="${PERSONAL_BASE_IMAGE:-mcgillrobotics/auv_2026:isaac-ros-base}"
APPLICATION_IMAGE="${APPLICATION_IMAGE:-mcgillrobotics/auv_2026:latest-jetson}"

if [[ ! -d "${ISAAC_ROS_DIR}/.git" ]]; then
    git clone --depth 1 https://github.com/NVIDIA-ISAAC-ROS/isaac-ros-cli.git "${ISAAC_ROS_DIR}"
fi

docker build \
    --build-arg PLATFORM=arm64 \
    --build-arg ISAAC_ROS_PLATFORM=arm64 \
    -f "${ISAAC_ROS_DIR}/docker/Dockerfile.isaac_ros" \
    -t "${ISAAC_BASE_IMAGE}" \
    "${ISAAC_ROS_DIR}"

docker build \
    --build-arg BASE_IMAGE="${ISAAC_BASE_IMAGE}" \
    -f "${ROOT_DIR}/Docker/jetson/Dockerfile.base" \
    -t "${PERSONAL_BASE_IMAGE}" \
    "${ROOT_DIR}"

docker build \
    --build-arg BASE_IMAGE="${PERSONAL_BASE_IMAGE}" \
    -f "${ROOT_DIR}/Docker/jetson/Dockerfile" \
    -t "${APPLICATION_IMAGE}" \
    "${ROOT_DIR}"