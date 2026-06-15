#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
target_dir="${repo_root}/.external_worlds/aws-robomaker-small-house-world"

mkdir -p "${repo_root}/.external_worlds"

if [[ -d "${target_dir}/.git" ]]; then
  git -C "${target_dir}" fetch --depth 1 origin ros1
  git -C "${target_dir}" checkout ros1
else
  git clone --depth 1 --branch ros1 \
    https://github.com/aws-robotics/aws-robomaker-small-house-world.git \
    "${target_dir}"
fi

echo "AWS RoboMaker Small House World is available at ${target_dir}"
