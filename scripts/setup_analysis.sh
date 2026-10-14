#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
tools_dir=/workspace/.go2-research-tools
cd "$repo_dir"
command -v git >/dev/null
command -v python3 >/dev/null
command -v g++ >/dev/null
command -v make >/dev/null
# No --remote: preserve the pinned upstream commits. Never reset dirty sources.
git submodule update --init --recursive
python3 scripts/validate_sources.py
if [[ ! -x "$tools_dir/bin/python" ]]; then
  python3 -m venv "$tools_dir"
fi
"$tools_dir/bin/python" -m pip install --no-cache-dir --disable-pip-version-check cmake==3.31.6
"$tools_dir/bin/cmake" -S references/unitree_actuator_open_sdk -B build/open-sdk \
  -DCMAKE_BUILD_TYPE=Release -DUNITREE_BUILD_EXAMPLES=OFF -DUNITREE_BUILD_PYTHON=OFF
"$tools_dir/bin/cmake" --build build/open-sdk --target unitree_actuator_open_sdk_core --parallel 2
g++ -std=c++17 -Wall -Wextra -I references/unitree_actuator_open_sdk/include \
  scripts/offline_frame_check.cpp build/open-sdk/libunitree_actuator_open_sdk_core.a \
  -o build/offline_frame_check
./build/offline_frame_check
python3 scripts/validate_sources.py
