#!/usr/bin/env bash
# 兼容入口：OctoMap快照保存实现位于 apps/data_tools。
set -eo pipefail

AIRY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec "${AIRY_ROOT}/localmap/apps/data_tools/save_octomap_snapshot.sh" "$@"
