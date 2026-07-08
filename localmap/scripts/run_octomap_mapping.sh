#!/usr/bin/env bash
# 兼容入口：OctoMap实时建图实现位于 apps/perception。
set -eo pipefail

AIRY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec "${AIRY_ROOT}/localmap/apps/perception/run_octomap_mapping.sh" "$@"
