#!/usr/bin/env bash
# 兼容入口：OctoMap周期reset实现位于 apps/perception。
set -eo pipefail

AIRY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec "${AIRY_ROOT}/localmap/apps/perception/reset_octomap_periodically.sh" "$@"
