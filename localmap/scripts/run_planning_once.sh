#!/usr/bin/env bash
# 兼容入口：一次性规划流程实现位于 apps/planning。
set -eo pipefail

AIRY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec "${AIRY_ROOT}/localmap/apps/planning/run_planning_once.sh" "$@"
