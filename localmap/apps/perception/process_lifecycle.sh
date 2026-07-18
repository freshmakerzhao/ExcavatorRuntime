#!/usr/bin/env bash

# Children must be session leaders. Stop their whole process groups so wrappers cannot
# leave ROS grandchildren behind.
terminate_processes() {
  local grace_s="$1"
  shift
  local -a child_pids=("$@")
  local deadline_s=$((SECONDS + grace_s))
  local pid
  local any_alive

  for pid in "${child_pids[@]}"; do
    kill -TERM -- "-${pid}" >/dev/null 2>&1 || true
  done

  while ((SECONDS < deadline_s)); do
    any_alive=0
    for pid in "${child_pids[@]}"; do
      if kill -0 -- "-${pid}" >/dev/null 2>&1; then
        any_alive=1
        break
      fi
    done
    if ((any_alive == 0)); then
      break
    fi
    sleep 0.1
  done

  for pid in "${child_pids[@]}"; do
    if kill -0 -- "-${pid}" >/dev/null 2>&1; then
      kill -KILL -- "-${pid}" >/dev/null 2>&1 || true
    fi
  done
  for pid in "${child_pids[@]}"; do
    wait "${pid}" >/dev/null 2>&1 || true
  done
}
