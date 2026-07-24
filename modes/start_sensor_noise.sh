#!/usr/bin/env bash
ROOT="$(cd -- "$(dirname -- "$0")/.." && pwd)"
exec "${ROOT}/start_sim.sh" experiment sensor_noise --rviz "$@"
