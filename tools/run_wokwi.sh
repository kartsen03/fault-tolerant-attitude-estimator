#!/usr/bin/env bash
# Run the RP2040 firmware in the Wokwi simulator (Pico + MPU6050) with an
# automation scenario, saving the serial output.
#
# Needs a Wokwi CI token, from WOKWI_CLI_TOKEN or the file ~/.wokwi_token
# (the token is never printed). Free accounts get 50 simulation minutes per
# 30 days; one smoke run uses well under a minute.
#
# Usage: tools/run_wokwi.sh [scenario.yaml] [serial-log]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENARIO="${1:-${ROOT}/wokwi/scenarios/smoke_tilt.yaml}"
LOG="${2:-${ROOT}/results/tmp/wokwi_serial.log}"
WOKWI_CLI="${WOKWI_CLI:-$(command -v wokwi-cli || echo "${HOME}/.local/bin/wokwi-cli")}"

if [ -z "${WOKWI_CLI_TOKEN:-}" ]; then
  if [ ! -s "${HOME}/.wokwi_token" ]; then
    echo "No Wokwi token: set WOKWI_CLI_TOKEN or create ~/.wokwi_token" >&2
    exit 2
  fi
  WOKWI_CLI_TOKEN="$(cat "${HOME}/.wokwi_token")"
  export WOKWI_CLI_TOKEN
fi

if [ ! -f "${ROOT}/build/rp2040/ftae_fw.uf2" ]; then
  echo "Firmware not built: run tools/build_firmware.sh first" >&2
  exit 2
fi

mkdir -p "$(dirname "${LOG}")"
"${WOKWI_CLI}" --timeout 60000 --scenario "${SCENARIO}" --serial-log-file "${LOG}" "${ROOT}/wokwi"
