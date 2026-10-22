#!/usr/bin/env bash
# This workstation's rocket workflow. Only hover/fly accept --execute.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
source scripts/local_env.sh
PLAN="$ROOT/output/rocket-2p75m-prepared/target-plan.json"
if [[ ! -f "$PLAN" ]]; then
  PLAN="$ROOT/experiments/rocket-2p75m-20261022T015434Z/target-plan.json"
fi
ACTION="${1:-review}"
shift "$(( $# > 0 ? 1 : 0 ))"
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != --execute ) ]]; then
  echo 'Usage: bash scripts/rocket.sh {server|check|hover|review|fly|land} [--execute]' >&2
  exit 2
fi
if [[ $# -eq 1 && "$ACTION" != hover && "$ACTION" != fly ]]; then
  echo '--execute is only valid for hover or fly.' >&2
  exit 2
fi
case "$ACTION" in
  server) exec bash scripts/start_ground_checks.sh --vicon-only ;;
  check)
    # Terminal A must be running, with the aircraft grounded at XY [-1.375,0].
    pdg mission --config config/rocket-lab.yaml --plan-file "$PLAN" --output output/rocket-setup-review
    python scripts/check_hardware.py --vicon-only --controller-id 1
    python scripts/check_stream_continuity.py --seconds 30
    python scripts/check_mocap_path.py --seconds 30 --output "output/rocket-ground-$(date -u +%Y%m%dT%H%M%SZ)"
    python scripts/calibrate_ground.py --seconds 10 --preserve-heading
    python scripts/prepare_rocket_hover.py
    python scripts/configure_pdg_tracker.py
    python scripts/configure_pdg_tracker.py --apply
    python scripts/check_hardware.py --vicon-only --controller-id 6
    ;;
  hover)
    exec python scripts/hover_check.py --config output/setup/rocket-hover.yaml --vicon-only --controller mellinger --height 0.6 "$@"
    ;;
  review)
    exec pdg mission --config config/rocket-lab.yaml --plan-file "$PLAN" --output "output/rocket-review-$(date -u +%Y%m%dT%H%M%SZ)"
    ;;
  fly)
    exec pdg mission --config config/rocket-lab.yaml --plan-file "$PLAN" --output "output/rocket-flight-$(date -u +%Y%m%dT%H%M%SZ)" "$@"
    ;;
  land) exec bash scripts/land_now.sh ;;
  *) echo 'Usage: bash scripts/rocket.sh {server|check|hover|review|fly|land} [--execute]' >&2; exit 2 ;;
esac
