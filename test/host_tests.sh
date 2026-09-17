#!/usr/bin/env bash
# The router host tests in one command (no hardware, no PlatformIO).
#
#   test/host_tests.sh          gcc/clang -Os: both router legs (og, v5) plus
#                               the random sweeps with an exact ratchet
#   test/host_tests.sh ubsan    the same, under clang's undefined / bounds /
#                               implicit-conversion / integer sanitizers with
#                               recovery OFF - any report is a failure. This is
#                               the leg that finds chipStates[-1]. Run it after
#                               touching a router.
#
# The sweep gate is a RATCHET, not a floor: shorts must be 0, and each seed's
# per-bridge unrouted count must EQUAL the recorded value - any routing change,
# better or worse, updates the numbers in the same commit, which makes "this
# change does not alter routing" mechanical. The generator is an in-file
# xorshift32, so the numbers hold on glibc, macOS and musl. Unrouted bridges
# are lane exhaustion on the crossbar, not bugs; recorded 2026-09-17 (the nano
# header is in the sweep only with OG_SWEEP_NANO=1 - see K4 in the harness).
set -euo pipefail
cd "$(dirname "$0")/.."
TMP=${RUNNER_TEMP:-/tmp}

want() {   # $1 = og|v5, $2 = seed -> recorded per-bridge unrouted count
  case "$1-$2" in
    og-1) echo 6533 ;; og-2) echo 6510 ;; og-7) echo 6807 ;;
    v5-1) echo 1752 ;; v5-2) echo 1607 ;; v5-7) echo 1664 ;;
    *) echo "no recorded value for $1 seed $2" >&2; exit 1 ;;
  esac
}

sweep() {   # $1 = binary, $2 = og|v5
  local bin=$1 board=$2 seed line shorts unrouted w
  for seed in 1 2 7; do
    w=$(want "$board" "$seed")
    line=$("$bin" rand "$seed" 10000 | tail -1); echo "$line"
    shorts=$(sed -E 's/.*, ([0-9]+) with SHORTS.*/\1/' <<<"$line")
    unrouted=$(sed -E 's/.*PathHealth: [0-9]+ bridges, ([0-9]+) unrouted.*/\1/' <<<"$line")
    [ "$shorts" -eq 0 ] || { echo "FAIL: $board seed $seed: $shorts trials shorted"; exit 1; }
    [ "$unrouted" -eq "$w" ] || { echo "FAIL: $board seed $seed: $unrouted unrouted bridges, recorded $w - routing changed; check the diff, then update want() in the same commit"; exit 1; }
  done
}

if [ "${1:-}" = ubsan ]; then
  for board in og v5; do
    echo "== test_og_router $board (ubsan, no recovery)"
    out=$(BOARD=$board CXX=${CXX:-clang++} CXXFLAGS="-O1 -g -fsanitize=undefined,bounds,implicit-conversion,integer -fno-sanitize=enum,unsigned-shift-base,unsigned-integer-overflow -fno-sanitize-recover=all -w" \
          BUILD_DIR="$TMP/ubsan" bash test/test_og_router/run.sh 2>&1) || { echo "$out" | tail -5; exit 1; }
    echo "$out" | tail -1
    sweep "$(sed -n 's/^binary: //p' <<<"$out")" $board
  done
  echo "host tests (ubsan): all pass"
  exit 0
fi

for board in og v5; do
  echo "== test_og_router ($board leg)"
  out=$(BOARD=$board bash test/test_og_router/run.sh) || { echo "$out" | tail -20; exit 1; }
  echo "$out" | tail -1
  echo "== test_og_router sweeps ($board)"
  sweep "$(sed -n 's/^binary: //p' <<<"$out")" $board
done
echo "host tests: all pass"
