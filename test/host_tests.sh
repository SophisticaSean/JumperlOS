#!/usr/bin/env bash
# Every host test in one command (no hardware, no PlatformIO).
#
#   test/host_tests.sh          gcc/clang -Os legs: the harnesses + the OG
#                               router sweeps (exact per-seed ratchet)
#   test/host_tests.sh ubsan    the same router harness under clang's
#                               undefined/bounds/implicit-conversion/integer
#                               sanitizers with recovery OFF - any report
#                               is a failure. Run it after touching the router.
#
# The sweep gate is a RATCHET, not a floor: shorts must be 0, and the
# per-bridge unrouted count of each seed must EQUAL the recorded value - any
# routing change, better or worse, updates the numbers in the same commit,
# which also makes "this change does not alter routing" mechanical. The
# generator is an in-file xorshift32, so the numbers hold on glibc, macOS
# and musl. Unrouted bridges are lane exhaustion on the rev 2 crossbar, not
# bugs; the numbers below were recorded on og-routing-perf 2026-09-15 with
# the widened generator (SF nodes, ISENSE, rails, NANO_RESET/AREF; the nano
# header is OG_SWEEP_NANO=1 only - see K4 in the harness).
#
# Measured against: local g++ 11 / clang 14 / gawk 5.1; CI ubuntu-latest
# (newer clang, wider implicit-conversion group) and macos-latest (BSD awk,
# libc). Recover-mode enumeration of sanitizer sites, when the hard leg
# turns red on new code:
#   CXX=clang++ CXXFLAGS="-O1 -g -fsanitize=undefined,bounds,implicit-conversion,integer \
#     -fno-sanitize=enum,unsigned-shift-base -fsanitize-recover=all -w" \
#     BUILD_DIR=$TMPDIR/og_ubsan test/test_og_router/run.sh
#   $TMPDIR/og_ubsan/og/test_og_router rand 2 10000 2>&1 | grep 'runtime error' | sort -u
set -euo pipefail
cd "$(dirname "$0")/.."
TMP=${RUNNER_TEMP:-/tmp}
declare -A WANT_og=( [1]=7429 [2]=7347 [7]=7671 )   # per-bridge unrouted, seeds x 10 000 trials
declare -A WANT_v5=( [1]=1752 [2]=1607 [7]=1664 )

sweep() {   # $1 = binary, $2 = og|v5
  local bin=$1 board=$2 seed line shorts unrouted want
  for seed in 1 2 7; do
    want=$(eval "echo \${WANT_${board}[$seed]}")
    line=$("$bin" rand "$seed" 10000 | tail -1); echo "$line"
    shorts=$(sed -E 's/.*, ([0-9]+) with SHORTS.*/\1/' <<<"$line")
    unrouted=$(sed -E 's/.*PathHealth: [0-9]+ bridges, ([0-9]+) unrouted.*/\1/' <<<"$line")
    [ "$shorts" -eq 0 ] || { echo "FAIL: seed $seed: $shorts trials shorted"; exit 1; }
    [ "$unrouted" -eq "$want" ] || { echo "FAIL: $board seed $seed: $unrouted unrouted bridges, recorded $want - routing changed; inspect the digest, then update WANT_$board in the same commit"; exit 1; }
  done
}

if [ "${1:-}" = ubsan ]; then
  for board in og v5; do
    echo "== test_og_router $board (ubsan, no recovery)"
    out=$(BOARD=$board CXX=${CXX:-clang++} CXXFLAGS="-O1 -g -fsanitize=undefined,bounds,implicit-conversion,integer -fno-sanitize=enum,unsigned-shift-base -fno-sanitize-recover=all -w" \
          BUILD_DIR="$TMP/ubsan" bash test/test_og_router/run.sh 2>&1) || { echo "$out" | tail -5; exit 1; }
    echo "$out" | tail -1
    sweep "$(sed -n 's/^binary: //p' <<<"$out")" $board
  done
  echo "host tests (ubsan): all pass"
  exit 0
fi

for t in test_og_router test_og_analog test_pair_str test_rx_witness test_mp_rung test_slot_backstop test_qstr_table; do
  echo "== $t"
  out=$(bash "test/$t/run.sh") || { echo "$out" | tail -20; exit 1; }
  echo "$out" | tail -1
  [ "$t" = test_og_router ] && OG_BIN=$(sed -n 's/^binary: //p' <<<"$out")
done
echo "== test_og_router (v5 leg)"
out=$(BOARD=v5 bash test/test_og_router/run.sh) || { echo "$out" | tail -20; exit 1; }
echo "$out" | tail -1
V5_BIN=$(sed -n 's/^binary: //p' <<<"$out")
echo "== test_og_router sweeps (og)"
sweep "$OG_BIN" og
echo "== test_og_router sweeps (v5)"
sweep "$V5_BIN" v5
echo "host tests: all pass"
