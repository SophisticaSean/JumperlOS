#!/usr/bin/env bash
# Every host test in one command (no hardware, no PlatformIO): the four
# harnesses, then the OG router random sweep as a regression floor - shorts
# must stay at 0 and the unrouted-trial count must not grow past what
# og-routing-perf measured (seed 1, 2000 trials: 136; lane exhaustion, not bugs).
set -euo pipefail
cd "$(dirname "$0")/.."
for t in test_og_router test_og_analog test_pair_str test_rx_witness test_mp_rung; do
  echo "== $t"; bash "test/$t/run.sh" | tail -1
done
echo "== test_og_router rand 1 2000"
line=$("${RUNNER_TEMP:-/tmp}/test_og_router/og/test_og_router" rand 1 2000 | tail -1); echo "$line"
failed=$(sed -E 's/.*: ([0-9]+)\/2000 trials failed.*/\1/' <<<"$line")
shorts=$(sed -E 's/.*, ([0-9]+) with SHORTS.*/\1/' <<<"$line")
[ "$shorts" -eq 0 ] || { echo "FAIL: $shorts trials shorted"; exit 1; }
[ "$failed" -le 136 ] || { echo "FAIL: $failed unrouted trials > floor 136"; exit 1; }
echo "host tests: all pass"
