#!/usr/bin/env bash
# Branch-coverage proof for fixed case Y0M (the Y0 / L-Y mirror guard):
#   gcov.sh           build a -O0 --coverage OG harness, run the fixed cases,
#                     require the reject arm of freeOrSameNetY's mirror guard
#                     (`if (other != -1 && other != net) return false;`) taken.
#                     Matched by SOURCE TEXT, not line number.
#   gcov.sh --revert  remove BOTH mirror blocks (the one in setChipYStatusSafe
#                     and the one in freeOrSameNetY) from a copy of the router
#                     and require Y0M to report SHORTED - the real flip. One
#                     block alone is not enough: the other converts the short
#                     into an unrouted path.
set -euo pipefail
cd "$(dirname "$0")/../.."
T=${RUNNER_TEMP:-/tmp}/test_og_router_gcov
rm -rf "$T"; mkdir -p "$T"
SRC=src/routing/NetsToChipConnections_OG.cpp
if [ "${1:-}" = --revert ]; then
  # Delete every "if (mirrorChip != -1) { ... }" block: the two mirror
  # guards are the only such blocks in the file.
  awk 'BEGIN{skip=0;depth=0}
       skip==0 && /if \(mirrorChip != -1\) \{/ {skip=1;depth=1;next}
       skip==1 { n=gsub(/\{/,"{"); m=gsub(/\}/,"}"); depth+=n-m; if(depth<=0){skip=0}; next }
       {print}' "$SRC" > "$T/reverted.cpp"
  removed=$(( $(wc -l < "$SRC") - $(wc -l < "$T/reverted.cpp") ))
  [ "$removed" -ge 8 ] || { echo "gcov.sh --revert: expected to remove both mirror blocks, removed $removed lines"; exit 1; }
  NTCC_SRC="$T/reverted.cpp" BUILD_DIR="$T/build" bash test/test_og_router/run.sh > "$T/out.txt" 2>&1 || true
  if grep -A4 '^=== Y0M' "$T/out.txt" | grep -q 'SHORTED\|touches'; then
    echo "Y0M with both mirror blocks removed: SHORTED (expected)"; exit 0
  fi
  echo "Y0M with both mirror blocks removed did NOT short - the case no longer exercises the fix"; grep -A4 '^=== Y0M' "$T/out.txt"; exit 1
fi
CXX=${CXX:-g++} CXXFLAGS="-O0 --coverage" BUILD_DIR="$T/build" bash test/test_og_router/run.sh > "$T/out.txt" 2>&1 || { tail -5 "$T/out.txt"; exit 1; }
cd "$T/build/og"
gcov -b -t ./*NetsToChipConnections_OG.gcda > gcov.txt 2>/dev/null
# the guard line inside freeOrSameNetY (the second occurrence in the file)
# branch 2 = the `other != net` test's fallthrough into `return false`
pct=$(awk '/bool freeOrSameNetY/{f=1} f && /if \(other != -1 && other != net\) return false;/{g=1;next} g && /^branch  2/{print; exit}' gcov.txt | sed -E 's/.*taken ([0-9]+)%.*/\1/')
[ -n "$pct" ] || { echo "gcov.sh: could not find the freeOrSameNetY mirror guard branch in gcov output"; exit 1; }
echo "freeOrSameNetY mirror guard: branch 2 taken ${pct}%"
[ "$pct" -gt 0 ]
