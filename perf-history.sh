#!/usr/bin/env bash
#
# Measure test cost across a run of commits, so a regression can be attributed to the commit
# that introduced it rather than merely detected.
#
# Every commit is built and measured in the same invocation on the same machine, which is what
# makes the comparison meaningful: absolute numbers drift with the toolchain, ratios measured
# back to back do not.
#
#   ./perf-history.sh [depth] [gtest-filter]
#
#   MODE=instructions  (default) cachegrind instruction counts. Deterministic to the instruction,
#                      so a 2% move is signal. Costs roughly 40x runtime.
#   MODE=wall          wall-clock, best of REPEATS. Only trust large moves: on a shared runner
#                      the same binary has been seen to vary by 9x.
#   PER_SUITE=1        measure every gtest suite separately instead of one filter, with the
#                      binary's fixed start-up cost subtracted so the number is the suite's own.
#   CSV=path           also write machine-readable rows, for plotting.
#
set -euo pipefail

DEPTH=${1:-4}
FILTER=${2:-OptimizationTrajectoryTest.*}
MODE=${MODE:-instructions}
REPEATS=${REPEATS:-7}
REF=${REF:-main}
PER_SUITE=${PER_SUITE:-0}
CSV=${CSV:-}

REPO=$(git rev-parse --show-toplevel)
WORK=$(mktemp -d)

# Reuse dependencies already fetched into ./build, or FetchContent clones Blaze and GoogleTest
# once per commit. CMakeLists applies the Blaze patch idempotently, so sharing the tree is safe.
EXTRA=()
[ -d "$REPO/build/_deps/blaze-src" ] && EXTRA+=("-DFETCHCONTENT_SOURCE_DIR_BLAZE=$REPO/build/_deps/blaze-src")
[ -d "$REPO/build/_deps/googletest-src" ] && EXTRA+=("-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=$REPO/build/_deps/googletest-src")

cleanup() {
  for wt in "$WORK"/*; do [ -d "$wt" ] && git -C "$REPO" worktree remove --force "$wt" 2>/dev/null || true; done
  git -C "$REPO" worktree prune 2>/dev/null || true
  rm -rf "$WORK"
}
trap cleanup EXIT

measure() {  # binary, filter
  if [ "$MODE" = wall ]; then
    local best="" ms start end
    for _ in $(seq "$REPEATS"); do
      start=$(date +%s%N); "$1" --gtest_filter="$2" >/dev/null 2>&1 || true; end=$(date +%s%N)
      ms=$(( (end - start) / 1000000 ))
      if [ -z "$best" ] || [ "$ms" -lt "$best" ]; then best=$ms; fi   # min, not mean: noise is one-sided
    done
    echo "$best"
  else
    valgrind --tool=cachegrind --cachegrind-out-file=/dev/null "$1" --gtest_filter="$2" 2>&1 >/dev/null \
      | awk '/I refs:/ { gsub(/,/, "", $4); print $4 }'
  fi
}

suites_of() {  # binary -> one gtest suite name per line
  # Suite lines are unindented and end in a dot; anchor on that, or gtest's own
  # "Running main() from ..." banner is read as a suite.
  "$1" --gtest_list_tests 2>/dev/null | grep -E '^[A-Za-z][A-Za-z0-9_/]*\.' | sed 's/\..*$//' | sort -u
}

# Collect newest-first, report oldest-first: a commit's delta belongs beside the commit that
# caused it, which is only knowable once its parent has been measured.
shas=(); subjects=(); values=(); rows=()

for i in $(seq 0 "$DEPTH"); do
  sha=$(git -C "$REPO" rev-parse --short "$REF~$i")
  subject=$(git -C "$REPO" log -1 --format=%s "$REF~$i" | cut -c1-48)
  wt="$WORK/$sha"

  git -C "$REPO" worktree add --detach "$wt" "$sha" >/dev/null 2>&1
  if ! cmake -S "$wt" -B "$wt/b" -DCMAKE_BUILD_TYPE=Release "${EXTRA[@]}" >/dev/null 2>&1 ||
     ! cmake --build "$wt/b" -j"$(nproc)" >/dev/null 2>&1; then
    shas+=("$sha"); subjects+=("$subject"); values+=("")
    continue
  fi

  bin="$wt/b/tests/vortex_tests"
  if [ "$PER_SUITE" = 1 ]; then
    # Subtract the cost of starting the binary and running nothing, so each number is the
    # suite's own work rather than mostly process start-up.
    base=$(measure "$bin" 'NoSuchTest.NoSuchCase')
    while read -r suite; do
      [ -z "$suite" ] && continue
      total=$(measure "$bin" "$suite.*")
      rows+=("$sha,$subject,$suite,$(( ${total:-0} - ${base:-0} ))")
    done < <(suites_of "$bin")
    values+=("$base")
  else
    values+=("$(measure "$bin" "$FILTER")")
  fi
  shas+=("$sha"); subjects+=("$subject")
done

if [ -n "$CSV" ]; then
  { echo "commit,subject,suite,$MODE"; printf '%s\n' "${rows[@]}"; } > "$CSV"
  echo "wrote $CSV ($(( ${#rows[@]} )) rows)"
fi

if [ "$PER_SUITE" != 1 ]; then
  echo "workload : $FILTER"
  echo "metric   : $MODE$([ "$MODE" = wall ] && echo " (best of $REPEATS)")"
  echo
  printf '%-9s  %-48s  %16s  %9s\n' COMMIT SUBJECT "$(tr '[:lower:]' '[:upper:]' <<<"$MODE")" 'VS PARENT'
  printf '%-9s  %-48s  %16s  %9s\n' '---------' "$(printf '%.0s-' {1..48})" '----------------' '---------'
  for (( i=${#shas[@]}-1; i>=0; i-- )); do
    v=${values[$i]}; p=${values[$i+1]:-}
    if [ -z "$v" ]; then printf '%-9s  %-48s  %16s  %9s\n' "${shas[$i]}" "${subjects[$i]}" 'BUILD FAILED' '-'; continue; fi
    d='-'
    [ -n "$p" ] && d=$(awk -v a="$v" -v b="$p" 'BEGIN { printf "%+.2f%%", (a - b) * 100.0 / b }')
    printf "%-9s  %-48s  %16s  %9s\n" "${shas[$i]}" "${subjects[$i]}" "$(printf "%'d" "$v")" "$d"
  done
fi
