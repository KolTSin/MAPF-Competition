#!/usr/bin/env bash
# Rerun only the levels that were not solved in a previous benchmark summary.csv.
#
# This version automatically searches for level files in both:
#   ./levels
#   ./complevels
#
# Typical use:
#   ./rerun_unsolved_from_summary_v3.sh benchmark_results_20260512_120000/summary.csv
#
# Or let it find the latest benchmark/rerun summary automatically:
#   ./rerun_unsolved_from_summary_v3.sh
#
# Optional extra level directories can still be provided if needed:
#   ./rerun_unsolved_from_summary_v3.sh summary.csv ./other-level-dir ./another-dir
#
# Optional environment overrides:
#   SERVER_JAR=./server.jar
#   CLIENT_CMD='build/searchclient --solver comp --heuristic gc'
#   TIMEOUT=180
#   OUT_DIR=benchmark_rerun_YYYYMMDD_HHMMSS
#   LEVELS_DIRS='./levels:./complevels:./other-level-dir'
#
# Requires Python 3 for robust CSV parsing.

set -u

SERVER_JAR="${SERVER_JAR:-./server.jar}"
CLIENT_CMD="${CLIENT_CMD:-build/searchclient --solver lns_htn --heuristic gc}"
TIMEOUT="${TIMEOUT:-180}"
OUT_DIR="${OUT_DIR:-benchmark_rerun_$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="$OUT_DIR/logs"
CSV="$OUT_DIR/summary.csv"
STATS="$OUT_DIR/summary_stats.txt"
UNSOLVED_LIST="$OUT_DIR/unsolved_input_levels.txt"
SOLVED_TIMES_TMP="$OUT_DIR/.solved_times.tmp"

INPUT_SUMMARY="${1:-}"
shift || true
EXTRA_LEVEL_DIRS=("$@")

find_latest_summary() {
    find . -maxdepth 2 -type f \( -path './benchmark_results_*/summary.csv' -o -path './benchmark_rerun_*/summary.csv' \) \
        | sort \
        | tail -n 1
}

if [[ -z "$INPUT_SUMMARY" ]]; then
    INPUT_SUMMARY="$(find_latest_summary)"
fi

if [[ -z "$INPUT_SUMMARY" || ! -f "$INPUT_SUMMARY" ]]; then
    echo "ERROR: Could not find a previous summary.csv." >&2
    echo "Usage: $0 [path/to/summary.csv] [optional_extra_levels_dir ...]" >&2
    echo "Example: $0 benchmark_results_20260512_133326/summary.csv" >&2
    exit 1
fi

if [[ ! -f "$SERVER_JAR" ]]; then
    echo "ERROR: Could not find $SERVER_JAR. Run this from the project root or set SERVER_JAR." >&2
    exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
    echo "ERROR: python3 is required to parse summary.csv safely." >&2
    exit 1
fi

mkdir -p "$LOG_DIR"
: > "$SOLVED_TIMES_TMP"

csv_escape() {
    local s="${1:-}"
    s=${s//\"/\"\"}
    printf '"%s"' "$s"
}

extract_last_match() {
    local pattern="$1"
    local file="$2"
    grep -E "$pattern" "$file" | tail -n 1 || true
}

is_number() {
    [[ "${1:-}" =~ ^[0-9]+([.][0-9]+)?$ ]]
}

# Extract levels_dir values from previous summary, if present.
# These are treated as hints only. We still always check ./levels and ./complevels.
infer_level_dirs_from_summary() {
    python3 - "$INPUT_SUMMARY" <<'PY'
import csv
import sys

path = sys.argv[1]
seen = set()
try:
    with open(path, newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames or 'levels_dir' not in reader.fieldnames:
            raise SystemExit(0)
        for row in reader:
            value = (row.get('levels_dir') or '').strip()
            if value and value not in seen:
                seen.add(value)
                print(value)
except FileNotFoundError:
    pass
PY
}

# Build level search path list.
# Priority:
#   1. extra directories provided after summary.csv
#   2. LEVELS_DIRS environment variable, colon-separated
#   3. levels_dir values stored in previous summary.csv, if present
#   4. ./levels
#   5. ./complevels
# We intentionally include ./levels and ./complevels even if the previous summary says
# something else, because old rerun summaries may contain a wrong fallback directory.
declare -a LEVEL_SEARCH_DIRS=()

add_level_dir() {
    local dir="$1"
    [[ -z "$dir" ]] && return 0
    local existing
    for existing in "${LEVEL_SEARCH_DIRS[@]:-}"; do
        [[ "$existing" == "$dir" ]] && return 0
    done
    LEVEL_SEARCH_DIRS+=("$dir")
}

for d in "${EXTRA_LEVEL_DIRS[@]:-}"; do
    add_level_dir "$d"
done

if [[ -n "${LEVELS_DIRS:-}" ]]; then
    IFS=':' read -r -a env_dirs <<< "$LEVELS_DIRS"
    for d in "${env_dirs[@]:-}"; do
        add_level_dir "$d"
    done
fi

while IFS= read -r inferred_dir; do
    add_level_dir "$inferred_dir"
done < <(infer_level_dirs_from_summary)

add_level_dir "./levels"
add_level_dir "./complevels"

find_level_path() {
    local level_file="$1"
    local dir
    for dir in "${LEVEL_SEARCH_DIRS[@]}"; do
        if [[ -f "$dir/$level_file" ]]; then
            printf '%s\n' "$dir/$level_file"
            return 0
        fi
    done
    return 1
}

level_dir_for_path() {
    local path="$1"
    dirname "$path"
}

# Extract unsolved level file names from the previous summary.
# Supports all earlier summary formats.
# We treat anything except solved=Yes/status=SOLVED as a rerun candidate.
python3 - "$INPUT_SUMMARY" > "$UNSOLVED_LIST" <<'PY'
import csv
import os
import sys

summary_path = sys.argv[1]

with open(summary_path, newline='', encoding='utf-8') as f:
    reader = csv.DictReader(f)
    fieldnames = set(reader.fieldnames or [])
    if 'level' not in fieldnames:
        raise SystemExit("summary.csv is missing required column: level")

    seen = set()
    for row in reader:
        raw_level = (row.get('level') or '').strip()
        if not raw_level:
            continue

        # Keep just the filename if an older/custom summary stored a path.
        level = os.path.basename(raw_level)

        status = (row.get('status') or '').strip().upper()
        solved = (row.get('solved') or '').strip().upper()

        # If a summary lacks solved/status columns, be conservative and rerun it.
        if (solved == 'YES') or (status == 'SOLVED'):
            continue

        if level not in seen:
            seen.add(level)
            print(level)
PY

mapfile -t LEVEL_FILES < "$UNSOLVED_LIST"

total_count=${#LEVEL_FILES[@]}

if [[ $total_count -eq 0 ]]; then
    echo "No unsolved levels found in: $INPUT_SUMMARY"
    echo "Nothing to rerun. Nice."
    exit 0
fi

printf 'level,levels_dir,level_path,status,solved,exit_code,wall_time_s,server_time_s,solution_length,log_file,source_summary\n' > "$CSV"

solved_count=0
unsolved_count=0
unknown_count=0
missing_count=0

cat > "$OUT_DIR/README.txt" <<EOF_README
This folder was created by rerun_unsolved_from_summary_v3.sh.

Source summary: $INPUT_SUMMARY
Level search dirs:
$(printf '  - %s\n' "${LEVEL_SEARCH_DIRS[@]}")
Client:         $CLIENT_CMD
Timeout:        ${TIMEOUT}s

To continue iterating after improving the solver, run:
  ./rerun_unsolved_from_summary_v3.sh $CSV

The script automatically checks ./levels and ./complevels, plus any levels_dir values stored in the summary.
EOF_README

echo "Previous summary: $INPUT_SUMMARY"
echo "Level search dirs:"
printf '  - %s\n' "${LEVEL_SEARCH_DIRS[@]}"
echo "Rerunning unsolved levels: $total_count"
echo "Client: $CLIENT_CMD"
echo "Timeout: ${TIMEOUT}s"
echo "Output: $OUT_DIR"
echo

for level_file in "${LEVEL_FILES[@]}"; do
    level_path="$(find_level_path "$level_file" || true)"
    level_name="${level_file%.lvl}"
    log_file="$LOG_DIR/${level_name}.log"

    echo "==> $level_file"

    if [[ -z "$level_path" ]]; then
        echo "    MISSING: not found in search dirs"
        missing_count=$((missing_count + 1))
        {
            csv_escape "$level_file"; printf ','
            csv_escape ""; printf ','
            csv_escape ""; printf ','
            csv_escape "MISSING_LEVEL_FILE"; printf ','
            csv_escape "No"; printf ','
            csv_escape ""; printf ','
            csv_escape ""; printf ','
            csv_escape ""; printf ','
            csv_escape ""; printf ','
            csv_escape ""; printf ','
            csv_escape "$INPUT_SUMMARY"; printf '\n'
        } >> "$CSV"
        continue
    fi

    actual_levels_dir="$(level_dir_for_path "$level_path")"
    echo "    found: $level_path"

    start_ts=$(date +%s)
    java -jar "$SERVER_JAR" -l "$level_path" -c "$CLIENT_CMD" -t "$TIMEOUT" > "$log_file" 2>&1
    exit_code=$?
    end_ts=$(date +%s)
    wall_time=$((end_ts - start_ts))

    solved_line="$(extract_last_match '\[server\]\[info\][[:space:]]+Level solved:' "$log_file")"
    if [[ "$solved_line" =~ Level[[:space:]]solved:[[:space:]]*Yes ]]; then
        solved="Yes"
        status="SOLVED"
        solved_count=$((solved_count + 1))
    elif [[ "$solved_line" =~ Level[[:space:]]solved:[[:space:]]*No ]]; then
        solved="No"
        status="UNSOLVED"
        unsolved_count=$((unsolved_count + 1))
    else
        solved="Unknown"
        status="UNKNOWN"
        unknown_count=$((unknown_count + 1))
    fi

    if [[ $exit_code -ne 0 && "$status" != "SOLVED" ]]; then
        status="EXIT_${exit_code}"
    fi
    if grep -Eiq 'timeout|timed out|time-out|OutOfMemory|killed' "$log_file" && [[ "$status" != "SOLVED" ]]; then
        status="TIMEOUT_OR_CRASH"
    fi

    server_time_line="$(extract_last_match '\[server\]\[info\][[:space:]]+Time to solve:' "$log_file")"
    server_time="$(printf '%s' "$server_time_line" | sed -E 's/.*Time to solve:[[:space:]]*([0-9.]+).*/\1/' || true)"
    [[ "$server_time" == "$server_time_line" ]] && server_time=""

    length_line="$(extract_last_match 'Found solution of length|Solution length|solution of length|joint actions|Actions used|Actions used:' "$log_file")"
    solution_length="$(printf '%s' "$length_line" | sed -E 's/.*(Found solution of length|Solution length|solution of length|joint actions|Actions used)[^0-9]*([0-9]+).*/\2/' || true)"
    [[ "$solution_length" == "$length_line" ]] && solution_length=""

    if [[ "$solved" == "Yes" ]]; then
        if is_number "$server_time"; then
            printf '%s\n' "$server_time" >> "$SOLVED_TIMES_TMP"
        else
            printf '%s\n' "$wall_time" >> "$SOLVED_TIMES_TMP"
        fi
    fi

    {
        csv_escape "$level_file"; printf ','
        csv_escape "$actual_levels_dir"; printf ','
        csv_escape "$level_path"; printf ','
        csv_escape "$status"; printf ','
        csv_escape "$solved"; printf ','
        csv_escape "$exit_code"; printf ','
        csv_escape "$wall_time"; printf ','
        csv_escape "$server_time"; printf ','
        csv_escape "$solution_length"; printf ','
        csv_escape "$log_file"; printf ','
        csv_escape "$INPUT_SUMMARY"; printf '\n'
    } >> "$CSV"

    printf '    %-16s solved=%s wall=%ss server=%ss length=%s\n' \
        "$status" "$solved" "$wall_time" "${server_time:-?}" "${solution_length:-?}"
done

failed_count=$((total_count - solved_count))
success_rate="$(awk -v solved="$solved_count" -v total="$total_count" 'BEGIN { if (total > 0) printf "%.2f", (100.0 * solved / total); else printf "0.00" }')"

if [[ -s "$SOLVED_TIMES_TMP" ]]; then
    avg_solve_time="$(awk '{ sum += $1; n += 1 } END { if (n > 0) printf "%.3f", sum / n; else printf "" }' "$SOLVED_TIMES_TMP")"
else
    avg_solve_time="-"
fi

remaining_unsolved="$OUT_DIR/remaining_unsolved_levels.txt"
python3 - "$CSV" > "$remaining_unsolved" <<'PY'
import csv
import sys

with open(sys.argv[1], newline='', encoding='utf-8') as f:
    for row in csv.DictReader(f):
        level = (row.get('level') or '').strip()
        status = (row.get('status') or '').strip().upper()
        solved = (row.get('solved') or '').strip().upper()
        if level and solved != 'YES' and status != 'SOLVED':
            print(level)
PY

{
    echo "source_summary=$INPUT_SUMMARY"
    echo "level_search_dirs=$(IFS=':'; echo "${LEVEL_SEARCH_DIRS[*]}")"
    echo "levels_rerun=$total_count"
    echo "levels_solved_this_rerun=$solved_count"
    echo "levels_still_failed=$failed_count"
    echo "levels_missing=$missing_count"
    echo "levels_unsolved_reported=$unsolved_count"
    echo "levels_unknown=$unknown_count"
    echo "success_rate_percent_this_rerun=$success_rate"
    echo "average_solve_time_s_this_rerun=$avg_solve_time"
    echo "average_solve_time_note=Average is over levels solved in this rerun only; uses server_time_s when available, otherwise wall_time_s."
    echo "remaining_unsolved_file=$remaining_unsolved"
} > "$STATS"

rm -f "$SOLVED_TIMES_TMP"

echo
echo "================ Rerun summary ================"
printf 'Levels rerun:        %s\n' "$total_count"
printf 'Solved this rerun:   %s\n' "$solved_count"
printf 'Still failed:        %s\n' "$failed_count"
printf 'Missing level files: %s\n' "$missing_count"
printf 'Success rate:        %s%%\n' "$success_rate"
printf 'Avg solve time:      %ss over newly solved levels\n' "$avg_solve_time"
echo "================================================"
echo
echo "New per-level CSV:     $CSV"
echo "New summary stats:     $STATS"
echo "Input unsolved list:   $UNSOLVED_LIST"
echo "Remaining unsolved:    $remaining_unsolved"
echo "Logs:                  $LOG_DIR"
echo
echo "Next iteration command:"
echo "  $0 $CSV"
echo
