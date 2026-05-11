#!/usr/bin/env bash
# Run every .lvl file in ./levels through MAvis using the competition solver.
# Usage:
#   ./benchmark_levels.sh
# Optional environment overrides:
#   LEVELS_DIR=./complevels CLIENT_CMD='build/searchclient --solver comp --heuristic gc' TIMEOUT=180 ./benchmark_levels.sh

set -u

LEVELS_DIR="${LEVELS_DIR:-./complevels}"
SERVER_JAR="${SERVER_JAR:-./server.jar}"
CLIENT_CMD="${CLIENT_CMD:-build/searchclient --solver comp --heuristic gc}"
TIMEOUT="${TIMEOUT:-180}"
OUT_DIR="${OUT_DIR:-benchmark_results_$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="$OUT_DIR/logs"
CSV="$OUT_DIR/summary.csv"
STATS="$OUT_DIR/summary_stats.txt"
SOLVED_TIMES_TMP="$OUT_DIR/.solved_times.tmp"

mkdir -p "$LOG_DIR"
: > "$SOLVED_TIMES_TMP"

csv_escape() {
    local s="${1:-}"
    # CSV escaping: double every quote, then wrap the whole field in quotes.
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

printf 'level,status,solved,exit_code,wall_time_s,server_time_s,solution_length,log_file\n' > "$CSV"

if [[ ! -f "$SERVER_JAR" ]]; then
    echo "ERROR: Could not find $SERVER_JAR. Run this from the project root or set SERVER_JAR." >&2
    exit 1
fi

if [[ ! -d "$LEVELS_DIR" ]]; then
    echo "ERROR: Could not find $LEVELS_DIR. Run this from the project root or set LEVELS_DIR." >&2
    exit 1
fi

mapfile -d '' LEVELS < <(find "$LEVELS_DIR" -maxdepth 1 -type f -name '*.lvl' -print0 | sort -z)

if [[ ${#LEVELS[@]} -eq 0 ]]; then
    echo "ERROR: No .lvl files found in $LEVELS_DIR" >&2
    exit 1
fi

total_count=${#LEVELS[@]}
solved_count=0
unsolved_count=0
unknown_count=0

printf 'Running %s levels\n' "$total_count"
echo "Client: $CLIENT_CMD"
echo "Timeout: ${TIMEOUT}s"
echo "Output: $OUT_DIR"
echo

for level_path in "${LEVELS[@]}"; do
    level_file="$(basename "$level_path")"
    level_name="${level_file%.lvl}"
    log_file="$LOG_DIR/${level_name}.log"

    echo "==> $level_file"

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

    length_line="$(extract_last_match 'Found solution of length|Solution length|solution of length|joint actions|Actions used' "$log_file")"
    solution_length="$(printf '%s' "$length_line" | sed -E 's/.*(Found solution of length|Solution length|solution of length|joint actions|Actions used)[^0-9]*([0-9]+).*/\2/' || true)"
    [[ "$solution_length" == "$length_line" ]] && solution_length=""

    if [[ "$solved" == "Yes" ]]; then
        # Prefer server-reported solve time. Fall back to measured wall time if the server line is missing.
        if is_number "$server_time"; then
            printf '%s\n' "$server_time" >> "$SOLVED_TIMES_TMP"
        else
            printf '%s\n' "$wall_time" >> "$SOLVED_TIMES_TMP"
        fi
    fi

    {
        csv_escape "$level_file"; printf ','
        csv_escape "$status"; printf ','
        csv_escape "$solved"; printf ','
        csv_escape "$exit_code"; printf ','
        csv_escape "$wall_time"; printf ','
        csv_escape "$server_time"; printf ','
        csv_escape "$solution_length"; printf ','
        csv_escape "$log_file"; printf '\n'
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

{
    echo "levels_tested=$total_count"
    echo "levels_solved=$solved_count"
    echo "levels_failed=$failed_count"
    echo "levels_unsolved_reported=$unsolved_count"
    echo "levels_unknown=$unknown_count"
    echo "success_rate_percent=$success_rate"
    echo "average_solve_time_s=$avg_solve_time"
    echo "average_solve_time_note=Average is over solved levels only; uses server_time_s when available, otherwise wall_time_s."
} > "$STATS"

rm -f "$SOLVED_TIMES_TMP"

echo
echo "================ Benchmark summary ================"
printf 'Levels tested:       %s\n' "$total_count"
printf 'Levels solved:       %s\n' "$solved_count"
printf 'Levels failed:       %s\n' "$failed_count"
printf 'Success rate:        %s%%\n' "$success_rate"
printf 'Avg solve time:      %ss over solved levels\n' "$avg_solve_time"
echo "==================================================="
echo
echo "Per-level CSV: $CSV"
echo "Summary stats: $STATS"
echo "Logs:          $LOG_DIR"
