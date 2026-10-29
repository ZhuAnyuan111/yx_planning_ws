#!/bin/bash

# Do not allow `bash -x` (or a caller's xtrace setting) to print credentials.
set +x

# NAS watchdog configuration. The password is intentionally kept local to this
# process; it is never exported, placed in an argv, or written to a log.
readonly NAS_PASSWORD='sy868SY868'

# OpenSSH executes SSH_ASKPASS with its own prompt as argv[1].  The environment
# marker is therefore the reliable way to enter this small helper mode.
if [[ "${NAS_ASKPASS_MODE:-0}" == "1" ]]; then
    printf '%s\n' "$NAS_PASSWORD"
    exit 0
fi

readonly NAS_USER='sanyNAS'
readonly NAS_HOST='192.168.1.245'
readonly NAS_APP_DIR='/volume1/web/app'
readonly NAS_APP_SCRIPT='DS923_app.py'
readonly NAS_HEALTH_URL='http://192.168.1.245:8082/'
readonly NAS_HEALTH_EXPECTED_TEXT='Sany Hello'
readonly NAS_CHECK_INTERVAL_S=30
readonly NAS_FAILURE_THRESHOLD=3
readonly NAS_STARTUP_CHECK_COUNT=6
readonly NAS_STARTUP_CHECK_INTERVAL_S=5
readonly NAS_REMOTE_PID_FILE='/volume1/web/app/.DS923_app_watchdog.pid'
readonly NAS_REMOTE_LOG_FILE='/volume1/web/app/DS923_app_watchdog.log'
readonly NAS_MOUNT_ROOT='/media/nvidia/G500'
readonly NAS_DATA_ROOT='/media/nvidia/G500/DataCapture'
readonly NAS_LOCAL_LOG_DIR='/media/nvidia/G500/DataCapture/nas_ds923_watchdog_logs'
readonly NAS_LOG_RETENTION_MINUTES=4320
readonly NAS_LOG_CLEANUP_INTERVAL_S=3600
readonly NAS_HEALTH_ERROR_LIMIT=1000

SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
if [[ -z "$SCRIPT_PATH" || ! -f "$SCRIPT_PATH" ]]; then
    printf '[NAS watchdog] ERROR: cannot resolve this script path.\n' >&2
    exit 1
fi
readonly SCRIPT_PATH

NAS_ACTIVE_LOG=''
NAS_ACTIVE_LOG_DAY=''
NAS_NEXT_LOG_CLEANUP_EPOCH=0
NAS_HEALTH_LAST_ERROR=''

nas_log_prefix() {
    printf '[NAS watchdog][%s]' "$(date '+%F %T')"
}

nas_prepare_log_directory() {
    local dependency
    for dependency in mountpoint mkdir mktemp chmod date find; do
        if ! command -v "$dependency" >/dev/null 2>&1; then
            printf '[NAS watchdog] ERROR: logging requires command %s.\n' "$dependency" >&2
            return 1
        fi
    done

    if [[ -L "$NAS_MOUNT_ROOT" ]] || ! mountpoint -q "$NAS_MOUNT_ROOT"; then
        printf '[NAS watchdog] ERROR: %s is not a mounted filesystem; refusing to write logs into the system disk.\n' \
            "$NAS_MOUNT_ROOT" >&2
        return 1
    fi
    if [[ -L "$NAS_DATA_ROOT" || ( -e "$NAS_DATA_ROOT" && ! -d "$NAS_DATA_ROOT" ) ]]; then
        printf '[NAS watchdog] ERROR: unsafe DataCapture path: %s\n' "$NAS_DATA_ROOT" >&2
        return 1
    fi
    if ! mkdir -p "$NAS_DATA_ROOT"; then
        printf '[NAS watchdog] ERROR: cannot create DataCapture path: %s\n' "$NAS_DATA_ROOT" >&2
        return 1
    fi
    if [[ -L "$NAS_LOCAL_LOG_DIR" || ( -e "$NAS_LOCAL_LOG_DIR" && ! -d "$NAS_LOCAL_LOG_DIR" ) ]]; then
        printf '[NAS watchdog] ERROR: unsafe local log directory: %s\n' "$NAS_LOCAL_LOG_DIR" >&2
        return 1
    fi
    if ! mkdir -p -m 700 "$NAS_LOCAL_LOG_DIR" || \
            ! chmod 700 "$NAS_LOCAL_LOG_DIR" 2>/dev/null; then
        printf '[NAS watchdog] ERROR: cannot prepare local log directory: %s\n' "$NAS_LOCAL_LOG_DIR" >&2
        return 1
    fi
    if [[ ! -O "$NAS_LOCAL_LOG_DIR" || ! -w "$NAS_LOCAL_LOG_DIR" ]]; then
        printf '[NAS watchdog] ERROR: local log directory must be owned and writable by this user: %s\n' \
            "$NAS_LOCAL_LOG_DIR" >&2
        return 1
    fi
}

nas_open_daily_log() {
    local log_day log_stamp new_log
    # Re-check the mounted filesystem and path ownership before every rotation;
    # a long-running watchdog must never follow a replaced log-directory link.
    nas_prepare_log_directory || return 1
    log_day="$(date '+%Y%m%d')" || return 1
    log_stamp="$(date '+%Y%m%d_%H%M%S')" || return 1
    new_log="$(mktemp "${NAS_LOCAL_LOG_DIR}/nas_ds923_watchdog_${log_stamp}_$$_XXXXXX.log" 2>/dev/null)" || {
        printf '[NAS watchdog] ERROR: cannot create a local watchdog log.\n' >&2
        return 1
    }
    if [[ ! -f "$new_log" || -L "$new_log" ]]; then
        printf '[NAS watchdog] ERROR: unsafe local watchdog log path: %s\n' "$new_log" >&2
        return 1
    fi
    chmod 600 "$new_log" 2>/dev/null || true

    if [[ -n "$NAS_ACTIVE_LOG" ]]; then
        printf '%s Rotating local log to %s.\n' "$(nas_log_prefix)" "$new_log"
    fi
    if ! exec >>"$new_log" 2>&1; then
        printf '[NAS watchdog] ERROR: cannot redirect output to %s.\n' "$new_log" >&2
        return 1
    fi
    NAS_ACTIVE_LOG="$new_log"
    NAS_ACTIVE_LOG_DAY="$log_day"
    printf '%s Local log opened: %s\n' "$(nas_log_prefix)" "$NAS_ACTIVE_LOG"
}

nas_cleanup_expired_logs() {
    local old_log removed_count=0
    nas_prepare_log_directory || return 1
    while IFS= read -r -d '' old_log; do
        [[ "$old_log" == "$NAS_ACTIVE_LOG" ]] && continue
        [[ -f "$old_log" && ! -L "$old_log" ]] || continue
        if rm -f -- "$old_log"; then
            removed_count=$((removed_count + 1))
        fi
    done < <(find "$NAS_LOCAL_LOG_DIR" -xdev -mindepth 1 -maxdepth 1 \
        -type f -name 'nas_ds923_watchdog_*.log' \
        -mmin "+${NAS_LOG_RETENTION_MINUTES}" -print0)
    if [[ "$removed_count" -gt 0 ]]; then
        printf '%s Removed %d local watchdog log(s) older than 72 hours.\n' \
            "$(nas_log_prefix)" "$removed_count"
    fi
}

nas_log_maintenance() {
    local today now_epoch
    today="$(date '+%Y%m%d')" || return 1
    if [[ "$today" != "$NAS_ACTIVE_LOG_DAY" ]]; then
        nas_open_daily_log || return 1
    fi
    now_epoch="$(date '+%s')" || return 1
    if (( now_epoch >= NAS_NEXT_LOG_CLEANUP_EPOCH )); then
        nas_cleanup_expired_logs
        NAS_NEXT_LOG_CLEANUP_EPOCH=$((now_epoch + NAS_LOG_CLEANUP_INTERVAL_S))
    fi
}

nas_initialize_logging() {
    umask 077
    nas_prepare_log_directory || return 1
    nas_open_daily_log || return 1
    nas_cleanup_expired_logs
    NAS_NEXT_LOG_CLEANUP_EPOCH=$(($(date '+%s') + NAS_LOG_CLEANUP_INTERVAL_S))
}

nas_watchdog_log() {
    printf '[NAS watchdog][%s] %s\n' "$(date '+%F %T')" "$*"
}

nas_health_check() {
    local response curl_rc clean_error
    NAS_HEALTH_LAST_ERROR=''
    response="$(curl --noproxy '*' --fail --silent --show-error \
        --connect-timeout 5 --max-time 8 "$NAS_HEALTH_URL" 2>&1)"
    curl_rc=$?
    clean_error="${response//$'\r'/ }"
    clean_error="${clean_error//$'\n'/ }"
    clean_error="${clean_error//$'\t'/ }"
    clean_error="${clean_error:0:NAS_HEALTH_ERROR_LIMIT}"
    if [[ "$curl_rc" -ne 0 ]]; then
        NAS_HEALTH_LAST_ERROR="curl exit ${curl_rc}: ${clean_error:-no response}"
        return 1
    fi
    if [[ "$response" != *"$NAS_HEALTH_EXPECTED_TEXT"* ]]; then
        NAS_HEALTH_LAST_ERROR="unexpected response: ${clean_error:-empty body}"
        return 1
    fi
    return 0
}

# Quote one string for a POSIX remote shell. The remote script contains no
# password and runs as the SSH login user.
nas_shell_single_quote() {
    local value="${1-}"
    value=${value//\'/\'\\\'\'}
    printf "'%s'" "$value"
}

nas_restart_remote_service() {
    local remote_script quoted_remote_script remote_command ssh_rc

    # This whole function runs with xtrace disabled.  The fixed remote script
    # validates PID identity before stopping anything and never uses pkill.
    set +x
    remote_script="$(cat <<'REMOTE_SCRIPT'
set -u
umask 077

APP_DIR='/volume1/web/app'
APP_SCRIPT='DS923_app.py'
PYTHON_BIN='/bin/python3'

if [ "$(id -un 2>/dev/null)" != 'sanyNAS' ]; then
    echo '[NAS remote] ERROR: remote script must run as sanyNAS.' >&2
    exit 70
fi
PID_FILE='/volume1/web/app/.DS923_app_watchdog.pid'
LOG_FILE='/volume1/web/app/DS923_app_watchdog.log'

if ! cd "$APP_DIR"; then
    echo '[NAS remote] ERROR: application directory is unavailable.' >&2
    exit 72
fi
APP_DIR=$(pwd -P) || exit 72

if [ ! -f "$APP_SCRIPT" ]; then
    echo '[NAS remote] ERROR: DS923_app.py was not found.' >&2
    exit 75
fi
if [ ! -x "$PYTHON_BIN" ] || ! "$PYTHON_BIN" -c 'import flask' >/dev/null 2>&1; then
    echo '[NAS remote] ERROR: /bin/python3 cannot import flask as sanyNAS.' >&2
    exit 74
fi

is_uint() {
    case "${1-}" in
        ''|*[!0-9]*) return 1 ;;
        *) return 0 ;;
    esac
}

process_start_time() {
    process_pid=${1-}
    stat_line=$(cat "/proc/$process_pid/stat" 2>/dev/null) || return 1
    # Remove everything through the final ") "; field 22 is then item 20.
    stat_tail=${stat_line##*) }
    set -- $stat_tail
    [ "$#" -ge 20 ] || return 1
    printf '%s\n' "${20}"
}

is_managed_process() {
    managed_pid=${1-}
    expected_start=${2-}
    is_uint "$managed_pid" && is_uint "$expected_start" || return 1
    [ "$managed_pid" -gt 1 ] || return 1
    [ -r "/proc/$managed_pid/stat" ] || return 1

    actual_start=$(process_start_time "$managed_pid") || return 1
    [ "$actual_start" = "$expected_start" ] || return 1

    actual_cwd=$(readlink "/proc/$managed_pid/cwd" 2>/dev/null) || return 1
    [ "$actual_cwd" = "$APP_DIR" ] || return 1

    [ -r "/proc/$managed_pid/cmdline" ] || return 1
    tr '\000' '\n' < "/proc/$managed_pid/cmdline" | grep -F -x -q "$APP_SCRIPT"
}

if [ ! -w "$APP_DIR" ]; then
    echo '[NAS remote] ERROR: sanyNAS cannot write to the application directory.' >&2
    exit 71
fi
if [ -L "$PID_FILE" ] || [ -d "$PID_FILE" ]; then
    echo '[NAS remote] ERROR: unsafe PID path; refusing to follow or replace it.' >&2
    exit 71
fi
if [ -e "$PID_FILE" ] && [ ! -f "$PID_FILE" ]; then
    echo '[NAS remote] ERROR: PID path is not a regular file.' >&2
    exit 71
fi

if [ -f "$PID_FILE" ] && [ ! -r "$PID_FILE" ]; then
    echo '[NAS remote] ERROR: sanyNAS cannot read the existing PID file.' >&2
    exit 71
fi
pid_probe=$(mktemp "${PID_FILE}.check.XXXXXX" 2>/dev/null) || {
    echo '[NAS remote] ERROR: sanyNAS cannot create PID files in the application directory.' >&2
    exit 71
}
if ! rm -f "$pid_probe"; then
    echo '[NAS remote] ERROR: sanyNAS cannot remove PID files from the application directory.' >&2
    exit 71
fi

if [ -L "$LOG_FILE" ] || [ -d "$LOG_FILE" ]; then
    echo '[NAS remote] ERROR: unsafe watchdog log path; refusing to follow or replace it.' >&2
    exit 76
fi
if [ -e "$LOG_FILE" ] && [ ! -f "$LOG_FILE" ]; then
    echo '[NAS remote] ERROR: watchdog log path is not a regular file.' >&2
    exit 76
fi
if ! ( : >>"$LOG_FILE" ) 2>/dev/null; then
    echo '[NAS remote] ERROR: sanyNAS cannot append to the watchdog log.' >&2
    exit 76
fi
if ! chmod 600 "$LOG_FILE" 2>/dev/null; then
    echo '[NAS remote] ERROR: sanyNAS cannot set watchdog log permissions.' >&2
    exit 76
fi

if [ -f "$PID_FILE" ]; then
    pid_record=$(cat "$PID_FILE" 2>/dev/null || true)
    set -- $pid_record
    if [ "$#" -eq 2 ] && is_managed_process "$1" "$2"; then
        old_pid=$1
        old_start=$2
        echo "[NAS remote] Stopping managed DS923_app.py PID $old_pid."
        kill -TERM "$old_pid" 2>/dev/null || true

        stop_wait=0
        while [ "$stop_wait" -lt 10 ] && is_managed_process "$old_pid" "$old_start"; do
            sleep 1
            stop_wait=$((stop_wait + 1))
        done

        if is_managed_process "$old_pid" "$old_start"; then
            echo "[NAS remote] PID $old_pid ignored TERM; sending KILL." >&2
            kill -KILL "$old_pid" 2>/dev/null || true
            sleep 1
        fi

        if is_managed_process "$old_pid" "$old_start"; then
            echo '[NAS remote] ERROR: managed process could not be stopped.' >&2
            exit 73
        fi
    else
        echo '[NAS remote] WARNING: stale or unsafe PID file ignored; no process was killed.' >&2
    fi
    if ! rm -f "$PID_FILE"; then
        echo '[NAS remote] ERROR: sanyNAS cannot remove the old PID file.' >&2
        exit 71
    fi
fi

nohup "$PYTHON_BIN" -u "$APP_SCRIPT" </dev/null >>"$LOG_FILE" 2>&1 &
new_pid=$!
sleep 2
new_start=$(process_start_time "$new_pid" 2>/dev/null || true)

if ! is_managed_process "$new_pid" "$new_start"; then
    echo '[NAS remote] ERROR: DS923_app.py exited during startup; inspect the watchdog log.' >&2
    if [ -f "$LOG_FILE" ] && [ ! -L "$LOG_FILE" ]; then
        echo '[NAS remote] Last 80 lines of DS923_app_watchdog.log:' >&2
        tail -n 80 "$LOG_FILE" >&2 || true
    fi
    exit 77
fi

pid_tmp=$(mktemp "${PID_FILE}.tmp.XXXXXX") || exit 78
[ -f "$pid_tmp" ] && [ ! -L "$pid_tmp" ] || exit 78
trap 'rm -f "$pid_tmp"' EXIT HUP INT TERM
printf '%s %s\n' "$new_pid" "$new_start" > "$pid_tmp" || exit 78
chmod 600 "$pid_tmp" 2>/dev/null || true
mv -f "$pid_tmp" "$PID_FILE" || exit 78
trap - EXIT HUP INT TERM

echo "[NAS remote] DS923_app.py started with PID $new_pid; log: $LOG_FILE"
REMOTE_SCRIPT
)"

    quoted_remote_script="$(nas_shell_single_quote "$remote_script")"
    remote_command="/bin/sh -c ${quoted_remote_script}"

    nas_watchdog_log "Health check failed ${NAS_FAILURE_THRESHOLD} times; connecting to ${NAS_USER}@${NAS_HOST}."
    env NAS_ASKPASS_MODE=1 \
            DISPLAY="${DISPLAY:-:0}" \
            SSH_ASKPASS="$SCRIPT_PATH" \
            SSH_ASKPASS_REQUIRE=force \
        setsid -w ssh -T \
            -o ConnectTimeout=10 \
            -o ConnectionAttempts=1 \
            -o ServerAliveInterval=10 \
            -o ServerAliveCountMax=3 \
            -o PreferredAuthentications=password,keyboard-interactive \
            -o PubkeyAuthentication=no \
            -o PasswordAuthentication=yes \
            -o NumberOfPasswordPrompts=1 \
            -o StrictHostKeyChecking=accept-new \
            "${NAS_USER}@${NAS_HOST}" "$remote_command" </dev/null
    ssh_rc=$?

    if [[ "$ssh_rc" -ne 0 ]]; then
        nas_watchdog_log "ERROR: SSH restart failed (exit ${ssh_rc}); will retry after another ${NAS_FAILURE_THRESHOLD} failed checks."
        return 1
    fi

    local check_index
    for ((check_index = 1; check_index <= NAS_STARTUP_CHECK_COUNT; ++check_index)); do
        sleep "$NAS_STARTUP_CHECK_INTERVAL_S"
        if nas_health_check; then
            nas_watchdog_log "DS923 service is healthy after restart."
            return 0
        fi
    done

    nas_watchdog_log "ERROR: remote command completed, but ${NAS_HEALTH_URL} is still unhealthy; inspect ${NAS_REMOTE_LOG_FILE}."
    return 1
}

run_nas_watchdog() {
    local dependency lock_root lock_dir lock_file failure_count=0
    umask 077

    if ! nas_initialize_logging; then
        printf '[NAS watchdog] ERROR: persistent logging could not be initialized; watchdog will exit.\n' >&2
        return 1
    fi

    for dependency in curl ssh setsid flock tail; do
        if ! command -v "$dependency" >/dev/null 2>&1; then
            nas_watchdog_log "ERROR: required command '$dependency' is not installed."
            return 1
        fi
    done

    lock_root="${XDG_RUNTIME_DIR:-/tmp}"
    if [[ ! -d "$lock_root" || ! -w "$lock_root" ]]; then
        lock_root='/tmp'
    fi
    lock_dir="${lock_root%/}/sy200_nas_ds923_watchdog_${UID}"
    if [[ -L "$lock_dir" ]]; then
        nas_watchdog_log "ERROR: unsafe watchdog lock directory $lock_dir."
        return 1
    fi
    if ! mkdir -p -m 700 "$lock_dir"; then
        nas_watchdog_log "ERROR: cannot create watchdog lock directory $lock_dir."
        return 1
    fi
    chmod 700 "$lock_dir" 2>/dev/null || true
    if [[ ! -d "$lock_dir" || -L "$lock_dir" || ! -O "$lock_dir" ]]; then
        nas_watchdog_log "ERROR: watchdog lock directory is not owned by the current user."
        return 1
    fi
    lock_file="$lock_dir/watchdog.lock"
    if [[ -L "$lock_file" || ( -e "$lock_file" && ! -f "$lock_file" ) ]]; then
        nas_watchdog_log "ERROR: unsafe watchdog lock file $lock_file."
        return 1
    fi

    # Append-open also avoids truncating the existing lock inode.
    exec {NAS_WATCHDOG_LOCK_FD}>>"$lock_file" || {
        nas_watchdog_log "ERROR: cannot open watchdog lock $lock_file."
        return 1
    }
    if ! flock -n "$NAS_WATCHDOG_LOCK_FD"; then
        nas_watchdog_log 'Another NAS watchdog is already running; this terminal will exit.'
        return 0
    fi

    trap 'nas_watchdog_log "Watchdog stopped."; exit 0' INT TERM
    nas_watchdog_log "Started: checking ${NAS_HEALTH_URL} every ${NAS_CHECK_INTERVAL_S}s; restart after ${NAS_FAILURE_THRESHOLD} consecutive failures."

    while true; do
        if ! nas_log_maintenance; then
            nas_watchdog_log 'ERROR: log rotation or cleanup failed; continuing with the current open log.'
        fi
        if nas_health_check; then
            if [[ "$failure_count" -gt 0 ]]; then
                nas_watchdog_log 'Health recovered; failure counter reset.'
            fi
            failure_count=0
        else
            failure_count=$((failure_count + 1))
            nas_watchdog_log "Health check failed (${failure_count}/${NAS_FAILURE_THRESHOLD}): ${NAS_HEALTH_LAST_ERROR:-unknown error}"
            if [[ "$failure_count" -ge "$NAS_FAILURE_THRESHOLD" ]]; then
                nas_restart_remote_service || true
                # Require three fresh failed checks before another SSH attempt,
                # preventing an unreachable NAS from being hammered every 30s.
                failure_count=0
            fi
        fi
        sleep "$NAS_CHECK_INTERVAL_S"
    done
}

# Run directly as the dedicated watchdog process. SSH_ASKPASS re-entry
# exits near the top of this file before reaching this call.
run_nas_watchdog
