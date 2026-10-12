#!/bin/sh
# Install the Kilo mind daemon as a systemd service. Idempotent.
#
#   sudo mind/systemd/install.sh --system [--user NAME] [--start] [--kilo PATH]
#        mind/systemd/install.sh [--start] [--kilo PATH] [--no-linger]
#
# --system (recommended; needs root, prints the sudo command otherwise):
# - writes kilo-system.service to /etc/systemd/system/kilo.service, running
#   as NAME (default: the user who ran sudo) with Group=dialout, the socket
#   in /run/kilo, settings from ~NAME/.config/kilo/env;
# - creates ~NAME/.config/kilo/env from env.example if missing, ~NAME/kilo-logs;
# - disables and stops the user unit and turns lingering off, if present;
# - systemctl daemon-reload, enable kilo.service; (re)starts it only with --start.
#
# Without --system (the fallback): kilo.service as NAME's *user* service in
# ~/.config/systemd/user/, the env file, lingering (loginctl enable-linger),
# systemctl --user daemon-reload + enable; (re)starts only with --start.
#
# --kilo PATH: the kilo executable (default ~/OpenCat/.venv/bin/kilo, or
# `kilo` on PATH in user mode when that does not exist).
set -eu

here=$(cd "$(dirname "$0")" && pwd)
self="$here/$(basename "$0")"
sysroot=${KILO_SYSROOT:-}   # tests only: a fake / for /etc and /var/lib
start=0
linger=1
system=0
target_user=""
kilo_bin=""
while [ $# -gt 0 ]; do
    case "$1" in
        --start) start=1 ;;
        --no-linger) linger=0 ;;
        --system) system=1 ;;
        --user) target_user=${2:?--user needs a name}; shift ;;
        --kilo) kilo_bin=${2:?--kilo needs a path}; shift ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "install.sh: unknown option $1" >&2; exit 2 ;;
    esac
    shift
done

# Copy $1 to $2 (mode 0644) unless identical; say which.
put() {
    if cmp -s "$1" "$2"; then
        echo "unit unchanged: $2"
    else
        install -m 0644 "$1" "$2"
        echo "installed $2"
    fi
}

tmp=$(mktemp)
trap 'rm -f "$tmp"' EXIT

# ---- system service -------------------------------------------------------------
if [ "$system" = 1 ]; then
    if [ -z "$target_user" ]; then
        if [ "$(id -u)" = 0 ]; then target_user=${SUDO_USER:-}; else target_user=$(id -un); fi
    fi
    if [ -z "$target_user" ] || [ "$target_user" = root ]; then
        echo "install.sh --system: say which user runs the daemon: --user NAME (not root)" >&2
        exit 2
    fi
    if [ "$(id -u)" != 0 ]; then
        cmd="sudo $self --system --user $target_user"
        [ -n "$kilo_bin" ] && cmd="$cmd --kilo $kilo_bin"
        [ "$start" = 1 ] && cmd="$cmd --start"
        echo "install.sh --system needs root. Run:" >&2
        echo "    $cmd" >&2
        exit 1
    fi
    pw=$(getent passwd "$target_user") || { echo "install.sh: no user $target_user" >&2; exit 2; }
    home=$(echo "$pw" | cut -d: -f6)
    group=dialout
    if ! getent group dialout >/dev/null; then
        group=$(id -gn "$target_user")
        echo "warning: no dialout group; running as group $group" >&2
    fi
    [ -n "$kilo_bin" ] || kilo_bin=$home/OpenCat/.venv/bin/kilo
    [ -x "$kilo_bin" ] || echo "warning: $kilo_bin not found (install the mind venv, or pass --kilo PATH)" >&2

    unit_dir=$sysroot/etc/systemd/system
    mkdir -p "$unit_dir"
    sed -e "s|@USER@|$target_user|g" -e "s|@GROUP@|$group|g" -e "s|@HOME@|$home|g" \
        -e "s|@KILO@|$kilo_bin|g" "$here/kilo-system.service" > "$tmp"
    put "$tmp" "$unit_dir/kilo.service"

    conf_dir=$home/.config/kilo
    for d in "$home/.config" "$conf_dir" "$home/kilo-logs"; do
        if [ ! -d "$d" ]; then mkdir -p "$d" && chown "$target_user:" "$d"; fi
    done
    if [ -f "$conf_dir/env" ]; then
        echo "kept $conf_dir/env"
    else
        install -m 0644 "$here/env.example" "$conf_dir/env"
        chown "$target_user:" "$conf_dir/env"
        echo "created $conf_dir/env (check KILO_PORT)"
    fi

    # The user unit would fight the system one for the port: disable and stop it.
    user_wants=$home/.config/systemd/user/default.target.wants/kilo.service
    if [ -e "$user_wants" ] || [ -L "$user_wants" ]; then
        if systemctl --user -M "$target_user@" disable --now kilo.service 2>/dev/null; then
            echo "disabled and stopped the user unit (systemctl --user kilo)"
        else
            rm -f "$user_wants"
            echo "disabled the user unit (its manager is not running)"
        fi
    elif systemctl --user -M "$target_user@" is-active --quiet kilo.service 2>/dev/null; then
        systemctl --user -M "$target_user@" stop kilo.service
        echo "stopped the user unit"
    fi
    if [ -e "$sysroot/var/lib/systemd/linger/$target_user" ] || \
            loginctl show-user "$target_user" -p Linger 2>/dev/null | grep -q '=yes'; then
        loginctl disable-linger "$target_user"
        echo "turned lingering off for $target_user (not needed by a system unit)"
    fi

    systemctl daemon-reload
    systemctl enable kilo.service
    if [ "$start" = 1 ]; then
        systemctl restart kilo.service
        echo "started: systemctl status kilo; journalctl -u kilo -f; kilo daemon"
    else
        echo "enabled; starts at the next boot (or now: sudo systemctl start kilo)"
    fi
    exit 0
fi

# ---- user service (fallback) ----------------------------------------------------------
if [ -e "$sysroot/etc/systemd/system/kilo.service" ]; then
    echo "warning: the system unit /etc/systemd/system/kilo.service is installed; two daemons would" \
         "fight for the port: sudo systemctl disable --now kilo first" >&2
fi
config=${XDG_CONFIG_HOME:-$HOME/.config}
unit_dir=$config/systemd/user
conf_dir=$config/kilo
mkdir -p "$unit_dir" "$conf_dir" "$HOME/kilo-logs"

default_bin=$HOME/OpenCat/.venv/bin/kilo
if [ -z "$kilo_bin" ] && [ ! -x "$default_bin" ] && command -v kilo >/dev/null 2>&1; then
    kilo_bin=$(command -v kilo)
fi
if [ -n "$kilo_bin" ]; then
    sed "s|^ExecStart=%h/OpenCat/.venv/bin/kilo |ExecStart=$kilo_bin |" "$here/kilo.service" > "$tmp"
else
    cp "$here/kilo.service" "$tmp"
    [ -x "$default_bin" ] || echo "warning: $default_bin not found (install the mind venv, or pass --kilo PATH)" >&2
fi
put "$tmp" "$unit_dir/kilo.service"

if [ -f "$conf_dir/env" ]; then
    echo "kept $conf_dir/env"
else
    install -m 0644 "$here/env.example" "$conf_dir/env"
    echo "created $conf_dir/env (check KILO_PORT)"
fi

if ! id -nG | tr ' ' '\n' | grep -qx dialout; then
    echo "warning: $(id -un) is not in the dialout group: sudo usermod -aG dialout $(id -un), then log out and in" >&2
fi

if [ "$linger" = 1 ]; then
    if loginctl show-user "$(id -un)" -p Linger 2>/dev/null | grep -q '=yes'; then
        echo "lingering already enabled"
    elif loginctl enable-linger "$(id -un)" 2>/dev/null; then
        echo "enabled lingering for $(id -un)"
    else
        echo "warning: could not enable lingering; run: sudo loginctl enable-linger $(id -un)" >&2
    fi
fi

systemctl --user daemon-reload
systemctl --user enable kilo.service
if [ "$start" = 1 ]; then
    systemctl --user restart kilo.service
    echo "started: systemctl --user status kilo; journalctl --user -u kilo -f"
else
    echo "not started (use --start, or: systemctl --user start kilo)"
fi
