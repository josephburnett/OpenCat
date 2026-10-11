#!/bin/sh
# Install the Kilo mind daemon as a systemd user service. Idempotent.
#
#   mind/systemd/install.sh [--start] [--kilo PATH] [--no-linger]
#
# - copies kilo.service to ~/.config/systemd/user/ (ExecStart rewritten when
#   --kilo is given, or when ~/OpenCat/.venv/bin/kilo does not exist but
#   `kilo` is on PATH);
# - creates ~/.config/kilo/env from env.example if it does not exist;
# - enables lingering (loginctl enable-linger) so the user manager, and with
#   it kilo.service, starts at boot without a login;
# - systemctl --user daemon-reload, enable kilo.service;
# - starts (restarts) it only with --start.
set -eu

here=$(cd "$(dirname "$0")" && pwd)
start=0
linger=1
kilo_bin=""
while [ $# -gt 0 ]; do
    case "$1" in
        --start) start=1 ;;
        --no-linger) linger=0 ;;
        --kilo) kilo_bin=${2:?--kilo needs a path}; shift ;;
        -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
        *) echo "install.sh: unknown option $1" >&2; exit 2 ;;
    esac
    shift
done

config=${XDG_CONFIG_HOME:-$HOME/.config}
unit_dir=$config/systemd/user
conf_dir=$config/kilo
mkdir -p "$unit_dir" "$conf_dir" "$HOME/kilo-logs"

default_bin=$HOME/OpenCat/.venv/bin/kilo
if [ -z "$kilo_bin" ] && [ ! -x "$default_bin" ] && command -v kilo >/dev/null 2>&1; then
    kilo_bin=$(command -v kilo)
fi
tmp=$(mktemp)
trap 'rm -f "$tmp"' EXIT
if [ -n "$kilo_bin" ]; then
    sed "s|^ExecStart=%h/OpenCat/.venv/bin/kilo |ExecStart=$kilo_bin |" "$here/kilo.service" > "$tmp"
else
    cp "$here/kilo.service" "$tmp"
    [ -x "$default_bin" ] || echo "warning: $default_bin not found (install the mind venv, or pass --kilo PATH)" >&2
fi
if cmp -s "$tmp" "$unit_dir/kilo.service"; then
    echo "unit unchanged: $unit_dir/kilo.service"
else
    install -m 0644 "$tmp" "$unit_dir/kilo.service"
    echo "installed $unit_dir/kilo.service"
fi

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
