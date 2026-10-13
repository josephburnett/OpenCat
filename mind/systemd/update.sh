#!/bin/sh
# Update the mind on the Pi and restart the daemon. No root needed after
# `install.sh --system` (it installs the sudoers rule for the restart).
#
#   mind/systemd/update.sh [--flash] [--dry-run]
#
# - git pull --ff-only (prints what came in)
# - .venv/bin/pip install -e mind, only if mind/pyproject.toml changed
# - sudo -n systemctl restart kilo.service, only if mind/ or protocol/ changed
#   (or --flash: body/ changed means the firmware is rebuilt and flashed after
#   the restart: Kilo goes limp for about a minute; floor only, owner informed)
# - kilo daemon: the daemon's one-line status at the end
# Writes only under the checkout and /run: works in safe mode.
set -eu

flash=0; dry=0
for a in "$@"; do
    case "$a" in
        --flash) flash=1 ;;
        --dry-run|-n) dry=1 ;;
        -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
        *) echo "update.sh: unknown argument $a" >&2; exit 2 ;;
    esac
done
repo=$(cd "$(dirname "$0")/../.." && pwd)
cd "$repo"
kilo=$repo/.venv/bin/kilo
pip=$repo/.venv/bin/pip
PATH=$HOME/.local/bin:$PATH; export PATH   # arduino-cli for kilo flash

run() { if [ "$dry" = 1 ]; then echo "would: $*"; else echo ">> $*"; "$@"; fi; }

old=$(git rev-parse HEAD)
if [ "$dry" = 1 ]; then
    git fetch -q origin
    new=$(git rev-parse origin/main)
    echo "would: git pull --ff-only  ($(git rev-parse --short "$old") -> $(git rev-parse --short "$new"))"
else
    git pull -q --ff-only origin main
    new=$(git rev-parse HEAD)
fi
if [ "$old" = "$new" ]; then
    echo "already up to date ($(git rev-parse --short "$new"))"
    changed=""
else
    git log --oneline "$old..$new" | sed 's/^/  /'
    changed=$(git diff --name-only "$old" "$new")
fi
in_changed() { printf '%s\n' "$changed" | grep -q "$1"; }

if in_changed '^mind/pyproject.toml'; then
    run "$pip" install -q -e mind
fi
if in_changed '^mind/' || in_changed '^protocol/'; then
    # the sudoers rule allows only these systemctl commands, so ask about that command
    # (sudo -n true would fail even with the rule in place)
    if sudo -n -l systemctl restart kilo.service >/dev/null 2>&1; then
        run sudo -n systemctl restart kilo.service
    else
        echo "restart needs a password here (no sudoers rule: sudo mind/systemd/install.sh --system adds it): run  sudo systemctl restart kilo" >&2
    fi
else
    [ -z "$changed" ] || echo "daemon code unchanged: no restart"
fi
if [ "$flash" = 1 ]; then
    if in_changed '^body/' || in_changed '^protocol/'; then
        run "$kilo" -v flash
    else
        echo "firmware unchanged: no flash"
    fi
elif in_changed '^body/' || in_changed '^protocol/'; then
    echo "firmware changed: flash it when Kilo is on the floor and the owner knows:  $0 --flash   (or: kilo flash)"
fi
[ "$dry" = 1 ] || { sleep 2; "$kilo" daemon 2>&1 | head -3; }
