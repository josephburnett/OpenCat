#!/bin/sh
# Trim the Pi's boot of things Kilo does not need (docs/setup.md "The daemon as a system service, and boot time").
# Run on the Pi, once, with sudo; safe to run again (it only changes what is
# still there to change), and prints every change. Reboot afterwards.
#
#   sudo mind/systemd/trim-boot.sh [--dry-run]
#
# What it does:
# - cloud-init (Pi Imager's first-boot provisioning, done long ago): touches
#   /etc/cloud/cloud-init.disabled and disables its units. Skipped unless the
#   Wi-Fi settings it wrote are on disk (/etc/netplan/*.yaml or
#   /etc/NetworkManager/system-connections/*), so the network still comes up.
# - NetworkManager-wait-online.service and systemd-networkd-wait-online.service:
#   masked, so nothing (network-online.target, a package upgrade) pulls them
#   in again and nothing at boot waits for the network. NetworkManager itself
#   is untouched: Wi-Fi still connects, also when the dongle is plugged in later.
# - the kernel line (/boot/firmware/cmdline.txt): drops `splash` (the plymouth
#   boot splash), keeps `quiet`; the old file is kept as cmdline.txt.kilo-backup.
#
# What it leaves alone: NetworkManager, wpa_supplicant, ssh, tailscaled, udev,
# and rpi-resize-swap-file.service (not known for certain to be a one-shot
# that has already done its work; see docs/setup.md for how to look).
set -eu

root=${KILO_SYSROOT:-}   # tests only: a fake /
dry=0
case "${1:-}" in
    --dry-run|-n) dry=1 ;;
    "") ;;
    -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
    *) echo "trim-boot.sh: unknown option $1" >&2; exit 2 ;;
esac
if [ "$dry" = 0 ] && [ "$(id -u)" != 0 ]; then
    echo "trim-boot.sh needs root. Run:  sudo $0   (or look first: $0 --dry-run)" >&2
    exit 1
fi

changed=0
# act DESCRIPTION CMD...: run CMD (or say it would), and count the change.
act() {
    what=$1; shift
    changed=$((changed + 1))
    if [ "$dry" = 1 ]; then
        echo "would: $what    ($*)"
    else
        echo "$what    ($*)"
        "$@"
    fi
}

# The unit's enablement state, or "" when there is no such unit.
state() { systemctl is-enabled "$1" 2>/dev/null || true; }

# ---- cloud-init ------------------------------------------------------------------------
wifi_on_disk=0
for f in "$root"/etc/netplan/*.yaml "$root"/etc/NetworkManager/system-connections/*; do
    [ -e "$f" ] && wifi_on_disk=1 && break
done
if [ ! -d "$root/etc/cloud" ]; then
    echo "cloud-init: not installed"
elif [ "$wifi_on_disk" = 0 ]; then
    echo "cloud-init: LEFT ON: no /etc/netplan/*.yaml and no NetworkManager connection file," >&2
    echo "  so cloud-init may be what sets up the Wi-Fi at each boot (nmcli connection show)" >&2
else
    if [ -e "$root/etc/cloud/cloud-init.disabled" ]; then
        echo "cloud-init: already disabled (/etc/cloud/cloud-init.disabled)"
    else
        act "cloud-init: disabled" touch "$root/etc/cloud/cloud-init.disabled"
    fi
    for u in cloud-init-local.service cloud-init-main.service cloud-init-network.service \
             cloud-init.service cloud-config.service cloud-final.service; do
        case $(state "$u") in
            enabled|enabled-runtime) act "$u: disabled" systemctl disable "$u" ;;
            "") ;;
            *) echo "$u: $(state "$u") (left as it is)" ;;
        esac
    done
fi

# ---- wait-online: nothing at boot waits for the network --------------------------------
for u in NetworkManager-wait-online.service systemd-networkd-wait-online.service; do
    s=$(state "$u")
    case $s in
        masked) echo "$u: already masked" ;;
        "") echo "$u: not installed" ;;
        *)
            case $s in enabled*) act "$u: disabled" systemctl disable "$u" ;; esac
            act "$u: masked (was $s)" systemctl mask "$u" ;;
    esac
done

# ---- boot splash -----------------------------------------------------------------------
cmdline=$root/boot/firmware/cmdline.txt
[ -f "$cmdline" ] || cmdline=$root/boot/cmdline.txt
if [ ! -f "$cmdline" ]; then
    echo "cmdline.txt: not found (looked in /boot/firmware and /boot)" >&2
elif tr ' ' '\n' < "$cmdline" | grep -qx splash; then
    new=$(tr -s ' \n' '  ' < "$cmdline" | awk '{s=""; for (i = 1; i <= NF; i++) if ($i != "splash") s = s (s == "" ? "" : " ") $i; print s}')
    if [ "$dry" = 1 ]; then
        changed=$((changed + 1))
        echo "would: $cmdline: drop splash, keep quiet; new line:"
        echo "    $new"
    else
        [ -e "$cmdline.kilo-backup" ] || cp -p "$cmdline" "$cmdline.kilo-backup"
        printf '%s\n' "$new" > "$cmdline.kilo-new"
        mv "$cmdline.kilo-new" "$cmdline"
        changed=$((changed + 1))
        echo "$cmdline: dropped splash (backup: $cmdline.kilo-backup); now:"
        echo "    $new"
    fi
else
    echo "cmdline.txt: no splash"
fi

echo "rpi-resize-swap-file.service: left alone (look: systemctl cat rpi-resize-swap-file.service)"
if [ "$changed" = 0 ]; then
    echo "nothing to change"
elif [ "$dry" = 1 ]; then
    echo "$changed change(s) would be made; run without --dry-run (with sudo) to make them"
else
    systemctl daemon-reload
    echo "$changed change(s) made; reboot to see the effect (sudo reboot), then: systemd-analyze; systemd-analyze critical-chain kilo.service"
fi
