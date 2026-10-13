#!/bin/sh
# Safe mode: the Pi's system files read-only, so a power cut cannot damage them
# (docs/setup.md "Power: safe mode"). Run on the Pi.
#
#   mind/systemd/safe-mode.sh status
#   sudo mind/systemd/safe-mode.sh on|off [--dry-run]
#
# on:  the root filesystem becomes an overlay (Raspberry Pi OS "overlay file
#      system", package overlayroot: writes go to RAM and are gone at the next
#      boot) and /boot/firmware is mounted read-only. /data (data-partition.sh)
#      and the bind mounts on it (/home/joe, /var/lib/tailscale,
#      /var/log/journal, /var/lib/systemd/timesync) stay writable. Refused
#      unless /data and every bind mount are in place, now and in /etc/fstab.
# off: the reverse (for apt, unit files, Wi-Fi profiles; see docs/setup.md).
# Both take effect at the next reboot, and print every change.
#
# How: `raspi-config nonint enable_bootro|disable_bootro|enable_overlayfs`
# (RPi-Distro/raspi-config, trixie branch), then the kernel line gets
# `overlayroot=tmpfs:recurse=0` instead of raspi-config's `overlayroot=tmpfs`:
# with overlayroot's default recurse=1 every fstab mount under / (so /data
# too) would get a RAM overlay and nothing would persist. While the overlay
# is active, /etc/fstab is edited on the real root via overlayroot-chroot.
set -eu

root=${KILO_SYSROOT:-}   # tests only: a fake /
cmd=""
dry=0
for a in "$@"; do
    case "$a" in
        on|off|status) cmd=$a ;;
        --dry-run|-n) dry=1 ;;
        -h|--help) sed -n '2,21p' "$0"; exit 0 ;;
        *) echo "safe-mode.sh: unknown argument $a" >&2; exit 2 ;;
    esac
done
[ -n "$cmd" ] || { echo "usage: safe-mode.sh on|off|status [--dry-run]" >&2; exit 2; }

user=${SAFE_MODE_USER:-${SUDO_USER:-joe}}
[ "$user" != root ] || user=joe
binds="/home/$user /var/lib/tailscale /var/log/journal /var/lib/systemd/timesync"
mounts=$root/proc/mounts
cmdline=$root/boot/firmware/cmdline.txt
token=overlayroot=tmpfs:recurse=0

# The fstab that the next boot reads: while the overlay is active, the one on
# the real (lower) root; /etc/fstab is overlayroot's generated copy.
overlay_now() { awk '$2 == "/" && $3 == "overlay" { f = 1 } END { exit !f }' "$mounts"; }
fstab=$root/etc/fstab
overlay_now && [ -f "$root/media/root-ro/etc/fstab" ] && fstab=$root/media/root-ro/etc/fstab

mounted() { awk -v t="$1" '$2 == t { f = 1 } END { exit !f }' "$mounts"; }
mount_opts() { awk -v t="$1" '$2 == t { o = $4 } END { print o }' "$mounts"; }
in_fstab() { awk -v t="$1" '$1 !~ /^#/ && $2 == t { f = 1 } END { exit !f }' "$fstab"; }
boot_ro_now() { case ",$(mount_opts /boot/firmware)," in *,ro,*) return 0 ;; esac; return 1; }
boot_ro_conf() { awk '$1 !~ /^#/ && $2 == "/boot/firmware" { n = split($4, o, ","); for (i = 1; i <= n; i++) if (o[i] == "ro") f = 1 } END { exit !f }' "$fstab"; }
# The overlayroot= word on the kernel line for the next boot, or "".
overlay_conf() { tr ' ' '\n' < "$cmdline" | grep '^overlayroot=' | head -1 || true; }

# ---- status -----------------------------------------------------------------------------
status() {
    if overlay_now; then echo "overlay root now:        ON  (system changes vanish at reboot)"
    else echo "overlay root now:        off (the root filesystem is written directly)"; fi
    c=$(overlay_conf)
    case $c in
        "$token") echo "overlay root next boot:  ON  ($token in $cmdline)" ;;
        "") echo "overlay root next boot:  off" ;;
        *) echo "overlay root next boot:  ON, BUT '$c' lacks recurse=0: /data would be under a RAM overlay too (run: sudo $0 on)" ;;
    esac
    if boot_ro_now; then echo "/boot/firmware now:      read-only"; else echo "/boot/firmware now:      writable"; fi
    if boot_ro_conf; then echo "/boot/firmware next boot: read-only"; else echo "/boot/firmware next boot: writable"; fi
    if mounted /data; then echo "/data:                   mounted"; else echo "/data:                   NOT mounted"; fi
    for b in $binds; do
        now=missing; mounted "$b" && now=mounted
        conf="not in fstab"; in_fstab "$b" && conf="in fstab"
        printf '%-24s %s, %s\n' "$b:" "$now" "$conf"
    done
    [ -n "$(ls "$root"/boot/firmware/initramfs* 2>/dev/null)" ] || \
        echo "warning: no /boot/firmware/initramfs*: overlayroot runs in the initramfs, so the overlay would not start"
}
if [ "$cmd" = status ]; then
    status
    exit 0
fi

if [ "$dry" = 0 ] && [ "$(id -u)" != 0 ]; then
    echo "safe-mode.sh $cmd needs root. Run:  sudo $0 $cmd   (or look first: $0 $cmd --dry-run)" >&2
    exit 1
fi

changed=0
act() {
    what=$1; shift
    changed=$((changed + 1))
    if [ "$dry" = 1 ]; then
        printf 'would: %s    (%s)\n' "$what" "$*"
    else
        printf '%s    (%s)\n' "$what" "$*"
        "$@"
    fi
}

# Set the kernel line's overlayroot= word to $1 ("" = none), keeping the rest.
set_cmdline_overlay() {
    new=$(tr -s ' \n' '  ' < "$cmdline" | awk -v t="$1" '{ s = t; for (i = 1; i <= NF; i++) if ($i !~ /^overlayroot=/) s = s (s == "" ? "" : " ") $i; print s }')
    changed=$((changed + 1))
    if [ "$dry" = 1 ]; then
        echo "would: $cmdline: ${1:-no overlayroot=}; new line:"
        echo "    $new"
        return
    fi
    ro=0
    if boot_ro_now; then ro=1; echo "remount /boot/firmware rw    (mount -o remount,rw /boot/firmware)"; mount -o remount,rw /boot/firmware; fi
    [ -e "$cmdline.kilo-safe-mode-backup" ] || cp -p "$cmdline" "$cmdline.kilo-safe-mode-backup"
    printf '%s\n' "$new" > "$cmdline.kilo-new"
    mv "$cmdline.kilo-new" "$cmdline"
    echo "$cmdline: ${1:-overlayroot= removed}; now:"
    echo "    $new"
    if [ "$ro" = 1 ]; then echo "remount /boot/firmware ro    (mount -o remount,ro /boot/firmware)"; mount -o remount,ro /boot/firmware; fi
}

# /boot/firmware read-only (on) or writable (off) from the next boot.
set_bootro() {
    if [ "$1" = on ]; then
        boot_ro_conf && { echo "/boot/firmware: already read-only at boot (/etc/fstab)"; return; }
        rc=enable_bootro sedx='s#^\([^#].*[[:space:]]/boot/firmware[[:space:]].*\)defaults#\1defaults,ro#'
        what="/boot/firmware: read-only from the next boot"
    else
        boot_ro_conf || { echo "/boot/firmware: already writable at boot (/etc/fstab)"; return; }
        rc=disable_bootro sedx='s#^\([^#].*[[:space:]]/boot/firmware[[:space:]].*\),ro\([[:space:],]\)#\1\2#'
        what="/boot/firmware: writable from the next boot"
    fi
    if overlay_now; then
        # raspi-config refuses while the overlay is active; edit the real root's fstab.
        act "$what (real root's /etc/fstab)" overlayroot-chroot sed -i -e "$sedx" /etc/fstab
    else
        act "$what" raspi-config nonint "$rc"
    fi
}

if [ "$cmd" = on ]; then
    missing=""
    mounted /data || missing="$missing
  /data is not mounted"
    in_fstab /data || missing="$missing
  /data has no line in /etc/fstab"
    for b in $binds; do
        mounted "$b" || missing="$missing
  $b is not bind-mounted from /data"
        in_fstab "$b" || missing="$missing
  $b has no line in /etc/fstab"
    done
    if [ -n "$missing" ]; then
        echo "REFUSED: safe mode would freeze these at their state on the root filesystem:$missing" >&2
        echo "Set up the data partition first (docs/setup.md \"Power: safe mode\": mind/systemd/data-partition.sh)." >&2
        exit 1
    fi
    command -v raspi-config >/dev/null || { echo "REFUSED: raspi-config is not installed" >&2; exit 1; }
    set_bootro on
    c=$(overlay_conf)
    if [ "$c" = "$token" ]; then
        echo "overlay root: already on for the next boot ($token)"
    else
        if [ -z "$c" ] && ! overlay_now; then
            # raspi-config installs overlayroot (apt; needs the network the first time) and adds overlayroot=tmpfs.
            act "overlay root: on (raspi-config)" raspi-config nonint enable_overlayfs
        fi
        set_cmdline_overlay "$token"
    fi
else
    c=$(overlay_conf)
    if [ -z "$c" ]; then
        echo "overlay root: already off for the next boot"
    else
        set_cmdline_overlay ""
    fi
    set_bootro off
fi

if [ "$changed" = 0 ]; then
    echo "nothing to change"
elif [ "$dry" = 1 ]; then
    echo "$changed change(s) would be made; run without --dry-run (with sudo) to make them"
else
    echo "$changed change(s) made; reboot to apply: sudo reboot   (then: $0 status)"
fi
