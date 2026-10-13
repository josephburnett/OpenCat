#!/bin/sh
# Give the Pi's SD card a writable data partition (docs/setup.md "Power: safe mode").
# Run on a Linux host (rtb, a laptop) with the card in a USB reader, as root.
# NOT on the Pi: the Pi's root cannot be shrunk while it is mounted.
#
#   sudo mind/systemd/data-partition.sh /dev/sdX --dry-run   # look first
#   sudo mind/systemd/data-partition.sh /dev/sdX --yes       # do it
#        [--root-size GIB] [--user NAME]
#
# What it does, each step only if still needed (safe to run again, also after
# a failure halfway), printing every change:
# - checks the card: a whole disk, not the host's own, nothing on it mounted,
#   p1 vfat with cmdline.txt, p2 ext4 (and no p3, or p3 ext4 "kilo-data");
# - e2fsck -f, then shrinks the p2 filesystem and partition to GIB (default 16)
#   GiB; same start sector, so root=PARTUUID=...-02 still matches;
# - p3 over the rest of the card: ext4, label kilo-data;
# - copies (cp -a: owners, modes, links kept) from p2 into p3:
#   /home/NAME (default joe), /var/lib/tailscale, /var/log/journal,
#   /var/lib/systemd/timesync, as /data/<same path>; a copy already on p3 is kept;
# - p2's /etc/fstab: /data plus four bind mounts (the paths above at their
#   usual places); the originals stay on p2, hidden under the mounts, and are
#   what you get if p3 ever fails to mount (Kilo then does not start);
# - drops a `resize` word from cmdline.txt if present (the first-boot
#   resize in the initramfs would try to grow p2 over p3).
# Without --yes it only shows the card and the plan.
set -eu

target_gib=16
user=joe
dev=""
dry=0
yes=0
while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run|-n) dry=1 ;;
        --yes) yes=1 ;;
        --root-size) target_gib=${2:?--root-size needs a number of GiB}; shift ;;
        --user) user=${2:?--user needs a name}; shift ;;
        -h|--help) sed -n '2,27p' "$0"; exit 0 ;;
        /dev/*) dev=$1 ;;
        *) echo "data-partition.sh: unknown argument $1" >&2; exit 2 ;;
    esac
    shift
done
case $target_gib in ''|*[!0-9]*) echo "data-partition.sh: --root-size takes whole GiB" >&2; exit 2 ;; esac
if [ -z "$dev" ]; then
    echo "usage: sudo $0 /dev/sdX [--dry-run|--yes]   (the card's whole device; lsblk shows it)" >&2
    exit 2
fi
if [ "$(id -u)" != 0 ]; then
    echo "data-partition.sh reads the card directly and needs root, also for --dry-run:  sudo $0 $dev --dry-run" >&2
    exit 1
fi
[ "$yes" = 1 ] || dry=1   # no --yes: show the plan only

die() { echo "REFUSED: $*" >&2; exit 1; }
gib() { awk -v b="$1" 'BEGIN { printf "%.1f GiB", b / 1073741824 }'; }

# ---- is this the Kilo card? -------------------------------------------------------------
[ "$(lsblk -dno TYPE "$dev" 2>/dev/null)" = disk ] || die "$dev is not a whole disk (lsblk -dno TYPE $dev); give /dev/sdX, not /dev/sdX1"
rootsrc=$(findmnt -nvo SOURCE / 2>/dev/null || true)
if [ -n "$rootsrc" ] && lsblk -nrpso NAME,TYPE "$rootsrc" 2>/dev/null | awk '$2 == "disk" { print $1 }' | grep -qx "$dev"; then
    die "$dev holds this host's own root filesystem"
fi
echo "== $dev"
lsblk -o NAME,SIZE,TYPE,FSTYPE,LABEL,MOUNTPOINT,MODEL "$dev"
mounted=$(lsblk -nrpo NAME,MOUNTPOINT "$dev" | awk 'NF > 1 { print "  " $1 " on " $2 }')
[ -z "$mounted" ] || die "something on $dev is mounted; unmount it first, then run this again:
$(lsblk -nrpo NAME,MOUNTPOINT "$dev" | awk 'NF > 1 { print "  sudo umount " $2 }')"

case $dev in *[0-9]) pfx=${dev}p ;; *) pfx=$dev ;; esac
p1=${pfx}1 p2=${pfx}2 p3=${pfx}3
nparts=$(lsblk -nrpo NAME,TYPE "$dev" | awk '$2 == "part"' | wc -l)
fstype() { blkid -p -o value -s TYPE "$1" 2>/dev/null || true; }
[ "$(fstype "$p1")" = vfat ] || die "$p1 is not vfat (not a Raspberry Pi OS card?)"
[ "$(fstype "$p2")" = ext4 ] || die "$p2 is not ext4 (not a Raspberry Pi OS card?)"
have_p3=0
case $nparts in
    2) ;;
    3) [ "$(fstype "$p3")" = ext4 ] && [ "$(blkid -p -o value -s LABEL "$p3" 2>/dev/null)" = kilo-data ] \
           || die "$p3 exists and is not ext4 labelled kilo-data: unknown layout, not touching it"
       # it must sit after p2 and be a real size (an earlier version let sfdisk drop it into the
       # 7 MiB gap before p1)
       p3_start=$(lsblk -ndo START "$p3" | tr -d ' ')
       p2_end=$(( $(lsblk -ndo START "$p2" | tr -d ' ') + $(lsblk -bdno SIZE "$p2" | tr -d ' ') / $(lsblk -dno LOG-SEC "$dev" | tr -d ' ') ))
       if [ "${p3_start:-0}" -lt "$p2_end" ] || [ "$(lsblk -bdno SIZE "$p3" | tr -d ' ')" -lt 2147483648 ]; then
           die "$p3 is misplaced or tiny (starts at sector ${p3_start:-?}, p2 ends at $p2_end; $(gib "$(lsblk -bdno SIZE "$p3" | tr -d ' ')")).
Delete it and run this again (only an empty or half-copied /data is lost):  sudo sfdisk --delete $dev 3"
       fi
       have_p3=1 ;;
    *) die "$dev has $nparts partitions; expected 2 (or 3 after an earlier run)" ;;
esac

# ---- mounts in a scratch dir, always undone ---------------------------------------------
m=$(mktemp -d "${TMPDIR:-/tmp}/kilo-card.XXXXXX")
mounted_here=""
cleanup() {
    # an interrupted du/cp may still hold the mount for a moment: retry, then lazy-unmount
    for d in $mounted_here; do
        n=0
        until umount "$m/$d" 2>/dev/null; do
            n=$((n + 1))
            if [ "$n" -ge 5 ]; then
                umount -l "$m/$d" 2>/dev/null || echo "warning: could not unmount $m/$d (sudo umount $m/$d)" >&2
                break
            fi
            sleep 1
        done
    done
    for d in boot root data; do [ -d "$m/$d" ] && rmdir "$m/$d" 2>/dev/null; done
    rmdir "$m" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 1' INT TERM HUP
# mnt NAME DEVICE ro|rw
mnt() {
    mkdir -p "$m/$1"
    mount -o "$3" "$2" "$m/$1"
    mounted_here="$1 $mounted_here"
}
unmnt() {
    umount "$m/$1"
    mounted_here=$(echo "$mounted_here" | tr ' ' '\n' | grep -vx "$1" | tr '\n' ' ')
}

mnt boot "$p1" ro
[ -f "$m/boot/cmdline.txt" ] || die "$p1 has no cmdline.txt (not a Raspberry Pi OS boot partition)"
resize_word=0
tr ' ' '\n' < "$m/boot/cmdline.txt" | grep -qx resize && resize_word=1
unmnt boot

mnt root "$p2" ro
[ -f "$m/root/etc/fstab" ] && [ -d "$m/root/home/$user" ] || die "$p2 has no /etc/fstab or no /home/$user"
dirs="home/$user var/lib/tailscale var/log/journal var/lib/systemd/timesync"
echo "== to copy from $p2 to /data on $p3  (measuring sizes over USB: this can take a minute or two)"
for d in $dirs; do
    if [ -d "$m/root/$d" ]; then echo "  /$d  ($(du -sxh "$m/root/$d" 2>/dev/null | cut -f1))"
    else echo "  /$d  (not on the card: created empty)"; fi
done
[ -e "$m/root/etc/fake-hwclock.data" ] && echo "  note: /etc/fake-hwclock.data exists; it stays on the read-only root (the clock file /var/lib/systemd/timesync is copied)"
unmnt root

# ---- sizes ------------------------------------------------------------------------------
logsec=$(lsblk -dno LOG-SEC "$dev" | tr -d ' ')
disk_bytes=$(lsblk -bdno SIZE "$dev" | tr -d ' ')
p2_bytes=$(lsblk -bdno SIZE "$p2" | tr -d ' ')
p2_start=$(lsblk -ndo START "$p2" | tr -d ' ')
[ "${p2_start:-0}" -gt 0 ] || die "cannot read the start sector of $p2 (lsblk -ndo START $p2)"
# fs_bytes and used_bytes from the superblock
eval "$(dumpe2fs -h "$p2" 2>/dev/null | awk -F: '
    /^Block count/ { c = $2 } /^Block size/ { s = $2 } /^Free blocks/ { f = $2 }
    END { printf "fs_bytes=%.0f used_bytes=%.0f\n", c * s, (c - f) * s }')"
[ "${fs_bytes:-0}" -gt 0 ] || die "cannot read the ext4 superblock of $p2 (dumpe2fs -h $p2)"
target_bytes=$((target_gib * 1073741824))
target_sectors=$((target_bytes / logsec))
# p3 starts right after the shrunk p2; given explicitly, or sfdisk would use the first free
# gap on the card (there is one of 7 MiB before p1)
p3_start=$((p2_start + target_sectors))

echo "== plan ($dev: $(gib "$disk_bytes"); p2 partition $(gib "$p2_bytes"), filesystem $(gib "$fs_bytes"), used $(gib "$used_bytes"))"
if [ "$fs_bytes" -gt "$target_bytes" ] || [ "$p2_bytes" -gt "$target_bytes" ]; then
    [ "$have_p3" = 0 ] || die "$p3 exists but p2 is still bigger than ${target_gib} GiB: unexpected, not touching it"
    [ $((used_bytes + 2 * 1073741824)) -le "$target_bytes" ] || \
        die "p2 holds $(gib "$used_bytes"); ${target_gib} GiB leaves under 2 GiB free. Use a bigger --root-size"
    [ $((disk_bytes - target_bytes)) -ge 2147483648 ] || die "less than 2 GiB would be left for /data"
fi
[ "$fs_bytes" -le "$target_bytes" ] || { echo "  e2fsck -f -p $p2; resize2fs $p2 ${target_gib}G   (shrink the root filesystem)"; }
[ "$p2_bytes" -le "$target_bytes" ] || { echo "  sfdisk -N 2 $dev: p2 size $target_sectors sectors (${target_gib} GiB), same start"; }
if [ "$have_p3" = 0 ]; then
    echo "  sfdisk --append $dev: p3 from sector $p3_start (right after p2) to the end of the card ($(gib $((disk_bytes - target_bytes))) - alignment)"
    echo "  mkfs.ext4 -L kilo-data $p3"
else
    echo "  $p3: already there (ext4, kilo-data), kept"
fi
echo "  copy to $p3 (skipping what is already there); fstab lines on $p2 (adding only what is missing)"
[ "$resize_word" = 0 ] || { echo "  $p1 cmdline.txt: drop the word 'resize' (backup cmdline.txt.kilo-data-backup)"; }

if [ "$dry" = 1 ]; then
    echo "nothing changed. Check the card above is the Kilo card, then: sudo $0 $dev --yes"
    exit 0
fi

# ---- do it ------------------------------------------------------------------------------
say() { echo ">> $*"; }
if [ "$fs_bytes" -gt "$target_bytes" ]; then
    say "e2fsck -f -p $p2"
    rc=0; e2fsck -f -p "$p2" || rc=$?
    [ "$rc" -lt 4 ] || die "e2fsck found errors it would not fix alone (exit $rc); run e2fsck -f $p2 by hand, then this again"
    say "resize2fs $p2 ${target_gib}G"
    resize2fs "$p2" "${target_gib}G"
fi
fs_now=$(dumpe2fs -h "$p2" 2>/dev/null | awk -F: '/^Block count/ { c = $2 } /^Block size/ { s = $2 } END { printf "%.0f", c * s }')
[ "$fs_now" -le "$target_bytes" ] || die "the filesystem on $p2 is still $(gib "$fs_now"); not shrinking the partition"
if [ "$p2_bytes" -gt "$target_bytes" ]; then
    say "sfdisk -N 2 $dev  (p2: $target_sectors sectors)"
    echo ", $target_sectors" | sfdisk -N 2 "$dev"
    udevadm settle || true
fi
if [ "$have_p3" = 0 ]; then
    if [ "$(lsblk -dno TYPE "$p3" 2>/dev/null)" != part ]; then
        say "sfdisk --append $dev  (p3, Linux, from sector $p3_start to the end of the card)"
        echo "$p3_start,,L" | sfdisk --append "$dev"
        udevadm settle || true
    fi
    [ "$(lsblk -dno TYPE "$p3" 2>/dev/null)" = part ] || die "$p3 did not appear; unplug and replug the reader, then run this again"
    [ "$(lsblk -ndo START "$p3" | tr -d ' ')" = "$p3_start" ] || die "$p3 does not start at sector $p3_start as asked; not formatting it. Look with: sudo sfdisk -l $dev"
    if [ -z "$(fstype "$p3")" ]; then
        say "mkfs.ext4 -L kilo-data $p3"
        mkfs.ext4 -q -L kilo-data "$p3"
    fi
    [ "$(blkid -p -o value -s LABEL "$p3" 2>/dev/null)" = kilo-data ] || die "$p3 is not labelled kilo-data"
fi

mnt root "$p2" rw
mnt data "$p3" rw
for d in $dirs; do
    if [ -e "$m/data/$d" ]; then
        echo "kept /data/$d (already on $p3)"
        continue
    fi
    mkdir -p "$m/data/$(dirname "$d")"
    if [ -d "$m/root/$d" ]; then
        say "cp -a /$d -> /data/$d"
        cp -a "$m/root/$d" "$m/data/$d"
    else
        say "mkdir /data/$d and /$d (not on the card yet)"
        mkdir -p "$m/data/$d" "$m/root/$d"
    fi
done
[ -d "$m/root/data" ] || { say "mkdir /data on $p2 (the mount point)"; mkdir "$m/root/data"; }

fstab=$m/root/etc/fstab
data_line="LABEL=kilo-data  /data  ext4  defaults,noatime,nofail,x-systemd.device-timeout=10s,x-systemd.before=local-fs.target  0  2"
has_line() { awk -v t="$1" '$1 !~ /^#/ && $2 == t { f = 1 } END { exit !f }' "$fstab"; }
add=""
has_line /data || add="$data_line"
for d in $dirs; do
    has_line "/$d" && continue
    add="$add
/data/$d  /$d  none  bind,nofail,x-systemd.requires=/data,x-systemd.before=local-fs.target  0  0"
done
if [ -n "$add" ]; then
    [ -e "$fstab.kilo-data-backup" ] || cp -p "$fstab" "$fstab.kilo-data-backup"
    printf '# Kilo data partition (mind/systemd/data-partition.sh): what must survive the read-only root.\n%s\n' \
        "$(echo "$add" | sed '/^$/d')" >> "$fstab"
    say "$p2 /etc/fstab: added (backup /etc/fstab.kilo-data-backup):"
    sed -n '/^# Kilo data partition/,$p' "$fstab"
else
    echo "/etc/fstab on $p2: all lines already there"
fi
unmnt data
unmnt root

if [ "$resize_word" = 1 ]; then
    mnt boot "$p1" rw
    c=$m/boot/cmdline.txt
    [ -e "$c.kilo-data-backup" ] || cp -p "$c" "$c.kilo-data-backup"
    tr -s ' \n' '  ' < "$c" | awk '{ s = ""; for (i = 1; i <= NF; i++) if ($i != "resize") s = s (s == "" ? "" : " ") $i; print s }' > "$c.kilo-new"
    mv "$c.kilo-new" "$c"
    say "$p1 cmdline.txt: dropped 'resize'"
    unmnt boot
fi
sync
echo "done. Final layout:"
lsblk -o NAME,SIZE,FSTYPE,LABEL "$dev"
echo "Put the card back in the Pi and boot it; then on the Pi: findmnt /data; mind/systemd/safe-mode.sh status"
