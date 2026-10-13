#!/bin/sh
# Recreate the Wi-Fi profile that was found empty (0 bytes) after the unclean shutdown.
# Reads the SSID/password Raspberry Pi Imager left in /boot/firmware/network-config.
set -e
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
SSID=$(sed -n "s/^ *\"\([^\"]*\)\":$/\1/p" /boot/firmware/network-config | head -1)
PW=$(sed -n "s/^ *password: *\"\?\([^\"]*\)\"\?/\1/p" /boot/firmware/network-config | head -1)
echo "ssid=$SSID password=${#PW} chars"
echo "== evidence: NetworkManager last-used record for the empty profile"
for f in /etc/netplan/90-NM-*.yaml; do u=$(basename "$f" .yaml | sed s/^90-NM-//); grep "$u" /var/lib/NetworkManager/timestamps 2>/dev/null | while IFS== read -r id ts; do echo "$id last used $(date -d @$ts)"; done; done
for f in /etc/netplan/90-NM-*.yaml; do [ -e "$f" ] && [ ! -s "$f" ] && { echo "removing empty $f"; rm -f "$f"; }; done
nmcli device wifi connect "$SSID" password "$PW" 2>&1 | sed "s/$PW/<hidden>/g"
nmcli -t -f NAME,TYPE,DEVICE,ACTIVE,AUTOCONNECT connection show
