#!/bin/bash
#
# rubik-hotspot-firstrun.sh -- one-shot boot hook that turns the Pi into a
# standalone WiFi access point WITHOUT needing SSH, a monitor, or a reflash.
#
# How it is triggered: from a Mac/PC, copy this file and rubik-hotspot.nmconnection
# onto the card's FAT "bootfs" partition and append to the single line in
# cmdline.txt:
#
#   systemd.run=/boot/firmware/rubik-hotspot-firstrun.sh systemd.run_success_action=reboot systemd.unit=kernel-command-line.target
#
# (The same mechanism Raspberry Pi Imager uses for its own firstrun.sh.)
# On the next boot systemd runs this script as root instead of the normal
# targets, then reboots. This script:
#   1. scrubs itself out of cmdline.txt FIRST (so a normal boot follows no
#      matter what happens below -- no boot loop is possible),
#   2. installs the NetworkManager hotspot keyfile (or removes it, if a file
#      named rubik-hotspot.disable exists on bootfs),
#   3. installs a small status reporter that writes the hotspot state to
#      bootfs on every boot, so you can diagnose from the Mac if the AP never
#      shows up,
#   4. optionally installs the API systemd service (INSTALL_API_SERVICE=1),
#   5. logs everything to bootfs/rubik-hotspot.log and always exits 0.
#
# Nothing else on the card is modified. The saved client WiFi profile is kept
# (lower priority), so removing the keyfile restores the old behaviour.

# ---- knobs -------------------------------------------------------------------
INSTALL_API_SERVICE=1                 # 1 = also start the Flask API on every boot
API_USER=pi
API_DIR=/home/pi/rubik_robot
API_DRIVER=pca9685
# -------------------------------------------------------------------------------

set -u   # deliberately NOT -e: we want to log and finish, never hang the boot

BOOT=/boot/firmware
[ -f "$BOOT/cmdline.txt" ] || BOOT=/boot            # older layout fallback
mountpoint -q "$BOOT" || mount "$BOOT" 2>/dev/null || true

LOG="$BOOT/rubik-hotspot.log"
KEYFILE_SRC="$BOOT/rubik-hotspot.nmconnection"
KEYFILE_DST=/etc/NetworkManager/system-connections/rubik-hotspot.nmconnection
REPORT_BIN=/usr/local/sbin/rubik-hotspot-report.sh
REPORT_UNIT=/etc/systemd/system/rubik-hotspot-report.service

log() { printf '%s %s\n' "$(date '+%F %T')" "$*" >> "$LOG"; }
run() { log "\$ $*"; "$@" >> "$LOG" 2>&1; local rc=$?; [ $rc -eq 0 ] || log "  -> exit $rc"; return $rc; }

{
  echo "==================================================================="
  echo " rubik-hotspot-firstrun.sh  $(date '+%F %T')"
  echo "==================================================================="
} >> "$LOG"

# --- 1. make the NEXT boot a normal one, before anything else ------------------
if grep -q 'systemd\.run=' "$BOOT/cmdline.txt"; then
    cp "$BOOT/cmdline.txt" "$BOOT/cmdline.txt.pre-hotspot"
    sed -i -e 's/ systemd\.run=[^ ]*//g' \
           -e 's/ systemd\.run_success_action=[^ ]*//g' \
           -e 's/ systemd\.run_failure_action=[^ ]*//g' \
           -e 's/ systemd\.unit=kernel-command-line\.target//g' "$BOOT/cmdline.txt"
    log "cmdline.txt restored to: $(cat "$BOOT/cmdline.txt")"
else
    log "cmdline.txt had no systemd.run= entry (manual run?)"
fi

# --- 2. environment facts (for the log) -----------------------------------------
log "os: $(. /etc/os-release && echo "$PRETTY_NAME")  kernel: $(uname -r)"
log "NetworkManager: $(NetworkManager --version 2>/dev/null || echo MISSING)"
if dpkg -s dnsmasq-base >/dev/null 2>&1; then
    log "dnsmasq-base: installed (required for ipv4.method=shared)"
else
    log "WARNING dnsmasq-base NOT installed -- NM shared mode will fail; install it over Ethernet"
fi
log "regdom: $(grep -o 'ieee80211_regdom=[A-Z]*' "$BOOT/cmdline.txt" || echo 'not set in cmdline')"
run rfkill unblock wifi

# --- 3. install or remove the hotspot profile -----------------------------------
if [ -e "$BOOT/rubik-hotspot.disable" ]; then
    log "rubik-hotspot.disable present -> REMOVING hotspot profile"
    run rm -f "$KEYFILE_DST"
    run rm -f "$BOOT/rubik-hotspot.disable"
elif [ -f "$KEYFILE_SRC" ]; then
    run install -o root -g root -m 600 "$KEYFILE_SRC" "$KEYFILE_DST"
    log "installed $KEYFILE_DST: $(stat -c '%U:%G %a' "$KEYFILE_DST")"
    log "ssid=$(sed -n 's/^ssid=//p' "$KEYFILE_DST")  ip=$(sed -n 's/^address1=//p' "$KEYFILE_DST")"
else
    log "ERROR $KEYFILE_SRC not found on bootfs -- nothing installed"
fi

# --- 4. per-boot status reporter (writes bootfs/rubik-hotspot-status.log) -------
cat > "$REPORT_BIN" <<'REPORT'
#!/bin/bash
# Writes the hotspot/network state to the FAT boot partition so it can be read
# from a Mac/PC if the AP does not work. Installed by rubik-hotspot-firstrun.sh.
# First snapshot 45 s after boot, then one every 60 s for ~15 min, so a card
# pulled after a failed client attempt carries wpa_supplicant's view of it.
BOOT=/boot/firmware; [ -d "$BOOT" ] || BOOT=/boot
OUT="$BOOT/rubik-hotspot-status.log"
sleep 45
for pass in $(seq 1 15); do
{
  echo "=== rubik hotspot status  pass $pass  $(date '+%F %T')  up $(cut -d. -f1 /proc/uptime)s ==="
  echo "--- nmcli general";            nmcli -t general status
  echo "--- active";                   nmcli -f NAME,TYPE,DEVICE,STATE connection show --active
  echo "--- effective wifi-security";  nmcli -f 802-11-wireless-security connection show rubik-hotspot 2>&1
  echo "--- wlan0";                    ip -brief addr show wlan0 2>&1
  echo "--- iw dev";                   iw dev 2>&1 | grep -E 'Interface|type|ssid|channel'
  echo "--- stations (iw station dump)"; iw dev wlan0 station dump 2>&1 | grep -E 'Station|authorized|authenticated|associated|connected time' || echo "none"
  echo "--- dhcp leases";              cat /var/lib/NetworkManager/dnsmasq-wlan0.leases 2>/dev/null || echo "none"
  echo "--- listening :5000";          ss -ltnp 2>/dev/null | grep ':5000' || echo "API not listening"
  echo "--- rubik-api.service";        systemctl is-enabled rubik-api.service 2>&1; systemctl is-active rubik-api.service 2>&1
  echo "--- wpa_supplicant journal (this boot, last 80 lines: look for AP-STA-*, MIC, EAPOL, handshake)"
  journalctl -b -u wpa_supplicant --no-pager -n 80 2>&1
  echo "--- NetworkManager journal (this boot, last 40 lines)"
  journalctl -b -u NetworkManager --no-pager -n 40 2>&1
} > "$OUT.tmp" 2>&1 && mv -f "$OUT.tmp" "$OUT"
sync
sleep 60
done
REPORT
chmod 755 "$REPORT_BIN"

cat > "$REPORT_UNIT" <<UNIT
[Unit]
Description=Write Rubik hotspot status to the boot partition
After=NetworkManager.service network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=$REPORT_BIN

[Install]
WantedBy=multi-user.target
UNIT
run systemctl enable rubik-hotspot-report.service \
  || run ln -sf ../rubik-hotspot-report.service /etc/systemd/system/multi-user.target.wants/rubik-hotspot-report.service

# --- 5. optional: API on boot -----------------------------------------------------
if [ "$INSTALL_API_SERVICE" = "1" ]; then
    cat > /etc/systemd/system/rubik-api.service <<UNIT
[Unit]
Description=Rubik Robot API server
After=network-online.target
Wants=network-online.target
StartLimitIntervalSec=300
StartLimitBurst=3

[Service]
Type=simple
User=$API_USER
WorkingDirectory=$API_DIR
ExecStart=/usr/bin/python3 -m rubik_robot.run --driver $API_DRIVER --host 0.0.0.0 --port 5000
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
UNIT
    run systemctl enable rubik-api.service \
      || run ln -sf ../rubik-api.service /etc/systemd/system/multi-user.target.wants/rubik-api.service
    log "rubik-api.service installed (user=$API_USER dir=$API_DIR driver=$API_DRIVER)"
else
    log "INSTALL_API_SERVICE=0 -> API not started on boot (start it over SSH: ssh $API_USER@192.168.4.1)"
fi

log "done; rebooting into normal boot (systemd.run_success_action=reboot)"
sync
exit 0
