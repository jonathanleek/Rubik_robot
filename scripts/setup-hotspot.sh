#!/usr/bin/env bash
#
# setup-hotspot.sh -- turn the Raspberry Pi into a self-contained WiFi
# access point so you can reach the Rubik Robot API without any external
# network (e.g. at a crowded conference with unknown WiFi).
#
# The Pi 3B has a single WiFi radio, so it can be a client OR an access
# point, not both. This script puts it into access-point mode: your
# laptop/phone joins the "rubik-robot" network and talks to the API
# directly. No internet passes through -- you don't need it for the API.
#
# It auto-detects the networking stack:
#   * NetworkManager (Raspberry Pi OS Bookworm and later) -> nmcli profile
#   * dhcpcd         (Raspberry Pi OS Bullseye)           -> hostapd+dnsmasq
#
# The hotspot is ALWAYS-ON: it comes up automatically on every boot.
#
# Usage:
#   sudo ./setup-hotspot.sh            # enable the always-on hotspot
#   sudo ./setup-hotspot.sh --disable  # tear it down, restore normal WiFi
#
# After running, the API is reachable at:  http://192.168.4.1:5000
#
set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration -- edit these if you want a different network name/password.
# ---------------------------------------------------------------------------
SSID="rubik-robot"
PSK="solvethecube"        # WPA2 password; must be 8-63 characters
COUNTRY="US"              # regulatory domain; AP mode requires this be set
IFACE="wlan0"
AP_IP="192.168.4.1"
DHCP_START="192.168.4.2"
DHCP_END="192.168.4.20"
NETMASK="255.255.255.0"
NM_CON_NAME="rubik-hotspot"
HOSTAPD_CONF="/etc/hostapd/hostapd.conf"
DNSMASQ_CONF="/etc/dnsmasq.d/rubik-hotspot.conf"
DHCPCD_MARKER="# --- rubik-hotspot (managed by setup-hotspot.sh) ---"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
log()  { printf '\033[1;36m[hotspot]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[hotspot]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[hotspot]\033[0m %s\n' "$*" >&2; exit 1; }

require_root() {
    if [[ "${EUID}" -ne 0 ]]; then
        die "Must run as root. Try:  sudo $0 $*"
    fi
}

# Detect which networking stack is in charge. NetworkManager is preferred
# when it is actually running the show.
detect_stack() {
    if systemctl is-active --quiet NetworkManager 2>/dev/null; then
        echo "networkmanager"
    else
        echo "dhcpcd"
    fi
}

# ---------------------------------------------------------------------------
# NetworkManager path (Bookworm+)
# ---------------------------------------------------------------------------
enable_networkmanager() {
    log "Using NetworkManager (nmcli)."

    command -v nmcli >/dev/null 2>&1 || die "nmcli not found but NetworkManager is active."

    # Regulatory domain + radio on (AP mode is blocked without a country set).
    if command -v raspi-config >/dev/null 2>&1; then
        raspi-config nonint do_wifi_country "${COUNTRY}" || true
    fi
    command -v rfkill >/dev/null 2>&1 && rfkill unblock wifi || true
    nmcli radio wifi on || true

    # Recreate the profile from scratch so re-runs are idempotent.
    nmcli connection delete "${NM_CON_NAME}" >/dev/null 2>&1 || true

    nmcli connection add type wifi ifname "${IFACE}" con-name "${NM_CON_NAME}" \
        autoconnect yes ssid "${SSID}"

    # mode ap        -> act as access point
    # band bg        -> 2.4 GHz (Pi 3B onboard radio; best device compatibility)
    # ipv4 shared    -> NetworkManager runs DHCP for clients automatically
    # ipv4.addresses -> pin the gateway IP so the API address is deterministic
    # wifi-sec.pmf 1 -> DISABLE 802.11w. brcmfmac (Pi 3B / Zero W) cannot do
    #                   Management Frame Protection in AP mode; with NM's default
    #                   (optional) Apple/Android clients negotiate it and the WPA2
    #                   handshake fails. raspberrypi/linux#3619
    # Deliberately NOT setting wifi-sec.proto/pairwise/group: pinning them breaks
    # the handshake on the BCM43438 (clients report "incorrect password").
    # https://forums.raspberrypi.com/viewtopic.php?t=358481
    nmcli connection modify "${NM_CON_NAME}" \
        802-11-wireless.mode ap \
        802-11-wireless.band bg \
        ipv4.method shared \
        ipv4.addresses "${AP_IP}/24" \
        wifi-sec.key-mgmt wpa-psk \
        wifi-sec.psk "${PSK}" \
        wifi-sec.pmf 1 \
        connection.autoconnect yes \
        connection.autoconnect-priority 100

    nmcli connection up "${NM_CON_NAME}"
    log "Hotspot profile '${NM_CON_NAME}' is up and set to autoconnect on boot."
}

disable_networkmanager() {
    log "Removing NetworkManager hotspot profile."
    nmcli connection down "${NM_CON_NAME}" >/dev/null 2>&1 || true
    nmcli connection delete "${NM_CON_NAME}" >/dev/null 2>&1 || true
    log "Done. The Pi will fall back to its other saved WiFi connections."
}

# ---------------------------------------------------------------------------
# dhcpcd / hostapd / dnsmasq path (Bullseye)
# ---------------------------------------------------------------------------
enable_dhcpcd() {
    log "Using dhcpcd + hostapd + dnsmasq."

    log "Installing hostapd and dnsmasq (if missing)..."
    DEBIAN_FRONTEND=noninteractive apt-get install -y hostapd dnsmasq

    command -v rfkill >/dev/null 2>&1 && rfkill unblock wifi || true

    # 1. Static IP for the AP interface, and stop the client supplicant from
    #    fighting us over wlan0.
    if ! grep -qF "${DHCPCD_MARKER}" /etc/dhcpcd.conf 2>/dev/null; then
        log "Configuring static IP on ${IFACE} in /etc/dhcpcd.conf"
        cat >> /etc/dhcpcd.conf <<EOF

${DHCPCD_MARKER}
interface ${IFACE}
    static ip_address=${AP_IP}/24
    nohook wpa_supplicant
EOF
    fi

    # 2. DHCP server for clients that join the hotspot.
    log "Writing ${DNSMASQ_CONF}"
    cat > "${DNSMASQ_CONF}" <<EOF
interface=${IFACE}
dhcp-range=${DHCP_START},${DHCP_END},${NETMASK},24h
domain=wlan
# Resolve rubik.local to the Pi for clients on the hotspot
address=/rubik.local/${AP_IP}
EOF

    # 3. The access point itself.
    log "Writing ${HOSTAPD_CONF}"
    mkdir -p "$(dirname "${HOSTAPD_CONF}")"
    cat > "${HOSTAPD_CONF}" <<EOF
country_code=${COUNTRY}
interface=${IFACE}
ssid=${SSID}
hw_mode=g
channel=7
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=${PSK}
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
EOF
    chmod 600 "${HOSTAPD_CONF}"

    # Point the hostapd service at our config.
    if [[ -f /etc/default/hostapd ]]; then
        sed -i 's|^#\?DAEMON_CONF=.*|DAEMON_CONF="'"${HOSTAPD_CONF}"'"|' /etc/default/hostapd
    else
        echo "DAEMON_CONF=\"${HOSTAPD_CONF}\"" > /etc/default/hostapd
    fi

    # 4. Enable services so the hotspot returns on every boot.
    systemctl unmask hostapd
    systemctl enable hostapd dnsmasq

    log "Restarting networking services..."
    systemctl restart dhcpcd
    systemctl restart dnsmasq
    systemctl restart hostapd

    log "Hotspot is up and enabled at boot (hostapd + dnsmasq)."
}

disable_dhcpcd() {
    log "Tearing down hostapd/dnsmasq hotspot."
    systemctl disable --now hostapd >/dev/null 2>&1 || true
    systemctl disable --now dnsmasq >/dev/null 2>&1 || true
    rm -f "${DNSMASQ_CONF}"

    # Strip our managed block from dhcpcd.conf.
    if grep -qF "${DHCPCD_MARKER}" /etc/dhcpcd.conf 2>/dev/null; then
        # Delete from the marker line to the next blank line (our whole block).
        sed -i "/${DHCPCD_MARKER}/,/^\s*$/d" /etc/dhcpcd.conf
    fi
    systemctl restart dhcpcd || true
    log "Done. Restore your normal WiFi via 'sudo raspi-config' (System > Wireless LAN)."
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
main() {
    local action="enable"
    if [[ "${1:-}" == "--disable" ]]; then
        action="disable"
    elif [[ -n "${1:-}" ]]; then
        die "Unknown argument: $1  (use --disable to tear down)"
    fi

    require_root "$@"

    if [[ ${#PSK} -lt 8 || ${#PSK} -gt 63 ]]; then
        die "WPA2 password (PSK) must be 8-63 characters; got ${#PSK}."
    fi

    local stack
    stack="$(detect_stack)"
    log "Detected networking stack: ${stack}"

    if [[ "${action}" == "enable" ]]; then
        case "${stack}" in
            networkmanager) enable_networkmanager ;;
            dhcpcd)         enable_dhcpcd ;;
        esac
        echo
        log "===================================================="
        log " Hotspot ready."
        log "   SSID:     ${SSID}"
        log "   Password: ${PSK}"
        log "   API URL:  http://${AP_IP}:5000   (also http://rubik.local:5000)"
        log " Reboot to confirm it comes up automatically."
        log "===================================================="
    else
        case "${stack}" in
            networkmanager) disable_networkmanager ;;
            dhcpcd)         disable_dhcpcd ;;
        esac
    fi
}

main "$@"
