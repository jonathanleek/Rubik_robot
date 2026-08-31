#!/usr/bin/env bash
#
# install-api-service.sh -- install the Rubik Robot API as a systemd service
# so it starts automatically on boot. Pairs with setup-hotspot.sh: with both
# installed, powering on the Pi gives you a hotspot AND a running API with no
# SSH needed -- ideal for demos and conferences.
#
# Usage:
#   sudo ./install-api-service.sh                 # driver defaults to pca9685
#   sudo ./install-api-service.sh --driver gpio   # use the GPIO variant
#   sudo ./install-api-service.sh --disable       # stop and remove the service
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT_SRC="${SCRIPT_DIR}/rubik-api.service"
UNIT_DST="/etc/systemd/system/rubik-api.service"
DRIVER="pca9685"
ACTION="install"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --driver) DRIVER="${2:?--driver needs a value}"; shift 2 ;;
        --disable) ACTION="disable"; shift ;;
        *) echo "Unknown argument: $1" >&2; exit 1 ;;
    esac
done

[[ "${EUID}" -eq 0 ]] || { echo "Must run as root. Try: sudo $0 $*" >&2; exit 1; }

if [[ "${ACTION}" == "disable" ]]; then
    systemctl disable --now rubik-api.service 2>/dev/null || true
    rm -f "${UNIT_DST}"
    systemctl daemon-reload
    echo "[rubik-api] Service removed."
    exit 0
fi

[[ -f "${UNIT_SRC}" ]] || { echo "Missing ${UNIT_SRC}" >&2; exit 1; }

# Install the unit, applying the chosen driver.
sed "s/--driver pca9685/--driver ${DRIVER}/" "${UNIT_SRC}" > "${UNIT_DST}"

systemctl daemon-reload
systemctl enable rubik-api.service
systemctl restart rubik-api.service

echo "[rubik-api] Installed and started (driver=${DRIVER})."
echo "[rubik-api] Logs:   journalctl -u rubik-api -f"
echo "[rubik-api] Status: systemctl status rubik-api"
