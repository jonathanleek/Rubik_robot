#!/usr/bin/env bash
#
# Provision a blank Raspberry Pi (Raspberry Pi OS Bookworm) to run the Rubik
# Robot API server.
#
# What it does:
#   * installs system packages (camera stack, I2C tools, build deps)
#   * enables the I2C interface (for the PCA9685) and the camera
#   * creates a Python virtualenv with access to the apt-installed picamera2
#   * installs the Python dependencies
#   * installs and enables a systemd service so the API starts on boot
#
# Run from the repository root on the Pi:
#   chmod +x setup/provision.sh
#   ./setup/provision.sh
#
# Re-runnable (idempotent). A reboot is recommended afterwards so the I2C/camera
# interface changes take effect.

set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_USER="${SUDO_USER:-$USER}"
VENV_DIR="$APP_DIR/.venv"
SERVICE_NAME="rubik-robot"

echo "==> App directory : $APP_DIR"
echo "==> Service user  : $RUN_USER"

# --------------------------------------------------------------------------- #
# 1. System packages
# --------------------------------------------------------------------------- #
echo "==> Installing system packages"
sudo apt-get update
sudo apt-get install -y \
    python3-venv python3-pip python3-dev \
    python3-picamera2 \
    i2c-tools python3-smbus \
    libatlas-base-dev \
    git

# --------------------------------------------------------------------------- #
# 2. Enable interfaces (I2C for PCA9685, camera auto-detect)
# --------------------------------------------------------------------------- #
echo "==> Enabling I2C and camera"
if command -v raspi-config >/dev/null 2>&1; then
    sudo raspi-config nonint do_i2c 0 || true
fi

# Ensure camera auto-detect is on (default on Bookworm; harmless if already set).
CONFIG_TXT=/boot/firmware/config.txt
[ -f "$CONFIG_TXT" ] || CONFIG_TXT=/boot/config.txt
if [ -f "$CONFIG_TXT" ] && ! grep -q "^camera_auto_detect=1" "$CONFIG_TXT"; then
    echo "camera_auto_detect=1" | sudo tee -a "$CONFIG_TXT" >/dev/null
fi

# --------------------------------------------------------------------------- #
# 3. Python virtualenv (with system site packages so picamera2 is visible)
# --------------------------------------------------------------------------- #
echo "==> Creating virtualenv at $VENV_DIR"
if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv --system-site-packages "$VENV_DIR"
fi
"$VENV_DIR/bin/pip" install --upgrade pip
"$VENV_DIR/bin/pip" install -r "$APP_DIR/requirements.txt"

mkdir -p "$APP_DIR/data"

# --------------------------------------------------------------------------- #
# 4. Quick sanity checks
# --------------------------------------------------------------------------- #
echo "==> I2C devices (expect 0x40 for the PCA9685):"
sudo i2cdetect -y 1 || echo "    (i2cdetect failed; check wiring / reboot)"

echo "==> Importing picamera2 in the venv:"
"$VENV_DIR/bin/python" -c "import picamera2; print('    picamera2 OK')" \
    || echo "    picamera2 import failed (camera may need a reboot)"

# --------------------------------------------------------------------------- #
# 5. systemd service
# --------------------------------------------------------------------------- #
echo "==> Installing systemd service: $SERVICE_NAME"
TMP_UNIT="$(mktemp)"
sed -e "s#__USER__#$RUN_USER#g" -e "s#__APP_DIR__#$APP_DIR#g" \
    "$APP_DIR/setup/rubik-robot.service" > "$TMP_UNIT"
sudo cp "$TMP_UNIT" "/etc/systemd/system/$SERVICE_NAME.service"
rm -f "$TMP_UNIT"

sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
sudo systemctl restart "$SERVICE_NAME"

echo
echo "==> Done. API should be live on http://$(hostname -I | awk '{print $1}'):8000"
echo "    Interactive docs:  http://<pi-ip>:8000/docs"
echo "    Service logs:      sudo journalctl -u $SERVICE_NAME -f"
echo "    A reboot is recommended if this was the first run."
