# Rubik Robot API

A Flask-based REST API for controlling a Rubik's Cube solving robot built on Raspberry Pi. The robot physically manipulates a Rubik's Cube using servo-controlled grippers and wrists, and uses a camera to scan the cube's state.

This project separates the **robot control** (this API) from the **solving algorithm**, allowing you to run any solver on a separate system and send the solution moves to the robot via HTTP.

Based on [DrVoHo/Rubik_solver](https://github.com/DrVoHo/Rubik_solver). Mechanical design: [Thingiverse](https://www.thingiverse.com/thing:3826740).

## Hardware Variants

The robot supports two servo control methods:

| Variant | Flag | Description |
|---------|------|-------------|
| **PCA9685** | `--driver pca9685` | Uses a PCA9685 I2C PWM driver board. Provides stable, jitter-free servo control. Calibrated via the API (`/config`); an OLED display + buttons are optional. **Recommended.** |
| **GPIO** | `--driver gpio` | Uses the Raspberry Pi's built-in GPIO pins for PWM. Simpler hardware setup but may exhibit servo jitter. No display or buttons -- calibration is done via the API. |

---

## Installation on Raspberry Pi

### Prerequisites

- Raspberry Pi (tested on Pi 3B, 3B+, and Pi 4)
- Raspberry Pi OS **Bookworm** (recommended) or Bullseye
- Python 3.9+
- Raspberry Pi Camera module (connected and enabled)
- Servo motors and mechanical assembly (see [Thingiverse](https://www.thingiverse.com/thing:3826740))
- For PCA9685 variant: Adafruit/compatible PCA9685 PWM driver board. An SSD1306
  OLED display and 3 push buttons are **optional** (on-device calibration UI) --
  the robot runs fine without them and is calibrated via the API.

> **Camera stack:** on **Bookworm** the camera uses **`picamera2`** (libcamera),
> installed as a system package (below). On older Bullseye you would instead use the
> legacy `picamera`; this codebase targets `picamera2`.

### Step 1: Enable required interfaces

```bash
sudo raspi-config nonint do_i2c 0      # enable I2C (PCA9685 + OLED)
```

On **Bookworm** the camera is auto-detected via libcamera -- no toggle needed.
Make sure the **legacy camera is OFF** (it disables libcamera). Check
`/boot/firmware/config.txt` has `camera_auto_detect=1` and no `start_x=1`, then
reboot. Verify with `rpicam-hello --list-cameras` (should list your sensor).

### Step 2: Install system dependencies

```bash
sudo apt-get update
sudo apt-get install -y git i2c-tools python3-pip python3-dev \
    python3-picamera2 libatlas-base-dev libjpeg-dev zlib1g-dev
```

`python3-picamera2` **must** come from apt (it depends on the system libcamera
stack) -- it is intentionally not a pip dependency.

### Step 3: Clone and install the project

```bash
cd /home/pi
git clone <your-repo-url> rubik_robot
cd rubik_robot
pip3 install -r requirements.txt --break-system-packages
```

On Bookworm, `--break-system-packages` is required (PEP 668). For a dedicated
appliance Pi this is fine; alternatively use a virtualenv created with
`--system-site-packages` so it can reach the apt-installed `picamera2`.

### Step 4: Install the font file (optional -- only if using the OLED)

If you use the optional SSD1306 OLED display, it needs the VCR OSD Mono font.
Download it from [dafont.com](https://www.dafont.com/vcr-osd-mono.font) and place it
in the home directory. Skip this if you have no display (the code falls back to a
null display automatically):

```bash
cp VCR_OSD_MONO_1.001.ttf /home/pi/
```

### Step 5: Install the Kociemba solver (optional)

If you want the Pi to validate scanned cube states:

```bash
pip3 install kociemba
```

### Step 6: Wire the hardware

**GPIO variant** (BOARD pin numbering):

| Servo | GPIO Pin |
|-------|----------|
| Left wrist (turn) | 33 |
| Left gripper | 31 |
| Right wrist (turn) | 37 |
| Right gripper | 35 |

**PCA9685 variant**:

| Component | Connection |
|-----------|------------|
| PCA9685 board | I2C (SDA/SCL), address 0x40 |
| Left wrist servo | PCA9685 channel 0 |
| Left gripper servo | PCA9685 channel 1 |
| Right wrist servo | PCA9685 channel 2 |
| Right gripper servo | PCA9685 channel 3 |
| OLED display *(optional)* | I2C, address 0x3C |
| Plus button *(optional)* | GPIO pin 11 (BOARD) |
| Minus button *(optional)* | GPIO pin 13 (BOARD) |
| Enter button *(optional)* | GPIO pin 15 (BOARD) |

> **Servo power:** power the servos from a dedicated 5&nbsp;V supply into the
> PCA9685 `V+` screw terminal -- **not** from the Pi. See
> [`docs/hardware-upgrade-plan.md`](docs/hardware-upgrade-plan.md) for a
> single-USB-C power build (PD trigger &rarr; 5&nbsp;V/10&nbsp;A buck &rarr; Pi + PCA9685).
>
> **No buttons?** On Raspberry Pi OS Bookworm, `RPi.GPIO` edge detection is
> unsupported, so physical-button setup is skipped automatically -- calibrate via
> the API instead.

### Step 7: Start the server

```bash
# PCA9685 variant (default)
python3 -m rubik_robot.run

# GPIO variant
python3 -m rubik_robot.run --driver gpio

# Custom port
python3 -m rubik_robot.run --port 8080
```

The API will be available at `http://<pi-ip>:5000`.

### Step 8: Auto-start on boot (optional)

Add to `/etc/rc.local` before `exit 0`:

```bash
sudo -H -u pi python3 /home/pi/rubik_robot/rubik_robot/run.py &
```

Or create a systemd service:

```ini
# /etc/systemd/system/rubik-robot.service
[Unit]
Description=Rubik Robot API
After=network.target

[Service]
User=pi
WorkingDirectory=/home/pi/rubik_robot
ExecStart=/usr/bin/python3 -m rubik_robot.run
Restart=always

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable rubik-robot
sudo systemctl start rubik-robot
```

### Step 9: Run as a WiFi hotspot (optional)

To reach the API where there's no usable WiFi (e.g. a crowded conference), the
Pi can host its own network instead of joining one. Connect your laptop/phone
to the robot's `rubik-robot` network and call the API at
`http://192.168.4.1:5000`.

```bash
sudo ./scripts/setup-hotspot.sh          # always-on hotspot, auto-detects OS stack
sudo ./scripts/install-api-service.sh    # start the API on boot too (no SSH needed)
```

No network access to the Pi at all? The hotspot can also be enabled by editing
only the SD card's `bootfs` partition on another computer -- see
[docs/hotspot.md](docs/hotspot.md), which also covers customization, reverting
to normal WiFi, and two Pi 3B-specific settings without which clients cannot
authenticate.

---

## Servo Calibration

Calibration has two layers:

1. **Servo geometry** (`/config`) -- the servo command values for each key position,
   plus per-servo safety clamps. Persisted to **`~/robot_config.json`**. This is the
   main calibration and is fully API-editable (no code edits).
2. **Runtime tuning** (`/calibration`) -- timing/behavior (`sleep`, `regrip_enabled`).
   Persisted to `~/tune_values.txt`.

### Servo geometry model

Every position is a raw servo command value (degrees; same units as
`/calibration/test`). `GET /config` returns:

| Servo | Keys | Meaning |
|-------|------|---------|
| `left_grip`, `right_grip` | `open`, `closed`, `load` | jaws open / firm grip / loose hold |
| `left_turn`, `right_turn` | `m`, `n`, `o` | wrist at 0° / 90° (neutral) / 180° |
| *(all servos)* | `min`, `max` | **safety clamp** -- robot moves are clamped to this range |

**Safety clamps:** the robot's own motions (moves, regrip, home) are clamped to each
servo's `[min, max]`, so a command can never drive a servo into a mechanical hard
stop. `POST /calibration/test` is intentionally **unclamped** so you can explore the
full range and re-clock horns during calibration.

### Calibration workflow (recommended)

1. **Detach the load** from the servo (disconnect the gripper linkage or pull the
   horn) so the servo turns unloaded -- zero risk of straining a part.
2. **Sweep** with `POST /calibration/test` (unclamped) to find the servo value for
   each key position (open/closed/load, or wrist m/n/o).
3. **Save** those values with `POST /config` (persisted to `robot_config.json`).
4. **Set `min`/`max`** just inside the hard stops so normal moves stay safe.
5. **Reconnect** and verify with `POST /home` and single moves.

```bash
# View the full geometry
curl http://<pi-ip>:5000/config

# Test a single servo position (UNCLAMPED -- for calibration)
curl -X POST http://<pi-ip>:5000/calibration/test \
  -H "Content-Type: application/json" \
  -d '{"servo": "left_grip", "value": 65}'

# Persist a calibrated value (any subset of servos/keys)
curl -X POST http://<pi-ip>:5000/config \
  -H "Content-Type: application/json" \
  -d '{"left_grip": {"closed": 65}, "right_turn": {"o": 235}}'

# Runtime tuning (timing/behavior)
curl -X POST http://<pi-ip>:5000/calibration \
  -H "Content-Type: application/json" \
  -d '{"sleep": 0.45, "regrip_enabled": true}'
```

### Calibration tips

1. **Grip (`closed`)**: tighten until the gripper holds the cube squarely without
   straining. Foam pads on the fingers add compliance and grip.
2. **Load**: the loose hold between moves -- cube retained but not clamped. Too open
   and it falls; too closed and it can't reseat during a regrip.
3. **Wrist neutral (`n`)**: the cube-straight position; `m`/`o` are ±90° from it. If a
   face turn falls a hair short, widen `m`/`o` (and the clamp) a few degrees.
4. **Inconsistent moves** (same command, different results) almost always mean a
   *mechanical* grip problem, not geometry -- geometry is deterministic.

### Calibrating via physical buttons (optional, PCA9685 + OLED only)

If the optional OLED + buttons are fitted, the old on-device flow still exists
(Enter starts; Plus/Minus adjust; Enter advances; double-click Enter cancels). It
edits the legacy `/calibration` tune values only. Most builds calibrate via the API.

---

## API Reference

All endpoints return JSON. The `Content-Type: application/json` header is required for POST requests with a body.

### GET /status

Get the current robot status. **Never blocks** -- returns immediately even if the robot is busy.

**Response:**
```json
{
  "status": "idle",
  "driver": "pca9685",
  "calibration": {
    "left_grip_tune": 4,
    "left_wrist_tune": 2,
    "right_grip_tune": 6,
    "right_wrist_tune": 0,
    "load": 30,
    "sleep": 0.45,
    "regrip_enabled": true
  }
}
```

Possible status values: `idle`, `initializing`, `scanning`, `moving`, `scrambling`, `calibrating`.

---

### GET /scan

Scan the cube and return its current state. The robot rotates the cube to photograph all six faces, then analyzes the sticker colors.

**Response (200):**
```json
{
  "state": "UBUULRUFURURFRBRDRFUFLFRFDFDFDLDRDBDLULBLFLDLBUBRBLBDB",
  "time": 18.5
}
```

The `state` field is a 54-character string in [Kociemba URFDLB notation](https://github.com/muodov/kociemba). Each character represents a sticker color, ordered by face: U (positions 0-8), R (9-17), F (18-26), D (27-35), L (36-44), B (45-53).

**Error (409):** Robot is busy with another operation.

**Error (500):** Scan failed (e.g., invalid color detection).

---

### POST /move

Execute a list of Rubik's notation moves on the physical cube.

**Request:**
```json
{
  "moves": ["R", "U'", "F2", "B", "L'", "D2"]
}
```

Valid moves: `U`, `U'`, `U2`, `D`, `D'`, `D2`, `R`, `R'`, `R2`, `L`, `L'`, `L2`, `F`, `F'`, `F2`, `B`, `B'`, `B2`

**Response (200):**
```json
{
  "moves_executed": 6,
  "time": 24.3
}
```

**Error (400):** Invalid move notation.

**Error (409):** Robot is busy.

---

### POST /execute

Execute a raw servo action string. This is a low-level endpoint for direct servo control.

**Request:**
```json
{
  "actions": "bXBaYA"
}
```

Valid action codes:
| Code | Action |
|------|--------|
| `A` | Left gripper close |
| `a` | Left gripper open |
| `B` | Right gripper close |
| `b` | Right gripper open |
| `M` | Left wrist to 0 degrees |
| `N` | Left wrist to 90 degrees |
| `O` | Left wrist to 180 degrees |
| `X` | Right wrist to 0 degrees |
| `Y` | Right wrist to 90 degrees |
| `Z` | Right wrist to 180 degrees |
| `R` | Regrip |
| `t` | Decrement move counter |

**Response (200):**
```json
{
  "actions_executed": 6,
  "time": 3.1
}
```

---

### POST /home

Return all servos to their home (neutral) position: wrists at 90 degrees, grippers at load position.

**Response (200):**
```json
{
  "status": "homed"
}
```

---

### POST /scramble

Scramble the cube with a random sequence of moves. Useful for testing.

**Request:**
```json
{
  "count": 20
}
```

The count is clamped to the range [1, 20].

**Response (200):**
```json
{
  "moves": ["R", "U'", "F2", "B", "L'", "D"],
  "count": 6,
  "time": 18.7
}
```

---

### GET /calibration

Get current servo calibration values.

**Response (200):**
```json
{
  "left_grip_tune": 4,
  "left_wrist_tune": 2,
  "right_grip_tune": 6,
  "right_wrist_tune": 0,
  "load": 30,
  "sleep": 0.45,
  "regrip_enabled": true
}
```

---

### POST /calibration

Update calibration values. Only include the fields you want to change.

**Request:**
```json
{
  "left_grip_tune": 4,
  "sleep": 0.45
}
```

**Response (200):** Full updated calibration values (same format as GET).

---

### POST /calibration/test

Move a single servo to a specific position for testing during calibration.

**Request:**
```json
{
  "servo": "left_grip",
  "value": 10
}
```

Valid servo names: `left_grip`, `left_turn`, `right_grip`, `right_turn`.

This move is **unclamped** (unlike the robot's normal moves) so you can explore the
full servo range and re-clock horns during calibration.

**Response (200):**
```json
{
  "servo": "left_grip",
  "value": 10
}
```

---

### GET /config

Get the servo geometry -- each servo's positions and safety-clamp limits. Never
blocks (safe to read while the robot is busy).

**Response (200):**
```json
{
  "geometry": {
    "left_grip":  {"open": 120, "closed": 65, "load": 80, "min": 62, "max": 145},
    "right_grip": {"open": 120, "closed": 90, "load": 94, "min": 62, "max": 145},
    "left_turn":  {"m": 45, "n": 135, "o": 225, "min": 40, "max": 230},
    "right_turn": {"m": 35, "n": 135, "o": 235, "min": 30, "max": 240}
  }
}
```

---

### POST /config

Update servo geometry, applied live and persisted to `~/robot_config.json`. Send
only the servos/keys you want to change. Grippers accept `open`/`closed`/`load`/
`min`/`max`; wrists accept `m`/`n`/`o`/`min`/`max`.

**Request:**
```json
{
  "right_grip": {"closed": 88},
  "left_turn": {"n": 134}
}
```

**Response (200):** the full updated geometry (same format as GET /config).

**Error (400):** unknown servo/key, or a non-numeric value.

**Error (409):** robot is busy.

---

## Typical Workflow

Here is how an external solver would use this API:

```bash
# 1. Check that the robot is ready
curl http://pi:5000/status

# 2. Scan the cube to get its current state
curl http://pi:5000/scan
# Response: {"state": "UBUULR...", "time": 18.5}

# 3. Feed the state string to your solver (on another machine)
#    The solver returns a list of moves, e.g.: R U R' U R U2 R'

# 4. Send the solution to the robot
curl -X POST http://pi:5000/move \
  -H "Content-Type: application/json" \
  -d '{"moves": ["R", "U", "R'\''", "U", "R", "U2", "R'\''"]}'

# 5. Verify the cube is solved by scanning again
curl http://pi:5000/scan
# Response: {"state": "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB", ...}
```

## Project Structure

```
rubik_robot/
    __init__.py              # Package marker
    __main__.py              # Allows: python -m rubik_robot
    app.py                   # Flask routes and API endpoint definitions
    config.py                # Hardware constants, servo geometry (robot_config.json), calibration, pixels
    robot.py                 # RobotController: central orchestrator
    run.py                   # CLI entry point with argument parsing
    hardware/
        __init__.py
        base.py              # Abstract ServoDriver interface
        gpio_driver.py       # GPIO PWM servo implementation
        pca9685_driver.py    # PCA9685 I2C PWM servo implementation
        display.py           # OLED display wrapper (+ NullDisplay fallback)
        buttons.py           # Physical button input with debouncing
    servo/
        __init__.py
        moves.py             # Low-level servo commands and macro moves
        solver.py            # Rubik's notation to servo action translation
        calibration.py       # Physical button calibration state machine
    scanner/
        __init__.py
        camera.py            # picamera2 (libcamera) wrapper
        color.py             # Sticker color detection via nearest-neighbor
        cube_reader.py       # Cube scanning choreography (rotate + photograph)
```
