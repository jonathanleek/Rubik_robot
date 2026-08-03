# Rubik Robot API

A Flask-based REST API for controlling a Rubik's Cube solving robot built on Raspberry Pi. The robot physically manipulates a Rubik's Cube using servo-controlled grippers and wrists, and uses a camera to scan the cube's state.

This project separates the **robot control** (this API) from the **solving algorithm**, allowing you to run any solver on a separate system and send the solution moves to the robot via HTTP.

Based on [DrVoHo/Rubik_solver](https://github.com/DrVoHo/Rubik_solver). Mechanical design: [Thingiverse](https://www.thingiverse.com/thing:3826740).

## Hardware Variants

The robot supports two servo control methods:

| Variant | Flag | Description |
|---------|------|-------------|
| **PCA9685** | `--driver pca9685` | Uses an Adafruit PCA9685 I2C PWM driver board. Provides stable, jitter-free servo control. Includes OLED display and physical buttons for calibration. **Recommended.** |
| **GPIO** | `--driver gpio` | Uses the Raspberry Pi's built-in GPIO pins for PWM. Simpler hardware setup but may exhibit servo jitter. No display or buttons -- calibration is done via the API. |

---

## Installation on Raspberry Pi

### Prerequisites

- Raspberry Pi (tested on Pi 3B+ and Pi 4)
- Raspbian/Raspberry Pi OS (Bullseye or later)
- Python 3.7+
- PiCamera module (connected and enabled)
- Servo motors and mechanical assembly (see [Thingiverse](https://www.thingiverse.com/thing:3826740))
- For PCA9685 variant: Adafruit PCA9685 PWM driver board, SSD1306 OLED display, 3 push buttons

### Step 1: Enable required interfaces

```bash
sudo raspi-config
```

Enable the following under **Interface Options**:
- **Camera** (for PiCamera)
- **I2C** (for PCA9685 and OLED display -- PCA9685 variant only)

Reboot after making changes.

### Step 2: Install system dependencies

```bash
sudo apt-get update
sudo apt-get install -y python3-pip python3-dev libatlas-base-dev libjpeg-dev zlib1g-dev
```

### Step 3: Clone and install the project

```bash
cd /home/pi
git clone <your-repo-url> rubik_robot
cd rubik_robot
pip3 install -r requirements.txt
```

### Step 4: Install the font file

The OLED display (PCA9685 variant) uses the VCR OSD Mono font. Download it from [dafont.com](https://www.dafont.com/vcr-osd-mono.font) and place it in the home directory:

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
| OLED display | I2C, address 0x3C |
| Plus button | GPIO pin 11 (BOARD) |
| Minus button | GPIO pin 13 (BOARD) |
| Enter button | GPIO pin 15 (BOARD) |

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

---

## Servo Calibration

Calibration adjusts servo positions so the grippers align properly with the cube. Calibration values are saved to `~/tune_values.txt` and persist across restarts.

### Calibration parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `left_grip_tune` | Offset for the left gripper closed position (degrees) | 0 |
| `left_wrist_tune` | Offset for the left wrist center position (degrees) | 0 |
| `right_grip_tune` | Offset for the right gripper closed position (degrees) | 0 |
| `right_wrist_tune` | Offset for the right wrist center position (degrees) | 0 |
| `load` | Gripper angle for the release/load position (degrees) | 30 |
| `sleep` | Servo settling delay in seconds | 0.5 |
| `regrip_enabled` | Whether to regrip between layer moves | true |

### Calibrating via physical buttons (PCA9685 only)

1. Press the **Enter** button to start calibration
2. The OLED display shows the current parameter and value
3. Press **Plus/Minus** to adjust the value (increments of 2)
4. Press **Enter** to advance to the next parameter
5. After the last parameter (Regrip), press **Enter** to save and exit
6. Double-click **Enter** at any point to cancel without saving

### Calibrating via the API

```bash
# View current calibration
curl http://<pi-ip>:5000/calibration

# Update specific values
curl -X POST http://<pi-ip>:5000/calibration \
  -H "Content-Type: application/json" \
  -d '{"left_grip_tune": 4, "sleep": 0.45}'

# Test a single servo position
curl -X POST http://<pi-ip>:5000/calibration/test \
  -H "Content-Type: application/json" \
  -d '{"servo": "left_grip", "value": 10}'
```

### Calibration tips

1. **First run**: On the first run, all servos go to 0 degrees. Attach the gripper arms so they are roughly aligned at this position.
2. **Grip tune**: Adjust until the gripper closes squarely on the cube without excessive force.
3. **Wrist tune**: Adjust until the wrist is centered at exactly 90 degrees (cube faces align with the camera).
4. **Load position**: This is how far the grippers open during a regrip. Too small and the cube won't seat properly; too large and the cube may fall.
5. **Sleep delay**: Increase if servos don't fully reach their target position; decrease for faster solving.

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

**Response (200):**
```json
{
  "servo": "left_grip",
  "value": 10
}
```

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
    config.py                # Hardware constants, calibration, pixel locations
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
        camera.py            # PiCamera wrapper
        color.py             # Sticker color detection via nearest-neighbor
        cube_reader.py       # Cube scanning choreography (rotate + photograph)
```
