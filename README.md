# Rubik Robot — Raspberry Pi hardware API

A two-arm robot that scans and turns a Rubik's cube, driven by a Raspberry Pi.

This Pi no longer runs the solving logic. Instead it runs a small **FastAPI
server** that exposes the hardware (servos + camera) over HTTP. A separate
client — in our case an **Apache Airflow** project — does the scanning
orchestration, solving (kociemba), scrambles, and patterns, and calls these
endpoints to drive the robot.

Mechanical build: <https://www.thingiverse.com/thing:3826740>
(this code targets the **PCA9685** servo-driver variant).

> **Project status, goals, and bring-up plan:** see [docs/PROJECT.md](docs/PROJECT.md).

---

## Architecture

```
┌─────────────────────┐         HTTP/JSON          ┌──────────────────────────┐
│  Client (Airflow)   │ ─────────────────────────► │  Raspberry Pi            │
│  - kociemba solver  │   POST /scan  -> facelets  │  FastAPI (this repo)     │
│  - scramble / drill │   POST /moves -> execute   │  - PCA9685 servos (I2C)  │
│  - orchestration    │ ◄───────────────────────── │  - Pi camera (picamera2) │
└─────────────────────┘                            └──────────────────────────┘
```

The Pi owns all hardware and performs one physical operation at a time
(serialised by a lock). The cube's **orientation is tracked on the Pi** between
calls, so individual moves sent one-at-a-time compose correctly (turning most
faces physically reorients the whole cube).

### Code layout

| Path | Purpose |
|---|---|
| `rubik_robot/config.py` | Constants + persisted calibration (`Calibration`) |
| `rubik_robot/moves.py` | Cube move → servo-action translation + orientation tracking |
| `rubik_robot/vision.py` | Colour detection → 54-char facelet string |
| `rubik_robot/hardware/servos.py` | PCA9685 driver (+ mock) |
| `rubik_robot/hardware/camera.py` | picamera2 driver (+ mock) |
| `rubik_robot/robot.py` | `Robot`: servo motion, scan sequence, move execution |
| `rubik_robot/api.py` | FastAPI app |
| `rubik_robot/__main__.py` | `python -m rubik_robot` entry point |
| `setup/provision.sh` | One-shot provisioning for a blank Pi |
| `tests/` | Move-parity tests (vs. original firmware) + API tests |

---

## Hardware

- Raspberry Pi (with camera connector) running **Raspberry Pi OS Bookworm**.
- Raspberry Pi Camera Module on the CSI port.
- **PCA9685** 16-channel PWM board on I2C (address `0x40`).
- Four servos on PCA9685 channels:
  `0` left wrist, `1` left grip, `2` right wrist, `3` right grip.
- 5V supply for the servos (do **not** power servos from the Pi).

> The 180° wrist moves assume ≥270° servos (`C180 = 1` in `config.py`). Set
> `C180 = 0` for classic 90°-only servos.

---

## Setup from a blank Pi

1. Flash **Raspberry Pi OS Bookworm** (64-bit) and boot the Pi (enable SSH).
2. Clone this repo and run the provisioner:

   ```bash
   git clone <this-repo> rubik-robot
   cd rubik-robot
   chmod +x setup/provision.sh
   ./setup/provision.sh
   sudo reboot      # first run only, so I2C/camera changes take effect
   ```

   The script installs system packages, enables I2C + camera, creates a
   virtualenv (`.venv`) with access to the apt-installed `picamera2`, installs
   Python deps, runs `i2cdetect` (you should see `0x40`), and installs +
   enables a **systemd service** that starts the API on boot.

3. Verify:

   ```bash
   curl http://<pi-ip>:8000/health
   # interactive docs:
   #   http://<pi-ip>:8000/docs
   sudo journalctl -u rubik-robot -f      # live logs
   ```

### Manual run (instead of the service)

```bash
.venv/bin/python -m rubik_robot --host 0.0.0.0 --port 8000
```

---

## Calibration

On first boot the robot is uncalibrated. Tune each servo via the API (or
`/docs`) and the values persist to `data/calibration.json`:

```bash
# read current calibration
curl http://<pi-ip>:8000/tune

# adjust (any subset of fields)
curl -X PUT http://<pi-ip>:8000/tune \
     -H 'Content-Type: application/json' \
     -d '{"left_grip_tune": 6, "load": 28, "sleep": 0.4}'
```

For fine mechanical setup you can fire raw single-servo actions with `/action`
(e.g. `{"actions": "a"}` opens the left gripper). See the action alphabet in
`rubik_robot/moves.py`.

| Field | Meaning |
|---|---|
| `left/right_grip_tune` | gripper closed-position offset (deg) |
| `left/right_wrist_tune` | wrist 90°-position offset (deg) |
| `load` | gripper open ("release") position |
| `sleep` | servo settle delay (s) |
| `regrip` | 1 = reseat cube before each layer turn |

---

## API reference

| Method & path | Body | Description |
|---|---|---|
| `GET /health` | — | Liveness + wrist positions |
| `POST /home` | — | Home wrists (90°) and grippers (load) |
| `POST /grip` | — | Close both grippers onto the cube |
| `POST /regrip` | — | Reseat the cube |
| `POST /scan` | — | Run scan sequence; returns `{"facelets": "<54 chars>"}` |
| `POST /move` | `{"move":"R'"}` | Execute one move (Singmaster notation) |
| `POST /moves` | `{"moves":[...], "reset_orientation":false}` | Execute a sequence |
| `POST /orientation/reset` | — | Reset cube-orientation tracking to home |
| `POST /action` | `{"actions":"abAB"}` | Raw single-servo actions (calibration) |
| `GET /tune` | — | Current calibration |
| `PUT /tune` | partial `Calibration` | Update + persist calibration |

Facelet strings are in **kociemba order** (`U R F D L B`, 9 stickers each).

### Typical client flow (e.g. an Airflow DAG)

**Solve:**
1. `POST /scan` → 54-char facelet string.
2. Client computes the solution with kociemba.
3. `POST /moves {"moves": [...solution...]}` (leave `reset_orientation` false —
   `/scan` already left the cube in the correct post-scan frame).

**Scramble:**
1. `POST /home`.
2. `POST /moves {"moves": [...scramble...], "reset_orientation": true}`.

> **Orientation note:** because the robot reorients the cube to reach most
> faces, move execution is stateful. After a `/scan` the Pi is in the correct
> frame for a freshly computed solution. When starting from a homed cube
> (scrambles, drills), pass `reset_orientation: true` or call
> `POST /orientation/reset` first.

---

## Development & testing (off-device)

The whole server runs against **mock hardware**, so you can develop the client
without a Pi:

```bash
python3 -m venv .venv-dev
.venv-dev/bin/pip install -r requirements-dev.txt
RUBIK_MOCK=1 .venv-dev/bin/python -m rubik_robot --mock --port 8000
```

Run the tests:

```bash
.venv-dev/bin/python -m pytest tests/ -q
```

- `tests/test_moves.py` fuzzes random move sequences and asserts the refactored
  translator produces **byte-identical** servo strings to the original firmware
  (both batch and one-move-at-a-time).
- `tests/test_api.py` exercises every endpoint against a mock robot.

---

## Notes / preserved behaviour

- Servo pulse calibration (`PWM_FREQUENCY = 300`, the pulse maths in
  `hardware/servos.py`) is carried over **verbatim** from the original working
  firmware, including its intentional overshoot, so mechanical behaviour is
  unchanged.
- The colour classifier is relative (each sticker matched to the centre
  stickers captured in the same session, with manual white balance), making it
  robust to camera RGB/BGR channel ordering.
- The original German, button UI, OLED display, and on-Pi solving have been
  removed; that logic now belongs to the client. The pre-refactor scripts remain
  in git history.
