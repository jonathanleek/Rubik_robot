# Project Notes — Goals, Status & Plan

A living document describing what this project is, where it stands, and what's
next. For architecture/setup/API details see the [README](../README.md); for the
day-to-day working checklist see `.context/todos.md` (gitignored scratch).

_Last updated: 2026-06-28._

---

## 1. Intentions / goals

Build a **portable Rubik's-cube robot** driven by a Raspberry Pi, where the Pi
is a **headless hardware server** and all the "thinking" lives in a separate
client.

- **The Pi exposes its hardware over an HTTP API** (FastAPI): move servos, scan
  the cube (returns a 54-character facelet string), home, calibrate.
- **A separate client orchestrates everything** — scanning sequence, solving
  (kociemba), scrambles, patterns, drills. That client is an **Apache Airflow**
  project living in a different repo; this repo is *only* the Pi side.
- **Portable / on-the-road:** the robot will be used in different locations, so
  connectivity must not depend on whatever WiFi is around.

Why this split: the original project was a single monolithic German script that
ran solving on the Pi with a button/OLED UI. We rearchitected it into a clean,
English, testable hardware service so the orchestration can evolve independently
(and be driven by Airflow), and so the Pi just does one job well.

---

## 2. Hardware

| Component | Detail |
|---|---|
| Compute | **Raspberry Pi 3 Model B Rev 1.2**, Debian 13 (Trixie), hostname `rubik` |
| Camera | **IMX219** (Camera Module v2); upright image needs no rotation (`CAMERA_ROTATION = 0`) |
| Servo driver | **PCA9685** 16-ch PWM over I2C (addr `0x40`); servos on channels 0–3 *(on order)* |
| Servos | 4 total: ch0 left wrist, ch1 left grip, ch2 right wrist, ch3 right grip |
| Servo power | Separate **5V/3A+** supply into the PCA9685 `V+` terminal (not from the Pi) |
| Pi power | **Needs a 5V/2.5A+ supply** — under-voltage currently detected (`throttled=0x50005`) |

**Note:** the robot was originally wired with servos **directly to the Pi GPIO**.
We're moving to the PCA9685 for jitter-free, repeatable servo control (and to
offload servo current from the Pi). The software targets the PCA9685.

---

## 3. Connectivity

- **mDNS:** reachable as `rubik.local` (avahi) wherever mDNS is allowed.
- **Wired hard-line (configured):** `eth0` is NetworkManager **shared mode** at a
  fixed **`192.168.50.1/24`**. A direct USB-Ethernet cable (Mac ↔ Pi) gives
  zero-config access with no router/WiFi: the Pi runs DHCP, the API is at
  `http://192.168.50.1:8000`, and Pi internet (if any) passes through to the Mac.
  _Untested until a USB-Ethernet adapter is on hand._
- **SSH:** a key from the dev machine is authorized (passwordless login); `sudo`
  on the Pi still requires a password.
- **Future option:** Pi WiFi hotspot (AP mode) for a cable-free fallback — not yet
  set up (single radio ⇒ lockout risk; only enable once Ethernet works).

---

## 4. Current status

**Done**
- Rearchitected to a headless FastAPI package (`rubik_robot/`): config, move
  translation (with cube-orientation tracking), vision/color detection, hardware
  drivers (PCA9685 + camera) **and mock drivers**, `Robot` controller, API,
  entrypoint.
- Move translation is **byte-parity tested** against the original firmware
  (`tests/test_moves.py`); API tested on mocks (`tests/test_api.py`).
- Pi provisioned (`setup/provision.sh`): OS deps, I2C enabled, venv, systemd
  service. Verified on-device: I2C bus healthy, camera detected, API serves.
- **Vision validated on a real cube** (hand-held in the grippers): rotation = 0,
  and all 6 colors read cleanly distinct (incl. the tricky orange/red pair).
- Tooling: `tools/capture_overlay.py` to align the 9 sticker-sample points;
  camera rotation config; independently selectable `--servos`/`--camera` backends
  so the camera runs before the PCA9685 is wired.
- Resilient wired connectivity configured (above).

**Blocked on hardware (in transit)**
- PCA9685 servo driver → gates all servo work.
- USB-Ethernet adapter → to test the wired link.
- Stronger Pi power supply → clear the under-voltage flag.

---

## 5. The plan

| Phase | What | Status |
|---|---|---|
| 1 | Flash OS, first boot, SSH | ✅ done |
| 2 | Deploy code + `provision.sh` | ✅ done |
| 3 | Verify hardware: I2C bus, camera, API (no motion) | ✅ done |
| 3b | Wire PCA9685, detect `0x40` | ⏳ awaiting board |
| 4 | **Servo bring-up** — center each servo carefully, set arm neutral positions, tune grip/wrist offsets via `/tune`, get a safe `/home` | ⏳ |
| 5 | **Scan tuning** — with the cube servo-held in its true position: confirm all 9 sample centers stay in-frame for all 6 faces, lock the grid in `config.py`, sort the white-balance reference | 🟡 prepped |
| 6 | **First solve** — scan → solve (client) → `/moves`; iterate calibration until reliable | ⏳ |
| 7 | **Airflow client integration** (separate repo) — DAGs that scan/solve/scramble via the API | ⏳ future |

### Known open items to handle along the way
- **Servo calibration from scratch** — the inherited pulse constants
  (`PWM_FREQUENCY=300`, etc.) are preserved but tuned for the original builder's
  servos; expect to re-tune offsets for this build.
- **Camera framing is tight** — the fixed camera mount sits close, so the cube
  face overfills and the top-row sticker *edges* clip. Not a blocker (the scan
  only samples 9 center patches and all centers are in-frame), but verify per-face
  during Phase 5. No wider field of view is possible (sensor FoV is maxed).
- **White-balance reference** — scans currently white-balance off background;
  add a fixed neutral grey/white card in view, or lock the camera's AWB/AE after
  warmup so all 6 faces match.
- **Provisional sample grid** (hand-held, for reference): `cols=[245,500,840]`,
  `rows=[85,320,600]`, rotation `0`. Will be re-derived with the servo-held cube.

---

## 6. Repo orientation

- `rubik_robot/` — the application (see README for module map)
- `setup/` — `provision.sh` + systemd unit for a blank-Pi bring-up
- `tools/capture_overlay.py` — scan-tuning aid
- `tests/` — move-parity + API tests
- `docs/PROJECT.md` — this file
- `.context/todos.md` — live bring-up checklist (gitignored)
