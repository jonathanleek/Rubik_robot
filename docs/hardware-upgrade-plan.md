# Hardware Upgrade Plan: PCA9685 + Single USB-C Power

Goal: move the robot off **Pi-powered servos** (current setup: 4 servos plugged
directly into the Pi's GPIO, with Wago connectors joining servo power/ground to
the Pi) onto a **PCA9685 servo controller** with a **dedicated servo power rail**,
and power the *entire robot from a single USB-C input*.

## Hardware inventory

| Item | Detail |
|------|--------|
| Raspberry Pi | **3B** (official supply 2.5A; real draw ~1A typical, ~2.5A peak) |
| Servos | **4 × Miuzei DS3218**, 270°, digital, metal gear |
| Servo controller | **HiLetgo PCA9685** 16-channel, 2-pack (one active @ `0x40`, one spare) |
| Power (ordered) | USB-C PD charger + cable → PD trigger (15V) → 5V/10A module → Pi + PCA9685 |

The two PCA9685 boards are the servo *controller* (the thing servos plug into and
the Pi commands over I2C). They are **not** a power supply and have **no USB-C
input** — servo power still comes from an external feed into the board's `V+`
screw terminal.

## Why this upgrade

Digital servos actively hold position under load, so a gripper closed on the cube
draws near its holding current continuously. Four DS3218s pulling that through the
Pi's 5V rail (via the current Wago setup) risks brown-outs / Pi damage. The
PCA9685 + dedicated servo supply keeps the servo current path off the Pi.

---

## Power budget (verified)

DS3218 datasheet figures:
- Operating voltage: **4.8–6.8V**
- **Stall current: ~1.5A @ 5V** (~1.8A @ 6.8V) per servo
- Idle/quiescent: a few mA (but holding under load approaches stall)

| Load | Current @ 5V |
|------|--------------|
| One DS3218 (stall/hold) | ~1.5A |
| Realistic concurrent (2 grippers holding + 1 wrist turning) | ~4.5–6A |
| Absolute worst (all 4 stalled) | ~6A |
| Raspberry Pi 3B | ~1A typ, ~2.5A peak |
| **Design target (servos + Pi, with margin)** | **~8–9A @ 5V** |

Conclusion: a **5V / 10A supply** covers even the worst case with headroom.

---

## Power architecture

```
USB-C PD wall charger (65W)
        │  e-marked USB-C cable (5A/100W rated)
        ▼
USB-C PD trigger board (CH224K)  ── set to 15V ──┐
        │  15V                                   │ (15V keeps the buck cooler /
        ▼                                         │  higher current than 20V)
5V module → 5V @ 10A (fixed 12/24V→5V module)
        │  regulated 5V
        ├──► Raspberry Pi  (GPIO 5V pin 2/4 + GND)   ~2.5A branch
        └──► PCA9685 V+ terminal (servo power)        ~6A branch
     ALL grounds common (buck GND = Pi GND = PCA9685 GND)
```

Single USB-C input, single regulated 5V rail, but servos draw from the buck — not
through the Pi. The Pi's GPIO 5V pins only ever carry the Pi's own ~2.5A.

### 5V vs 6V servo rail — decided: 5V

Use a **single 5V rail** for both Pi and servos. The board silkscreen states
**`V+: 6V Max`**, so 6V is the absolute ceiling with no margin — not worth the risk
for marginal extra torque. DS3218 runs fine at 5V and the robot's torque needs are
low. (The chosen 5V module is fixed-output anyway — it cannot be set to 6V.)

---

## Bill of materials — ORDERED

| # | Item | Purchased | Setting |
|---|------|-----------|---------|
| 1 | USB-C PD charger, 65W | Anker Nano II 65W (B08T5QN2TR) | must expose a **15V** PDO |
| 2 | USB-C cable | Anker 100W / 5A e-marked (B08PVPTNZL) | — |
| 3 | USB-C PD trigger board | GELRHONR CH224K DIP-switch (B0FNVBNNP1) | **set to 15V** |
| 4 | 5V power module | 12/24V→5V 10A 50W fixed, 3-pack (B0G51Q8VTP) | fixed 5V — verify ~5.0V |
| 5 | Servo extension leads | uxcell 10-pk, M→F, 22AWG (B0986TGGGH) | cut off the **MALE** end |
| 6 | Wago 221 lever-nuts | 221-412 (2-way) ×~12, 221-413 (3-way) ×~4 | reuse existing if 221 |

Already on hand: HiLetgo PCA9685 ×2 (**HW-170**, bulk cap already populated),
DS3218 ×4, Raspberry Pi 3B. **No bulk capacitor to buy** — it's on the board.

### Compatibility verification (researched)

- **Trigger set to 15V, not 20V.** 15V/3A = 45W comfortably covers the ~30W
  realistic load and keeps the step-down module cooler (less heat than 20V). The
  Anker 65W offers fixed 5/9/15/20V (no 12V), so 15V is the sweet spot it can
  actually deliver.
- **5V module:** the Pololu D24V90F5 (9A) was unavailable, so the plan uses a
  **fixed 12/24V→5V 10A module** (input 10–35V covers 15V). Fixed output = no way to
  accidentally over-volt the Pi. 10A rating gives margin over the ~6A real load
  (don't expect sustained full 10A from a generic module — not needed). Verify it
  reads ~5.0–5.1V with a meter before wiring the Pi.
- **Charger PDOs (Anker Nano II 65W):** 5V/3A, 9V/3A, 15V/3A, 20V/3.25A, PPS
  5–21V/3.25A. Must advertise **15V** (it does). Any 65W PD charger with a 15V PDO
  substitutes fine.
- **DS3218 PWM:** rated **50–333 Hz**, pulse 500–2500µs. Code sends **300 Hz**
  (`config.py:191`) — within spec. If servos jitter or run hot, lower `pwm_freq`
  in `config.py` toward 50–150 Hz (config-only change).
- **Servo pulse calibration:** the PCA9685 `ServoConfig` defaults in `config.py`
  (`offset=0.0`, `pwm_range=2.0`) map 0–270° to 0–2.0 ms, but the DS3218 wants
  ~0.5–2.5 ms. Expect to tune `offset`/`pwm_range` (≈0.5 / 2.0) for full travel
  during calibration — a config edit, not a parts issue.
- **Servo connectors:** DS3218 uses a standard JR/Futaba 3-pin connector; the
  PCA9685 channel headers and the extension leads all match. 22AWG handles the
  ~1.5A per servo easily.
- **PCA9685:** 40–1000 Hz range (300 Hz OK); `V+` fed at 5V (≤6V max) ✓.

Deferred / optional: SSD1306 OLED + 3 push buttons (on-device calibration UI;
API-based calibration works without them).

---

## Capacitor note — RESOLVED

Confirmed by photo: these boards are the **HW-170** revision, which ships with the
**bulk electrolytic capacitor already populated** (the large silver cylinder next to
the green terminal block). **No cap needs to be added.**

If, under heavy load, you ever see rail sag / Pi resets, you can parallel a larger
electrolytic across the green terminal — but with DS3218 stall at only ~1.5A and the
robot moving 1–2 servos at a time, the onboard cap should be sufficient.

---

## Servo connectors (no crimping)

The servos' original 3-pin connectors were cut off — their power/ground wires
currently end bare in Wagos. To restore a proper plug that mates with the PCA9685
channel headers **without hand-crimping Dupont pins**:

1. Take a **servo extension lead** and **cut off its male end**. This leaves a
   ready-made **3-pin female servo connector** on a short pigtail.
2. Join that pigtail's 3 wires to the servo's 3 bare wires with **Wagos** (match
   colors: signal→signal, red/V+→red, brown-black/GND→GND).
3. The female connector now plugs directly onto the servo's PCA9685 channel header
   (all 3 pins in one plug). One extension lead per servo (4 total).

No crimp tool and no soldering required — the connector comes pre-made on the
extension lead; Wagos do the joining. (Solder + heatshrink is an optional cleaner
alternative to the Wagos if preferred.)

## Assembly & wiring

> ⚠️ **GOLDEN RULE:** confirm the power chain outputs **5.0 V** on the bench
> **before** anything sensitive (Pi, PCA9685, servos) is connected. A mis-set
> trigger or bad module could push 15 V downstream and instantly destroy the Pi
> and servos. Do every wiring step below with the **charger unplugged**.

Follow these steps in order, top to bottom.

### Step 0 — Bench-test the power chain (nothing sensitive attached)
1. On the **PD trigger board**, set the DIP switch to **15 V** (per the board's
   printed voltage/switch table).
2. Connect only the front half of the chain:
   - **charger → USB-C cable → trigger USB-C input**
   - **trigger `OUT+` (15 V) → module `IN+` / `VIN+`**
   - **trigger `OUT−` (GND) → module `IN−` / `VIN−`**
3. Plug in the charger. With a multimeter:
   - trigger output reads **~15 V**, and
   - **module output (`OUT+`→`OUT−`) reads 5.0–5.1 V.**
4. If the module is **not** ~5 V, **STOP** and fix it. If good, **unplug the
   charger** and leave it unplugged for every step below.

### Step 1 — Rebuild the 4 servo connectors
Per *Servo connectors* above: cut the **male** end off each extension lead and
**Wago** its pigtail to the servo's bare wires with **221-412 (2-way)** — 3 joins
per servo (signal / red-V+ / brown-GND), 12 total. Each servo now ends in a **3-pin
female plug**. Do **not** plug them into the PCA yet.

### Step 2 — Remove the old GPIO wiring
Unplug the 4 servo signal wires from the Pi header and remove the old Wagos that
tapped servo power/ground to the Pi. Servos will now be powered only by the PCA9685.

### Step 3 — Logic wiring: Pi → PCA9685 (4 jumper wires)
| Pi pin (BOARD) | Pi signal | → PCA9685 pin |
|---|---|---|
| 1 | 3.3 V | `VCC` |
| 3 | SDA (GPIO2) | `SDA` |
| 5 | SCL (GPIO3) | `SCL` |
| 6 | GND | `GND` |

Leave `OE` unconnected. `VCC` here is **logic power only** — never servo power.

### Step 4 — Servo power: module 5 V → PCA9685 green terminal
- module **`OUT+` (5 V) → PCA9685 green screw terminal `V+`**
- module **`OUT−` (GND) → PCA9685 green screw terminal `GND`**

Always the **green screw terminal** (reverse-polarity protected), **never** the
side breakout header pins. Never exceed 6 V here.

### Step 5 — Pi power + the 5 V / GND splits
The module's 5 V feeds **two** loads (Pi and PCA `V+`) and its GND returns from
**two** loads, so use **221-413 (3-way) Wagos** as splitters:

- **5 V node (221-413):**
  - in: `module OUT+`
  - out: `→ Pi 5 V (pin 2)` and `→ PCA9685 V+` *(the wire from Step 4)*
- **GND node (221-413):**
  - in: `module OUT−`
  - out: `→ Pi GND (pin 9)` and `→ PCA9685 GND` *(the wire from Step 4)*

> Powering the Pi through its GPIO 5 V pins bypasses its onboard fuse — that is
> exactly why Step 0's 5.0 V check is non-negotiable.

### Step 6 — Verify common ground
With the meter in continuity/beep mode, confirm a beep between **Pi GND** and the
**PCA9685 green-terminal GND**. They must be one net (they will be, via the Step 5
GND node and the board's internal logic-GND ↔ V+-GND bridge).

### Step 7 — First power-up WITHOUT servos plugged in
The Pi and servos now share one 5 V rail, so powering the Pi also energizes the PCA
`V+`. That's fine — on boot the PCA sends **no** PWM and the servos stay limp — but
to be safe, **leave the servo plugs OUT of the PCA channels** for the first power-up
and the I2C check. Plug them in only after I2C is confirmed (Step 9).

## Software & bring-up

### Step 8 — Enable I2C and confirm the board
1. Plug the charger into the trigger; the Pi boots from the module's 5 V.
2. `sudo raspi-config` → Interface Options → **I2C** → Enable → reboot.
3. `sudo apt-get install -y i2c-tools && i2cdetect -y 1` → expect **`40`**.
   **Do not proceed until you see `40`.**
4. `pip3 install -r requirements.txt` (includes `Adafruit_PCA9685`).

### Step 9 — Plug in the servos
Power down, then plug each servo's female plug onto its channel
(`rubik_robot/hardware/pca9685_driver.py:26`):

| Channel | Servo |
|---|---|
| 0 | Left wrist (turn) |
| 1 | Left gripper |
| 2 | Right wrist (turn) |
| 3 | Right gripper |

Orientation: the servo's **brown/black (GND)** wire goes to the **GND pin** — the
header row nearest the board edge (the black row). Reversing can damage a servo.

### Step 10 — Run and calibrate
1. Power on and run `python3 -m rubik_robot.run` (PCA9685 is the **default** driver).
2. Nudge each servo at small values first to confirm channel + direction:
   ```bash
   curl -X POST http://<pi-ip>:5000/calibration/test \
     -H "Content-Type: application/json" \
     -d '{"servo": "left_grip", "value": 10}'
   ```
   Servos: `left_grip`, `left_turn`, `right_grip`, `right_turn`.
3. Re-calibrate fully — PCA9685 geometry differs from the old GPIO variant (270°,
   180° rotation; `pca9685_config()` at `rubik_robot/config.py:176`), so old GPIO
   tune values don't carry over. Expect to also tune `offset`/`pwm_range` in
   `config.py` so the DS3218 gets its full ~0.5–2.5 ms travel (see *Compatibility
   verification*).

---

## Notes / cautions

- **HiLetgo `V+` traces are modest.** At ~6A across four servos you're within range
  for a robot that moves a couple servos at a time, but don't expect all four to
  stall simultaneously safely. The bulk cap helps.
- **Two boards on one I2C bus** would collide at `0x40`; keep the spare offline, or
  jumper it to `0x41` (that path needs a driver code change — not part of this plan).
- No code changes are required for this upgrade; the PCA9685 driver already exists
  and is the default.

## Sources
- DS3218 datasheet spec (stall current): https://servodatabase.com/servo/dsservo/rds3218
- CH224K PD trigger review: https://www.beyondlogic.org/review-usb-c-power-delivery-trigger-board-ch224/
- Pololu D24V90F5 5V/9A buck: https://www.pololu.com/product/2866
