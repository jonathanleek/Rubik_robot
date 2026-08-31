# Running the robot as a WiFi hotspot

At a conference (or anywhere with unknown/captive WiFi), the robot can host
its **own** WiFi network so you can reach the API directly from a laptop or
phone — no external network required.

## How it works

The Pi 3B has a single WiFi radio, so it is *either* a client (joining an
existing network) *or* an access point (hosting its own) — not both. In
hotspot mode the Pi broadcasts a network you connect to and serves DHCP to
your device. The Flask API already binds to all interfaces (`0.0.0.0`), so it
is immediately reachable over the hotspot with no application changes.

There is **no internet** through the hotspot — you don't need it to call the
API.

| Setting | Value |
|---------|-------|
| SSID | `rubik-robot` |
| Password | `solvethecube` (WPA2) |
| Pi / gateway IP | `192.168.4.1` |
| API URL | `http://192.168.4.1:5000` (also `http://rubik.local:5000`) |
| Client DHCP range | `192.168.4.10` – `192.168.4.254` (NetworkManager) / `.2` – `.20` (hostapd path) |

Change these by editing the variables at the top of
[`scripts/setup-hotspot.sh`](../scripts/setup-hotspot.sh) before running it.

## Setup

Run once on the Pi (needs sudo; it makes system network changes):

```bash
cd ~/rubik_robot
sudo ./scripts/setup-hotspot.sh
```

The script auto-detects your OS's networking stack and configures the right one:

- **Bookworm and later** → NetworkManager (`nmcli` access-point profile).
- **Bullseye** → `hostapd` + `dnsmasq` (installed automatically if missing).

The hotspot is **always-on**: it comes back automatically on every boot.
Reboot once to confirm:

```bash
sudo reboot
```

## Setup without any network access (edit the SD card on a Mac/PC)

If you can't SSH in at all -- no usable WiFi, no Ethernet -- you can still turn
the hotspot on by editing **only the FAT `bootfs` partition** of the SD card.
This uses the same one-shot boot hook Raspberry Pi Imager uses for its own
first-boot customisation, so it works headless and nothing on the card is wiped
(calibration, code, the saved client-WiFi profile all survive).

1. Power the Pi off, put the card in your computer. The FAT partition mounts as
   `bootfs` (macOS: `/Volumes/bootfs`; it is the only partition macOS can write).
2. Copy [`scripts/bootfs/rubik-hotspot.nmconnection`](../scripts/bootfs/rubik-hotspot.nmconnection)
   and [`scripts/bootfs/rubik-hotspot-firstrun.sh`](../scripts/bootfs/rubik-hotspot-firstrun.sh)
   onto it. Edit the SSID/password in the keyfile and `INSTALL_API_SERVICE` at the
   top of the script first if you want something other than the defaults.
3. Append this to the **single line** in `cmdline.txt` (one leading space, no
   newline, LF line endings only):

   ```
    systemd.run=/boot/firmware/rubik-hotspot-firstrun.sh systemd.run_success_action=reboot systemd.unit=kernel-command-line.target
   ```

4. Eject and boot the Pi. On that boot systemd runs the script as root instead of
   the normal targets; it **first** scrubs the hook back out of `cmdline.txt`
   (so a boot loop is impossible), installs the keyfile as `root:root 0600`,
   installs a status reporter, optionally the API service, logs everything to
   `bootfs/rubik-hotspot.log`, and reboots. The SSID appears ~2 minutes after
   power-on.

**Diagnostics without a network:** pull the card and read
`bootfs/rubik-hotspot.log` (what the hook did, incl. OS/NM versions and whether
`dnsmasq-base` is present) and `bootfs/rubik-hotspot-status.log` (rewritten every
60 s for the first 15 min of every boot: `nmcli`/`ip`/`iw` state, DHCP leases,
the wpa_supplicant and NetworkManager journals -- so a failed client attempt shows
up there as `AP-STA-*` / `POSSIBLE-PSK-MISMATCH` lines).

**Worst case:** if the hook can't execute at all, systemd's default
`FailureAction` powers the Pi off (it will not hang or loop). Restore
`cmdline.txt` from the `cmdline.txt.pre-hotspot` copy the script keeps, or
remove the `systemd.run=...` words by hand.

## Auto-start the API too (recommended for conferences)

The API server does not start on boot by default. Since you'll be relying on
the hotspot to reach it, install it as a service so it's running the moment
the Pi powers on — no SSH needed:

```bash
sudo ./scripts/install-api-service.sh                 # PCA9685 (default)
sudo ./scripts/install-api-service.sh --driver gpio   # GPIO variant
```

Check it:

```bash
systemctl status rubik-api
journalctl -u rubik-api -f
```

> If you already set up the `rubik-robot` systemd service from README Step 8,
> you don't need this — that unit does the same job. Don't enable both at once
> (they'd both try to bind port 5000). This one adds a `--driver` flag and
> waits for `network-online.target` so it comes up cleanly with the hotspot.

## Using it at the conference

1. Power on the Pi. Wait ~30 s for the hotspot to come up.
2. On your laptop/phone, join WiFi **`rubik-robot`** (password `solvethecube`).
3. Hit the API:

   ```bash
   curl http://192.168.4.1:5000/status
   ```

## Reverting to normal WiFi

```bash
sudo ./scripts/setup-hotspot.sh --disable
```

On the NetworkManager path this removes the hotspot profile and the Pi falls
back to your other saved networks. On the Bullseye path it disables
`hostapd`/`dnsmasq` and removes the static-IP block; re-add your home network
with `sudo raspi-config` → *System Options* → *Wireless LAN*.

To also stop the API auto-starting:

```bash
sudo ./scripts/install-api-service.sh --disable
```

Without network access: create an empty file named `rubik-hotspot.disable` on
`bootfs`, re-add the `systemd.run=...` hook to `cmdline.txt` as above, and boot.
The hook removes the hotspot profile instead of installing it.

## Troubleshooting

- **No `rubik-robot` network appears.** Check the radio isn't soft-blocked:
  `rfkill list` (unblock with `sudo rfkill unblock wifi`). AP mode also needs a
  regulatory country set — the script sets it, but you can confirm with
  `iw reg get`. On the Bullseye path check `sudo systemctl status hostapd`.
- **Connected but can't reach the API.** Confirm the API is running
  (`systemctl status rubik-api` or start it manually with
  `python3 -m rubik_robot.run`) and that you're using `192.168.4.1`, not
  `rubik.local`, if your device doesn't resolve mDNS.
- **Two devices got the same IP / no IP.** Restart DHCP: NetworkManager path
  `sudo nmcli connection up rubik-hotspot`; Bullseye path
  `sudo systemctl restart dnsmasq`.
- **Network is visible but every device says "incorrect password"** (macOS CLI:
  error `-3925`, a WPA supplicant timeout). The password is fine -- the AP is
  failing the WPA2 handshake. See the Pi 3B gotchas below.

## Pi 3B / Zero W gotchas (brcmfmac BCM43438) -- verified 2026-08-30

Verified on a Pi 3B, Raspberry Pi OS Bookworm (kernel 6.12.93), NetworkManager
1.42.4, with a macOS 26 laptop and an Android phone. Both apply to the
`nmcli` path and to the keyfile; the shipped files already include them.

1. **Disable PMF (802.11w).** `wifi-sec.pmf disable` / keyfile `pmf=1`. The
   brcmfmac driver can't do Management Frame Protection in AP mode; with NM's
   default (*optional*) Apple and Android clients negotiate it and the 4-way
   handshake times out. [raspberrypi/linux#3619](https://github.com/raspberrypi/linux/issues/3619)
2. **Don't pin `proto`/`pairwise`/`group`.** Setting `proto=rsn`,
   `pairwise=ccmp`, `group=ccmp` explicitly (a common "WPA2-only" hardening)
   breaks the handshake on this chip -- every client reports a wrong password.
   Leave them to NetworkManager's defaults. The Zero 2 W / Pi 4 don't care.
   [Pi forums t=358481](https://forums.raspberrypi.com/viewtopic.php?t=358481)
3. NM 1.42 still lists the `WPA-PSK-SHA256` AKM in `key_mgmt` even with PMF
   disabled (there is no connection setting to suppress it). Harmless once 1 and
   2 are in place.

What was verified working: macOS + Android join; DHCP leases in `192.168.4.x`;
`curl http://192.168.4.1:5000/status`; `rubik.local` resolves via mDNS from
macOS; key-based SSH to `pi@192.168.4.1`. On the laptop, a phone tethered over
USB supplied the default route while WiFi talked to the robot -- the
`192.168.4.0/24` route stays on WiFi, so both work at once.
