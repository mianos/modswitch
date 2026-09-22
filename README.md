# mosnode

A generic **Modbus RTU → MOSFET node** on a classic ESP32. It listens on RS485
as a Modbus slave and drives N MOSFET channels from N coils.

Built as the front-of-bike half of [mqttcan](../mqttcan): that board watches a
BMW R1200GS's CAN bus, counts a triple click of the high beam, and writes the
coils here; this one switches the auxiliary driving lights. Nothing about it is
specific to that — it is coils to gates, and the pin map is one header away in
`main/Config.h`.

Hardware: **"ESP MOS X4"**, DC 5–60 V, ESP32-D0WD-V3 (rev 3.1), 4 MB flash, no
PSRAM, four MOSFET channels. ESP-IDF v6.0.1.

## The protocol

State, not events. The master writes **every coil's desired value** roughly once
a second whether or not anything changed, so this node needs no resync logic of
any kind: it browns out, reboots, or gets plugged in ten minutes late, and it is
correct again within one heartbeat. No handshake, no sequence numbers, no
recovery path to get wrong.

| | |
|---|---|
| Unit id | `1` |
| Serial | 19200 8N1 |
| Function codes | `0x0F` write multiple coils from coil 0; `0x01` read coils works but the master never uses it |
| Coils | `0`–`3` → channels 1–4. Coil set ⇒ gate on |

## The failsafe

**All channels are forced off after `failsafe_ms` (default 5000) with no coil
write.** This is the safety property the whole system rests on, not a nicety.

The mqttcan master runs from the bike's *switched* 12 V. If this node is fed
from a *constant* circuit, then at ignition-off the master simply stops
existing — and without a watchdog here, nothing would ever tell the lights to go
out. They would burn until the battery was flat.

`off_when: ignition off` in the master's decode table is **not** the failsafe; it
cannot be, because it needs the master to be alive to act. This is.

Keep `failsafe_ms` comfortably above the master's `modbus_period_ms` (1000 ms) or
ordinary bus noise trips it. The default is five missed heartbeats.

The failsafe clears the coil image too, not just the gates, so what the node
*reports* and what it is *doing* never disagree. The master's next heartbeat
restores the true state anyway.

`POST /output` **feeds** the watchdog but is not exempt from it: the watchdog
guards against losing *all* control, not Modbus specifically, and an HTTP client
actively setting outputs is a controller. A channel set that way holds for
`failsafe_ms` from the last command and is then dropped like any other, so one
call is a pulse, not a latch. For a long bench test, repeat the call or raise
`failsafe_ms` first.

## Verified end to end

Against the real mqttcan master over RS485, 2026-09-22:

| | |
|---|---|
| Link | `modbus_link: up`, `modbus_ok: 60`, `modbus_err: 0` |
| Write counts | 60 at the master, 60 at the node — nothing dropped |
| Echo | none. `modbus_err` stayed 0, so the auto-direction transceiver is not feeding its own transmissions back |
| Gesture | a triple click injected on the CAN bus lit both lamps; a single click turned them off |
| Failsafe | master stopped with the lamps on → still on at t+3s and t+6s, **off at t+9s** against an 8000ms timeout |
| Recovery | master restarted → lamps restored on the next heartbeat, no resync, `tripped` back to false |

That last row is the "state, not events" design paying for itself: the node
converged on the correct state with no handshake and no recovery path, because
there is nothing to recover.

## Pins

| Function | GPIO |
|---|---|
| Channels 1–4 | 16, 17, 26, 27 |
| RS485 TX → transceiver DI | 32 |
| RS485 RX → transceiver RO | 33 |
| RS485 DE + /RE (tied) | 25 |
| Link LED (board) | 23 |
| Console UART0 | 1, 3 |

The channel pins come from the [blakadder Tasmota
template](https://templates.blakadder.com/diynow_ESP32_MOS_X4.html)'s function
table. **That page is internally inconsistent**: its table says 16/17/26/27, but
decoding the raw GPIO array in its template JSON appears to give 12/13/22/23.
The table matches the board this was built against, so the table wins — but if
this lands on a board where nothing switches, suspect that first and set
`MOSNODE_WALK_ON_BOOT 1`, which switches each channel on for a second in turn so
you can see which output is coil 0 without tracing the PCB.

The three RS485 pins are chosen to be safe under *either* claim, so a loom does
not have to be redone if the channel map turns out to be the other one. They
also avoid 16/17, which carry external PSRAM on a WROVER module.

UART2's *defaults* are GPIO16/17 — which are channels 1 and 2 here. Left on
defaults, Modbus traffic shows up as two flickering outputs.

**Fit a pull-down at every gate.** ESP32 pins float from power-on until firmware
runs, and a floating gate on a logic-level MOSFET will conduct. The firmware
parks each pin at its inactive level before enabling the pad and sets a matching
internal pull, but nothing in software can cover the window before software
runs.

## HTTP

Wi-Fi is ESP-Touch v2 on first boot (`WiFiManager: Not provisioned`), then
reconnects.

| | |
|---|---|
| `POST /firmware` | OTA — raw `.bin` body, reboots into it |
| `GET /firmware` | running version / partition |
| `GET`/`POST /config` | settings, NVS-backed |
| `GET /status` | channel states, coil image, write count, failsafe trips |
| `POST /output` | `{"channel":0,"set":"on"\|"off"\|"toggle"}` |
| `GET /healthz`, `POST /reset`, `POST /set_hostname` | shared `WebServer` base |

```sh
curl -s http://mosnode.local/status | jq
curl -X POST -d '{"channel":0,"set":"on"}' http://mosnode.local/output
curl -X POST --data-binary @build/mosnode.bin http://mosnode.local/firmware
curl -X POST -d '{"failsafe_ms":8000}' http://mosnode.local/config
```

`POST /reset` clears Wi-Fi credentials and reboots into provisioning. It is not
a reboot — do not reach for it to restart the board.

## Startup order, and why

Gates are parked, then Modbus and the failsafe start, and **only then** Wi-Fi.

This node's job is to switch four gates on command and drop them when command is
lost. None of that may wait on an access point, a DHCP lease, or a provisioning
flow that might never complete. A network failure cannot take the actuator with
it.

## Build and flash

```sh
idf.py set-target esp32
idf.py build
idf.py -p /dev/cu.usbserial-XXXX -b 230400 flash
```

**Flash over the wire once, then use OTA.** The partition table is 4 MB
dual-OTA (two 1.875 MB slots against a ~1 MB image) specifically so the serial
port stops mattering, and on this hardware that is worth real effort to avoid:

- The USB-serial adapter has **no DTR wired to IO0**, so esptool cannot put the
  chip into download mode by itself. It resets (RTS → EN works) and then boots
  straight into flash, and esptool reports either `Wrong boot mode detected
  (0x13)` or `Invalid head of packet` — the latter being the application's own
  log arriving where a sync response was expected.
- Recovery is a jumper from **IO0 to GND** plus a power cycle.

OTA rollback is armed (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`). A new image
marks itself valid only once it has an IP; otherwise the bootloader reverts to
the previous one. Connectivity is the right health check even though the real
job is Modbus, because an image that cannot reach the network cannot be replaced
except with that jumper. Pending-verify only happens straight after an OTA,
which by definition happened somewhere with Wi-Fi, so this cannot strand a node
that is simply out riding.

**Serial gotcha that cost an hour:** an `idf.py monitor` left open on
`/dev/tty.usbserial-*` locks the same device as `/dev/cu.usbserial-*`, and both
esptool and any other reader then get complete silence. It presents exactly like
a dead board. Check `lsof /dev/cu.usbserial-*` before believing a hardware
theory.
