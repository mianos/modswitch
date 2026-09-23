# modswitch

A generic **Modbus RTU switch node** on a classic ESP32. It listens on RS485
as a Modbus slave and drives N relay channels from N coils. (Formerly
`mosnode`.)

Built as the front-of-bike half of [mqttcan](../mqttcan): that board watches a
BMW R1200GS's CAN bus, counts a triple click of the high beam, and writes the
coils here; this one switches the auxiliary driving lights. Nothing about it is
specific to that — it is coils to outputs, and the pin map is one header away in
`main/Config.h`.

Target hardware: **ESP32_Relay_30A_X2** — ESP32-WROOM-32, 7–28 V input, two
SONGLE SLA-05VDC-SL-C relays (20 A as SPDT, 5 V coil), BOOT and RST buttons.
ESP-IDF v6.0.1.

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
| Function codes | `0x0F` write multiple coils from coil 0; `0x10` write holding registers 0–1 for the clock; `0x01` read coils works but the master never uses it |
| Coils | `0`–`3` → channels 1–4. Coil set ⇒ relay closed |
| Holding regs | `0`–`1` → 32-bit Unix epoch, high word first |

## The clock

This node has no RTC and no network, so without the master it has no idea what
time it is. Modbus has **no function code for time** — the spec has no notion of
it — so the master writes a 32-bit Unix epoch into holding registers 0–1 with
FC 0x10, roughly once a minute. The address is a shared constant on both sides
(`cfg::kTimeReg` here, `modbusbus::kTimeReg` in mqttcan); a mismatch would be a
silent no-op.

The point is not tidy log lines. It is that **`failsafe_trips` becomes "the link
dropped at 14:49:30"** instead of a bare number you cannot place in a ride.

Incoming epochs are sanity-gated rather than trusted: a half-written pair or a
corrupt frame that happened to pass CRC would otherwise throw the clock to 1970
or 2106 and every timestamp after it. Until a plausible one arrives, `/status`
reports `"unknown"` rather than presenting 1970 as a fact.

**A clock write does not feed the failsafe.** Only a coil write does. The
watchdog measures how fresh the *lamp command* is, and a clock update is not a
lamp command — a master that somehow kept sending the time while no longer
driving the coils must still trip it.

`tz` (default `AEST-10AEDT,M10.1.0,M4.1.0/3`, matching mqttcan) is what turns
the UTC epoch into a local timestamp for reporting.

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

The failsafe clears the coil image too, not just the outputs, so what the node
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
| Channels | 16, 17, 26, 27 — **provisional**, see below |
| RS485 TX → transceiver DI | 32 |
| RS485 RX → transceiver RO | 33 |
| RS485 DE + /RE (tied) | 25 |
| Console UART0 | 1, 3 |

**The channel pins are not confirmed for the relay board yet.** The nearest
published map is LC Technology's [`ESP32_Relay_X2`](https://templates.blakadder.com/ESP32_Relay_X2.html)
Tasmota template, which exists in two revisions:

| Revision | Relay 1 | Relay 2 | LED |
|---|---|---|---|
| A | GPIO16 | GPIO17 | GPIO23 |
| B | GPIO26 | GPIO25 | GPIO23 |

That listing is a 5–60 V board, not the 7–28 V 30 A one, so treat it as a lead.
**Revision B collides with RS485 DE on GPIO25** — if the board turns out to be
B, move DE. `MODSWITCH_WALK_ON_BOOT 1` switches each channel on for a second in
turn, coil 0 first, which settles it against the hardware. Then set
`kChannels` in `Config.h`.

Reading Tasmota templates: on ESP32 the GPIO array is **not** indexed by GPIO
number. Its slots run `0,1,2,3,4,5,9,10,12,13,…,27,6,7,8,11,32…39`.

UART2's defaults are GPIO16/17. The RS485 pins are assigned explicitly so
Modbus traffic can never land on an output.

ESP32 pins float from power-on until firmware runs. The firmware parks each
output at its inactive level before enabling the pad, with a matching internal
pull, but only a physical pull-down on the relay driver's input covers the
window before that. Check the board has one.

## Wiring the load

The relay contacts are isolated from the board, so the lamp circuit and the
board's own supply are independent:

- Board `7-28V` input ← **switched** 12 V.
- Relay **COM** ← battery positive through a fuse sized for the lamp.
- Relay **NO** → lamp positive; lamp negative → ground.

Use **NO**, not NC: a de-energised relay — no power, no firmware yet, failsafe
tripped — then means lamp off.

The 7–28 V input has little headroom over an automotive load dump. Fit a TVS
across it.

## HTTP

Wi-Fi is ESP-Touch v2 on first boot (`WiFiManager: Not provisioned`), then
reconnects.

| | |
|---|---|
| `POST /firmware` | OTA — raw `.bin` body, reboots into it |
| `GET /firmware` | running version / partition / `ota_state` — **check this is `valid` after an OTA before powering down** |
| `GET`/`POST /config` | settings, NVS-backed |
| `GET /status` | channel states, coil image, write count, failsafe trips, clock and `last_trip_at` |
| `POST /output` | `{"channel":0,"set":"on"\|"off"\|"toggle"}` |
| `POST /uart_test` | transmit on RS485 so the module's TX LED lights — bring-up only |
| `GET /healthz`, `POST /reset`, `POST /set_hostname` | shared `WebServer` base |

`POST /uart_test` **returns 409 while a master is driving the bus.** It puts raw
bytes on the shared pair for seconds at a time, which collides with every Modbus
transaction for the duration and trips the failsafe — on a moving vehicle, that
is the lights going out. It was written when reaching it meant a serial cable;
over Wi-Fi it needs to refuse by itself. Add `"force":true` to override, and the
node is considered idle once nothing has written a coil for `failsafe_ms`.

```sh
curl -s http://modswitch.local/status | jq
curl -X POST -d '{"channel":0,"set":"on"}' http://modswitch.local/output
curl -X POST --data-binary @build/modswitch.bin http://modswitch.local/firmware
curl -X POST -d '{"failsafe_ms":8000}' http://modswitch.local/config
```

`POST /reset` clears Wi-Fi credentials and reboots into provisioning. It is not
a reboot — do not reach for it to restart the board.

## Startup order, and why

Outputs are parked, then Modbus and the failsafe start, and **only then** Wi-Fi.

This node's job is to switch its outputs on command and drop them when command is
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
dual-OTA (two 1.875 MB slots against a ~1 MB image). For the wired flash, if the
USB-serial adapter has no auto-reset, hold **BOOT**, tap **RST**, then release
BOOT to enter download mode.

OTA rollback is armed (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`). A new image
marks itself valid once it has an IP. Connectivity is the right health check
even though the real job is Modbus, because an image that cannot reach the
network can only be replaced over a serial cable.

**That wait has no deadline, deliberately.** The bootloader already covers both
real failure modes: an image that crashes never reaches the check and is rolled
back on the next boot, and one that runs but cannot join Wi-Fi stays
unconfirmed and is rolled back at the next power cycle. A timeout on top of
that adds exactly one behaviour — rebooting a node that is working fine,
mid-ride, because it happens to be out of range. On a node holding the driving
lights on, that is the lamps going dark for the two seconds it takes to get
back to the coil task. Nothing is worth that.

**Serial gotcha that cost an hour:** an `idf.py monitor` left open on
`/dev/tty.usbserial-*` locks the same device as `/dev/cu.usbserial-*`, and both
esptool and any other reader then get complete silence. It presents exactly like
a dead board. Check `lsof /dev/cu.usbserial-*` before believing a hardware
theory.
