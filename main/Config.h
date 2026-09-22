#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"

// Everything board-specific lives here, so porting to another MOSFET board is
// editing one file rather than hunting through main.cpp.

namespace cfg {

// ---------------------------------------------------------------------------
// Outputs
// ---------------------------------------------------------------------------

// The MOSFET gates, in coil order: coil 0 drives kChannels[0], and so on.
//
// Pins for the "ESP MOS X4", per the blakadder Tasmota template's function
// table (Relay 1-4 on GPIO 16/17/26/27, link LED on GPIO23):
//   https://templates.blakadder.com/diynow_ESP32_MOS_X4.html
//
// That page is internally inconsistent and it is worth knowing which half to
// trust. Its human-readable table says 16/17/26/27, but decoding the raw
// GPIO array in its template JSON appears to give 12/13/22/23. The table
// matches the board this was built against, so the table wins here — but if
// this firmware ever lands on a board where nothing switches, that is the
// first thing to suspect. MOSNODE_WALK_ON_BOOT below settles it against the
// hardware rather than by argument.
//
// Do not use GPIO 34-39: they are input-only on the classic ESP32 and cannot
// drive a gate. Avoid 6-11 (SPI flash) and, for an output, the strapping pins
// 0/2/5/12/15 — a gate pull-down on a strapping pin can stop the board booting.
constexpr gpio_num_t kChannels[] = {
    GPIO_NUM_16,
    GPIO_NUM_17,
    GPIO_NUM_26,
    GPIO_NUM_27,
};
constexpr int kChannelCount = sizeof(kChannels) / sizeof(kChannels[0]);

// true if the board switches the load when the gate is LOW.
constexpr bool kActiveLow = false;

// ---------------------------------------------------------------------------
// RS485 / Modbus RTU
// ---------------------------------------------------------------------------

// UART2, on explicitly assigned pins. Its *defaults* are GPIO16/17, which are
// MOSFET channels 1 and 2 on this board — left on defaults, Modbus traffic
// would show up as two flickering outputs.
//
// These three are chosen to be safe under every pinout claim made about this
// board, so a wiring loom does not have to be redone if the channel map turns
// out to be the other one. They avoid:
//   16/17/26/27  channels per the template's table
//   12/13/22/23  channels per the template's raw JSON
//   23           link LED
//   1/3          UART0, the USB-serial console
//   6-11         SPI flash
//   0/2/5/12/15  strapping pins
//   34-39        input-only, so unusable for TX or DE
// Avoiding 16/17 has a second benefit: on a WROVER module those two carry the
// external PSRAM, so nothing here can collide with it whatever module is
// fitted.
constexpr uart_port_t kUartPort = UART_NUM_2;
constexpr int         kPinTxd   = 32;   // -> transceiver DI
constexpr int         kPinRxd   = 33;   // -> transceiver RO
constexpr int         kPinDe    = 25;   // -> transceiver DE and /RE, tied together

// Must match the master. mqttcan ships modbus_baud 19200, modbus_parity none,
// and addresses this node as slave 1.
constexpr int           kSlaveAddr = 1;
constexpr int           kBaud      = 19200;
constexpr uart_parity_t kParity    = UART_PARITY_DISABLE;

// First coil address. The master writes from coil 0 upward in one FC 0x0F
// transaction covering every channel.
constexpr uint16_t kCoilStart = 0;

// Holding registers 0-1 carry a 32-bit Unix epoch, high word first, written by
// the master with FC 0x10. Modbus has no function code for time, so this is a
// convention between our two ends rather than a standard; mqttcan's
// ModbusBus.h declares the same address, and a mismatch would be a silent
// no-op. This node has no clock and no network, so this is the only way its
// logs and its failsafe trips can carry a real timestamp.
constexpr uint16_t kTimeReg      = 0;
constexpr uint16_t kTimeRegCount = 2;

// ---------------------------------------------------------------------------
// Failsafe
// ---------------------------------------------------------------------------

// All channels are forced off after this long with no coil write from the
// master.
//
// This is the failsafe for the whole system, not a nicety. The master runs off
// the bike's switched 12V; if this node is fed from a constant circuit, then at
// ignition-off the master simply stops existing and nothing else would ever
// tell this board to turn the lights off. They would burn until the battery
// was flat.
//
// The master re-asserts state every modbus_period_ms (1000ms by default), so
// this must be comfortably longer than that or ordinary bus noise trips it.
// 5 seconds is five missed heartbeats.
// This is the compiled-in default only. The live value is settings.failsafe_ms
// (Settings.h), so the timeout can be tuned over HTTP without a reflash.
constexpr uint32_t kCommsTimeoutMs = 5000;

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

// 1 = on boot, switch each channel on for a second in turn, lowest coil first,
// then all off. This is how you find out which physical output is coil 0
// without tracing the PCB, and how to check kChannels against a board whose
// pinout is in doubt.
//
// Leave it 0 in service. On a vehicle it would flash the driving lights every
// time the ignition was switched on.
#define MOSNODE_WALK_ON_BOOT 0

}  // namespace cfg
