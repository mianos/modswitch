#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"

// Everything board-specific lives here, so porting to another board is
// editing one file rather than hunting through main.cpp.

namespace cfg {

// ---------------------------------------------------------------------------
// Outputs
// ---------------------------------------------------------------------------

// The outputs, in coil order: coil 0 drives kChannels[0], and so on.
//
// ESP32_Relay_30A_X2: relays on GPIO12 and GPIO13, traced with a multimeter on
// the board itself. Neither published Tasmota map for LC Technology's
// ESP32_Relay_X2 (16/17 or 26/25) applies to this 30A board — trust the
// meter, not the template.
//
// GPIO12 is a strapping pin: at reset it selects the flash voltage, and held
// high it asks for 1.8V, which stops a 3.3V-flash module booting. Safe here
// because its reset default is an internal pull-down, which also keeps the
// relay open through boot. Never fit a pull-up to it.
//
// Do not use GPIO 34-39 for an output: they are input-only on the classic
// ESP32. Avoid 6-11 (SPI flash).
constexpr gpio_num_t kChannels[] = {
    GPIO_NUM_12,   // relay 1
    GPIO_NUM_13,   // relay 2
};
constexpr int kChannelCount = sizeof(kChannels) / sizeof(kChannels[0]);

// true if the board closes a relay when its pin is LOW.
constexpr bool kActiveLow = false;

// ---------------------------------------------------------------------------
// RS485 / Modbus RTU
// ---------------------------------------------------------------------------

// UART2, on explicitly assigned pins rather than its GPIO16/17 defaults, so
// the port's placement is written down here rather than implied.
//
// These three avoid:
//   12/13        the relays
//   23           link LED on the candidate relay boards
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
#define MODSWITCH_WALK_ON_BOOT 0

}  // namespace cfg
