#pragma once

#include <cstdint>

// The narrow surface the HTTP layer is allowed to see, so the actuator's
// internals stay in main.cpp rather than becoming globals in a header.
namespace node {

// Current coil image, bit n = channel n.
uint8_t coils();

// Whether channel `ch` is physically energised right now. This can disagree
// with coils() for the moment between a write and the next apply.
bool channelState(int ch);

// Drive one channel by hand, for bench work and for checking which physical
// output is which without a master on the bus.
//
// This feeds the comms watchdog, deliberately: the watchdog guards against
// losing *all* control, not against losing Modbus specifically, and an HTTP
// client actively setting outputs is a controller.
//
// It feeds it, it does not exempt from it. A channel set here holds for
// failsafe_ms from the *last* command and is then dropped like any other, so a
// single call is a pulse rather than a latch. That is the point of a dead-man's
// switch — a curl command that scrolled off someone's terminal is not evidence
// that anybody still wants the lights on. To hold a channel for a long bench
// test, repeat the call, or raise failsafe_ms first.
//
// Returns false for an out-of-range channel.
bool setChannel(int ch, bool on);

// How many coil writes have been accepted from the master since boot, and how
// long ago the last one was. A write count stuck at zero with the master
// running means the bus, not the firmware.
uint32_t writeCount();
uint32_t msSinceLastWrite();

// Failsafe trips since boot, and whether it is tripped now. A trip count that
// climbs during a ride is the signal that the RS485 run is marginal.
uint32_t tripCount();
bool     tripped();

int channelCount();

// Transmit a pattern on the RS485 port for `ms` milliseconds and return the
// byte count.
//
// Bring-up only. A Modbus slave never speaks unbidden, so with no master on
// the bus this node is silent forever and the module's TX LED never lights —
// which looks identical to "the UART pin is wrong" or "the transceiver is
// dead". This makes the port say something on demand, so the transmit half of
// the wiring can be proved before the master exists.
//
// 0x55 is 01010101, so every bit toggles: it is the easiest pattern to
// recognise on a scope and it keeps the LED evenly lit rather than flickering
// at whatever duty a text string happens to have.
int uartTest(int ms);

}  // namespace node
