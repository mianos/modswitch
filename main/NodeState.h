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
// This also feeds the comms watchdog, and that is deliberate: the watchdog
// guards against losing *all* control, not against losing Modbus specifically.
// An HTTP client that is actively setting outputs is a controller, and having
// the failsafe cut the lights out from under it five seconds later would be
// astonishing rather than safe. Returns false for an out-of-range channel.
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

}  // namespace node
