#pragma once

#include "SettingsBase.h"

// mosnode settings — persistence, reset and logging live in mianesp's
// settingsbase, the same store mqttcan uses. Member initialisers are the
// compiled-in defaults; missing or unparseable NVS config falls back to them.
//
// SettingsBase stores only std::string and int, so booleans are 0/1 ints.
//
// Pin assignments are deliberately NOT here. They are compile-time constants in
// Config.h, because a settings write that moved a MOSFET gate to the wrong pin
// could energise a driving light at 70km/h with no way to undo it remotely.
// Timing is safe to tune at runtime; wiring is not.
struct Settings : SettingsBase {
    std::string sensorName = "mosnode";

    // Wi-Fi regulatory domain. ESP-IDF defaults to "01" (worldwide), which
    // permits only channels 1-11, so an AP on channel 12 or 13 is invisible
    // and association fails with reason 201 NO_AP_FOUND even though the
    // credentials are correct.
    std::string wifiCountry = "AU";

    // How long without a coil write before every channel is forced off.
    //
    // This is the failsafe for the whole system. The mqttcan master runs from
    // the bike's switched 12V; if this node is fed from a constant circuit then
    // at ignition-off the master simply stops existing, and without this
    // nothing would ever tell the lights to go out. They would burn until the
    // battery was flat.
    //
    // The master re-asserts coil state every modbus_period_ms (1000ms by
    // default), so this must be comfortably longer than that or ordinary bus
    // noise trips it. 5000ms is five missed heartbeats. Applies live.
    int failsafeMs = 5000;

    // Timezone for reporting only. The master pushes UTC over RS485 (see
    // Config.h kTimeReg); this is what turns it into a local timestamp in the
    // log and in /status. Matches mqttcan's default so both ends of the bus
    // read the same.
    std::string tz = "AEST-10AEDT,M10.1.0,M4.1.0/3";

    // Wi-Fi power save: "max" | "min" | "none". Applies live.
    //
    // This node sits on the bike's battery, and the radio is the largest thing
    // it does that it almost never needs — its only network job is an
    // occasional OTA. "max" sleeps through several DTIM beacons instead of
    // waking for each, which is the cheapest real saving available without
    // touching the Modbus path.
    //
    // The cost is latency: HTTP replies get slower and a ~1MB OTA can take
    // noticeably longer. If that becomes a nuisance, POST {"wifi_ps":"min"}
    // before the upload and put it back after. It is a setting rather than a
    // compile-time choice precisely so recovering from that does not need the
    // IO0 jumper.
    //
    // "none" keeps the radio awake permanently, which is what mqttcan does
    // because it is a reporter on switched power. Wrong trade here.
    std::string wifiPs = "max";

    explicit Settings(NvsStorageManager& nvs) : SettingsBase(nvs) {
        field("sensor_name",  sensorName);
        field("wifi_country", wifiCountry);
        field("failsafe_ms",  failsafeMs);
        field("tz",           tz);
        field("wifi_ps",      wifiPs);
        load();
    }
};
