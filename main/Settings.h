#pragma once

#include "SettingsBase.h"

// modswitch settings — persistence, reset and logging live in mianesp's
// settingsbase, the same store mqttcan uses. Member initialisers are the
// compiled-in defaults; missing or unparseable NVS config falls back to them.
//
// SettingsBase stores only std::string and int, so booleans are 0/1 ints.
//
// Pin assignments are deliberately NOT here. They are compile-time constants in
// Config.h, because a settings write that moved an output to the wrong pin
// could energise a driving light at 70km/h with no way to undo it remotely.
// Timing is safe to tune at runtime; wiring is not.
struct Settings : SettingsBase {
    std::string sensorName = "modswitch";

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
    // The radio's only job here is the occasional OTA, so "max" lets it sleep
    // through several DTIM beacons. The cost is latency: HTTP replies are
    // slower and a ~1MB OTA takes longer. If that is a nuisance, POST
    // {"wifi_ps":"min"} before the upload and put it back after.
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
