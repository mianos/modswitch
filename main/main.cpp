// modswitch — a generic Modbus RTU switch node.
//
// Listens on RS485 as a Modbus slave and drives N relay channels from N
// coils. Built as the front-of-bike half of mqttcan: that board watches a BMW
// R1200GS's CAN bus, counts a triple click of the high beam, and writes the
// coils here; this one switches the auxiliary driving lights. Nothing in it is
// specific to that, though — it is coils to outputs, and the pin map is one
// header away in Config.h.
//
// It has Wi-Fi, an HTTP surface and OTA so it can be updated where it is
// mounted rather than on the bench. All of it is management only, started
// last and after the actuator, and none of it is in the path that drives an
// output.
//
// It deliberately has NO MQTT. It is mounted on the bike and is out of
// broker range for almost every minute it is powered, so a client there would
// be a blocking network dependency next to the actuator in exchange for
// telemetry that almost never arrives. Everything it has to say fits in GET
// /status, and if that state is ever wanted on a broker the master is the
// place to do it: it already has a client, already talks to this node every
// second, and can read it back over Modbus without this end growing a second
// network stack. Its entire job is to make its pins follow its coils.
//
// Wiring:
//   RS485 module DI <- GPIO32, RO -> GPIO33, DE+/RE <- GPIO25 (tie DE and /RE)
//   A/B to the master's pair; 120R termination at both ends of the run
//   Output pins are in Config.h, which also records why those three UART
//   pins and not the obvious ones
//
// The protocol is state, not events: the master writes every coil's desired
// value roughly once a second whether or not anything changed. So this node
// needs no resync logic — it browns out, reboots, and is correct again within
// one heartbeat.

// Standard library before the IDF headers: under picolibc, hal/assert.h
// redefines __noreturn as [[noreturn]] and the C library then uses it as a
// trailing attribute, which is a hard error in C++ if the order is reversed.
#include <cinttypes>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <sys/time.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mbcontroller.h"

#include "NvsStorageManager.h"
#include "WebServer.h"
#include "WifiManager.h"

#include "Config.h"
#include "SwitchWebServer.h"
#include "NodeState.h"
#include "Settings.h"

namespace {

constexpr const char* TAG = "modswitch";

void* g_slave = nullptr;

// Signalled by the got-IP handler; otaVerifyTask waits on it.
SemaphoreHandle_t s_got_ip = nullptr;

void onGotIp(void*, esp_event_base_t base, int32_t id, void*) {
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP && s_got_ip != nullptr) {
        xSemaphoreGive(s_got_ip);
    }
}

// The coil bits, written directly by the Modbus stack when the master sends
// FC 0x0F. One byte is eight coils, which is room for twice the channels this
// board has.
//
// Volatile because the stack writes it from its own task while ours reads it.
// That is not a substitute for synchronisation in general, but a single byte
// is written atomically on this architecture and a torn read is impossible, so
// the worst case is acting on a value one transaction stale — which the next
// heartbeat corrects anyway.
volatile uint8_t g_coils = 0;

// Set on every coil write from the master; the failsafe measures its age.
volatile uint64_t g_lastWriteUs = 0;

// Reported on the console and through GET /status.
uint32_t g_tripCount  = 0;
uint32_t g_writeCount = 0;
bool     g_trippedNow = false;

// The clock the master pushes into holding registers 0-1. Written by the Modbus
// stack from its own task, same argument as g_coils.
volatile uint16_t g_timeRegs[cfg::kTimeRegCount] = {};

// Set once the master has sent a plausible epoch, so /status can say whether
// its timestamps mean anything rather than quietly reporting 1970.
bool     g_timeSet   = false;
uint32_t g_timeSyncs = 0;

// Wall clock of the last failsafe trip, 0 when none or when the clock was not
// yet known. This is the whole reason the node wants the time at all: a trip
// count that climbed during a ride is far more useful with an hour attached.
time_t   g_lastTripAt = 0;

// Live copy of settings.failsafeMs, so the timeout can be tuned over HTTP
// without a reflash. Read by failsafeTask on every pass.
uint32_t g_failsafeMs = 5000;

// Whether each channel is currently energised, so a change can be logged
// without spamming a line per heartbeat.
bool g_applied[cfg::kChannelCount] = {};

// Serialises applyCoils. Three unrelated contexts drive the outputs — the coil
// task, the failsafe, and an HTTP POST /output — and without this two of them
// can interleave inside the compare-drive-record sequence below and leave a pin
// disagreeing with what the node reports.
//
// It does not cover the Modbus stack's own write into g_coils, which happens
// inside the component where we cannot reach. That one is harmless: the worst
// case is acting on a value one transaction stale, and the next heartbeat
// rewrites the true state.
SemaphoreHandle_t g_applyLock = nullptr;

void driveChannel(int i, bool on) {
    gpio_set_level(cfg::kChannels[i], (on != cfg::kActiveLow) ? 1 : 0);
}

// Park every output at its inactive level *before* the pad becomes an output,
// so enabling the driver cannot produce a brief pulse on the load. The
// internal pull matches, which also holds the line through reset — though the
// real fix for that window is a physical pull-down on the relay driver's
// input, because the pin floats from power-on until this function runs.
void initChannels() {
    g_applyLock = xSemaphoreCreateMutex();
    for (int i = 0; i < cfg::kChannelCount; ++i) {
        const gpio_num_t p = cfg::kChannels[i];
        gpio_reset_pin(p);
        gpio_set_level(p, cfg::kActiveLow ? 1 : 0);
        gpio_config_t c = {};
        c.pin_bit_mask = 1ULL << p;
        c.mode         = GPIO_MODE_OUTPUT;
        c.pull_up_en   = cfg::kActiveLow ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
        c.pull_down_en = cfg::kActiveLow ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;
        c.intr_type    = GPIO_INTR_DISABLE;
        gpio_config(&c);
        gpio_set_level(p, cfg::kActiveLow ? 1 : 0);
        g_applied[i] = false;
    }
    ESP_LOGI(TAG, "%d channel(s) parked off, active_%s",
             cfg::kChannelCount, cfg::kActiveLow ? "low" : "high");
}

// Push the coil bits onto the outputs. Logged only when something moves.
void applyCoils(uint8_t bits, const char* why) {
    bool changed = false;
    char s[cfg::kChannelCount + 1];

    if (g_applyLock != nullptr) xSemaphoreTake(g_applyLock, portMAX_DELAY);
    for (int i = 0; i < cfg::kChannelCount; ++i) {
        const bool on = (bits >> i) & 1u;
        if (on != g_applied[i]) {
            driveChannel(i, on);
            g_applied[i] = on;
            changed = true;
        }
        s[i] = g_applied[i] ? '1' : '0';
    }
    s[cfg::kChannelCount] = '\0';
    if (g_applyLock != nullptr) xSemaphoreGive(g_applyLock);

    // Logged outside the lock: a console line is a blocking UART write of a
    // millisecond or so, and the failsafe must never queue behind one.
    if (changed) ESP_LOGI(TAG, "channels %s (%s)", s, why);
}

#if MODSWITCH_WALK_ON_BOOT
// Identify which physical output is which coil, for a board whose pinout is in
// doubt. Coil 0 first, one second each.
void walkChannels() {
    ESP_LOGW(TAG, "channel walk: each output on for 1s, coil 0 first");
    for (int i = 0; i < cfg::kChannelCount; ++i) {
        ESP_LOGW(TAG, "  coil %d -> GPIO%d", i, (int)cfg::kChannels[i]);
        driveChannel(i, true);
        vTaskDelay(pdMS_TO_TICKS(1000));
        driveChannel(i, false);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    ESP_LOGW(TAG, "channel walk done");
}
#endif

// Forces everything off when the master goes quiet, and says so once rather
// than every cycle.
//
// It also clears the coil image, not just the pins. If it only dropped the
// outputs, a master that came back and read the coils would be told the lights
// were on while they were dark. Clearing it keeps what this node reports and
// what it is doing the same thing, and costs nothing: the master writes the
// true state on its next heartbeat regardless.
void failsafeTask(void*) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        const uint64_t last = g_lastWriteUs;
        const uint64_t age  = (uint64_t)esp_timer_get_time() - last;
        if (age < (uint64_t)g_failsafeMs * 1000ULL) {
            if (g_trippedNow) {
                g_trippedNow = false;
                ESP_LOGW(TAG, "master back after %" PRIu32 " trip(s)", g_tripCount);
            }
            continue;
        }
        if (g_trippedNow) continue;   // already off; stay quiet until comms return
        g_trippedNow = true;
        g_tripCount++;
        g_lastTripAt = g_timeSet ? time(nullptr) : 0;
        g_coils = 0;
        applyCoils(0, "FAILSAFE: no master");
        ESP_LOGE(TAG, "no coil write for %" PRIu32 "ms - all channels off (trip %" PRIu32 ")",
                 g_failsafeMs, g_tripCount);
    }
}

// Wi-Fi power save. Applied after the stack is up, and again on every settings
// change so the radio can be woken for an OTA and put back afterwards without
// a reflash.
void applyWifiPs(const std::string& mode) {
    wifi_ps_type_t ps = WIFI_PS_MAX_MODEM;
    if      (mode == "none") ps = WIFI_PS_NONE;
    else if (mode == "min")  ps = WIFI_PS_MIN_MODEM;
    else if (mode != "max") {
        ESP_LOGW(TAG, "wifi_ps '%s' unknown, using max", mode.c_str());
    }
    const esp_err_t err = esp_wifi_set_ps(ps);
    ESP_LOGI(TAG, "wifi power save '%s': %s", mode.c_str(), esp_err_to_name(err));
}

esp_err_t modbusStart() {
    mb_communication_info_t comm = {};
    comm.ser_opts.port      = cfg::kUartPort;
    comm.ser_opts.mode      = MB_RTU;
    comm.ser_opts.baudrate  = cfg::kBaud;
    comm.ser_opts.parity    = cfg::kParity;
    comm.ser_opts.uid       = cfg::kSlaveAddr;
    comm.ser_opts.data_bits = UART_DATA_8_BITS;
    comm.ser_opts.stop_bits = UART_STOP_BITS_1;

    esp_err_t err = mbc_slave_create_serial(&comm, &g_slave);
    if (err != ESP_OK || g_slave == nullptr) {
        ESP_LOGE(TAG, "mbc_slave_create_serial: %s", esp_err_to_name(err));
        return err == ESP_OK ? ESP_FAIL : err;
    }

    // The stack writes master traffic straight into g_coils.
    mb_register_area_descriptor_t area = {};
    area.type         = MB_PARAM_COIL;
    area.start_offset = cfg::kCoilStart;
    area.address      = (void*)&g_coils;
    area.size         = sizeof(g_coils);
    area.access       = MB_ACCESS_RW;
    err = mbc_slave_set_descriptor(g_slave, area);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mbc_slave_set_descriptor(coils): %s", esp_err_to_name(err));
        return err;
    }

    // Holding registers for the clock the master pushes at us. A separate area
    // from the coils on purpose: different function code, different event bit,
    // and nothing about a clock update should be able to disturb the coil
    // image that is holding the lights on.
    mb_register_area_descriptor_t timeArea = {};
    timeArea.type         = MB_PARAM_HOLDING;
    timeArea.start_offset = cfg::kTimeReg;
    timeArea.address      = (void*)&g_timeRegs[0];
    timeArea.size         = sizeof(g_timeRegs);
    timeArea.access       = MB_ACCESS_RW;
    err = mbc_slave_set_descriptor(g_slave, timeArea);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mbc_slave_set_descriptor(holding): %s", esp_err_to_name(err));
        return err;
    }

    // Pins and half-duplex mode go on after the controller has created the
    // UART driver and before the stack starts — the ordering the component's
    // own examples use.
    //
    // DE is driven as the UART's RTS rather than by hand: the peripheral
    // releases it only once the last stop bit has physically left the shift
    // register. Toggling DE in software races the FIFO and clips the tail of a
    // reply, which shows up as the master reporting intermittent CRC errors
    // under load rather than as an obvious fault.
    err = uart_set_pin(cfg::kUartPort, cfg::kPinTxd, cfg::kPinRxd, cfg::kPinDe,
                       UART_PIN_NO_CHANGE);
    if (err == ESP_OK) err = uart_set_mode(cfg::kUartPort, UART_MODE_RS485_HALF_DUPLEX);
    if (err == ESP_OK) err = mbc_slave_start(g_slave);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RS485 bring-up failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "modbus slave %d up: %d baud, TX=%d RX=%d DE=%d (half duplex), coils %d..%d",
             cfg::kSlaveAddr, cfg::kBaud, cfg::kPinTxd, cfg::kPinRxd, cfg::kPinDe,
             cfg::kCoilStart, cfg::kCoilStart + cfg::kChannelCount - 1);
    return ESP_OK;
}

// Writes from the master: coils, and the clock in the holding registers.
// mbc_slave_check_event blocks until one arrives, so this costs nothing while
// the bus is idle.
//
// Only a *coil* write feeds the failsafe. That is deliberate: the watchdog
// measures how fresh the lamp command is, and a clock update is not a lamp
// command. A master that had somehow gone on sending the time while no longer
// driving the coils must still trip it.
void coilTask(void*) {
    for (;;) {
        mbc_slave_check_event(
            g_slave, (mb_event_group_t)(MB_EVENT_COILS_WR | MB_EVENT_HOLDING_REG_WR));

        mb_param_info_t info;
        if (mbc_slave_get_param_info(g_slave, &info, MB_PAR_INFO_TOUT) != ESP_OK) continue;

        if (info.type & MB_EVENT_COILS_WR) {
            g_lastWriteUs = (uint64_t)esp_timer_get_time();
            g_writeCount++;
            applyCoils(g_coils, "master");
        }

        if (info.type & MB_EVENT_HOLDING_REG_WR) {
            const uint32_t epoch = ((uint32_t)g_timeRegs[0] << 16) | (uint32_t)g_timeRegs[1];
            // Sanity-gate it rather than trusting the wire. A half-written pair
            // or a corrupted frame that passed CRC would otherwise throw the
            // clock to 1970 or 2106, and every timestamp after it.
            if (epoch > 1700000000u) {
                struct timeval tv = { .tv_sec = (time_t)epoch, .tv_usec = 0 };
                settimeofday(&tv, nullptr);
                g_timeSyncs++;
                if (!g_timeSet) {
                    g_timeSet = true;
                    ESP_LOGI(TAG, "clock set from master: %" PRIu32, epoch);
                }
            } else {
                ESP_LOGW(TAG, "ignoring implausible epoch %" PRIu32 " from master", epoch);
            }
        }
    }
}

// Confirms a freshly OTA'd image once it has proved it can reach the network.
//
// Connectivity is the right health check even though this node's real job is
// Modbus, because an image that cannot reach the network can only be
// replaced over a serial cable on the bench.
//
// There is deliberately no deadline on that wait. The bootloader already covers
// both real failure modes: an image that crashes never reaches this point and is
// rolled back on the next boot by CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE, and an
// image that runs but cannot join Wi-Fi is left unconfirmed and rolled back at
// the next power cycle. A timeout on top of that adds one behaviour only —
// rebooting a node that is working, mid-ride, because it is out of Wi-Fi range.
// This node is holding the driving lights on; a reboot drops them for the two
// seconds it takes to get back to the coil task. Nothing is worth that.
void otaVerifyTask(void*) {
    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGW(TAG, "OTA: image pending verify; waiting for an IP");
        xSemaphoreTake(s_got_ip, portMAX_DELAY);
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGI(TAG, "OTA: connectivity confirmed, image marked valid");
    }
    vTaskDelete(nullptr);
}

}  // namespace

// The NodeState surface the HTTP layer uses. Defined here so the actuator's
// internals stay private to this file.
namespace node {

uint8_t coils() { return g_coils; }
int     channelCount() { return cfg::kChannelCount; }

bool channelState(int ch) {
    if (ch < 0 || ch >= cfg::kChannelCount) return false;
    return g_applied[ch];
}

bool setChannel(int ch, bool on) {
    if (ch < 0 || ch >= cfg::kChannelCount) return false;
    uint8_t bits = g_coils;
    if (on) bits |=  (uint8_t)(1u << ch);
    else    bits &= (uint8_t)~(1u << ch);
    g_coils = bits;
    // Counts as control activity, so the failsafe does not undo a deliberate
    // bench setting five seconds later. See NodeState.h.
    g_lastWriteUs = (uint64_t)esp_timer_get_time();
    applyCoils(bits, "http");
    return true;
}

uint32_t writeCount() { return g_writeCount; }
uint32_t tripCount()  { return g_tripCount; }
bool     tripped()    { return g_trippedNow; }

uint32_t msSinceLastWrite() {
    return (uint32_t)(((uint64_t)esp_timer_get_time() - g_lastWriteUs) / 1000ULL);
}

bool     timeSet()    { return g_timeSet; }
uint32_t timeSyncs()  { return g_timeSyncs; }
uint32_t lastTripAt() { return (uint32_t)g_lastTripAt; }

int uartTest(int ms) {
    if (ms < 100)   ms = 100;
    if (ms > 30000) ms = 30000;

    uint8_t buf[32];
    memset(buf, 0x55, sizeof(buf));

    // Safe to write straight at the port even though esp-modbus owns it: the
    // driver is installed and, with no master on the bus, the slave state
    // machine is parked waiting on RX and has nothing in flight to corrupt.
    // uart_write_bytes drives RTS/DE for us in half-duplex mode, and on an
    // auto-direction transceiver that pin simply goes nowhere.
    const int64_t end = esp_timer_get_time() + (int64_t)ms * 1000;
    int total = 0;
    while (esp_timer_get_time() < end) {
        const int n = uart_write_bytes(cfg::kUartPort, buf, sizeof(buf));
        if (n > 0) total += n;
        // Drain before pausing, so DE is released between bursts and the gap
        // is a real gap rather than the tail of the previous burst.
        uart_wait_tx_done(cfg::kUartPort, pdMS_TO_TICKS(200));
        vTaskDelay(pdMS_TO_TICKS(60));
    }
    ESP_LOGW(TAG, "uart test: %d bytes out on TX=%d", total, cfg::kPinTxd);
    return total;
}

}  // namespace node

extern "C" void app_main(void) {
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set("switchweb", ESP_LOG_INFO);
    esp_log_level_set("WiFiManager", ESP_LOG_INFO);
    esp_log_level_set("esp_netif_handlers", ESP_LOG_INFO);   // prints "sta ip: ..."
    esp_log_level_set("settings", ESP_LOG_INFO);

    // Outputs first, before anything can block: the pins must be parked whatever
    // happens to the bus or the network afterwards.
    initChannels();

#if MODSWITCH_WALK_ON_BOOT
    walkChannels();
#endif

    static NvsStorageManager nvs;      // constructing this initialises NVS flash
    static Settings settings(nvs);
    settings.log();
    g_failsafeMs = (uint32_t)settings.failsafeMs;
    settings.onChange("failsafe_ms", [] { g_failsafeMs = (uint32_t)settings.failsafeMs; });

    // The master sends UTC; this is what makes the log read in local time.
    setenv("TZ", settings.tz.c_str(), 1);
    tzset();
    settings.onChange("tz", [] {
        setenv("TZ", settings.tz.c_str(), 1);
        tzset();
    });

    // Modbus and the failsafe come up before Wi-Fi, deliberately. This node's
    // job is to switch its outputs on command and to drop them when command is
    // lost; none of that may wait on an access point, a DHCP lease, or a
    // provisioning flow that might never complete.
    if (modbusStart() != ESP_OK) {
        // Nothing to fall back to, and dark lights is the correct failure.
        // Reboot rather than sit here, in case the fault was transient.
        ESP_LOGE(TAG, "rebooting in 5s");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }

    // Start the clock now, so a node powered up before its master gets the full
    // timeout before declaring a failure rather than tripping instantly.
    g_lastWriteUs = (uint64_t)esp_timer_get_time();
    xTaskCreate(coilTask,     "coils",    4096, nullptr, 6, nullptr);
    xTaskCreate(failsafeTask, "failsafe", 3072, nullptr, 5, nullptr);

    // Everything below is for management only — OTA, config, diagnostics. A
    // failure here must never take the actuator with it.
    s_got_ip = xSemaphoreCreateBinary();

    static WiFiManager wifi(nvs, onGotIp, nullptr);
    std::string host = settings.sensorName;
    wifi.configSetHostName(host);

    // Regulatory domain. The IDF default "01" caps scanning at channel 11, so
    // an AP on ch12/13 is never found however correct the credentials are.
    // Must be set after the Wi-Fi stack is up.
    esp_err_t cr = esp_wifi_set_country_code(settings.wifiCountry.c_str(), true);
    ESP_LOGI(TAG, "wifi country '%s': %s", settings.wifiCountry.c_str(), esp_err_to_name(cr));

    applyWifiPs(settings.wifiPs);
    settings.onChange("wifi_ps", [] { applyWifiPs(settings.wifiPs); });

    static WebContext webctx(&wifi);
    static SwitchWebServer web(&webctx, settings);
    web.start();

    xTaskCreate(otaVerifyTask, "ota_verify", 4096, nullptr, 4, nullptr);

    ESP_LOGI(TAG, "modswitch started");
}
