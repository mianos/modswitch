#include "MosWebServer.h"

#include <algorithm>
#include <array>
#include <cinttypes>
#include <string>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "Config.h"
#include "NodeState.h"
#include "Settings.h"

namespace {

constexpr const char* TAG = "mosweb";

// /config and /output bodies are tiny. Bound the input so a hostile
// Content-Length cannot trigger a huge std::string allocation; /firmware has
// its own bound, the OTA partition size.
constexpr size_t kMaxJsonBodyBytes = 4 * 1024;

std::string read_request_body(httpd_req_t* req) {
    std::string body;
    body.reserve(req->content_len);
    char buf[256];
    int remaining = req->content_len;
    while (remaining > 0) {
        int got = httpd_req_recv(req, buf, std::min<int>(remaining, (int)sizeof(buf)));
        if (got <= 0) break;
        body.append(buf, got);
        remaining -= got;
    }
    return body;
}

esp_err_t send_json(httpd_req_t* req, const JsonWrapper& json) {
    httpd_resp_set_type(req, "application/json");
    std::string out = json.ToString();
    return httpd_resp_sendstr(req, out.c_str());
}

std::string uptimeString() {
    uint32_t s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    uint32_t d = s / 86400; s %= 86400;
    uint32_t h = s / 3600;  s %= 3600;
    return std::to_string(d) + "d " + std::to_string(h) + "h " +
           std::to_string(s / 60) + "m";
}

}  // namespace

MosWebServer::MosWebServer(WebContext* ctx, Settings& settings)
    : WebServer(ctx), settings_(settings) {}

void MosWebServer::populate_healthz_fields(WebContext*, JsonWrapper& json) {
    json.AddItem("uptime",    uptimeString());
    json.AddItem("heap_free", (int)esp_get_free_heap_size());
    json.AddItem("tripped",   node::tripped());
}

esp_err_t MosWebServer::start() {
    esp_err_t err = WebServer::start();
    if (err != ESP_OK) return err;

    struct Route {
        const char*   uri;
        httpd_method_t method;
        esp_err_t (*handler)(httpd_req_t*);
    };
    static const std::array<Route, 7> routes = {{
        {"/firmware",  HTTP_POST, firmware_post_handler},
        {"/firmware",  HTTP_GET,  firmware_get_handler},
        {"/config",    HTTP_GET,  config_get_handler},
        {"/config",    HTTP_POST, config_post_handler},
        {"/status",    HTTP_GET,  status_get_handler},
        {"/output",    HTTP_POST, output_post_handler},
        {"/uart_test", HTTP_POST, uart_test_post_handler},
    }};

    for (const Route& r : routes) {
        httpd_uri_t uri = {
            .uri      = r.uri,
            .method   = r.method,
            .handler  = r.handler,
            .user_ctx = this,
        };
        esp_err_t e = httpd_register_uri_handler(server, &uri);
        if (e != ESP_OK) ESP_LOGE(TAG, "register %s: %s", r.uri, esp_err_to_name(e));
    }
    return ESP_OK;
}

// POST /firmware — raw .bin body into the inactive OTA slot, then reboot.
//
// The image comes up pending-verify; main.cpp confirms it once it has an IP,
// and the bootloader rolls back to this image if it never does. That rollback
// is the whole safety net for a board whose serial recovery needs a jumper.
esp_err_t MosWebServer::firmware_post_handler(httpd_req_t* req) {
    if (req->content_len <= 0) return sendJsonError(req, 400, "Content-Length required");

    const esp_partition_t* target = esp_ota_get_next_update_partition(nullptr);
    if (target == nullptr) return sendJsonError(req, 500, "no OTA partition available");
    if (req->content_len > (int)target->size) {
        return sendJsonError(req, 413, "image larger than OTA partition");
    }
    ESP_LOGW(TAG, "OTA: writing %d bytes to %s @ 0x%" PRIx32,
             req->content_len, target->label, target->address);

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (err != ESP_OK) return sendJsonError(req, 500, esp_err_to_name(err));

    char buf[1024];
    int remaining = req->content_len;
    int written = 0;
    while (remaining > 0) {
        int got = httpd_req_recv(req, buf, std::min<int>(remaining, (int)sizeof(buf)));
        if (got == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (got <= 0) {
            esp_ota_abort(handle);
            return sendJsonError(req, 400, "request body truncated");
        }
        err = esp_ota_write(handle, buf, got);
        if (err != ESP_OK) {
            esp_ota_abort(handle);
            return sendJsonError(req, 500, esp_err_to_name(err));
        }
        written   += got;
        remaining -= got;
    }

    err = esp_ota_end(handle);
    if (err != ESP_OK) return sendJsonError(req, 400, esp_err_to_name(err));
    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) return sendJsonError(req, 500, esp_err_to_name(err));

    ESP_LOGW(TAG, "OTA: %d bytes written to %s; rebooting", written, target->label);
    JsonWrapper resp;
    resp.AddItem("status",    std::string("ok"));
    resp.AddItem("written",   written);
    resp.AddItem("partition", std::string(target->label));
    send_json(req, resp);

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

esp_err_t MosWebServer::firmware_get_handler(httpd_req_t* req) {
    const esp_app_desc_t*  desc    = esp_app_get_description();
    const esp_partition_t* running = esp_ota_get_running_partition();
    JsonWrapper resp;
    resp.AddItem("version",   std::string(desc->version));
    resp.AddItem("idf_ver",   std::string(desc->idf_ver));
    resp.AddItem("date",      std::string(desc->date));
    resp.AddItem("time",      std::string(desc->time));
    resp.AddItem("partition", std::string(running->label));
    return send_json(req, resp);
}

esp_err_t MosWebServer::config_get_handler(httpd_req_t* req) {
    auto* self = static_cast<MosWebServer*>(req->user_ctx);
    JsonWrapper resp = self->settings_.toJson();
    return send_json(req, resp);
}

esp_err_t MosWebServer::config_post_handler(httpd_req_t* req) {
    auto* self = static_cast<MosWebServer*>(req->user_ctx);
    if (req->content_len > (int)kMaxJsonBodyBytes) {
        return sendJsonError(req, 413, "request body too large");
    }
    std::string body = read_request_body(req);
    if (body.empty()) return sendJsonError(req, 400, "empty body");
    JsonWrapper json = JsonWrapper::Parse(body);
    if (json.Empty()) return sendJsonError(req, 400, "invalid JSON");

    self->settings_.loadFromJson(json);
    self->settings_.save();
    self->settings_.log();

    JsonWrapper resp = self->settings_.toJson();
    return send_json(req, resp);
}

// GET /status — what this node is doing and whether the bus is healthy.
//
// ch<N> is flattened one key per channel rather than an array because
// JsonWrapper has no nested containers.
esp_err_t MosWebServer::status_get_handler(httpd_req_t* req) {
    auto* self = static_cast<MosWebServer*>(req->user_ctx);
    JsonWrapper resp;
    resp.AddItem("uptime",     uptimeString());
    resp.AddItem("heap_free",  (int)esp_get_free_heap_size());
    resp.AddItem("slave_addr", cfg::kSlaveAddr);
    resp.AddItem("baud",       cfg::kBaud);
    resp.AddItem("channels",   node::channelCount());
    resp.AddItem("coils",      (int)node::coils());
    for (int i = 0; i < node::channelCount(); ++i) {
        resp.AddItem("ch" + std::to_string(i),
                     std::string(node::channelState(i) ? "on" : "off"));
    }
    resp.AddItem("writes",          (int)node::writeCount());
    resp.AddItem("ms_since_write",  (int)node::msSinceLastWrite());
    resp.AddItem("failsafe_ms",     self->settings_.failsafeMs);
    resp.AddItem("failsafe_trips",  (int)node::tripCount());
    resp.AddItem("tripped",         node::tripped());
    return send_json(req, resp);
}

// POST /output — {"channel":0,"set":"on"|"off"|"toggle"}
//
// For bench work and for identifying which physical output is which. Note this
// feeds the comms watchdog (see NodeState.h), so a channel set here stays set;
// it is not quietly undone five seconds later.
esp_err_t MosWebServer::output_post_handler(httpd_req_t* req) {
    if (req->content_len > (int)kMaxJsonBodyBytes) {
        return sendJsonError(req, 413, "request body too large");
    }
    std::string body = read_request_body(req);
    if (body.empty()) return sendJsonError(req, 400, "empty body");
    JsonWrapper json = JsonWrapper::Parse(body);
    if (json.Empty()) return sendJsonError(req, 400, "invalid JSON");

    int ch = -1;
    if (!json.GetField("channel", ch)) {
        return sendJsonError(req, 400, "need an integer 'channel'");
    }
    std::string set;
    if (!json.GetField("set", set)) {
        return sendJsonError(req, 400, "need 'set' of on, off or toggle");
    }

    bool want;
    if (set == "on")          want = true;
    else if (set == "off")    want = false;
    else if (set == "toggle") want = !node::channelState(ch);
    else return sendJsonError(req, 400, "'set' must be on, off or toggle");

    if (!node::setChannel(ch, want)) {
        return sendJsonError(req, 400, "channel out of range");
    }

    JsonWrapper resp;
    resp.AddItem("channel", ch);
    resp.AddItem("state",   std::string(want ? "on" : "off"));
    resp.AddItem("coils",   (int)node::coils());
    return send_json(req, resp);
}

// POST /uart_test — {"ms": 3000}, transmit for that long so the RS485 module's
// TX LED lights and the transmit half of the wiring can be proved before a
// master exists. Blocks for the duration; the response comes afterwards.
esp_err_t MosWebServer::uart_test_post_handler(httpd_req_t* req) {
    int ms = 3000;
    if (req->content_len > 0 && req->content_len <= (int)kMaxJsonBodyBytes) {
        JsonWrapper json = JsonWrapper::Parse(read_request_body(req));
        if (!json.Empty()) json.GetField("ms", ms);
    }

    const int sent = node::uartTest(ms);

    JsonWrapper resp;
    resp.AddItem("sent_bytes", sent);
    resp.AddItem("tx_gpio",    cfg::kPinTxd);
    resp.AddItem("baud",       cfg::kBaud);
    return send_json(req, resp);
}
