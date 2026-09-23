#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#include "JsonWrapper.h"
#include "WebServer.h"

struct Settings;

// modswitch's HTTP control surface, layered on the shared WebServer base
// (/healthz, /reset, /set_hostname). Adds:
//   POST /firmware   raw .bin body -> inactive OTA slot -> reboot
//   GET  /firmware   running image version / partition
//   GET  /config     current settings as JSON
//   POST /config     apply + persist a subset of settings
//   GET  /status     channel states, coil image, failsafe and link counters
//   POST /output     drive one channel by hand: {"channel":0,"set":"on"}
//   POST /uart_test  transmit on RS485 so the module's TX LED lights
//
// POST /firmware is the reason this class exists: once this is running,
// updates go over Wi-Fi and the serial port is only needed for recovery.
//
// Handlers recover this instance from req->user_ctx.
class SwitchWebServer : public WebServer {
public:
    SwitchWebServer(WebContext* ctx, Settings& settings);

    esp_err_t start() override;

protected:
    void populate_healthz_fields(WebContext* ctx, JsonWrapper& json) override;

private:
    static esp_err_t firmware_post_handler(httpd_req_t* req);
    static esp_err_t firmware_get_handler(httpd_req_t* req);
    static esp_err_t config_get_handler(httpd_req_t* req);
    static esp_err_t config_post_handler(httpd_req_t* req);
    static esp_err_t status_get_handler(httpd_req_t* req);
    static esp_err_t output_post_handler(httpd_req_t* req);
    static esp_err_t uart_test_post_handler(httpd_req_t* req);

    Settings& settings_;
};
