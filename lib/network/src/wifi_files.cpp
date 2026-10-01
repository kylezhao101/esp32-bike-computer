#include "wifi_files.hpp"
#include "wifi_build_secrets.h"

#include "sd_access.hpp"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>

namespace wifi_files {
namespace {
constexpr char tag[] = "wifi_files";
constexpr char setup_ssid[] = "BikeComputer-Setup";
#ifndef BIKE_SETUP_PASSWORD
#error "BIKE_SETUP_PASSWORD must be provided by the local build environment"
#endif
constexpr char setup_password[] = BIKE_SETUP_PASSWORD;
constexpr size_t max_relative_path = 384;
constexpr size_t max_entries = 48;

struct Entry { char name[256]; bool directory; int64_t modified; };
constexpr size_t recent_fit_count = 10;
httpd_handle_t server = nullptr;
portMUX_TYPE status_mux = portMUX_INITIALIZER_UNLOCKED;
bool setup_mode = false;
bool connected = false;
bool server_ready = false;
bool initialized = false;
bool start_requested = false;
bool startup_failed = false;
char ip_address[16] = {};
unsigned disconnect_count = 0;
bool recovery_task_running = false;
char saved_ssid[33] = {};
char saved_password[65] = {};

void set_status(bool setup, bool online, const char* ip)
{
    portENTER_CRITICAL(&status_mux);
    setup_mode = setup;
    connected = online;
    std::snprintf(ip_address, sizeof(ip_address), "%s", ip ? ip : "");
    portEXIT_CRITICAL(&status_mux);
}

void start_setup_ap()
{
    wifi_config_t ap{};
    std::snprintf(reinterpret_cast<char*>(ap.ap.ssid), sizeof(ap.ap.ssid), "%s", setup_ssid);
    std::snprintf(reinterpret_cast<char*>(ap.ap.password), sizeof(ap.ap.password), "%s", setup_password);
    ap.ap.ssid_len = std::strlen(setup_ssid);
    ap.ap.channel = 1;
    ap.ap.max_connection = 2;
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    set_status(true, false, "192.168.4.1");
    esp_wifi_disconnect();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_LOGW(tag, "Setup Wi-Fi: %s, open http://192.168.4.1/", setup_ssid);
}

void wifi_recovery_task(void* context)
{
    const bool switch_to_setup = reinterpret_cast<uintptr_t>(context) != 0;
    // Let the driver finish tearing down the disconnected association before
    // asking it to reconnect or changing its mode.
    vTaskDelay(pdMS_TO_TICKS(750));
    if (switch_to_setup) {
        start_setup_ap();
    } else {
        const esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK)
            ESP_LOGW(tag, "Wi-Fi reconnect request failed: %s", esp_err_to_name(err));
    }
    recovery_task_running = false;
    vTaskDelete(nullptr);
}

void wifi_event(void*, esp_event_base_t base, int32_t id, void* event_data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        bool currently_in_setup;
        portENTER_CRITICAL(&status_mux);
        currently_in_setup = setup_mode;
        portEXIT_CRITICAL(&status_mux);
        if (!saved_ssid[0] || currently_in_setup) return;
        const auto* event = static_cast<const wifi_event_sta_disconnected_t*>(event_data);
        set_status(false, false, "");
        ++disconnect_count;
        ESP_LOGW(tag, "Disconnected from saved Wi-Fi (reason %u, attempt %u)",
                 event ? event->reason : 0, disconnect_count);
        if (!recovery_task_running) {
            recovery_task_running = true;
            const bool switch_to_setup = disconnect_count >= 5;
            if (xTaskCreate(wifi_recovery_task, "wifi_recovery", 3072,
                            reinterpret_cast<void*>(switch_to_setup ? 1 : 0), 3, nullptr) != pdPASS) {
                recovery_task_running = false;
                ESP_LOGE(tag, "Could not create Wi-Fi recovery task");
            }
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const auto* event = static_cast<const ip_event_got_ip_t*>(event_data);
        char ip[16];
        std::snprintf(ip, sizeof(ip), IPSTR, IP2STR(&event->ip_info.ip));
        wifi_ps_type_t power_save = WIFI_PS_MIN_MODEM;
        const esp_err_t ps_err = esp_wifi_get_ps(&power_save);
        disconnect_count = 0;
        set_status(false, true, ip);
        ESP_LOGI(tag, "File browser available at http://%s/ (power save %s, mode %d)", ip,
                 ps_err == ESP_OK ? "configured" : esp_err_to_name(ps_err), power_save);
    }
}

bool load_credentials()
{
    nvs_handle_t nvs;
    if (nvs_open("bike_wifi", NVS_READONLY, &nvs) != ESP_OK) return false;
    size_t ssid_size = sizeof(saved_ssid), password_size = sizeof(saved_password);
    const bool ok = nvs_get_str(nvs, "ssid", saved_ssid, &ssid_size) == ESP_OK &&
                    nvs_get_str(nvs, "password", saved_password, &password_size) == ESP_OK &&
                    saved_ssid[0];
    nvs_close(nvs);
    return ok;
}

bool safe_relative(const char* path)
{
    if (!path || path[0] == '/' || std::strlen(path) > max_relative_path) return false;
    if (std::strchr(path, '\\') || std::strchr(path, ':')) return false;
    const char* part = path;
    while (*part) {
        while (*part == '/') ++part;
        const char* end = std::strchr(part, '/');
        const size_t length = end ? size_t(end - part) : std::strlen(part);
        if (length == 2 && part[0] == '.' && part[1] == '.') return false;
        if (!end) break;
        part = end + 1;
    }
    return true;
}

bool is_partial_fit(const char* name)
{
    const size_t length = std::strlen(name);
    constexpr char suffix[] = ".fit.part";
    return length >= sizeof(suffix) - 1 &&
           std::strcmp(name + length - (sizeof(suffix) - 1), suffix) == 0;
}

void html_escape(char* output, size_t capacity, const char* input);
void url_encode(char* output, size_t capacity, const char* input);

bool is_fit_file(const char* name)
{
    const size_t length = std::strlen(name);
    return length >= 4 && name[length - 4] == '.' &&
           (name[length - 3] == 'f' || name[length - 3] == 'F') &&
           (name[length - 2] == 'i' || name[length - 2] == 'I') &&
           (name[length - 1] == 't' || name[length - 1] == 'T');
}

bool is_recent_entry(const Entry* recent, size_t count, const char* name)
{
    for (size_t i = 0; i < count; ++i)
        if (std::strcmp(recent[i].name, name) == 0) return true;
    return false;
}

bool decode_component(const char* encoded, char* output, size_t capacity)
{
    size_t out = 0;
    for (size_t i = 0; encoded[i]; ++i) {
        if (out + 1 >= capacity) return false;
        if (encoded[i] == '+') output[out++] = ' ';
        else if (encoded[i] == '%' && encoded[i + 1] && encoded[i + 2]) {
            char hex[3] = {encoded[i + 1], encoded[i + 2], 0};
            char* end = nullptr;
            const long value = std::strtol(hex, &end, 16);
            if (!end || *end) return false;
            output[out++] = static_cast<char>(value);
            i += 2;
        } else output[out++] = encoded[i];
    }
    output[out] = 0;
    return true;
}

void send_file_link(httpd_req_t* req, const char* relative, const char* name)
{
    char path[max_relative_path + 1];
    const size_t parent_length = std::strlen(relative);
    const size_t name_length = std::strlen(name);
    size_t used = 0;
    if (parent_length) {
        std::memcpy(path, relative, parent_length);
        used = parent_length;
        path[used++] = '/';
    }
    std::memcpy(path + used, name, name_length + 1);
    char encoded[1200], escaped[1280];
    url_encode(encoded, sizeof(encoded), path);
    html_escape(escaped, sizeof(escaped), name);
    httpd_resp_sendstr_chunk(req, "<li><a href='/download?file=");
    httpd_resp_sendstr_chunk(req, encoded);
    httpd_resp_sendstr_chunk(req, "' download>");
    httpd_resp_sendstr_chunk(req, escaped);
    httpd_resp_sendstr_chunk(req, "</a></li>");
}

void send_page_link(httpd_req_t* req, const char* relative, size_t offset, const char* label)
{
    char encoded[1200], offset_text[16];
    url_encode(encoded, sizeof(encoded), relative);
    std::snprintf(offset_text, sizeof(offset_text), "%u", static_cast<unsigned>(offset));
    httpd_resp_sendstr_chunk(req, "<a href='/?dir=");
    httpd_resp_sendstr_chunk(req, encoded);
    httpd_resp_sendstr_chunk(req, "&amp;offset=");
    httpd_resp_sendstr_chunk(req, offset_text);
    httpd_resp_sendstr_chunk(req, "'>");
    httpd_resp_sendstr_chunk(req, label);
    httpd_resp_sendstr_chunk(req, "</a>");
}

void close_file(FILE* file)
{
    if (!file) return;
    if (sd_access::lock(portMAX_DELAY)) {
        std::fclose(file);
        sd_access::unlock();
    }
}

bool get_query_value(httpd_req_t* req, const char* key, char* output, size_t capacity)
{
    char query[1280];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return false;
    char encoded[1200];
    if (httpd_query_key_value(query, key, encoded, sizeof(encoded)) != ESP_OK) return false;
    size_t out = 0;
    for (size_t i = 0; encoded[i] && out + 1 < capacity; ++i) {
        if (encoded[i] == '+') output[out++] = ' ';
        else if (encoded[i] == '%' && encoded[i + 1] && encoded[i + 2]) {
            char hex[3] = {encoded[i + 1], encoded[i + 2], 0};
            char* end = nullptr;
            const long value = std::strtol(hex, &end, 16);
            if (!end || *end) return false;
            output[out++] = static_cast<char>(value);
            i += 2;
        } else output[out++] = encoded[i];
    }
    output[out] = 0;
    return !encoded[0] || out + 1 < capacity;
}

void html_escape(char* output, size_t capacity, const char* input)
{
    size_t used = 0;
    for (; *input && used + 7 < capacity; ++input) {
        const char* replacement = nullptr;
        switch (*input) {
            case '&': replacement = "&amp;"; break;
            case '<': replacement = "&lt;"; break;
            case '>': replacement = "&gt;"; break;
            case '"': replacement = "&quot;"; break;
            case '\'': replacement = "&#39;"; break;
            default: output[used++] = *input; continue;
        }
        const size_t n = std::strlen(replacement);
        std::memcpy(output + used, replacement, n);
        used += n;
    }
    output[used] = 0;
}

void url_encode(char* output, size_t capacity, const char* input)
{
    static constexpr char hex[] = "0123456789ABCDEF";
    size_t used = 0;
    for (; *input && used + 4 < capacity; ++input) {
        const unsigned char c = static_cast<unsigned char>(*input);
        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '/') {
            output[used++] = static_cast<char>(c);
        } else {
            output[used++] = '%';
            output[used++] = hex[c >> 4];
            output[used++] = hex[c & 0x0f];
        }
    }
    output[used] = 0;
}

esp_err_t send_setup_page(httpd_req_t* req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    const char* page =
        "<!doctype html><meta name='viewport' content='width=device-width'>"
        "<title>Bike computer setup</title><h1>Connect to Wi-Fi</h1>"
        "<form method='post' action='/wifi'>"
        "<label>Network name (SSID)<br><input name='ssid' maxlength='32' required></label><p>"
        "<label>Password<br><input name='password' type='password' maxlength='63'></label><p>"
        "<button>Save and connect</button></form>";
    return httpd_resp_sendstr(req, page);
}

esp_err_t root_handler(httpd_req_t* req)
{
    bool setup;
    portENTER_CRITICAL(&status_mux); setup = setup_mode; portEXIT_CRITICAL(&status_mux);
    if (setup) return send_setup_page(req);

    char relative[max_relative_path + 1] = {};
    char query[1280];
    size_t offset = 0;
    const esp_err_t query_result = httpd_req_get_url_query_str(req, query, sizeof(query));
    if (query_result == ESP_OK) {
        char encoded_dir[1200];
        const esp_err_t dir_result = httpd_query_key_value(query, "dir", encoded_dir, sizeof(encoded_dir));
        if (dir_result == ESP_OK && !decode_component(encoded_dir, relative, sizeof(relative)))
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid directory");
        if (dir_result != ESP_OK && dir_result != ESP_ERR_NOT_FOUND)
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid directory");

        char offset_text[16];
        const esp_err_t offset_result = httpd_query_key_value(query, "offset", offset_text, sizeof(offset_text));
        if (offset_result == ESP_OK) {
            char* end = nullptr;
            const unsigned long parsed = std::strtoul(offset_text, &end, 10);
            if (!offset_text[0] || !end || *end || parsed > 1000000UL)
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid page offset");
            offset = parsed;
        } else if (offset_result != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid page offset");
        }
    } else if (query_result != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid query");
    }
    if (!safe_relative(relative))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid directory");
    char directory[448];
    if (relative[0]) std::snprintf(directory, sizeof(directory), "/sdcard/%s", relative);
    else std::snprintf(directory, sizeof(directory), "/sdcard");

    Entry* entries = static_cast<Entry*>(std::calloc(max_entries, sizeof(Entry)));
    Entry* recent = relative[0] ? nullptr : static_cast<Entry*>(std::calloc(recent_fit_count, sizeof(Entry)));
    if (!entries || (!relative[0] && !recent)) {
        std::free(entries);
        std::free(recent);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
    }
    size_t recent_count = 0;
    size_t count = 0;
    size_t total_count = 0;
    bool has_more = false;
    bool opened = false;
    if (sd_access::lock(pdMS_TO_TICKS(1500))) {
        DIR* dir = opendir(directory);
        if (dir) {
            opened = true;
            if (!relative[0]) {
                while (dirent* item = readdir(dir)) {
                    if (!is_fit_file(item->d_name) || is_partial_fit(item->d_name)) continue;
                    char child[448];
                    std::snprintf(child, sizeof(child), "/sdcard/%s", item->d_name);
                    struct stat st{};
                    if (stat(child, &st) != 0 || !S_ISREG(st.st_mode)) continue;
                    size_t position = 0;
                    while (position < recent_count && recent[position].modified >= st.st_mtime) ++position;
                    if (position >= recent_fit_count) continue;
                    if (recent_count < recent_fit_count) ++recent_count;
                    for (size_t i = recent_count - 1; i > position; --i) recent[i] = recent[i - 1];
                    std::snprintf(recent[position].name, sizeof(recent[position].name), "%s", item->d_name);
                    recent[position].directory = false;
                    recent[position].modified = st.st_mtime;
                }

                rewinddir(dir);
            }
            while (dirent* item = readdir(dir)) {
                if (!std::strcmp(item->d_name, ".") || !std::strcmp(item->d_name, "..") ||
                    is_partial_fit(item->d_name) || is_recent_entry(recent, recent_count, item->d_name)) continue;
                if (total_count >= offset && count < max_entries) {
                    Entry& entry = entries[count];
                    std::snprintf(entry.name, sizeof(entry.name), "%s", item->d_name);
                    char child[448];
                    struct stat st{};
                    const size_t directory_length = std::strlen(directory);
                    const size_t item_length = std::strlen(item->d_name);
                    if (directory_length + 1 + item_length < sizeof(child)) {
                        std::memcpy(child, directory, directory_length);
                        child[directory_length] = '/';
                        std::memcpy(child + directory_length + 1, item->d_name, item_length + 1);
                        entry.directory = stat(child, &st) == 0 && S_ISDIR(st.st_mode);
                    } else {
                        entry.directory = false;
                    }
                    entry.modified = st.st_mtime;
                    ++count;
                } else if (total_count >= offset + max_entries) {
                    has_more = true;
                }
                ++total_count;
            }
            closedir(dir);
        }
        sd_access::unlock();
    }
    if (!opened) {
        std::free(entries);
        std::free(recent);
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Directory unavailable");
    }

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr_chunk(req, "<!doctype html><meta name='viewport' content='width=device-width'>"
        "<title>Bike files</title><style>body{font:18px sans-serif;max-width:720px;margin:2em auto;padding:0 1em}"
        "li{padding:.5em}a{color:#0759a5}</style><h1>SD card</h1>");
    if (recent_count) {
        httpd_resp_sendstr_chunk(req, "<h2>Recent rides</h2><ul>");
        for (size_t i = 0; i < recent_count; ++i)
            send_file_link(req, "", recent[i].name);
        httpd_resp_sendstr_chunk(req, "</ul>");
    }
    httpd_resp_sendstr_chunk(req, "<h2>Files</h2><ul>");
    if (relative[0]) {
        char parent[max_relative_path + 1];
        std::snprintf(parent, sizeof(parent), "%s", relative);
        char* slash = std::strrchr(parent, '/');
        if (slash) *slash = 0; else parent[0] = 0;
        char encoded[1200]; url_encode(encoded, sizeof(encoded), parent);
        httpd_resp_sendstr_chunk(req, "<li><a href='/?dir=");
        httpd_resp_sendstr_chunk(req, encoded);
        httpd_resp_sendstr_chunk(req, "'>[..]</a></li>");
    }
    for (size_t i = 0; i < count; ++i) {
        const size_t parent_length = std::strlen(relative);
        const size_t name_length = std::strlen(entries[i].name);
        if (parent_length + (relative[0] ? 1 : 0) + name_length > max_relative_path) {
            char escaped_name[1280];
            html_escape(escaped_name, sizeof(escaped_name), entries[i].name);
            httpd_resp_sendstr_chunk(req, "<li>");
            httpd_resp_sendstr_chunk(req, escaped_name);
            httpd_resp_sendstr_chunk(req, " (path too long)</li>");
            continue;
        }
        if (entries[i].directory) {
            httpd_resp_sendstr_chunk(req, "<li>[DIR] <a href='/?dir=");
            char child_rel[max_relative_path + 1], encoded[1200], escaped[1280];
            size_t used = 0;
            if (parent_length) {
                std::memcpy(child_rel, relative, parent_length);
                used = parent_length;
                child_rel[used++] = '/';
            }
            std::memcpy(child_rel + used, entries[i].name, std::strlen(entries[i].name) + 1);
            url_encode(encoded, sizeof(encoded), child_rel);
            html_escape(escaped, sizeof(escaped), entries[i].name);
            httpd_resp_sendstr_chunk(req, encoded);
            httpd_resp_sendstr_chunk(req, "'>");
            httpd_resp_sendstr_chunk(req, escaped);
            httpd_resp_sendstr_chunk(req, "/</a></li>");
        } else {
            send_file_link(req, relative, entries[i].name);
        }
    }
    std::free(entries);
    std::free(recent);
    httpd_resp_sendstr_chunk(req, "</ul>");
    if (total_count) {
        char range[80];
        std::snprintf(range, sizeof(range), "<p>Showing %u-%u of %u entries.</p>",
                      static_cast<unsigned>(offset + 1), static_cast<unsigned>(offset + count),
                      static_cast<unsigned>(total_count));
        httpd_resp_sendstr_chunk(req, range);
    } else {
        httpd_resp_sendstr_chunk(req, recent_count ? "<p>No other files in this folder.</p>" :
                                                     "<p>No files in this folder.</p>");
    }
    httpd_resp_sendstr_chunk(req, "<nav>");
    if (offset) {
        httpd_resp_sendstr_chunk(req, " ");
        send_page_link(req, relative, offset > max_entries ? offset - max_entries : 0, "&laquo; Previous");
    }
    if (has_more) {
        httpd_resp_sendstr_chunk(req, " ");
        send_page_link(req, relative, offset + max_entries, "Next &raquo;");
    }
    httpd_resp_sendstr_chunk(req, "</nav>");
    return httpd_resp_sendstr_chunk(req, nullptr);
}

esp_err_t download_handler(httpd_req_t* req)
{
    char relative[max_relative_path + 1];
    if (!get_query_value(req, "file", relative, sizeof(relative)) || !relative[0] || !safe_relative(relative))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid file path");
    char path[448];
    std::snprintf(path, sizeof(path), "/sdcard/%s", relative);
    if (!sd_access::lock(pdMS_TO_TICKS(1500)))
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "SD card busy");
    FILE* file = std::fopen(path, "rb");
    struct stat st{};
    const bool regular = file && stat(path, &st) == 0 && S_ISREG(st.st_mode);
    sd_access::unlock();
    if (!regular) {
        close_file(file);
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
    }
    if (is_partial_fit(relative)) {
        close_file(file);
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
    }
    const char* filename = std::strrchr(relative, '/');
    filename = filename ? filename + 1 : relative;
    char safe_name[128]; size_t n = 0;
    for (; filename[n] && n + 1 < sizeof(safe_name); ++n) {
        const unsigned char c = filename[n];
        safe_name[n] = ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                        (c >= '0' && c <= '9') || c == '.' || c == '-' || c == '_') ? c : '_';
    }
    safe_name[n] = 0;
    char disposition[180];
    std::snprintf(disposition, sizeof(disposition), "attachment; filename=\"%s\"", safe_name);
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", disposition);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    char buffer[1024];
    while (true) {
        if (!sd_access::lock(pdMS_TO_TICKS(1500))) { close_file(file); return ESP_FAIL; }
        const size_t bytes = std::fread(buffer, 1, sizeof(buffer), file);
        const bool read_error = std::ferror(file);
        sd_access::unlock();
        if (read_error) { close_file(file); return ESP_FAIL; }
        if (!bytes) break;
        if (httpd_resp_send_chunk(req, buffer, bytes) != ESP_OK) { close_file(file); return ESP_FAIL; }
    }
    close_file(file);
    return httpd_resp_send_chunk(req, nullptr, 0);
}

void restart_task(void*)
{
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();
}

bool decode_form_value(const char* body, const char* key, char* output, size_t capacity)
{
    char query[300]; std::snprintf(query, sizeof(query), "%s", body);
    char encoded[256];
    if (httpd_query_key_value(query, key, encoded, sizeof(encoded)) != ESP_OK) return false;
    size_t out = 0;
    for (size_t i = 0; encoded[i] && out + 1 < capacity; ++i) {
        if (encoded[i] == '+') output[out++] = ' ';
        else if (encoded[i] == '%' && encoded[i + 1] && encoded[i + 2]) {
            char hex[3] = {encoded[i + 1], encoded[i + 2], 0};
            output[out++] = static_cast<char>(std::strtol(hex, nullptr, 16)); i += 2;
        } else output[out++] = encoded[i];
    }
    output[out] = 0;
    return true;
}

esp_err_t save_wifi_handler(httpd_req_t* req)
{
    if (req->content_len <= 0 || req->content_len >= 300)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid form size");
    char body[300];
    int received = 0;
    while (received < req->content_len) {
        const int n = httpd_req_recv(req, body + received, req->content_len - received);
        if (n <= 0) return ESP_FAIL;
        received += n;
    }
    body[received] = 0;
    char ssid[33] = {}, password[65] = {};
    if (!decode_form_value(body, "ssid", ssid, sizeof(ssid)) || !ssid[0] ||
        !decode_form_value(body, "password", password, sizeof(password)) ||
        (password[0] && std::strlen(password) < 8))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID or password is invalid");

    nvs_handle_t nvs;
    esp_err_t err = nvs_open("bike_wifi", NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_str(nvs, "ssid", ssid);
        if (err == ESP_OK) err = nvs_set_str(nvs, "password", password);
        if (err == ESP_OK) err = nvs_commit(nvs);
        nvs_close(nvs);
    }
    if (err != ESP_OK) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Could not save Wi-Fi settings");
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_sendstr(req, "<meta name='viewport' content='width=device-width'><h1>Saved</h1><p>Bike computer restarting and joining your Wi-Fi. Check the display for its address.</p>");
    xTaskCreate(restart_task, "wifi_restart", 2048, nullptr, 3, nullptr);
    return ESP_OK;
}

bool start_server()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    // The SD listing builds escaped names and encoded paths on this task's
    // stack. Keep room for those temporaries even on deeply nested directories.
    config.stack_size = 12288;
    config.max_uri_handlers = 3;
    config.max_open_sockets = 3;
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(tag, "httpd_start failed: %s (free heap %u, largest block %u)",
                 esp_err_to_name(err), static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_8BIT)),
                 static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
        return false;
    }
    const httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_handler, .user_ctx = nullptr};
    const httpd_uri_t download = {.uri = "/download", .method = HTTP_GET, .handler = download_handler, .user_ctx = nullptr};
    const httpd_uri_t save = {.uri = "/wifi", .method = HTTP_POST, .handler = save_wifi_handler, .user_ctx = nullptr};
    err = httpd_register_uri_handler(server, &root);
    if (err == ESP_OK) err = httpd_register_uri_handler(server, &download);
    if (err == ESP_OK) err = httpd_register_uri_handler(server, &save);
    if (err != ESP_OK) {
        ESP_LOGE(tag, "HTTP route registration failed: %s", esp_err_to_name(err));
        httpd_stop(server);
        server = nullptr;
        return false;
    }
    return true;
}
} // namespace

bool initialize_network()
{
    const bool has_credentials = load_credentials();
    const esp_err_t netif_err = esp_netif_init();
    if (netif_err != ESP_OK && netif_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(tag, "esp_netif_init failed: %s", esp_err_to_name(netif_err));
        return false;
    }
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(tag, "event loop creation failed: %s", esp_err_to_name(err));
        return false;
    }
    if (!esp_netif_create_default_wifi_sta() || !esp_netif_create_default_wifi_ap()) {
        ESP_LOGE(tag, "Could not create Wi-Fi network interfaces (free heap %u)",
                 static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_8BIT)));
        return false;
    }
    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&wifi_config);
    if (err != ESP_OK) {
        ESP_LOGE(tag, "esp_wifi_init failed: %s (free heap %u, largest block %u)",
                 esp_err_to_name(err), static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_8BIT)),
                 static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
        return false;
    }
    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) {
        ESP_LOGE(tag, "Wi-Fi storage setup failed: %s", esp_err_to_name(err));
        return false;
    }
    // This ESP32/IDF combination panics in the station power-save null-frame
    // path shortly after association. Wi-Fi is only enabled on demand here,
    // so keep the station awake for a stable file transfer session.
    err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (err != ESP_OK) {
        ESP_LOGE(tag, "Could not disable Wi-Fi modem sleep: %s", esp_err_to_name(err));
        return false;
    }
    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event, nullptr);
    if (err == ESP_OK) err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event, nullptr);
    if (err != ESP_OK) {
        ESP_LOGE(tag, "Wi-Fi event handler registration failed: %s", esp_err_to_name(err));
        return false;
    }
    if (!has_credentials) {
        start_setup_ap();
    } else {
        wifi_config_t station{};
        std::memcpy(station.sta.ssid, saved_ssid, std::strlen(saved_ssid));
        std::memcpy(station.sta.password, saved_password, std::strlen(saved_password));
        err = esp_wifi_set_mode(WIFI_MODE_STA);
        if (err == ESP_OK) err = esp_wifi_set_config(WIFI_IF_STA, &station);
        if (err != ESP_OK) {
            ESP_LOGE(tag, "Saved Wi-Fi configuration failed: %s", esp_err_to_name(err));
            return false;
        }
        set_status(false, false, "");
    }
    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(tag, "esp_wifi_start failed: %s (free heap %u, largest block %u)",
                 esp_err_to_name(err), static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_8BIT)),
                 static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
        return false;
    }
    // Set it again after start: on this ESP-IDF build the pre-start setting
    // did not prevent the modem-sleep/null-frame crash after association.
    err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (err != ESP_OK) {
        ESP_LOGE(tag, "Could not disable Wi-Fi modem sleep after start: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(tag, "Wi-Fi modem sleep disabled");
    if (!start_server()) return false;
    portENTER_CRITICAL(&status_mux);
    server_ready = true;
    portEXIT_CRITICAL(&status_mux);
    return true;
}

void network_start_task(void*)
{
    if (!initialize_network()) {
        portENTER_CRITICAL(&status_mux);
        startup_failed = true;
        portEXIT_CRITICAL(&status_mux);
        ESP_LOGE(tag, "Could not start Wi-Fi file browser; free heap %u, largest block %u",
                 static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_8BIT)),
                 static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
    }
    vTaskDelete(nullptr);
}

bool init()
{
    if (!sd_access::init()) {
        portENTER_CRITICAL(&status_mux); startup_failed = true; portEXIT_CRITICAL(&status_mux);
        return false;
    }
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_INVALID_STATE) err = ESP_OK; // Bluetooth may have initialized NVS first.
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if (nvs_flash_erase() != ESP_OK) {
            portENTER_CRITICAL(&status_mux); startup_failed = true; portEXIT_CRITICAL(&status_mux);
            return false;
        }
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        portENTER_CRITICAL(&status_mux); startup_failed = true; portEXIT_CRITICAL(&status_mux);
        return false;
    }
    initialized = true;
    return true;
}

bool start()
{
    if (!initialized) return false;
    portENTER_CRITICAL(&status_mux);
    if (start_requested || startup_failed) {
        portEXIT_CRITICAL(&status_mux);
        return false;
    }
    start_requested = true;
    portEXIT_CRITICAL(&status_mux);
    if (xTaskCreate(network_start_task, "wifi_file_start", 6144, nullptr, 4, nullptr) != pdPASS) {
        portENTER_CRITICAL(&status_mux);
        start_requested = false;
        startup_failed = true;
        portEXIT_CRITICAL(&status_mux);
        return false;
    }
    return true;
}

void display_status(char* buffer, size_t size)
{
    if (!buffer || !size) return;
    bool setup, online; char ip[sizeof(ip_address)];
    portENTER_CRITICAL(&status_mux);
    setup = setup_mode; online = connected;
    const bool serving = server_ready;
    const bool started = start_requested;
    const bool failed = startup_failed;
    std::snprintf(ip, sizeof(ip), "%s", ip_address);
    portEXIT_CRITICAL(&status_mux);
    if (failed)
        std::snprintf(buffer, size, "Wi-Fi could not start; reboot to retry");
    else if (!started)
        std::snprintf(buffer, size, "Wi-Fi is off. Tap Start Wi-Fi below.");
    else if (!serving)
        std::snprintf(buffer, size, "Starting Wi-Fi file browser...");
    else if (setup)
        std::snprintf(buffer, size, "Wi-Fi: %s\nPass: %s | http://192.168.4.1", setup_ssid, setup_password);
    else if (online)
        std::snprintf(buffer, size, "Files: http://%s/", ip);
    else
        std::snprintf(buffer, size, "Joining saved Wi-Fi...");
}
} // namespace wifi_files
