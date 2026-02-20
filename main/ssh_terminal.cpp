/*
 * SSHTerminal Implementation
 * Core SSH terminal functionality including WiFi connectivity, libssh2 session management,
 * command execution, terminal display rendering with LVGL, and command history management.
 */

#include "ssh_terminal.hpp"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_crc.h"
#include "esp_ota_ops.h"
#include "esp_image_format.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#if defined(TPAGER_TARGET)
#include "esp_lvgl_port.h"
#include "tpager_sd.hpp"
#else
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#endif
#include <cstring>
#include <algorithm>
#include <cinttypes>
#include <cctype>
#include <cstdlib>
#include <cstdio>
#include <limits>
#include <set>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/select.h>
#include <cerrno>

static const char *TAG = "SSH_TERMINAL";

#if defined(TPAGER_TARGET)
extern "C" void tpager_request_shutdown(void);
#endif

namespace {
// Font fallback contract: keep UI readable even when smaller Montserrat faces
// are disabled in sdkconfig/LVGL.
bool display_lock(uint32_t timeout_ms)
{
#if defined(TPAGER_TARGET)
    return lvgl_port_lock(timeout_ms);
#else
    return bsp_display_lock(timeout_ms);
#endif
}

void display_unlock()
{
#if defined(TPAGER_TARGET)
    lvgl_port_unlock();
#else
    bsp_display_unlock();
#endif
}

const lv_font_t* ui_font_small()
{
#if defined(LV_FONT_MONTSERRAT_10) && LV_FONT_MONTSERRAT_10
    return &lv_font_montserrat_10;
#elif defined(LV_FONT_MONTSERRAT_12) && LV_FONT_MONTSERRAT_12
    return &lv_font_montserrat_12;
#elif defined(LV_FONT_MONTSERRAT_14) && LV_FONT_MONTSERRAT_14
    return &lv_font_montserrat_14;
#else
    return LV_FONT_DEFAULT;
#endif
}

const lv_font_t* ui_font_body()
{
#if defined(LV_FONT_MONTSERRAT_12) && LV_FONT_MONTSERRAT_12
    return &lv_font_montserrat_12;
#elif defined(LV_FONT_MONTSERRAT_14) && LV_FONT_MONTSERRAT_14
    return &lv_font_montserrat_14;
#else
    return LV_FONT_DEFAULT;
#endif
}

const lv_font_t* ui_font_terminal_big()
{
#if defined(LV_FONT_MONTSERRAT_14) && LV_FONT_MONTSERRAT_14
    return &lv_font_montserrat_14;
#elif defined(LV_FONT_MONTSERRAT_12) && LV_FONT_MONTSERRAT_12
    return &lv_font_montserrat_12;
#else
    return ui_font_body();
#endif
}

// Scrollback contract: retain at least ~3 full terminal screens on-device even
// during bursty output, while still bounding LVGL text area memory growth.
constexpr size_t kTerminalScrollbackBytes = 12288;
constexpr size_t kTerminalAppendChunkBytes = 1024;
constexpr size_t kTerminalIngressMaxBytes = 16384;
constexpr size_t kTerminalIngressKeepBytes = 12288;
constexpr int64_t kTerminalFlushIntervalMs = 250;

void log_heap_snapshot(const char *stage)
{
    const uint32_t free8 = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    const uint32_t largest8 = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    const uint32_t free32 = heap_caps_get_free_size(MALLOC_CAP_32BIT);
    const uint32_t largest32 = heap_caps_get_largest_free_block(MALLOC_CAP_32BIT);
    const uint32_t free_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    const uint32_t largest_spiram = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "heap[%s] free8=%" PRIu32 " largest8=%" PRIu32
                  " free32=%" PRIu32 " largest32=%" PRIu32
                  " free_psram=%" PRIu32 " largest_psram=%" PRIu32,
             stage, free8, largest8, free32, largest32, free_spiram, largest_spiram);
}

int free_percent_for_caps(uint32_t caps)
{
    const uint32_t total = heap_caps_get_total_size(caps);
    if (total == 0) {
        return -1;
    }
    const uint32_t free_bytes = heap_caps_get_free_size(caps);
    return static_cast<int>((static_cast<uint64_t>(free_bytes) * 100ULL) / total);
}

int app_flash_headroom_percent()
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == nullptr || running->size == 0) {
        return -1;
    }

    esp_partition_pos_t part = {};
    part.offset = running->address;
    part.size = running->size;

    esp_image_metadata_t metadata = {};
    if (esp_image_get_metadata(&part, &metadata) != ESP_OK || metadata.image_len == 0 ||
        metadata.image_len > running->size) {
        return -1;
    }

    const uint32_t free_bytes = running->size - metadata.image_len;
    return static_cast<int>((static_cast<uint64_t>(free_bytes) * 100ULL) / running->size);
}
}  // namespace

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static int s_retry_num = 0;
#define WIFI_MAXIMUM_RETRY 5

static esp_event_handler_instance_t s_instance_any_id = NULL;
static esp_event_handler_instance_t s_instance_got_ip = NULL;

namespace {
#if defined(TPAGER_TARGET)
class ScopedSDMount {
public:
    ScopedSDMount()
    {
        const bool was_mounted = tpager::sd_is_mounted();
        tpager::SdDiagStats stats = {};
        const esp_err_t ret = tpager::sd_mount_and_scan_keys(&stats);
        last_err_ = ret;
        if (ret != ESP_OK) {
            // Launcher/runtime variant: SD may already be mounted by another app
            // at /sd or /sdcard. Accept that as usable without taking ownership.
            struct stat st_sdcard = {};
            struct stat st_sd = {};
            const bool has_sdcard = (stat("/sdcard", &st_sdcard) == 0) && S_ISDIR(st_sdcard.st_mode);
            const bool has_sd = (stat("/sd", &st_sd) == 0) && S_ISDIR(st_sd.st_mode);
            if (has_sdcard || has_sd) {
                ESP_LOGW(TAG, "SD mount call failed (%s), using existing mountpoint(s): /sdcard=%d /sd=%d",
                         esp_err_to_name(ret), has_sdcard ? 1 : 0, has_sd ? 1 : 0);
                ok_ = true;
                mounted_ = false;
                return;
            }

            ESP_LOGW(TAG, "Failed to mount SD for runtime file access: %s", esp_err_to_name(ret));
            ok_ = false;
            return;
        }
        mounted_ = !was_mounted;
    }

    ~ScopedSDMount()
    {
        // Runtime contract: keep SD mounted once acquired to avoid launcher/app
        // transition races and repeated SDSPI remount timeouts.
    }

    bool ok() const { return ok_; }
    esp_err_t last_err() const { return last_err_; }

private:
    bool ok_ = true;
    bool mounted_ = false;
    esp_err_t last_err_ = ESP_OK;
};
#endif

constexpr const char *kSshConfigPath = "/sdcard/ssh_keys/ssh_config";
constexpr const char *kSshConfigPathRoot = "/sdcard/ssh_config";
constexpr const char *kSshConfigPathAlt = "/sd/ssh_keys/ssh_config";
constexpr const char *kSshConfigPathAltRoot = "/sd/ssh_config";
constexpr const char *kSshKeysRoot = "/sdcard/ssh_keys/";
constexpr const char *kSshKeysRootAlt = "/sd/ssh_keys/";
constexpr const char *kSshKeysDir = "/sdcard/ssh_keys";
constexpr const char *kSshKeysDirAlt = "/sd/ssh_keys";
constexpr const char *kWifiConfigPath = "/sdcard/ssh_keys/wifi_config";
constexpr const char *kWifiConfigPathRoot = "/sdcard/wifi_config";
constexpr const char *kWifiConfigPathAlt = "/sd/ssh_keys/wifi_config";
constexpr const char *kWifiConfigPathAltRoot = "/sd/wifi_config";
constexpr const char *kDefaultSerialRxFilename = "PocketSSH-TPager.bin";

struct SSHConfigOptions {
    bool has_host_name = false;
    std::string host_name;

    bool has_user = false;
    std::string user;

    bool has_port = false;
    int port = 22;

    bool has_identities_only = false;
    bool identities_only = false;
    std::vector<std::string> identity_files;

    bool has_connect_timeout = false;
    int connect_timeout = 0;

    bool has_server_alive_interval = false;
    int server_alive_interval = 0;

    bool has_server_alive_count_max = false;
    int server_alive_count_max = 0;

    bool has_strict_host_key_checking = false;
    std::string strict_host_key_checking;

    bool has_network = false;
    std::string network;

    bool has_fontsize = false;
    bool fontsize_big = false;
};

struct SSHConfigHostBlock {
    std::vector<std::string> patterns;
    SSHConfigOptions options;
};

struct SSHConfigFile {
    SSHConfigOptions global_options;
    std::vector<SSHConfigHostBlock> host_blocks;
    std::vector<std::string> aliases;
};

struct ResolvedSSHConfig {
    bool matched = false;
    std::string alias;
    std::string host_name;
    std::string user;
    int port = 22;
    bool identities_only = false;
    std::vector<std::string> identity_files;
    std::string strict_host_key_checking = "ask";
    std::string network;
};

struct WifiProfile {
    std::string network_name;
    std::string ssid;
    std::string password;
    bool auto_connect = false;
    bool has_auto_connect = false;
    int file_order = 0;
};

std::string resolve_ssh_config_path();
std::string resolve_wifi_config_path();
bool parse_ssh_config_file(SSHConfigFile *parsed);
bool parse_wifi_config_file(std::vector<WifiProfile> *profiles);
bool serial_receive_to_sd_file(SSHTerminal *terminal, const std::string &target_name);

std::string abbreviate_status_value(const std::string &value, size_t max_len)
{
    if (max_len == 0) {
        return "";
    }
    if (value.size() <= max_len) {
        return value;
    }
    if (max_len <= 3) {
        return value.substr(0, max_len);
    }
    return value.substr(0, max_len - 3) + "...";
}

bool resolve_host_ipv4(const char *host, int port, struct sockaddr_in *out_addr)
{
    if (host == nullptr || out_addr == nullptr || port <= 0 || port > 65535) {
        return false;
    }

    std::memset(out_addr, 0, sizeof(*out_addr));
    out_addr->sin_family = AF_INET;
    out_addr->sin_port = htons(port);

    // Fast path: literal IPv4 string.
    if (inet_pton(AF_INET, host, &out_addr->sin_addr) == 1) {
        return true;
    }

    struct addrinfo hints = {};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    struct addrinfo *results = nullptr;
    int rc = getaddrinfo(host, nullptr, &hints, &results);
    if (rc != 0 || results == nullptr) {
        ESP_LOGE(TAG, "DNS lookup failed for %s: %d", host, rc);
        return false;
    }

    bool ok = false;
    for (struct addrinfo *it = results; it != nullptr; it = it->ai_next) {
        if (it->ai_family != AF_INET || it->ai_addrlen < static_cast<int>(sizeof(struct sockaddr_in))) {
            continue;
        }
        auto *sin = reinterpret_cast<struct sockaddr_in *>(it->ai_addr);
        out_addr->sin_addr = sin->sin_addr;
        ok = true;
        break;
    }
    freeaddrinfo(results);
    return ok;
}

std::vector<std::string> split_nonempty_whitespace(const std::string &input)
{
    std::vector<std::string> parts;
    std::string token;
    for (char c : input) {
        if (c == ' ' || c == '\t') {
            if (!token.empty()) {
                parts.push_back(token);
                token.clear();
            }
            continue;
        }
        token.push_back(c);
    }
    if (!token.empty()) {
        parts.push_back(token);
    }
    return parts;
}

std::vector<std::string> split_quoted_arguments(const std::string &input, size_t start_pos)
{
    std::vector<std::string> args;
    std::string token;
    bool in_quotes = false;
    char quote_char = '\0';

    for (size_t i = start_pos; i < input.size(); ++i) {
        const char c = input[i];

        if ((c == '"' || c == '\'') && (!in_quotes || c == quote_char)) {
            if (in_quotes) {
                in_quotes = false;
                quote_char = '\0';
            } else {
                in_quotes = true;
                quote_char = c;
            }
            continue;
        }

        if (!in_quotes && (c == ' ' || c == '\t')) {
            if (!token.empty()) {
                args.push_back(token);
                token.clear();
            }
            continue;
        }

        token.push_back(c);
    }

    if (!token.empty()) {
        args.push_back(token);
    }

    return args;
}

std::string trim_ascii(const std::string &value)
{
    size_t start = 0;
    while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start])) != 0) {
        ++start;
    }

    size_t end = value.size();
    while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1])) != 0) {
        --end;
    }

    return value.substr(start, end - start);
}

std::string strip_inline_comment(const std::string &line)
{
    bool in_quotes = false;
    char quote_char = '\0';

    for (size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if ((c == '"' || c == '\'') && (!in_quotes || c == quote_char)) {
            if (in_quotes) {
                in_quotes = false;
                quote_char = '\0';
            } else {
                in_quotes = true;
                quote_char = c;
            }
            continue;
        }
        if (!in_quotes && c == '#') {
            return line.substr(0, i);
        }
    }
    return line;
}

std::string lowercase_ascii(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

std::string trim_matching_quotes(const std::string &value)
{
    if (value.size() >= 2) {
        const char first = value.front();
        const char last = value.back();
        if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
            return value.substr(1, value.size() - 2);
        }
    }
    return value;
}

bool starts_with_ascii_ci(const std::string &value, const std::string &prefix)
{
    if (value.size() < prefix.size()) {
        return false;
    }
    return lowercase_ascii(value.substr(0, prefix.size())) == lowercase_ascii(prefix);
}

bool split_directive(const std::string &line, std::string *key, std::string *value)
{
    if (key == nullptr || value == nullptr) {
        return false;
    }

    const size_t eq = line.find('=');
    if (eq != std::string::npos) {
        *key = trim_ascii(line.substr(0, eq));
        *value = trim_ascii(line.substr(eq + 1));
        return !key->empty() && !value->empty();
    }

    const size_t ws = line.find_first_of(" \t");
    if (ws == std::string::npos) {
        return false;
    }

    *key = trim_ascii(line.substr(0, ws));
    *value = trim_ascii(line.substr(ws + 1));
    return !key->empty() && !value->empty();
}

bool parse_int32(const std::string &value, int *out)
{
    if (out == nullptr || value.empty()) {
        return false;
    }

    char *end = nullptr;
    const long parsed = std::strtol(value.c_str(), &end, 10);
    if (end == value.c_str() || *end != '\0') {
        return false;
    }
    if (parsed < std::numeric_limits<int>::min() || parsed > std::numeric_limits<int>::max()) {
        return false;
    }

    *out = static_cast<int>(parsed);
    return true;
}

bool parse_bool_flag(const std::string &value, bool *out)
{
    if (out == nullptr) {
        return false;
    }

    const std::string lowered = lowercase_ascii(trim_ascii(value));
    if (lowered == "yes" || lowered == "true" || lowered == "on" || lowered == "1") {
        *out = true;
        return true;
    }
    if (lowered == "no" || lowered == "false" || lowered == "off" || lowered == "0") {
        *out = false;
        return true;
    }
    return false;
}

bool parse_fontsize_token(const std::string &value, bool *out_big)
{
    if (out_big == nullptr) {
        return false;
    }

    const std::string lowered = lowercase_ascii(trim_ascii(value));
    if (lowered == "big" || lowered == "large") {
        *out_big = true;
        return true;
    }
    if (lowered == "normal" || lowered == "small") {
        *out_big = false;
        return true;
    }
    return false;
}

std::string expand_identity_file_path(std::string path)
{
    path = trim_matching_quotes(trim_ascii(path));
    if (path.empty()) {
        return path;
    }

    if (path.rfind("~/.ssh/", 0) == 0) {
        return std::string(kSshKeysRoot) + path.substr(7);
    }

    if (path.rfind("/sd/ssh_keys/", 0) == 0) {
        return path;
    }

    if (path.rfind("/ssh_keys/", 0) == 0) {
        return std::string(kSshKeysRoot) + path.substr(10);
    }

    if (path.rfind("/sdcard/ssh_keys/", 0) == 0) {
        return path;
    }

    if (path.rfind("/sd/", 0) == 0) {
        return path;
    }

    if (path.rfind("sdcard/", 0) == 0) {
        return std::string("/") + path;
    }

    if (path.rfind("ssh_keys/", 0) == 0) {
        return std::string(kSshKeysRoot) + path.substr(9);
    }

    if (path[0] != '/') {
        return std::string(kSshKeysRoot) + path;
    }

    return path;
}

std::string base_name(const std::string &path)
{
    const size_t sep = path.find_last_of('/');
    if (sep == std::string::npos) {
        return path;
    }
    return path.substr(sep + 1);
}

bool is_probably_metadata_file(const std::string &name)
{
    const std::string lower = lowercase_ascii(base_name(name));
    if (starts_with_ascii_ci(lower, "._")) {
        return true;
    }
    // FAT short aliases for AppleDouble files commonly begin with '_' and
    // include a tilde sequence (for example: _SSH_C~1).
    return !lower.empty() && lower[0] == '_' && lower.find('~') != std::string::npos;
}

void push_unique_path(std::vector<std::string> *paths, const std::string &candidate)
{
    if (paths == nullptr || candidate.empty()) {
        return;
    }
    for (const auto &existing : *paths) {
        if (existing == candidate) {
            return;
        }
    }
    paths->push_back(candidate);
}

std::vector<std::string> identity_path_candidates(const std::string &raw_identity_path)
{
    std::vector<std::string> paths;
    const std::string configured = trim_matching_quotes(trim_ascii(raw_identity_path));
    const std::string expanded = expand_identity_file_path(configured);
    const std::string leaf = base_name(expanded.empty() ? configured : expanded);

    push_unique_path(&paths, configured);
    push_unique_path(&paths, expanded);
    if (!leaf.empty()) {
        push_unique_path(&paths, std::string(kSshKeysRoot) + leaf);
        push_unique_path(&paths, std::string(kSshKeysRootAlt) + leaf);
        push_unique_path(&paths, std::string("/sdcard/") + leaf);
        push_unique_path(&paths, std::string("/sd/") + leaf);
    }

    if (!expanded.empty() && expanded.rfind("/sdcard/", 0) == 0) {
        push_unique_path(&paths, std::string("/sd/") + expanded.substr(8));
    } else if (!expanded.empty() && expanded.rfind("/sd/", 0) == 0) {
        push_unique_path(&paths, std::string("/sdcard/") + expanded.substr(4));
    }
    return paths;
}

bool wildcard_match(const std::string &pattern, const std::string &candidate)
{
    const std::string pat = lowercase_ascii(pattern);
    const std::string text = lowercase_ascii(candidate);

    size_t p = 0;
    size_t t = 0;
    size_t star = std::string::npos;
    size_t match = 0;

    while (t < text.size()) {
        if (p < pat.size() && (pat[p] == '?' || pat[p] == text[t])) {
            ++p;
            ++t;
        } else if (p < pat.size() && pat[p] == '*') {
            star = p++;
            match = t;
        } else if (star != std::string::npos) {
            p = star + 1;
            t = ++match;
        } else {
            return false;
        }
    }

    while (p < pat.size() && pat[p] == '*') {
        ++p;
    }

    return p == pat.size();
}

bool host_block_matches(const SSHConfigHostBlock &block, const std::string &alias)
{
    bool has_positive = false;
    bool positive_match = false;

    for (const std::string &raw_pattern : block.patterns) {
        if (raw_pattern.empty()) {
            continue;
        }

        if (raw_pattern[0] == '!') {
            const std::string neg = raw_pattern.substr(1);
            if (!neg.empty() && wildcard_match(neg, alias)) {
                return false;
            }
            continue;
        }

        has_positive = true;
        if (wildcard_match(raw_pattern, alias)) {
            positive_match = true;
        }
    }

    return has_positive && positive_match;
}

void apply_option(const std::string &directive, const std::string &raw_value, SSHConfigOptions *target)
{
    if (target == nullptr) {
        return;
    }

    const std::string value = trim_matching_quotes(trim_ascii(raw_value));

    if (directive == "hostname") {
        target->host_name = value;
        target->has_host_name = true;
        return;
    }
    if (directive == "user") {
        target->user = value;
        target->has_user = true;
        return;
    }
    if (directive == "port") {
        int parsed_port = 0;
        if (parse_int32(value, &parsed_port) && parsed_port > 0 && parsed_port <= 65535) {
            target->port = parsed_port;
            target->has_port = true;
        }
        return;
    }
    if (directive == "identityfile") {
        const std::string expanded = expand_identity_file_path(value);
        if (!expanded.empty()) {
            target->identity_files.push_back(expanded);
        }
        return;
    }
    if (directive == "identitiesonly") {
        bool parsed = false;
        if (parse_bool_flag(value, &parsed)) {
            target->identities_only = parsed;
            target->has_identities_only = true;
        }
        return;
    }
    if (directive == "connecttimeout") {
        int timeout = 0;
        if (parse_int32(value, &timeout) && timeout >= 0) {
            target->connect_timeout = timeout;
            target->has_connect_timeout = true;
        }
        return;
    }
    if (directive == "serveraliveinterval") {
        int interval = 0;
        if (parse_int32(value, &interval) && interval >= 0) {
            target->server_alive_interval = interval;
            target->has_server_alive_interval = true;
        }
        return;
    }
    if (directive == "serveralivecountmax") {
        int max_count = 0;
        if (parse_int32(value, &max_count) && max_count >= 0) {
            target->server_alive_count_max = max_count;
            target->has_server_alive_count_max = true;
        }
        return;
    }
    if (directive == "stricthostkeychecking") {
        target->strict_host_key_checking = lowercase_ascii(value);
        target->has_strict_host_key_checking = true;
        return;
    }
    if (directive == "network" || directive == "tpagernetwork") {
        target->network = value;
        target->has_network = true;
        return;
    }
    if (directive == "fontsize") {
        bool parsed_big = false;
        if (parse_fontsize_token(value, &parsed_big)) {
            target->fontsize_big = parsed_big;
            target->has_fontsize = true;
        }
        return;
    }
}

void merge_options(const SSHConfigOptions &source, SSHConfigOptions *target)
{
    if (target == nullptr) {
        return;
    }

    if (source.has_host_name) {
        target->host_name = source.host_name;
        target->has_host_name = true;
    }
    if (source.has_user) {
        target->user = source.user;
        target->has_user = true;
    }
    if (source.has_port) {
        target->port = source.port;
        target->has_port = true;
    }
    if (source.has_identities_only) {
        target->identities_only = source.identities_only;
        target->has_identities_only = true;
    }
    if (!source.identity_files.empty()) {
        target->identity_files.insert(target->identity_files.end(),
                                      source.identity_files.begin(),
                                      source.identity_files.end());
    }
    if (source.has_connect_timeout) {
        target->connect_timeout = source.connect_timeout;
        target->has_connect_timeout = true;
    }
    if (source.has_server_alive_interval) {
        target->server_alive_interval = source.server_alive_interval;
        target->has_server_alive_interval = true;
    }
    if (source.has_server_alive_count_max) {
        target->server_alive_count_max = source.server_alive_count_max;
        target->has_server_alive_count_max = true;
    }
    if (source.has_strict_host_key_checking) {
        target->strict_host_key_checking = source.strict_host_key_checking;
        target->has_strict_host_key_checking = true;
    }
    if (source.has_network) {
        target->network = source.network;
        target->has_network = true;
    }
    if (source.has_fontsize) {
        target->fontsize_big = source.fontsize_big;
        target->has_fontsize = true;
    }
}

bool path_exists_regular_file(const std::string &path)
{
    struct stat st = {};
    if (stat(path.c_str(), &st) != 0) {
        return false;
    }
    return S_ISREG(st.st_mode);
}

bool path_exists_dir(const std::string &path)
{
    struct stat st = {};
    if (stat(path.c_str(), &st) != 0) {
        return false;
    }
    return S_ISDIR(st.st_mode);
}

int count_pem_files_in_dir(const char *dir_path)
{
    if (dir_path == nullptr) {
        return -1;
    }
    DIR *dir = opendir(dir_path);
    if (dir == nullptr) {
        return -1;
    }
    int count = 0;
    struct dirent *entry = nullptr;
    while ((entry = readdir(dir)) != nullptr) {
        const std::string name = lowercase_ascii(entry->d_name);
        if (name.size() > 4 && name.rfind(".pem") == name.size() - 4) {
            count++;
        }
    }
    closedir(dir);
    return count;
}

void append_dir_listing(SSHTerminal *terminal, const char *dir_path, int max_entries)
{
    if (terminal == nullptr || dir_path == nullptr || max_entries <= 0) {
        return;
    }
    DIR *dir = opendir(dir_path);
    if (dir == nullptr) {
        terminal->append_text("sdcheck: ls ");
        terminal->append_text(dir_path);
        terminal->append_text(" -> <unavailable>\n");
        return;
    }
    terminal->append_text("sdcheck: ls ");
    terminal->append_text(dir_path);
    terminal->append_text(":\n");
    int shown = 0;
    struct dirent *entry = nullptr;
    while ((entry = readdir(dir)) != nullptr && shown < max_entries) {
        const std::string name = entry->d_name;
        if (name == "." || name == "..") {
            continue;
        }
        terminal->append_text("  ");
        terminal->append_text(name.c_str());
        terminal->append_text("\n");
        shown++;
    }
    closedir(dir);
}

void append_sd_probe(SSHTerminal *terminal)
{
    if (terminal == nullptr) {
        return;
    }

#if defined(TPAGER_TARGET)
    ScopedSDMount mount_guard = {};
    if (!mount_guard.ok()) {
        terminal->append_text("sdcheck: mount call failed: ");
        terminal->append_text(esp_err_to_name(mount_guard.last_err()));
        terminal->append_text("; probing existing paths\n");
    }
#endif

    const bool has_sdcard = path_exists_dir("/sdcard");
    const bool has_sd = path_exists_dir("/sd");
    const bool has_sdcard_keys = path_exists_dir(kSshKeysDir);
    const bool has_sd_keys = path_exists_dir(kSshKeysDirAlt);

    char line[192];
    std::snprintf(line, sizeof(line), "sdcheck: dir /sdcard=%d /sd=%d /sdcard/ssh_keys=%d /sd/ssh_keys=%d\n",
                  has_sdcard ? 1 : 0, has_sd ? 1 : 0, has_sdcard_keys ? 1 : 0, has_sd_keys ? 1 : 0);
    terminal->append_text(line);

    const int pem_sdcard = count_pem_files_in_dir(kSshKeysDir);
    const int pem_sd = count_pem_files_in_dir(kSshKeysDirAlt);
    std::snprintf(line, sizeof(line), "sdcheck: pem count /sdcard/ssh_keys=%d /sd/ssh_keys=%d\n", pem_sdcard, pem_sd);
    terminal->append_text(line);

    std::snprintf(line, sizeof(line), "sdcheck: ssh_config /sdcard=%d /sd=%d\n",
                  path_exists_regular_file(kSshConfigPath) ? 1 : 0,
                  path_exists_regular_file(kSshConfigPathAlt) ? 1 : 0);
    terminal->append_text(line);

    std::snprintf(line, sizeof(line), "sdcheck: wifi_config /sdcard=%d /sd=%d\n",
                  path_exists_regular_file(kWifiConfigPath) ? 1 : 0,
                  path_exists_regular_file(kWifiConfigPathAlt) ? 1 : 0);
    terminal->append_text(line);

    const std::string ssh_path = resolve_ssh_config_path();
    const std::string wifi_path = resolve_wifi_config_path();
    terminal->append_text("sdcheck: resolved ssh_config -> ");
    terminal->append_text(ssh_path.c_str());
    terminal->append_text("\n");
    terminal->append_text("sdcheck: resolved wifi_config -> ");
    terminal->append_text(wifi_path.c_str());
    terminal->append_text("\n");

    SSHConfigFile parsed_ssh = {};
    const bool ssh_ok = parse_ssh_config_file(&parsed_ssh);
    std::snprintf(line, sizeof(line), "sdcheck: parse ssh_config=%d aliases=%d host_blocks=%d\n",
                  ssh_ok ? 1 : 0,
                  static_cast<int>(parsed_ssh.aliases.size()),
                  static_cast<int>(parsed_ssh.host_blocks.size()));
    terminal->append_text(line);

    std::vector<WifiProfile> profiles;
    const bool wifi_ok = parse_wifi_config_file(&profiles);
    std::snprintf(line, sizeof(line), "sdcheck: parse wifi_config=%d profiles=%d\n",
                  wifi_ok ? 1 : 0,
                  static_cast<int>(profiles.size()));
    terminal->append_text(line);

    append_dir_listing(terminal, kSshKeysDir, 12);
    append_dir_listing(terminal, kSshKeysDirAlt, 12);
}

bool parse_u64_decimal(const std::string &text, uint64_t *out)
{
    if (out == nullptr || text.empty()) {
        return false;
    }
    char *end = nullptr;
    errno = 0;
    const unsigned long long value = std::strtoull(text.c_str(), &end, 10);
    if (errno != 0 || end == text.c_str() || *end != '\0') {
        return false;
    }
    *out = static_cast<uint64_t>(value);
    return true;
}

bool parse_u32_hex(const std::string &text, uint32_t *out)
{
    if (out == nullptr || text.empty()) {
        return false;
    }
    char *end = nullptr;
    errno = 0;
    const unsigned long value = std::strtoul(text.c_str(), &end, 16);
    if (errno != 0 || end == text.c_str() || *end != '\0' || value > 0xFFFFFFFFUL) {
        return false;
    }
    *out = static_cast<uint32_t>(value);
    return true;
}

bool serial_read_line_with_timeout(int timeout_ms, std::string *line_out)
{
    if (line_out == nullptr) {
        return false;
    }
    line_out->clear();

    const int stdin_fd = fileno(stdin);
    if (stdin_fd < 0) {
        return false;
    }

    while (true) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(stdin_fd, &readfds);

        struct timeval tv = {};
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        const int sel = select(stdin_fd + 1, &readfds, nullptr, nullptr, &tv);
        if (sel <= 0) {
            return false;
        }

        char ch = '\0';
        const ssize_t n = read(stdin_fd, &ch, 1);
        if (n <= 0) {
            return false;
        }
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            return true;
        }
        if (line_out->size() < 2048) {
            line_out->push_back(ch);
        }
    }
}

int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return 10 + (c - 'a');
    }
    if (c >= 'A' && c <= 'F') {
        return 10 + (c - 'A');
    }
    return -1;
}

bool decode_hex_payload(const std::string &hex, std::vector<uint8_t> *bytes_out)
{
    if (bytes_out == nullptr || (hex.size() % 2) != 0) {
        return false;
    }
    bytes_out->clear();
    bytes_out->reserve(hex.size() / 2);
    for (size_t i = 0; i < hex.size(); i += 2) {
        const int hi = hex_nibble(hex[i]);
        const int lo = hex_nibble(hex[i + 1]);
        if (hi < 0 || lo < 0) {
            return false;
        }
        bytes_out->push_back(static_cast<uint8_t>((hi << 4) | lo));
    }
    return true;
}

bool valid_serial_target_name(const std::string &name)
{
    if (name.empty() || name.size() > 64) {
        return false;
    }
    if (name.find('/') != std::string::npos || name.find('\\') != std::string::npos ||
        name.find("..") != std::string::npos) {
        return false;
    }
    return true;
}

bool serial_receive_to_sd_file(SSHTerminal *terminal, const std::string &target_name)
{
    if (terminal == nullptr) {
        return false;
    }
    struct SerialRxFlagGuard {
        SSHTerminal *terminal = nullptr;
        explicit SerialRxFlagGuard(SSHTerminal *t) : terminal(t)
        {
            if (terminal != nullptr) {
                terminal->set_serial_rx_in_progress(true);
            }
        }
        ~SerialRxFlagGuard()
        {
            if (terminal != nullptr) {
                terminal->set_serial_rx_in_progress(false);
            }
        }
    } serial_rx_guard(terminal);

    if (!valid_serial_target_name(target_name)) {
        terminal->append_text("serialrx: invalid target filename\n");
        return false;
    }

#if defined(TPAGER_TARGET)
    ScopedSDMount mount_guard = {};
    if (!mount_guard.ok()) {
        terminal->append_text("serialrx: SD mount failed\n");
        return false;
    }
#endif

    const char *root_dir = path_exists_dir("/sdcard") ? "/sdcard" : (path_exists_dir("/sd") ? "/sd" : nullptr);
    if (root_dir == nullptr) {
        terminal->append_text("serialrx: no SD root mountpoint\n");
        return false;
    }

    const std::string target_path = std::string(root_dir) + "/" + target_name;
    FILE *out = std::fopen(target_path.c_str(), "wb");
    if (out == nullptr) {
        terminal->append_text("serialrx: failed to open target file\n");
        return false;
    }

    terminal->append_text("serialrx: waiting for BEGIN <size> <crc32hex>\n");
    terminal->append_text("serialrx: send DATA <hex> lines, then END\n");
    ESP_LOGI(TAG, "serialrx ready: target=%s", target_path.c_str());
    ESP_LOGI(TAG, "POCKETCTL serialrx_ready target=%s", target_path.c_str());

    std::string line;
    if (!serial_read_line_with_timeout(30000, &line)) {
        terminal->append_text("serialrx: timeout waiting for BEGIN\n");
        std::fclose(out);
        return false;
    }

    const std::vector<std::string> begin_parts = split_nonempty_whitespace(line);
    if (begin_parts.size() < 3 || lowercase_ascii(begin_parts[0]) != "begin") {
        terminal->append_text("serialrx: invalid BEGIN header\n");
        std::fclose(out);
        return false;
    }

    uint64_t expected_size_u64 = 0;
    uint32_t expected_crc = 0;
    if (!parse_u64_decimal(begin_parts[1], &expected_size_u64) ||
        !parse_u32_hex(begin_parts[2], &expected_crc)) {
        terminal->append_text("serialrx: invalid BEGIN arguments\n");
        std::fclose(out);
        return false;
    }
    const size_t expected_size = static_cast<size_t>(expected_size_u64);

    char hdr[128];
    std::snprintf(hdr, sizeof(hdr), "serialrx: receiving %u bytes to %s\n",
                  static_cast<unsigned>(expected_size),
                  target_path.c_str());
    terminal->append_text(hdr);

    std::vector<uint8_t> chunk;
    size_t received = 0;
    uint32_t crc = 0;
    int last_percent = -1;

    while (received < expected_size) {
        if (!serial_read_line_with_timeout(20000, &line)) {
            terminal->append_text("serialrx: timeout during transfer\n");
            std::fclose(out);
            std::remove(target_path.c_str());
            return false;
        }

        const std::vector<std::string> parts = split_nonempty_whitespace(line);
        if (parts.empty()) {
            continue;
        }
        const std::string cmd = lowercase_ascii(parts[0]);
        if (cmd == "abort") {
            terminal->append_text("serialrx: aborted by host\n");
            std::fclose(out);
            std::remove(target_path.c_str());
            return false;
        }
        if (cmd != "data" || parts.size() < 2) {
            continue;
        }

        if (!decode_hex_payload(parts[1], &chunk)) {
            terminal->append_text("serialrx: invalid DATA hex payload\n");
            std::fclose(out);
            std::remove(target_path.c_str());
            return false;
        }
        if (chunk.empty()) {
            continue;
        }
        if (received + chunk.size() > expected_size) {
            terminal->append_text("serialrx: DATA exceeds expected size\n");
            std::fclose(out);
            std::remove(target_path.c_str());
            return false;
        }
        const size_t written = std::fwrite(chunk.data(), 1, chunk.size(), out);
        if (written != chunk.size()) {
            terminal->append_text("serialrx: write failure\n");
            std::fclose(out);
            std::remove(target_path.c_str());
            return false;
        }
        crc = esp_crc32_le(crc, chunk.data(), static_cast<uint32_t>(chunk.size()));
        received += chunk.size();

        const int pct = (expected_size == 0) ? 100 : static_cast<int>((received * 100U) / expected_size);
        if (pct >= last_percent + 10 || pct == 100) {
            last_percent = pct;
            char linebuf[64];
            std::snprintf(linebuf, sizeof(linebuf), "serialrx: %d%% (%u/%u)\n",
                          pct,
                          static_cast<unsigned>(received),
                          static_cast<unsigned>(expected_size));
            terminal->append_text(linebuf);
        }
    }

    if (!serial_read_line_with_timeout(5000, &line)) {
        terminal->append_text("serialrx: missing END marker\n");
        std::fclose(out);
        std::remove(target_path.c_str());
        return false;
    }

    const std::vector<std::string> end_parts = split_nonempty_whitespace(line);
    if (end_parts.empty() || lowercase_ascii(end_parts[0]) != "end") {
        terminal->append_text("serialrx: invalid END marker\n");
        std::fclose(out);
        std::remove(target_path.c_str());
        return false;
    }

    std::fflush(out);
    std::fclose(out);

    if (crc != expected_crc) {
        terminal->append_text("serialrx: CRC mismatch, file removed\n");
        ESP_LOGE(TAG, "serialrx CRC mismatch expected=%08" PRIx32 " actual=%08" PRIx32, expected_crc, crc);
        std::remove(target_path.c_str());
        return false;
    }

    terminal->append_text("serialrx: transfer complete\n");
    ESP_LOGI(TAG, "serialrx complete: path=%s bytes=%u crc=%08" PRIx32,
             target_path.c_str(), static_cast<unsigned>(received), crc);
    return true;
}

std::string resolve_ssh_config_path()
{
    const std::string preferred = kSshConfigPath;
    const std::string root_preferred = kSshConfigPathRoot;
    const std::string alt_preferred = kSshConfigPathAlt;
    const std::string alt_root_preferred = kSshConfigPathAltRoot;

    auto candidate_score = [](const std::string &lower_name) -> int {
        if (lower_name == "ssh_config") {
            return 100;
        }
        if (starts_with_ascii_ci(lower_name, "ssh_config")) {
            return 95;
        }
        if (starts_with_ascii_ci(lower_name, "ssh_co~") ||
            starts_with_ascii_ci(lower_name, "ssh_c~") ||
            starts_with_ascii_ci(lower_name, "sshco~") ||
            starts_with_ascii_ci(lower_name, "sshc~")) {
            return 90;
        }
        if (lower_name == "sshcfg" || lower_name == "ssh.cfg" || lower_name == "ssh_cfg") {
            return 80;
        }
        if (starts_with_ascii_ci(lower_name, "sshcfg") || starts_with_ascii_ci(lower_name, "ssh_cfg")) {
            return 70;
        }
        if (lower_name.find("ssh") != std::string::npos && lower_name.find("config") != std::string::npos) {
            return 60;
        }
        if (starts_with_ascii_ci(lower_name, "ssh") && lower_name.find('~') != std::string::npos) {
            return 55;
        }
        return 0;
    };

    auto tilde_index = [](const std::string &name) -> int {
        const size_t tilde = name.find('~');
        if (tilde == std::string::npos || tilde + 1 >= name.size()) {
            return -1;
        }
        int value = 0;
        bool saw_digit = false;
        for (size_t i = tilde + 1; i < name.size(); ++i) {
            const char ch = name[i];
            if (ch >= '0' && ch <= '9') {
                saw_digit = true;
                value = value * 10 + (ch - '0');
            } else {
                break;
            }
        }
        return saw_digit ? value : -1;
    };

    struct SSHConfigCandidate {
        std::string path;
        std::string lower_name;
        int score = 0;
        int tilde = -1;
        time_t mtime = 0;
        int dir_rank = 0;
    };

    auto scan_dir = [&](const char *dir_path, int dir_rank, std::vector<SSHConfigCandidate> *out) {
        if (dir_path == nullptr || out == nullptr) {
            return;
        }
        DIR *dir = opendir(dir_path);
        if (dir == nullptr) {
            ESP_LOGW(TAG, "ssh_config resolve: unable to open %s (errno=%d %s)",
                     dir_path, errno, strerror(errno));
            return;
        }
        struct dirent *entry = nullptr;
        while ((entry = readdir(dir)) != nullptr) {
            const std::string name = entry->d_name;
            if (name == "." || name == "..") {
                continue;
            }
            const std::string lower = lowercase_ascii(name);
            if (starts_with_ascii_ci(lower, "._")) {
                continue;
            }
            const int score = candidate_score(lower);
            if (score <= 0) {
                continue;
            }

            const std::string path = std::string(dir_path) + "/" + name;
            struct stat st = {};
            if (stat(path.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) {
                continue;
            }

            SSHConfigCandidate candidate = {};
            candidate.path = path;
            candidate.lower_name = lower;
            candidate.score = score;
            candidate.tilde = tilde_index(name);
            candidate.mtime = st.st_mtime;
            candidate.dir_rank = dir_rank;
            out->push_back(candidate);
        }
        closedir(dir);
    };

    std::vector<SSHConfigCandidate> candidates;
    scan_dir(kSshKeysDir, 1, &candidates);
    scan_dir(kSshKeysDirAlt, 1, &candidates);
    scan_dir("/sdcard", 0, &candidates);
    scan_dir("/sd", 0, &candidates);

    if (candidates.empty()) {
        ESP_LOGW(TAG,
                 "ssh_config resolve: no candidate in %s, %s, /sdcard, or /sd; expected %s, %s, %s, or %s",
                 kSshKeysDir,
                 kSshKeysDirAlt,
                 preferred.c_str(),
                 root_preferred.c_str(),
                 alt_preferred.c_str(),
                 alt_root_preferred.c_str());
        return preferred;
    }

    const SSHConfigCandidate *best = &candidates[0];
    for (const auto &candidate : candidates) {
        bool take = false;
        if (candidate.score > best->score) {
            take = true;
        } else if (candidate.score == best->score && candidate.mtime > best->mtime) {
            take = true;
        } else if (candidate.score == best->score && candidate.mtime == best->mtime &&
                   candidate.tilde > best->tilde) {
            take = true;
        } else if (candidate.score == best->score && candidate.mtime == best->mtime &&
                   candidate.tilde == best->tilde && candidate.dir_rank > best->dir_rank) {
            take = true;
        } else if (candidate.score == best->score && candidate.mtime == best->mtime &&
                   candidate.tilde == best->tilde && candidate.dir_rank == best->dir_rank &&
                   candidate.lower_name > best->lower_name) {
            take = true;
        }
        if (take) {
            best = &candidate;
        }
    }

    ESP_LOGW(TAG, "ssh_config resolve: selected %s (score=%d mtime=%" PRId64 " tilde=%d)",
             best->path.c_str(), best->score, static_cast<int64_t>(best->mtime), best->tilde);
    return best->path;
}

std::string resolve_wifi_config_path()
{
    const std::string preferred = kWifiConfigPath;
    const std::string root_preferred = kWifiConfigPathRoot;
    const std::string alt_preferred = kWifiConfigPathAlt;
    const std::string alt_root_preferred = kWifiConfigPathAltRoot;

    auto candidate_score = [](const std::string &lower_name) -> int {
        if (lower_name == "wifi_config") {
            return 100;
        }
        if (starts_with_ascii_ci(lower_name, "wifi_config")) {
            return 95;
        }
        if (starts_with_ascii_ci(lower_name, "wifi_co~") ||
            starts_with_ascii_ci(lower_name, "wifi_c~") ||
            starts_with_ascii_ci(lower_name, "wifico~") ||
            starts_with_ascii_ci(lower_name, "wific~")) {
            return 90;
        }
        if (lower_name == "wificfg" || lower_name == "wifi.cfg" || lower_name == "wifi_cfg") {
            return 80;
        }
        if (starts_with_ascii_ci(lower_name, "wificfg") || starts_with_ascii_ci(lower_name, "wifi_cfg")) {
            return 70;
        }
        if (starts_with_ascii_ci(lower_name, "wifi") && lower_name.find('~') != std::string::npos) {
            return 60;
        }
        return 0;
    };

    auto tilde_index = [](const std::string &name) -> int {
        const size_t tilde = name.find('~');
        if (tilde == std::string::npos || tilde + 1 >= name.size()) {
            return -1;
        }
        int value = 0;
        bool saw_digit = false;
        for (size_t i = tilde + 1; i < name.size(); ++i) {
            const char ch = name[i];
            if (ch >= '0' && ch <= '9') {
                saw_digit = true;
                value = value * 10 + (ch - '0');
            } else {
                break;
            }
        }
        return saw_digit ? value : -1;
    };

    struct WifiConfigCandidate {
        std::string path;
        std::string lower_name;
        int score = 0;
        int tilde = -1;
        time_t mtime = 0;
        int dir_rank = 0;
    };

    auto scan_dir = [&](const char *dir_path, int dir_rank, std::vector<WifiConfigCandidate> *out) {
        if (dir_path == nullptr || out == nullptr) {
            return;
        }
        DIR *dir = opendir(dir_path);
        if (dir == nullptr) {
            ESP_LOGW(TAG, "wifi_config resolve: unable to open %s (errno=%d %s)",
                     dir_path, errno, strerror(errno));
            return;
        }
        struct dirent *entry = nullptr;
        while ((entry = readdir(dir)) != nullptr) {
            const std::string name = entry->d_name;
            if (name == "." || name == "..") {
                continue;
            }
            const std::string lower = lowercase_ascii(name);
            if (starts_with_ascii_ci(lower, "._")) {
                continue;
            }
            const int score = candidate_score(lower);
            if (score <= 0) {
                continue;
            }

            const std::string path = std::string(dir_path) + "/" + name;
            struct stat st = {};
            if (stat(path.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) {
                continue;
            }

            WifiConfigCandidate candidate = {};
            candidate.path = path;
            candidate.lower_name = lower;
            candidate.score = score;
            candidate.tilde = tilde_index(name);
            candidate.mtime = st.st_mtime;
            candidate.dir_rank = dir_rank;
            out->push_back(candidate);
        }
        closedir(dir);
    };

    std::vector<WifiConfigCandidate> candidates;
    scan_dir(kSshKeysDir, 1, &candidates);
    scan_dir(kSshKeysDirAlt, 1, &candidates);
    scan_dir("/sdcard", 0, &candidates);
    scan_dir("/sd", 0, &candidates);

    if (candidates.empty()) {
        ESP_LOGW(TAG,
                 "wifi_config resolve: no candidate in %s, %s, /sdcard, or /sd; expected %s, %s, %s, or %s",
                 kSshKeysDir,
                 kSshKeysDirAlt,
                 preferred.c_str(),
                 root_preferred.c_str(),
                 alt_preferred.c_str(),
                 alt_root_preferred.c_str());
        return preferred;
    }

    const WifiConfigCandidate *best = &candidates[0];
    for (const auto &candidate : candidates) {
        bool take = false;
        if (candidate.score > best->score) {
            take = true;
        } else if (candidate.score == best->score && candidate.mtime > best->mtime) {
            take = true;
        } else if (candidate.score == best->score && candidate.mtime == best->mtime &&
                   candidate.tilde > best->tilde) {
            take = true;
        } else if (candidate.score == best->score && candidate.mtime == best->mtime &&
                   candidate.tilde == best->tilde && candidate.dir_rank > best->dir_rank) {
            take = true;
        } else if (candidate.score == best->score && candidate.mtime == best->mtime &&
                   candidate.tilde == best->tilde && candidate.dir_rank == best->dir_rank &&
                   candidate.lower_name > best->lower_name) {
            take = true;
        }
        if (take) {
            best = &candidate;
        }
    }

    ESP_LOGW(TAG, "wifi_config resolve: selected %s (score=%d mtime=%" PRId64 " tilde=%d)",
             best->path.c_str(), best->score, static_cast<int64_t>(best->mtime), best->tilde);
    return best->path;
}

void maybe_push_wifi_profile(const WifiProfile &candidate, std::vector<WifiProfile> *profiles)
{
    if (profiles == nullptr) {
        return;
    }
    // Accept either named profile or direct SSID profile; runtime command matching
    // supports both `wifi <NetworkName>` and `wifi <SSID>`.
    if (candidate.network_name.empty() && candidate.ssid.empty()) {
        return;
    }
    profiles->push_back(candidate);
}

bool parse_wifi_config_file(std::vector<WifiProfile> *profiles)
{
    if (profiles == nullptr) {
        return false;
    }
    profiles->clear();

#if defined(TPAGER_TARGET)
    ScopedSDMount mount_guard = {};
    if (!mount_guard.ok()) {
        ESP_LOGW(TAG, "parse_wifi_config_file: mount helper failed, continuing with direct path probes");
    }
#endif

    auto parse_wifi_path = [&](const std::string &config_path, std::vector<WifiProfile> *out_profiles) -> bool {
        if (out_profiles == nullptr) {
            return false;
        }
        out_profiles->clear();

        FILE *file = std::fopen(config_path.c_str(), "r");
        if (file == nullptr) {
            return false;
        }

        WifiProfile current = {};
        bool in_profile = false;
        int order = 0;
        char line_buffer[512];
        while (std::fgets(line_buffer, sizeof(line_buffer), file) != nullptr) {
            std::string line = line_buffer;
            line = trim_ascii(strip_inline_comment(line));
            if (line.empty()) {
                continue;
            }

            std::string key;
            std::string value;
            if (!split_directive(line, &key, &value)) {
                continue;
            }

            const std::string directive = lowercase_ascii(key);
            const std::string cleaned_value = trim_matching_quotes(trim_ascii(value));
            if (directive == "network") {
                if (in_profile) {
                    maybe_push_wifi_profile(current, out_profiles);
                }
                current = {};
                current.network_name = cleaned_value;
                current.file_order = order++;
                in_profile = true;
                continue;
            }

            if (!in_profile) {
                // Tolerate top-level keys by implicitly creating a profile.
                current = {};
                current.file_order = order++;
                in_profile = true;
            }

            if (directive == "ssid") {
                current.ssid = cleaned_value;
            } else if (directive == "password") {
                current.password = cleaned_value;
            } else if (directive == "autoconnect") {
                bool parsed = false;
                if (parse_bool_flag(cleaned_value, &parsed)) {
                    current.auto_connect = parsed;
                    current.has_auto_connect = true;
                }
            } else if (directive == "priority") {
                // Accepted for compatibility with requirement doc; currently unused.
            }
        }

        if (in_profile) {
            maybe_push_wifi_profile(current, out_profiles);
        }

        std::fclose(file);
        return true;
    };

    std::vector<std::string> candidates;
    auto add_candidate = [&](const std::string &path) {
        if (path.empty()) {
            return;
        }
        for (const auto &existing : candidates) {
            if (existing == path) {
                return;
            }
        }
        candidates.push_back(path);
    };

    add_candidate(kWifiConfigPath);
    add_candidate(kWifiConfigPathRoot);
    add_candidate(kWifiConfigPathAlt);
    add_candidate(kWifiConfigPathAltRoot);
    add_candidate(resolve_wifi_config_path());

    auto scan_candidates = [&](const char *dir_path) {
        if (dir_path == nullptr) {
            return;
        }
        DIR *dir = opendir(dir_path);
        if (dir == nullptr) {
            return;
        }
        struct dirent *entry = nullptr;
        while ((entry = readdir(dir)) != nullptr) {
            const std::string name = entry->d_name;
            if (name == "." || name == "..") {
                continue;
            }
            const std::string lower = lowercase_ascii(name);
            if (starts_with_ascii_ci(lower, "._")) {
                continue;
            }
            if (lower.find("wifi") == std::string::npos) {
                continue;
            }
            const std::string path = std::string(dir_path) + "/" + name;
            struct stat st = {};
            if (stat(path.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) {
                continue;
            }
            add_candidate(path);
        }
        closedir(dir);
    };

    scan_candidates(kSshKeysDir);
    scan_candidates(kSshKeysDirAlt);
    scan_candidates("/sdcard");
    scan_candidates("/sd");

    bool opened_any = false;
    for (const auto &path : candidates) {
        std::vector<WifiProfile> parsed_profiles;
        if (!parse_wifi_path(path, &parsed_profiles)) {
            continue;
        }
        opened_any = true;
        ESP_LOGI(TAG, "wifi_config open: %s (profiles=%d)", path.c_str(), static_cast<int>(parsed_profiles.size()));
        if (!parsed_profiles.empty()) {
            *profiles = std::move(parsed_profiles);
            return true;
        }
    }

    if (!opened_any) {
        ESP_LOGW(TAG, "wifi_config open failed for all candidates");
        return false;
    }

    ESP_LOGW(TAG, "wifi_config parsed but no profiles found");
    return true;
}

const WifiProfile *find_wifi_profile(const std::vector<WifiProfile> &profiles, const std::string &name_or_ssid)
{
    const std::string query = lowercase_ascii(trim_ascii(name_or_ssid));
    if (query.empty()) {
        return nullptr;
    }

    for (const auto &profile : profiles) {
        if (!profile.network_name.empty() && lowercase_ascii(profile.network_name) == query) {
            return &profile;
        }
    }
    for (const auto &profile : profiles) {
        if (!profile.ssid.empty() && lowercase_ascii(profile.ssid) == query) {
            return &profile;
        }
    }
    return nullptr;
}

bool parse_ssh_config_file(SSHConfigFile *parsed)
{
    if (parsed == nullptr) {
        return false;
    }

    *parsed = {};

#if defined(TPAGER_TARGET)
    ScopedSDMount mount_guard = {};
    if (!mount_guard.ok()) {
        ESP_LOGW(TAG, "parse_ssh_config_file: mount helper failed, continuing with direct path probes");
    }
#endif

    auto parse_ssh_path = [&](const std::string &config_path, SSHConfigFile *out_parsed) -> bool {
        if (out_parsed == nullptr) {
            return false;
        }
        *out_parsed = {};

        FILE *file = std::fopen(config_path.c_str(), "r");
        if (file == nullptr) {
            return false;
        }

        std::set<std::string> alias_seen;
        SSHConfigHostBlock *active_host = nullptr;
        bool saw_host = false;
        char line_buffer[512];
        while (std::fgets(line_buffer, sizeof(line_buffer), file) != nullptr) {
            std::string line = line_buffer;
            line = trim_ascii(strip_inline_comment(line));
            if (line.empty()) {
                continue;
            }

            std::string key;
            std::string value;
            if (!split_directive(line, &key, &value)) {
                continue;
            }

            const std::string directive = lowercase_ascii(key);
            if (directive == "host") {
                const std::vector<std::string> patterns = split_nonempty_whitespace(value);
                if (patterns.empty()) {
                    continue;
                }

                saw_host = true;
                out_parsed->host_blocks.push_back({});
                active_host = &out_parsed->host_blocks.back();
                active_host->patterns = patterns;

                for (const std::string &pattern : patterns) {
                    if (pattern.empty() || pattern[0] == '!') {
                        continue;
                    }
                    if (pattern.find('*') != std::string::npos || pattern.find('?') != std::string::npos) {
                        continue;
                    }
                    if (alias_seen.insert(pattern).second) {
                        out_parsed->aliases.push_back(pattern);
                    }
                }
                continue;
            }

            SSHConfigOptions *target = (!saw_host || active_host == nullptr)
                                           ? &out_parsed->global_options
                                           : &active_host->options;
            apply_option(directive, value, target);
        }

        std::fclose(file);
        return true;
    };

    std::vector<std::string> candidates;
    auto add_candidate = [&](const std::string &path) {
        if (path.empty()) {
            return;
        }
        for (const auto &existing : candidates) {
            if (existing == path) {
                return;
            }
        }
        candidates.push_back(path);
    };

    add_candidate(kSshConfigPath);
    add_candidate(kSshConfigPathRoot);
    add_candidate(kSshConfigPathAlt);
    add_candidate(kSshConfigPathAltRoot);
    add_candidate(resolve_ssh_config_path());

    auto scan_candidates = [&](const char *dir_path) {
        if (dir_path == nullptr) {
            return;
        }
        DIR *dir = opendir(dir_path);
        if (dir == nullptr) {
            return;
        }
        struct dirent *entry = nullptr;
        while ((entry = readdir(dir)) != nullptr) {
            const std::string name = entry->d_name;
            if (name == "." || name == "..") {
                continue;
            }
            const std::string lower = lowercase_ascii(name);
            if (starts_with_ascii_ci(lower, "._")) {
                continue;
            }
            if (lower.find("ssh") == std::string::npos || lower.find("config") == std::string::npos) {
                continue;
            }
            const std::string path = std::string(dir_path) + "/" + name;
            struct stat st = {};
            if (stat(path.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) {
                continue;
            }
            add_candidate(path);
        }
        closedir(dir);
    };

    scan_candidates(kSshKeysDir);
    scan_candidates(kSshKeysDirAlt);
    scan_candidates("/sdcard");
    scan_candidates("/sd");

    bool opened_any = false;
    SSHConfigFile first_parsed = {};
    bool have_first_parsed = false;
    for (const auto &path : candidates) {
        SSHConfigFile parsed_candidate = {};
        if (!parse_ssh_path(path, &parsed_candidate)) {
            continue;
        }
        opened_any = true;
        ESP_LOGI(TAG, "ssh_config open: %s (aliases=%d host_blocks=%d)",
                 path.c_str(),
                 static_cast<int>(parsed_candidate.aliases.size()),
                 static_cast<int>(parsed_candidate.host_blocks.size()));

        if (!have_first_parsed) {
            first_parsed = parsed_candidate;
            have_first_parsed = true;
        }
        if (!parsed_candidate.host_blocks.empty() || !parsed_candidate.aliases.empty()) {
            *parsed = std::move(parsed_candidate);
            return true;
        }
    }

    if (!opened_any) {
        ESP_LOGW(TAG, "ssh_config open failed for all candidates");
        return false;
    }

    if (have_first_parsed) {
        *parsed = std::move(first_parsed);
    }
    ESP_LOGW(TAG, "ssh_config parsed but no Host blocks found");
    return true;
}

bool resolve_ssh_alias(const std::string &alias, ResolvedSSHConfig *resolved)
{
    if (resolved == nullptr || alias.empty()) {
        return false;
    }

    SSHConfigFile parsed = {};
    if (!parse_ssh_config_file(&parsed)) {
        return false;
    }

    SSHConfigOptions effective = {};
    merge_options(parsed.global_options, &effective);

    bool matched = false;
    for (const auto &block : parsed.host_blocks) {
        if (host_block_matches(block, alias)) {
            merge_options(block.options, &effective);
            matched = true;
        }
    }

    if (!matched) {
        return false;
    }

    resolved->matched = true;
    resolved->alias = alias;
    resolved->host_name = effective.has_host_name ? effective.host_name : alias;
    resolved->user = effective.has_user ? effective.user : "";
    resolved->port = effective.has_port ? effective.port : 22;
    resolved->identities_only = effective.has_identities_only ? effective.identities_only : false;
    resolved->identity_files = effective.identity_files;
    resolved->strict_host_key_checking = effective.has_strict_host_key_checking
                                             ? effective.strict_host_key_checking
                                             : "ask";
    resolved->network = effective.has_network ? effective.network : "";
    return true;
}

bool read_default_fontsize_big_from_config(bool *out_big)
{
    if (out_big == nullptr) {
        return false;
    }

    SSHConfigFile parsed = {};
    if (!parse_ssh_config_file(&parsed)) {
        return false;
    }
    if (!parsed.global_options.has_fontsize) {
        return false;
    }

    *out_big = parsed.global_options.fontsize_big;
    return true;
}

bool read_file_contents(const std::string &path, std::string *contents)
{
    if (contents == nullptr) {
        return false;
    }

#if defined(TPAGER_TARGET)
    ScopedSDMount mount_guard = {};
    if (!mount_guard.ok()) {
        ESP_LOGW(TAG, "read_file_contents: mount helper failed, trying direct open for %s", path.c_str());
    }
#endif

    FILE *file = std::fopen(path.c_str(), "rb");
    if (file == nullptr) {
        return false;
    }

    if (std::fseek(file, 0, SEEK_END) != 0) {
        std::fclose(file);
        return false;
    }
    const long file_size = std::ftell(file);
    if (file_size <= 0) {
        std::fclose(file);
        return false;
    }
    if (std::fseek(file, 0, SEEK_SET) != 0) {
        std::fclose(file);
        return false;
    }

    std::string data(static_cast<size_t>(file_size), '\0');
    const size_t read_count = std::fread(data.data(), 1, static_cast<size_t>(file_size), file);
    std::fclose(file);

    if (read_count != static_cast<size_t>(file_size)) {
        return false;
    }

    *contents = std::move(data);
    return true;
}

bool has_pem_extension(const std::string &name)
{
    if (is_probably_metadata_file(name)) {
        return false;
    }
    const std::string lowered = lowercase_ascii(name);
    if (lowered.size() >= 4 && lowered.rfind(".pem") == lowered.size() - 4) {
        return true;
    }
    // Some FAT aliases may not contain the dot separator.
    return lowered.size() >= 3 && lowered.rfind("pem") == lowered.size() - 3;
}

std::string short_name_prefix(const std::string &name)
{
    std::string stem = lowercase_ascii(base_name(name));
    const size_t dot = stem.rfind('.');
    if (dot != std::string::npos) {
        stem = stem.substr(0, dot);
    }
    const size_t tilde = stem.find('~');
    if (tilde == std::string::npos || tilde == 0) {
        return "";
    }
    std::string prefix = stem.substr(0, tilde);
    std::string normalized;
    normalized.reserve(prefix.size());
    for (char c : prefix) {
        if (std::isalnum(static_cast<unsigned char>(c)) != 0) {
            normalized.push_back(c);
        }
    }
    return normalized;
}

int load_keys_from_sd_if_needed(SSHTerminal *terminal)
{
    if (terminal == nullptr) {
        return 0;
    }
    if (!terminal->get_loaded_key_names().empty()) {
        return 0;
    }

#if defined(TPAGER_TARGET)
    ScopedSDMount mount_guard = {};
    if (!mount_guard.ok()) {
        ESP_LOGW(TAG, "On-demand key load: mount helper failed, trying direct directory access");
    }
#endif

    const char *active_keys_dir = kSshKeysDir;
    DIR *dir = opendir(kSshKeysDir);
    if (dir == nullptr) {
        active_keys_dir = kSshKeysDirAlt;
        dir = opendir(kSshKeysDirAlt);
    }
    if (dir == nullptr) {
        ESP_LOGW(TAG, "On-demand key load skipped: unable to open %s or %s", kSshKeysDir, kSshKeysDirAlt);
        return 0;
    }

    int loaded = 0;
    struct dirent *entry = nullptr;
    while ((entry = readdir(dir)) != nullptr) {
        const std::string filename = entry->d_name;
        if (filename == "." || filename == ".." || is_probably_metadata_file(filename) || !has_pem_extension(filename)) {
            continue;
        }

        const std::string full_path = std::string(active_keys_dir) + "/" + filename;
        FILE *file = std::fopen(full_path.c_str(), "rb");
        if (file == nullptr) {
            continue;
        }
        if (std::fseek(file, 0, SEEK_END) != 0) {
            std::fclose(file);
            continue;
        }
        const long file_size = std::ftell(file);
        if (file_size <= 0 || std::fseek(file, 0, SEEK_SET) != 0) {
            std::fclose(file);
            continue;
        }

        std::string key_data(static_cast<size_t>(file_size), '\0');
        const size_t read_count = std::fread(key_data.data(), 1, static_cast<size_t>(file_size), file);
        std::fclose(file);
        if (read_count != static_cast<size_t>(file_size)) {
            continue;
        }

        terminal->load_key_from_memory(filename.c_str(), key_data.c_str(), key_data.size());
        loaded++;
    }
    closedir(dir);
    return loaded;
}

bool parse_short_83_name(const std::string &name, std::string *prefix, std::string *ext)
{
    if (prefix == nullptr || ext == nullptr) {
        return false;
    }

    const std::string lower = lowercase_ascii(base_name(name));
    const size_t dot = lower.rfind('.');
    const std::string stem = (dot == std::string::npos) ? lower : lower.substr(0, dot);
    if (stem.empty()) {
        return false;
    }
    const size_t tilde = stem.find('~');
    if (tilde == std::string::npos || tilde == 0 || tilde + 1 >= stem.size()) {
        return false;
    }

    for (size_t i = tilde + 1; i < stem.size(); ++i) {
        const char c = stem[i];
        if (!std::isdigit(static_cast<unsigned char>(c))) {
            return false;
        }
    }

    *prefix = stem.substr(0, tilde);
    *ext = (dot == std::string::npos) ? std::string() : lower.substr(dot);
    return !prefix->empty();
}

std::string normalize_key_stem(const std::string &filename)
{
    std::string stem = lowercase_ascii(base_name(filename));
    const size_t dot = stem.rfind('.');
    if (dot != std::string::npos) {
        stem = stem.substr(0, dot);
    }

    std::string normalized;
    normalized.reserve(stem.size());
    for (char c : stem) {
        if (std::isalnum(static_cast<unsigned char>(c)) != 0) {
            normalized.push_back(c);
        }
    }
    return normalized;
}

int score_identity_candidate(const std::string &desired_name, const std::string &candidate_name)
{
    if (desired_name.empty() || candidate_name.empty()) {
        return 0;
    }
    if (is_probably_metadata_file(candidate_name) || !has_pem_extension(candidate_name)) {
        return 0;
    }

    const std::string desired_lower = lowercase_ascii(base_name(desired_name));
    const std::string candidate_lower = lowercase_ascii(base_name(candidate_name));
    if (candidate_lower == desired_lower) {
        return 300;
    }

    const std::string target_stem = normalize_key_stem(desired_lower);
    const std::string candidate_stem = normalize_key_stem(candidate_lower);
    if (target_stem.empty() || candidate_stem.empty()) {
        return 0;
    }
    if (candidate_stem == target_stem) {
        return 250;
    }

    if (target_stem.rfind(candidate_stem, 0) == 0 || candidate_stem.rfind(target_stem, 0) == 0) {
        return 220;
    }

    const std::string candidate_short_prefix = short_name_prefix(candidate_lower);
    if (!candidate_short_prefix.empty() && target_stem.rfind(candidate_short_prefix, 0) == 0) {
        return 200;
    }

    return 0;
}

std::string resolve_identity_file_on_sd(const std::string &identity_path)
{
    const std::string desired_name = base_name(identity_path);
    if (desired_name.empty()) {
        return "";
    }

    struct Candidate {
        std::string path;
        int score = 0;
    };

    Candidate best = {};
    int pem_file_count = 0;
    std::string single_pem_path;

    const char *dirs[] = {kSshKeysDir, kSshKeysDirAlt, "/sdcard", "/sd"};
    for (const char *dir_path : dirs) {
        DIR *dir = opendir(dir_path);
        if (dir == nullptr) {
            continue;
        }

        struct dirent *entry = nullptr;
        while ((entry = readdir(dir)) != nullptr) {
            const std::string entry_name = entry->d_name;
            if (entry_name == "." || entry_name == "..") {
                continue;
            }
            if (!has_pem_extension(entry_name)) {
                continue;
            }

            const std::string candidate_path = std::string(dir_path) + "/" + entry_name;
            struct stat st = {};
            if (stat(candidate_path.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) {
                continue;
            }

            pem_file_count++;
            single_pem_path = candidate_path;

            const int score = score_identity_candidate(desired_name, entry_name);
            if (score > best.score || (score == best.score && !best.path.empty() && candidate_path < best.path) ||
                (score == best.score && best.path.empty())) {
                best.path = candidate_path;
                best.score = score;
            }
        }
        closedir(dir);
    }

    if (best.score > 0) {
        ESP_LOGI(TAG, "identity resolve: %s -> %s (score=%d)",
                 desired_name.c_str(),
                 best.path.c_str(),
                 best.score);
        return best.path;
    }
    if (pem_file_count == 1) {
        ESP_LOGI(TAG, "identity resolve: only one key present, using %s", single_pem_path.c_str());
        return single_pem_path;
    }
    ESP_LOGW(TAG, "identity resolve: no SD match for %s (pem candidates=%d)", desired_name.c_str(), pem_file_count);
    return "";
}

const char *find_loaded_key_with_fallback(SSHTerminal *terminal,
                                          const std::string &identity_path,
                                          std::string *resolved_key_name,
                                          size_t *resolved_len)
{
    if (terminal == nullptr) {
        return nullptr;
    }

    const std::string desired_name = base_name(identity_path);
    const char *direct = terminal->get_loaded_key(desired_name.c_str(), resolved_len);
    if (direct != nullptr) {
        if (resolved_key_name != nullptr) {
            *resolved_key_name = desired_name;
        }
        return direct;
    }

    const std::vector<std::string> key_names = terminal->get_loaded_key_names();
    if (key_names.empty()) {
        return nullptr;
    }

    const std::string target_stem = normalize_key_stem(desired_name);

    // FAT may expose only short 8.3 aliases (e.g. PRODMI~5.PEM). Prefer an
    // unambiguous stem match; otherwise fallback to single loaded key.
    std::string matched_name;
    int match_count = 0;
    for (const std::string &candidate : key_names) {
        const std::string candidate_stem = normalize_key_stem(candidate);
        if (candidate_stem.empty() || target_stem.empty()) {
            continue;
        }
        bool this_match = (candidate_stem == target_stem ||
                           target_stem.rfind(candidate_stem, 0) == 0 ||
                           candidate_stem.rfind(target_stem, 0) == 0);
        const std::string cand_short_prefix = short_name_prefix(candidate);
        if (!this_match && !cand_short_prefix.empty() && target_stem.rfind(cand_short_prefix, 0) == 0) {
            this_match = true;
        }
        if (this_match) {
            matched_name = candidate;
            match_count++;
        }
    }

    if (match_count == 1) {
        if (resolved_key_name != nullptr) {
            *resolved_key_name = matched_name;
        }
        return terminal->get_loaded_key(matched_name.c_str(), resolved_len);
    }

    std::string short_prefix;
    std::string short_ext;
    if (parse_short_83_name(desired_name, &short_prefix, &short_ext)) {
        std::string short_match;
        int short_match_count = 0;
        for (const std::string &candidate : key_names) {
            const std::string candidate_lower = lowercase_ascii(base_name(candidate));
            const size_t dot = candidate_lower.rfind('.');
            const std::string candidate_stem =
                (dot == std::string::npos) ? candidate_lower : candidate_lower.substr(0, dot);
            const std::string candidate_ext =
                (dot == std::string::npos) ? std::string() : candidate_lower.substr(dot);

            if (!short_ext.empty() && candidate_ext != short_ext) {
                continue;
            }
            if (candidate_stem.rfind(short_prefix, 0) == 0) {
                short_match = candidate;
                short_match_count++;
            }
        }

        if (short_match_count == 1) {
            if (resolved_key_name != nullptr) {
                *resolved_key_name = short_match;
            }
            return terminal->get_loaded_key(short_match.c_str(), resolved_len);
        }
    }

    if (key_names.size() == 1) {
        if (resolved_key_name != nullptr) {
            *resolved_key_name = key_names.front();
        }
        return terminal->get_loaded_key(key_names.front().c_str(), resolved_len);
    }

    return nullptr;
}

esp_err_t connect_wifi_profile(SSHTerminal *terminal, const WifiProfile &profile, const char *context, bool verbose_ui)
{
    if (terminal == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (profile.ssid.empty()) {
        if (verbose_ui) {
            terminal->append_text("ERROR: wifi_config profile missing SSID\n");
        } else {
            ESP_LOGW(TAG, "wifi profile missing SSID");
        }
        return ESP_FAIL;
    }

    if (verbose_ui && context != nullptr && context[0] != '\0') {
        terminal->append_text(context);
        terminal->append_text("\n");
    }

    if (verbose_ui) {
        terminal->append_text("WiFi profile connect: ");
        if (!profile.network_name.empty()) {
            terminal->append_text(profile.network_name.c_str());
            terminal->append_text(" -> ");
        }
        terminal->append_text(profile.ssid.c_str());
        terminal->append_text("\n");
    } else {
        ESP_LOGI(TAG, "wifi auto boot: trying profile '%s' ssid='%s'",
                 profile.network_name.empty() ? "<unnamed>" : profile.network_name.c_str(),
                 profile.ssid.c_str());
    }

    return terminal->init_wifi(profile.ssid.c_str(), profile.password.c_str());
}

bool auto_connect_wifi_profiles(SSHTerminal *terminal, bool verbose_ui = true)
{
    if (terminal == nullptr) {
        return false;
    }

    std::vector<WifiProfile> profiles;
    if (!parse_wifi_config_file(&profiles)) {
        if (verbose_ui) {
            terminal->append_text("wifi auto: no wifi_config found (/sdcard/ssh_keys/wifi_config, /sdcard/wifi_config, /sd/ssh_keys/wifi_config, or /sd/wifi_config)\n");
        } else {
            ESP_LOGI(TAG, "wifi auto boot: no wifi_config");
        }
        return false;
    }

    bool attempted = false;
    for (const auto &profile : profiles) {
        if (!profile.has_auto_connect || !profile.auto_connect) {
            continue;
        }
        attempted = true;
        if (connect_wifi_profile(terminal, profile, "wifi auto: trying profile", verbose_ui) == ESP_OK) {
            if (verbose_ui) {
                terminal->append_text("wifi auto: connected\n");
            } else {
                ESP_LOGI(TAG, "wifi auto boot: connected");
            }
            return true;
        }
        if (verbose_ui) {
            terminal->append_text("wifi auto: failed, trying next profile\n");
        } else {
            ESP_LOGI(TAG, "wifi auto boot: failed profile, trying next");
        }
    }

    if (!attempted) {
        if (verbose_ui) {
            terminal->append_text("wifi auto: no AutoConnect true profiles\n");
        } else {
            ESP_LOGI(TAG, "wifi auto boot: no AutoConnect=true profiles");
        }
    }
    return false;
}

bool connect_wifi_profile_by_name_or_ssid(SSHTerminal *terminal, const std::string &name_or_ssid)
{
    if (terminal == nullptr) {
        return false;
    }

    std::vector<WifiProfile> profiles;
    if (!parse_wifi_config_file(&profiles)) {
        terminal->append_text("No wifi_config found at /sdcard/ssh_keys/wifi_config, /sdcard/wifi_config, /sd/ssh_keys/wifi_config, or /sd/wifi_config\n");
        return false;
    }

    const WifiProfile *profile = find_wifi_profile(profiles, name_or_ssid);
    if (profile == nullptr) {
        terminal->append_text("No matching wifi profile for: ");
        terminal->append_text(name_or_ssid.c_str());
        terminal->append_text("\n");
        return false;
    }

    return connect_wifi_profile(terminal, *profile, nullptr, true) == ESP_OK;
}

bool connect_with_alias_identities(SSHTerminal *terminal, const ResolvedSSHConfig &resolved)
{
    if (terminal == nullptr) {
        return false;
    }

    const int loaded_now = load_keys_from_sd_if_needed(terminal);
    if (loaded_now > 0) {
        char line[96];
        std::snprintf(line, sizeof(line), "Loaded %d key(s) from SD on demand\n", loaded_now);
        terminal->append_text(line);
    }

    const std::vector<std::string> loaded_key_names = terminal->get_loaded_key_names();
    ESP_LOGI(TAG, "alias connect: alias=%s host=%s user=%s identities=%d loaded_keys=%d",
             resolved.alias.c_str(),
             resolved.host_name.c_str(),
             resolved.user.c_str(),
             static_cast<int>(resolved.identity_files.size()),
             static_cast<int>(loaded_key_names.size()));
    for (const auto &loaded_name : loaded_key_names) {
        ESP_LOGI(TAG, "alias connect: loaded key name=%s", loaded_name.c_str());
    }

    bool attempted_identity = false;
    bool connected = false;
    for (const std::string &identity_path : resolved.identity_files) {
        std::string key_name = base_name(identity_path);
        size_t key_len = 0;
        const char *loaded_key = find_loaded_key_with_fallback(terminal, identity_path, &key_name, &key_len);
        ESP_LOGI(TAG, "alias connect: trying identity=%s", identity_path.c_str());

        terminal->append_text("Trying identity: ");
        terminal->append_text(identity_path.c_str());
        terminal->append_text("\n");

        attempted_identity = true;
        if (loaded_key != nullptr && key_len > 0) {
            ESP_LOGI(TAG, "alias connect: using loaded key=%s len=%d",
                     key_name.c_str(),
                     static_cast<int>(key_len));
            if (key_name != base_name(identity_path)) {
                terminal->append_text("  Using loaded key alias: ");
                terminal->append_text(key_name.c_str());
                terminal->append_text("\n");
            }
            if (terminal->connect_with_key(resolved.host_name.c_str(),
                                           resolved.port,
                                           resolved.user.c_str(),
                                           loaded_key,
                                           key_len) == ESP_OK) {
                connected = true;
                break;
            }
            continue;
        }

        std::string key_data;
        std::string resolved_path;
        bool key_read_ok = false;
        const std::vector<std::string> candidates = identity_path_candidates(identity_path);
        for (const auto &candidate : candidates) {
            ESP_LOGI(TAG, "alias connect: trying key file candidate=%s", candidate.c_str());
            if (read_file_contents(candidate, &key_data)) {
                resolved_path = candidate;
                key_read_ok = true;
                ESP_LOGI(TAG, "alias connect: read key file candidate=%s", candidate.c_str());
                break;
            }
        }
        if (!key_read_ok) {
            const std::string resolved_sd_path = resolve_identity_file_on_sd(identity_path);
            if (!resolved_sd_path.empty()) {
                ESP_LOGI(TAG, "alias connect: trying resolved SD key path=%s", resolved_sd_path.c_str());
            }
            if (!resolved_sd_path.empty() && read_file_contents(resolved_sd_path, &key_data)) {
                resolved_path = resolved_sd_path;
                key_read_ok = true;
                ESP_LOGI(TAG, "alias connect: read resolved SD key path=%s", resolved_sd_path.c_str());
            }
        }

        if (!key_read_ok) {
            ESP_LOGW(TAG, "alias connect: unable to read any key file for identity=%s", identity_path.c_str());
            terminal->append_text("  Skipping: unable to read key file\n");
            continue;
        }
        if (!resolved_path.empty() && resolved_path != identity_path) {
            terminal->append_text("  Using key file path: ");
            terminal->append_text(resolved_path.c_str());
            terminal->append_text("\n");
        }

        if (terminal->connect_with_key(resolved.host_name.c_str(),
                                       resolved.port,
                                       resolved.user.c_str(),
                                       key_data.c_str(),
                                       key_data.size()) == ESP_OK) {
            connected = true;
            break;
        }
    }

    if (!connected) {
        if (!attempted_identity) {
            terminal->append_text("ERROR: Alias has no IdentityFile entries\n");
            terminal->append_text("Add IdentityFile in ssh_config or use ssh/sshkey command directly.\n");
        } else {
            terminal->append_text("ERROR: All configured identity files failed\n");
        }
    }

    return connected;
}

void connect_using_ssh_alias(SSHTerminal *terminal, const std::string &alias)
{
    if (terminal == nullptr || alias.empty()) {
        return;
    }

    ResolvedSSHConfig resolved = {};
    if (!resolve_ssh_alias(alias, &resolved)) {
        terminal->append_text("ERROR: Host alias not found in /sdcard/ssh_keys/ssh_config or /sd/ssh_keys/ssh_config\n");
        terminal->append_text("Hint: run 'hosts' to list available aliases.\n");
        return;
    }

    if (resolved.user.empty()) {
        terminal->append_text("ERROR: ssh_config alias missing User directive\n");
        return;
    }

    char resolved_line[196];
    std::snprintf(resolved_line, sizeof(resolved_line),
                  "Resolved %s -> %s:%d as %s\n",
                  resolved.alias.c_str(),
                  resolved.host_name.c_str(),
                  resolved.port,
                  resolved.user.c_str());
    terminal->append_text(resolved_line);

    if (!terminal->is_wifi_connected()) {
        if (!resolved.network.empty()) {
            terminal->append_text("WiFi is disconnected; alias requests network profile: ");
            terminal->append_text(resolved.network.c_str());
            terminal->append_text("\n");
            if (!connect_wifi_profile_by_name_or_ssid(terminal, resolved.network)) {
                terminal->append_text("ERROR: failed to connect required WiFi profile\n");
                return;
            }
        } else {
            terminal->append_text("ERROR: WiFi not connected\n");
            terminal->append_text("Use: connect <SSID> <PASSWORD> or configure 'Network' in ssh_config alias\n");
            return;
        }
    }

    if (connect_with_alias_identities(terminal, resolved)) {
        return;
    }

    if (!resolved.network.empty() && terminal->is_wifi_connected()) {
        terminal->append_text("Retrying after reconnecting alias network profile...\n");
        if (connect_wifi_profile_by_name_or_ssid(terminal, resolved.network)) {
            (void)connect_with_alias_identities(terminal, resolved);
        }
    }
}
} // namespace

namespace {
void print_sta_netinfo(SSHTerminal *terminal)
{
    if (terminal == nullptr) {
        return;
    }

    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta == nullptr) {
        ESP_LOGW(TAG, "netinfo: STA netif not initialized");
        terminal->append_text("netinfo: STA netif not initialized\n");
        return;
    }

    esp_netif_ip_info_t ip_info = {};
    esp_err_t rc = esp_netif_get_ip_info(sta, &ip_info);
    if (rc != ESP_OK) {
        ESP_LOGW(TAG, "netinfo: failed to read IP info (%s)", esp_err_to_name(rc));
        terminal->append_text("netinfo: failed to read IP info\n");
        return;
    }

    char line[96];
    std::snprintf(line, sizeof(line), "IP      : " IPSTR "\n", IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "netinfo %s", line);
    terminal->append_text(line);
    std::snprintf(line, sizeof(line), "Netmask : " IPSTR "\n", IP2STR(&ip_info.netmask));
    ESP_LOGI(TAG, "netinfo %s", line);
    terminal->append_text(line);
    std::snprintf(line, sizeof(line), "Gateway : " IPSTR "\n", IP2STR(&ip_info.gw));
    ESP_LOGI(TAG, "netinfo %s", line);
    terminal->append_text(line);
}
} // namespace

SSHTerminal::SSHTerminal() 
    : terminal_screen(NULL), 
      terminal_output(NULL), 
      input_label(NULL),
      status_bar(NULL),
      byte_counter_label(NULL),
      side_panel(NULL),
      cursor_pos(0),
      bytes_received(0),
      history_index(-1),
      cursor_blink_timer(NULL),
      cursor_visible(true),
      battery_update_timer(NULL),
      debug_metrics_timer(NULL),
      history_needs_save(false),
      history_save_timer(NULL),
      last_display_update(0),
      wifi_connected(false),
      boot_wifi_auto_connect_attempted(false),
      ssh_connected(false),
      battery_initialized(false),
      serial_rx_in_progress(false),
      flash_headroom_percent(-1),
      ssh_socket(-1),
      session(NULL),
      channel(NULL),
      hostname(NULL),
      port_number(22),
      connected_wifi_ssid(""),
      connected_ssh_host(""),
      terminal_font_big(false),
      touch_scrub_active(false),
      touch_scrub_moved(false),
      touch_scrub_axis_locked(false),
      touch_scrub_vertical_mode(false),
      touch_scrub_last_x(0),
      touch_scrub_last_y(0),
      touch_scrub_accum_x(0)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "Initializing battery measurement...");
    esp_err_t battery_ret = battery.init();
    if (battery_ret == ESP_OK) {
        battery_initialized = true;
        ESP_LOGI(TAG, "Battery measurement initialized successfully");
        float test_voltage = battery.readBatteryVoltage();
        ESP_LOGI(TAG, "Test battery read: %.2fV", test_voltage);
    } else {
        battery_initialized = false;
        ESP_LOGE(TAG, "Battery measurement initialization FAILED: %s", esp_err_to_name(battery_ret));
    }
    
    load_history_from_nvs();
}

SSHTerminal::~SSHTerminal() 
{
    disconnect();
    if (hostname) {
        free(hostname);
    }
    if (cursor_blink_timer) {
        lv_timer_del(cursor_blink_timer);
    }
    if (battery_update_timer) {
        lv_timer_del(battery_update_timer);
    }
    if (debug_metrics_timer) {
        lv_timer_del(debug_metrics_timer);
    }
    if (history_save_timer) {
        lv_timer_del(history_save_timer);
        if (history_needs_save) {
            save_history_to_nvs();
        }
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    SSHTerminal* terminal = (SSHTerminal*)arg;
    
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"Connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        
        s_retry_num = 0;
        ESP_LOGI(TAG, "Setting WIFI_CONNECTED_BIT in event group %p", s_wifi_event_group);
        EventBits_t result = xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Event group bits after setting: 0x%x", result);
        (void)terminal;
    }
}

esp_err_t SSHTerminal::init_wifi(const char* ssid, const char* password)
{
    ESP_LOGI(TAG, "Initializing WiFi...");
    
    static bool wifi_initialized = false;
    if (wifi_initialized) {
        ESP_LOGI(TAG, "Cleaning up previous WiFi instance...");
        
        if (s_instance_any_id) {
            esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, s_instance_any_id);
            s_instance_any_id = NULL;
        }
        if (s_instance_got_ip) {
            esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, s_instance_got_ip);
            s_instance_got_ip = NULL;
        }
        
        esp_wifi_stop();
        esp_wifi_deinit();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    s_retry_num = 0;
    
    if (!s_wifi_event_group) {
        s_wifi_event_group = xEventGroupCreate();
    }
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    
    if (!wifi_initialized) {
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_sta();
        wifi_initialized = true;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    // Keep WiFi credentials ephemeral; profiles are sourced from SD config at runtime.
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        this,
                                                        &s_instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        this,
                                                        &s_instance_got_ip));

    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi init finished. Event group: %p", s_wifi_event_group);

    const int max_wait_ms = 15000;
    const int check_interval_ms = 500;
    int elapsed_ms = 0;
    
    while (elapsed_ms < max_wait_ms) {
        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                pdFALSE,
                pdFALSE,
                pdMS_TO_TICKS(check_interval_ms));
        
        if (elapsed_ms % 5000 == 0) {
            ESP_LOGI(TAG, "Waiting... bits: 0x%x, elapsed: %d ms", bits, elapsed_ms);
        }

        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "Connected to AP SSID:%s", ssid);
            wifi_connected = true;
            connected_wifi_ssid = (ssid != nullptr) ? ssid : "";
            
            if (display_lock(0)) {
                update_status_bar();
                append_text("WiFi Connected\n");
                print_sta_netinfo(this);
                display_unlock();
            }
            return ESP_OK;
        } else if (bits & WIFI_FAIL_BIT) {
            ESP_LOGI(TAG, "Failed to connect to SSID:%s", ssid);
            wifi_connected = false;
            connected_wifi_ssid.clear();
            
            if (display_lock(0)) {
                update_status_bar();
                display_unlock();
            }
            return ESP_FAIL;
        }
        
        if (display_lock(0)) {
            append_text(".");
            display_unlock();
        }
        elapsed_ms += check_interval_ms;
    }
    
    ESP_LOGE(TAG, "Connection timeout");
    wifi_connected = false;
    connected_wifi_ssid.clear();
    s_retry_num = 0;
    
    if (display_lock(0)) {
        update_status_bar();
        display_unlock();
    }
    return ESP_FAIL;
}

bool SSHTerminal::is_wifi_connected()
{
    return wifi_connected;
}

bool SSHTerminal::is_serial_rx_in_progress() const
{
    return serial_rx_in_progress.load();
}

void SSHTerminal::set_serial_rx_in_progress(bool in_progress)
{
    serial_rx_in_progress.store(in_progress);
}

void SSHTerminal::try_boot_wifi_auto_connect()
{
    if (boot_wifi_auto_connect_attempted) {
        return;
    }
    boot_wifi_auto_connect_attempted = true;

    if (wifi_connected) {
        ESP_LOGI(TAG, "wifi auto boot: already connected");
        return;
    }

    ESP_LOGI(TAG, "wifi auto boot: attempt");
    if (!auto_connect_wifi_profiles(this, false)) {
        ESP_LOGI(TAG, "wifi auto boot: skipped/failed");
    }
}

lv_obj_t* SSHTerminal::create_terminal_screen()
{
    // Keep a minimal side inset so the 1px border is fully visible on panel edges
    // while maximizing horizontal character columns on the T-Pager display.
    constexpr int kTPagerHorizontalInsetPx = 1;

    terminal_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(terminal_screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(terminal_screen, LV_OPA_COVER, 0);
    lv_obj_clear_flag(terminal_screen, LV_OBJ_FLAG_SCROLLABLE);

    status_bar = lv_label_create(terminal_screen);
    lv_label_set_text(status_bar, "Status: Disconnected");
    #if defined(TPAGER_TARGET)
    lv_obj_set_style_text_color(status_bar, lv_color_hex(0xD9F2E6), 0);
    #else
    lv_obj_set_style_text_color(status_bar, lv_color_hex(0x00FF00), 0);
    #endif
    lv_obj_set_style_text_font(status_bar, ui_font_body(), 0);
    #if defined(TPAGER_TARGET)
    lv_obj_align(status_bar, LV_ALIGN_TOP_LEFT, 4, 2);
    #else
    lv_obj_align(status_bar, LV_ALIGN_TOP_LEFT, 5, 5);
    #endif
    
    byte_counter_label = lv_label_create(terminal_screen);
    lv_label_set_text(byte_counter_label, "S-- P-- F--");
    #if defined(TPAGER_TARGET)
    lv_obj_set_style_text_color(byte_counter_label, lv_color_hex(0xAEE6FF), 0);
    #else
    lv_obj_set_style_text_color(byte_counter_label, lv_color_hex(0x00FFFF), 0);
    #endif
    lv_obj_set_style_text_font(byte_counter_label, ui_font_small(), 0);
    #if defined(TPAGER_TARGET)
    lv_obj_align(byte_counter_label, LV_ALIGN_TOP_RIGHT, -4, 2);
    #else
    lv_obj_align(byte_counter_label, LV_ALIGN_TOP_RIGHT, -5, 5);
    #endif

    terminal_output = lv_textarea_create(terminal_screen);
    #if defined(TPAGER_TARGET)
    lv_obj_set_size(terminal_output, lv_pct(100) - (kTPagerHorizontalInsetPx * 2), lv_pct(76));
    lv_obj_align(terminal_output, LV_ALIGN_TOP_MID, 0, 18);
    #else
    lv_obj_set_size(terminal_output, lv_pct(100), lv_pct(75));
    lv_obj_align(terminal_output, LV_ALIGN_TOP_MID, 0, 25);
    #endif
    lv_obj_set_style_bg_color(terminal_output, lv_color_black(), 0);
    #if defined(TPAGER_TARGET)
    lv_obj_set_style_text_color(terminal_output, lv_color_hex(0xF7FFF9), 0);
    #else
    lv_obj_set_style_text_color(terminal_output, lv_color_hex(0x00FF00), 0);
    #endif
    lv_obj_set_style_text_font(terminal_output, ui_font_small(), 0);
    #if defined(TPAGER_TARGET)
    lv_obj_set_style_border_color(terminal_output, lv_color_hex(0x48A878), 0);
    lv_obj_set_style_border_width(terminal_output, 1, 0);
    #else
    lv_obj_set_style_border_color(terminal_output, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_border_width(terminal_output, 2, 0);
    #endif
    lv_textarea_set_cursor_click_pos(terminal_output, false);
    lv_textarea_set_one_line(terminal_output, false);
    lv_obj_set_scrollbar_mode(terminal_output, LV_SCROLLBAR_MODE_OFF);
    
    lv_obj_clear_flag(terminal_output, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_obj_set_style_anim_time(terminal_output, 0, LV_PART_CURSOR);
    lv_obj_set_style_opa(terminal_output, LV_OPA_TRANSP, LV_PART_CURSOR);
    
    lv_obj_set_scroll_snap_x(terminal_output, LV_SCROLL_SNAP_NONE);
    lv_obj_set_scroll_snap_y(terminal_output, LV_SCROLL_SNAP_NONE);
    lv_obj_clear_flag(terminal_output, LV_OBJ_FLAG_SCROLL_MOMENTUM);
    lv_obj_clear_flag(terminal_output, LV_OBJ_FLAG_SCROLL_ELASTIC);

    lv_obj_t* input_container = lv_obj_create(terminal_screen);
    #if defined(TPAGER_TARGET)
    lv_obj_set_size(input_container, lv_pct(100) - (kTPagerHorizontalInsetPx * 2), 22);
    #else
    lv_obj_set_size(input_container, lv_pct(100) - 10, 25);
    #endif
    lv_obj_set_style_bg_opa(input_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(input_container, 0, 0);
    lv_obj_set_style_pad_all(input_container, 0, 0);
    lv_obj_set_scrollbar_mode(input_container, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(input_container, LV_DIR_HOR);
    #if defined(TPAGER_TARGET)
    lv_obj_align(input_container, LV_ALIGN_BOTTOM_MID, 0, -2);
    #else
    lv_obj_align(input_container, LV_ALIGN_BOTTOM_LEFT, 5, -5);
    #endif
    
    input_label = lv_label_create(input_container);
    lv_label_set_text(input_label, "> ");
    #if defined(TPAGER_TARGET)
    lv_obj_set_style_text_color(input_label, lv_color_hex(0xFFE9A8), 0);
    #else
    lv_obj_set_style_text_color(input_label, lv_color_hex(0xFFFF00), 0);
    #endif
    lv_obj_set_style_text_font(input_label, ui_font_body(), 0);
    lv_label_set_long_mode(input_label, LV_LABEL_LONG_CLIP);
    lv_obj_align(input_label, LV_ALIGN_LEFT_MID, 0, 0);
    
    // Touch contract:
    // - tap positions the cursor
    // - horizontal drag ("scrub") emits repeated left/right cursor moves
    lv_obj_add_flag(input_label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(input_label, input_touch_event_cb, LV_EVENT_PRESSED, this);
    lv_obj_add_event_cb(input_label, input_touch_event_cb, LV_EVENT_PRESSING, this);
    lv_obj_add_event_cb(input_label, input_touch_event_cb, LV_EVENT_RELEASED, this);
    lv_obj_add_event_cb(input_label, input_touch_event_cb, LV_EVENT_CLICKED, this);

    create_side_panel();
    
    lv_obj_add_event_cb(terminal_screen, gesture_event_cb, LV_EVENT_GESTURE, this);
    lv_obj_clear_flag(terminal_screen, LV_OBJ_FLAG_GESTURE_BUBBLE);
    
    cursor_blink_timer = lv_timer_create(cursor_blink_cb, 500, this);
    
    battery_update_timer = lv_timer_create(battery_update_cb, 60000, this);
    debug_metrics_timer = lv_timer_create(debug_metrics_cb, 1000, this);

    load_default_terminal_font_mode_from_config();
    apply_terminal_font_mode();
    update_status_bar();
    update_debug_metrics();
    
    history_save_timer = lv_timer_create(history_save_cb, 5000, this);

    #if defined(TPAGER_TARGET)
    const char* logo =
        "PocketSSH T-Pager\n"
        "Type 'help' for commands.\n"
        "Start with: wifi auto, hosts, or connect <alias>\n\n";
    #else
    const char* logo = 
        "\n"
        "  ================================================\n"
        "           POCKET SSH TERM - ESP32-S3\n"
        "  ================================================\n"
        "\n"
        "  Commands:\n"
        "   wifi - List configured WiFi profiles\n"
        "   wifi <Network|SSID> - Connect from wifi_config\n"
        "   wifi auto - Try AutoConnect profiles in file order\n"
        "   connect <ALIAS> - Resolve via ssh_config and SSH key\n"
        "   connect <SSID> <PASSWORD>  - WiFi connect\n"
        "   fontsize [big|normal] - Toggle/set font size\n"
        "     Use quotes for spaces: connect \"My WiFi\" \"my pass\"\n"
        "   hosts - List aliases from /sdcard/ssh_keys/ssh_config or /sd/ssh_keys/ssh_config\n"
        "   ssh <HOST> <PORT> <USER> <PASS> - SSH\n"
        "   sshkey <HOST> <PORT> <USER> <KEYFILE> - SSH key\n"
        "   disconnect - WiFi off | exit - SSH off\n"
        "   clear - Clear screen | help - Show help\n"
        "\n"
        "  Ready. Type 'connect' to start...\n\n";
    #endif
    
    lv_textarea_set_text(terminal_output, logo);

    return terminal_screen;
}

void SSHTerminal::append_text(const char* text)
{
    if (!terminal_output || !text) {
        return;
    }
    
    int64_t start_time = esp_timer_get_time() / 1000;
    
    const char* current_text = lv_textarea_get_text(terminal_output);
    size_t current_len = current_text ? strlen(current_text) : 0;
    size_t new_len = strlen(text);
    
    if (new_len > kTerminalAppendChunkBytes) {
        text = text + (new_len - kTerminalAppendChunkBytes);
        new_len = kTerminalAppendChunkBytes;
    }

    const size_t projected_len = current_len + new_len;
    if (projected_len > kTerminalScrollbackBytes && current_len > 0) {
        const size_t keep_len = (new_len >= kTerminalScrollbackBytes) ? 0 : (kTerminalScrollbackBytes - new_len);
        if (keep_len == 0) {
            lv_textarea_set_text(terminal_output, "");
            current_len = 0;
        } else if (current_len > keep_len) {
            std::string keep_copy(current_text + (current_len - keep_len), keep_len);
            lv_textarea_set_text(terminal_output, keep_copy.c_str());
            current_len = keep_copy.size();
        }
    }

    if (current_len + new_len <= kTerminalScrollbackBytes) {
        lv_textarea_add_text(terminal_output, text);
    }
    
    int64_t total_time = (esp_timer_get_time() / 1000) - start_time;
    if (total_time > 1000) {
        ESP_LOGW(TAG, "append_text took %lld ms - LVGL heap may be fragmented", total_time);
    }
}

void SSHTerminal::clear_terminal()
{
    if (terminal_output) {
        lv_textarea_set_text(terminal_output, "");
    }
}

void SSHTerminal::handle_key_input(char key)
{
    if (key == '\n' || key == '\r') {
        if (!current_input.empty()) {
            append_text("\n> ");
            append_text(current_input.c_str());
            append_text("\n");
            
            if (current_input.rfind("connect ", 0) == 0) {
                const std::vector<std::string> args = split_quoted_arguments(current_input, 8);

                if (args.size() == 1) {
                    connect_using_ssh_alias(this, args[0]);
                } else if (args.size() >= 2) {
                    std::string ssid = args[0];
                    std::string password = args[1];
                    
                    append_text("Connecting to WiFi: ");
                    append_text(ssid.c_str());
                    append_text("\n");
                    
                    if (init_wifi(ssid.c_str(), password.c_str()) == ESP_OK) {
                        append_text("WiFi connected successfully!\n");
                    } else {
                        append_text("WiFi connection failed!\n");
                    }
                } else {
                    append_text("Usage:\n");
                    append_text("  connect <ALIAS>\n");
                    append_text("  connect <SSID> <PASSWORD>\n");
                    append_text("  Use quotes for SSIDs/passwords with spaces: connect \"My WiFi\" password\n");
                }
            }
            else if (current_input.rfind("wifi", 0) == 0) {
                const std::vector<std::string> args = split_quoted_arguments(current_input, 4);
                if (args.empty()) {
                    std::vector<WifiProfile> profiles;
                    if (!parse_wifi_config_file(&profiles)) {
                        append_text("No wifi_config found at /sdcard/ssh_keys/wifi_config, /sdcard/wifi_config, /sd/ssh_keys/wifi_config, or /sd/wifi_config\n");
                    } else if (profiles.empty()) {
                        append_text("No WiFi profiles found in wifi_config\n");
                    } else {
                        append_text("Configured WiFi profiles:\n");
                        for (const auto &profile : profiles) {
                            append_text("  ");
                            if (!profile.network_name.empty()) {
                                append_text(profile.network_name.c_str());
                            } else {
                                append_text("<unnamed>");
                            }
                            append_text(" (SSID: ");
                            append_text(profile.ssid.empty() ? "<missing>" : profile.ssid.c_str());
                            append_text(")");
                            if (profile.has_auto_connect && profile.auto_connect) {
                                append_text(" [auto]");
                            }
                            append_text("\n");
                        }
                    }
                } else if (lowercase_ascii(args[0]) == "auto") {
                    (void)auto_connect_wifi_profiles(this);
                } else {
                    if (connect_wifi_profile_by_name_or_ssid(this, args[0])) {
                        append_text("WiFi connected via profile\n");
                    } else {
                        append_text("WiFi profile connection failed\n");
                    }
                }
            }
            else if (current_input == "sdcheck") {
                append_sd_probe(this);
            }
            else if (current_input.rfind("serialrx", 0) == 0) {
                if (ssh_connected) {
                    append_text("serialrx unavailable during active SSH session\n");
                } else {
                    const std::vector<std::string> args = split_quoted_arguments(current_input, 8);
                    const std::string target_name = args.empty() ? kDefaultSerialRxFilename : args[0];
                    if (!serial_receive_to_sd_file(this, target_name)) {
                        append_text("serialrx: failed\n");
                    }
                }
            }
            else if (current_input.rfind("ssh ", 0) == 0) {
                std::vector<std::string> parts = split_nonempty_whitespace(current_input);
                
                if (parts.size() == 2) {
                    connect_using_ssh_alias(this, parts[1]);
                } else if (parts.size() >= 5) {
                    std::string host = parts[1];
                    int port = std::atoi(parts[2].c_str());
                    std::string user = parts[3];
                    std::string pass = parts[4];

                    if (port <= 0 || port > 65535) {
                        append_text("ERROR: Invalid port for ssh command\n");
                    } else {
                        connect(host.c_str(), port, user.c_str(), pass.c_str());
                    }
                } else {
                    append_text("Usage: ssh <ALIAS>\n");
                    append_text("Usage: ssh <HOST> <PORT> <USER> <PASS>\n");
                }
            }
            else if (current_input.rfind("sshkey ", 0) == 0) {
                std::vector<std::string> parts = split_nonempty_whitespace(current_input);
                
                if (parts.size() >= 5) {
                    std::string host = parts[1];
                    int port = std::atoi(parts[2].c_str());
                    std::string user = parts[3];
                    std::string keyfile = parts[4];

                    if (port <= 0 || port > 65535) {
                        append_text("ERROR: Invalid port for sshkey command\n");
                    } else {
                        // Try to load key from memory
                        size_t key_len = 0;
                        const char* key_data = get_loaded_key(keyfile.c_str(), &key_len);

                        if (key_data && key_len > 0) {
                            append_text("Using key file: ");
                            append_text(keyfile.c_str());
                            append_text("\n");
                            connect_with_key(host.c_str(), port, user.c_str(), key_data, key_len);
                        } else {
                            append_text("ERROR: Key file not found: ");
                            append_text(keyfile.c_str());
                            append_text("\n");
                            append_text("Available keys: ");
                            for (const auto& kv : loaded_keys) {
                                append_text(kv.first.c_str());
                                append_text(" ");
                            }
                            append_text("\n");
                        }
                    }
                } else {
                    append_text("Usage: sshkey <HOST> <PORT> <USER> <KEYFILE>\n");
                    append_text("  Example: sshkey 192.168.1.100 22 pi default.pem\n");
                }
            }
            else if (current_input == "disconnect") {
                if (wifi_connected) {
                    append_text("Disconnecting WiFi...\n");
                    s_retry_num = WIFI_MAXIMUM_RETRY;
                    esp_wifi_disconnect();
                    wifi_connected = false;
                    connected_wifi_ssid.clear();
                    update_status_bar();
                    append_text("WiFi disconnected\n");
                } else {
                    append_text("WiFi not connected\n");
                }
            }
            else if (current_input.rfind("fontsize", 0) == 0) {
                if (ssh_connected) {
                    append_text("fontsize is only available when not in an SSH session\n");
                } else {
                    const std::vector<std::string> args = split_quoted_arguments(current_input, 8);
                    if (args.empty()) {
                        set_terminal_font_mode(!terminal_font_big, true);
                    } else {
                        bool parsed_big = false;
                        if (parse_fontsize_token(args[0], &parsed_big)) {
                            set_terminal_font_mode(parsed_big, true);
                        } else {
                            append_text("Usage: fontsize [big|normal]\n");
                        }
                    }
                }
            }
            else if (current_input == "exit") {
                disconnect();
            }
            else if (current_input == "clear") {
                clear_terminal();
            }
            else if (current_input == "help") {
                append_text("Available commands:\n");
                append_text("  wifi - List configured WiFi profiles\n");
                append_text("  wifi <Network|SSID> - Connect using wifi_config\n");
                append_text("  wifi auto - Try AutoConnect profiles\n");
                append_text("  hosts - List aliases from /sdcard/ssh_keys/ssh_config or /sd/ssh_keys/ssh_config\n");
                append_text("  connect <ALIAS> - Resolve alias from ssh_config and connect via key\n");
                append_text("  connect <SSID> <PASSWORD> - Connect to WiFi\n");
                append_text("    Use quotes for spaces: connect \"My WiFi\" password\n");
                append_text("  netinfo - Show WiFi IP/netmask/gateway\n");
                append_text("  sdcheck - Probe SD mountpoints and config visibility\n");
                append_text("  serialrx [filename] - Receive file into SD root (default: PocketSSH-TPager.bin)\n");
                append_text("    Protocol: BEGIN <size> <crc32hex>, DATA <hex>, END\n");
                append_text("  ssh <ALIAS> - Resolve alias from ssh_config and connect via key\n");
                append_text("  ssh <HOST> <PORT> <USER> <PASS> - Connect via SSH\n");
                append_text("  sshkey <HOST> <PORT> <USER> <KEYFILE> - Connect via SSH with private key\n");
                append_text("    Note: Place .pem keys in /sdcard/ssh_keys/ or /sd/ssh_keys/\n");
                append_text("  shutdown | poweroff - Deep sleep (wake via BOOT or encoder button)\n");
                append_text("  disconnect - Disconnect WiFi\n");
                append_text("  fontsize - Toggle terminal font size (not during SSH)\n");
                append_text("  fontsize big|normal - Set terminal font size\n");
                append_text("  exit - Disconnect SSH\n");
                append_text("  clear - Clear terminal\n");
                append_text("  help - Show this help\n");
            }
            else if (current_input == "shutdown" || current_input == "poweroff") {
#if defined(TPAGER_TARGET)
                append_text("Shutting down. Wake with BOOT or encoder button.\n");
                tpager_request_shutdown();
#else
                append_text("Shutdown is only supported on TPAGER target builds\n");
#endif
            }
            else if (current_input == "hosts") {
                SSHConfigFile parsed = {};
                if (!parse_ssh_config_file(&parsed)) {
                    append_text("No ssh_config found at /sdcard/ssh_keys/ssh_config or /sd/ssh_keys/ssh_config\n");
                } else if (parsed.aliases.empty()) {
                    append_text("No explicit Host aliases found in ssh_config\n");
                } else {
                    append_text("Configured Host aliases:\n");
                    for (const std::string &alias : parsed.aliases) {
                        append_text("  ");
                        append_text(alias.c_str());
                        append_text("\n");
                    }
                }
            }
            else if (current_input == "netinfo") {
                if (!wifi_connected) {
                    append_text("WiFi not connected\n");
                } else {
                    print_sta_netinfo(this);
                }
            }
            else if (ssh_connected) {
                send_command(current_input.c_str());
            } else {
                append_text("Unknown command. Type 'help' for commands.\n");
            }
            
            auto it = std::find(command_history.begin(), command_history.end(), current_input);
            if (it != command_history.end()) {
                command_history.erase(it);
            }
            command_history.push_back(current_input);
            history_needs_save = true;
            current_input.clear();
            cursor_pos = 0;
            history_index = -1;
        }
    } else if (key == 8 || key == 127) {
        // Backspace - delete character before cursor
        if (cursor_pos > 0 && !current_input.empty()) {
            current_input.erase(cursor_pos - 1, 1);
            cursor_pos--;
        }
    } else if (key >= 32 && key <= 126) {
        // Insert character at cursor position
        current_input.insert(cursor_pos, 1, key);
        cursor_pos++;
    }
    
    update_input_display();
}

void SSHTerminal::update_input_display()
{
    if (!input_label) {
        return;
    }
    
    // Ensure cursor_pos is within bounds
    if (cursor_pos > current_input.length()) {
        cursor_pos = current_input.length();
    }
    
    std::string full_text = "> " + current_input;
    
    // Insert cursor at correct position
    if (cursor_visible) {
        size_t display_pos = 2 + cursor_pos; // 2 = length of "> "
        full_text.insert(display_pos, "|");
    }
    
    lv_label_set_text(input_label, full_text.c_str());
    
    lv_obj_t* container = lv_obj_get_parent(input_label);
    if (container) {
        lv_obj_scroll_to_x(container, LV_COORD_MAX, LV_ANIM_OFF);
    }
}

void SSHTerminal::input_touch_event_cb(lv_event_t* e)
{
    SSHTerminal* terminal = (SSHTerminal*)lv_event_get_user_data(e);
    lv_obj_t* input_label = (lv_obj_t* )lv_event_get_target(e);
    const lv_event_code_t code = lv_event_get_code(e);
    
    if (!terminal || !input_label) {
        return;
    }

    lv_indev_t* indev = lv_indev_get_act();
    if (indev == nullptr) {
        return;
    }

    lv_point_t point;
    lv_indev_get_point(indev, &point);

    if (code == LV_EVENT_PRESSED) {
        terminal->touch_scrub_active = true;
        terminal->touch_scrub_moved = false;
        terminal->touch_scrub_axis_locked = false;
        terminal->touch_scrub_vertical_mode = false;
        terminal->touch_scrub_last_x = point.x;
        terminal->touch_scrub_last_y = point.y;
        terminal->touch_scrub_accum_x = 0;
        return;
    }

    if (code == LV_EVENT_PRESSING) {
        if (!terminal->touch_scrub_active) {
            return;
        }
        constexpr int32_t kAxisLockThresholdPx = 4;
        constexpr int32_t kHorizontalStepPx = 12;
        const int32_t delta_x = point.x - terminal->touch_scrub_last_x;
        const int32_t delta_y = point.y - terminal->touch_scrub_last_y;
        terminal->touch_scrub_last_x = point.x;
        terminal->touch_scrub_last_y = point.y;

        if (!terminal->touch_scrub_axis_locked &&
            (std::abs(delta_x) >= kAxisLockThresholdPx || std::abs(delta_y) >= kAxisLockThresholdPx)) {
            terminal->touch_scrub_axis_locked = true;
            terminal->touch_scrub_vertical_mode = (std::abs(delta_y) > std::abs(delta_x));
        }

        if (!terminal->touch_scrub_axis_locked) {
            return;
        }

        if (terminal->touch_scrub_vertical_mode) {
            if (terminal->terminal_output != nullptr && delta_y != 0) {
                lv_obj_scroll_by(terminal->terminal_output, 0, -delta_y, LV_ANIM_OFF);
                terminal->touch_scrub_moved = true;
            }
            return;
        }

        terminal->touch_scrub_accum_x += delta_x;
        while (terminal->touch_scrub_accum_x >= kHorizontalStepPx) {
            terminal->move_cursor_right();
            terminal->touch_scrub_accum_x -= kHorizontalStepPx;
            terminal->touch_scrub_moved = true;
        }
        while (terminal->touch_scrub_accum_x <= -kHorizontalStepPx) {
            terminal->move_cursor_left();
            terminal->touch_scrub_accum_x += kHorizontalStepPx;
            terminal->touch_scrub_moved = true;
        }
        return;
    }

    if (code == LV_EVENT_RELEASED) {
        terminal->touch_scrub_active = false;
        terminal->touch_scrub_axis_locked = false;
        terminal->touch_scrub_vertical_mode = false;
        return;
    }

    if (code != LV_EVENT_CLICKED) {
        return;
    }

    if (terminal->touch_scrub_moved) {
        terminal->touch_scrub_moved = false;
        return;
    }

    // Convert to label coordinates for point-to-cursor tap placement.
    lv_obj_t* label = input_label;
    lv_area_t label_coords;
    lv_obj_get_coords(label, &label_coords);
    
    int32_t click_x = point.x - label_coords.x1;
    
    // Get font and calculate character position
    const lv_font_t* font = lv_obj_get_style_text_font(label, LV_PART_MAIN);
    
    // Account for "> " prefix (2 characters)
    const char* prefix = "> ";
    lv_point_t prefix_size;
    lv_txt_get_size(&prefix_size, prefix, font, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    int32_t prefix_width = prefix_size.x;
    
    if (click_x <= prefix_width) {
        // Clicked on prefix, set cursor to start
        terminal->cursor_pos = 0;
    } else {
        // Calculate which character was clicked
        int32_t text_x = click_x - prefix_width;
        size_t best_pos = 0;
        int32_t min_distance = INT32_MAX;
        
        // Check each character position
        for (size_t i = 0; i <= terminal->current_input.length(); i++) {
            std::string substr = terminal->current_input.substr(0, i);
            lv_point_t substr_size;
            lv_txt_get_size(&substr_size, substr.c_str(), font, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
            int32_t char_x = substr_size.x;
            int32_t distance = abs(text_x - char_x);
            
            if (distance < min_distance) {
                min_distance = distance;
                best_pos = i;
            }
        }
        
        terminal->cursor_pos = best_pos;
    }
    
    // Reset cursor blink to make it visible
    terminal->cursor_visible = true;
    terminal->update_input_display();
    
    ESP_LOGI(TAG, "Cursor moved to position: %d", static_cast<int>(terminal->cursor_pos));
}

void SSHTerminal::cursor_blink_cb(lv_timer_t* timer)
{
    SSHTerminal* terminal = (SSHTerminal*)lv_timer_get_user_data(timer);
    if (terminal) {
        terminal->cursor_visible = !terminal->cursor_visible;
        terminal->update_input_display();
    }
}

void SSHTerminal::battery_update_cb(lv_timer_t* timer)
{
    SSHTerminal* terminal = (SSHTerminal*)lv_timer_get_user_data(timer);
    if (terminal) {
        if (display_lock(0)) {
            terminal->update_status_bar();
            display_unlock();
        }
    }
}

void SSHTerminal::debug_metrics_cb(lv_timer_t* timer)
{
    SSHTerminal* terminal = (SSHTerminal*)lv_timer_get_user_data(timer);
    if (terminal == nullptr) {
        return;
    }
    if (!display_lock(0)) {
        return;
    }
    terminal->update_debug_metrics();
    display_unlock();
}

void SSHTerminal::history_save_cb(lv_timer_t* timer)
{
    SSHTerminal* terminal = (SSHTerminal*)lv_timer_get_user_data(timer);
    if (terminal && terminal->history_needs_save) {
        terminal->history_needs_save = false;
        terminal->save_history_to_nvs();
    }
}

void SSHTerminal::navigate_history(int direction)
{
    if (command_history.empty()) {
        return;
    }
    
    if (direction > 0) {
        if (history_index < (int)command_history.size() - 1) {
            history_index++;
            current_input = command_history[command_history.size() - 1 - history_index];
        }
    } else if (direction < 0) {
        if (history_index > 0) {
            history_index--;
            current_input = command_history[command_history.size() - 1 - history_index];
        } else if (history_index == 0) {
            history_index = -1;
            current_input.clear();
        }
    }
    
    // Move cursor to end of input
    cursor_pos = current_input.length();
    update_input_display();
}

void SSHTerminal::move_cursor_left()
{
    if (cursor_pos > 0) {
        cursor_pos--;
        cursor_visible = true;
        update_input_display();
    }
}

void SSHTerminal::move_cursor_right()
{
    if (cursor_pos < current_input.length()) {
        cursor_pos++;
        cursor_visible = true;
        update_input_display();
    }
}

void SSHTerminal::move_cursor_home()
{
    cursor_pos = 0;
    cursor_visible = true;
    update_input_display();
}

void SSHTerminal::move_cursor_end()
{
    cursor_pos = current_input.length();
    cursor_visible = true;
    update_input_display();
}

void SSHTerminal::scroll_terminal_output(int steps)
{
    if (terminal_output == nullptr || steps == 0) {
        return;
    }
    constexpr int kPixelsPerStep = 20;
    lv_obj_scroll_by(terminal_output, 0, steps * kPixelsPerStep, LV_ANIM_OFF);
}

void SSHTerminal::delete_current_history_entry()
{
    if (command_history.empty() || history_index < 0) {
        ESP_LOGW(TAG, "No history entry to delete (empty or not navigating)");
        return;
    }
    
    size_t actual_index = command_history.size() - 1 - history_index;
    
    ESP_LOGI(TAG, "Deleting history entry: '%s' (index %d)", 
             command_history[actual_index].c_str(), (int)actual_index);
    
    command_history.erase(command_history.begin() + actual_index);
    
    history_needs_save = true;
    
    if (command_history.empty()) {
        history_index = -1;
        current_input.clear();
    } else if (history_index >= (int)command_history.size()) {
        history_index = command_history.size() - 1;
        current_input = command_history[command_history.size() - 1 - history_index];
    } else if (actual_index < command_history.size()) {
        current_input = command_history[command_history.size() - 1 - history_index];
    } else {
        history_index = -1;
        current_input.clear();
    }
    
    std::string display_text = "> " + current_input;
    if (input_label) {
        lv_label_set_text(input_label, display_text.c_str());
    }
    
    ESP_LOGI(TAG, "History entry deleted. Remaining entries: %d", (int)command_history.size());
}

void SSHTerminal::send_current_history_command()
{
    if (command_history.empty() || history_index < 0) {
        ESP_LOGW(TAG, "No history command to send (empty or not navigating)");
        return;
    }
    
    size_t actual_index = command_history.size() - 1 - history_index;
    std::string cmd_to_send = command_history[actual_index];
    
    ESP_LOGI(TAG, "Sending history command: '%s'", cmd_to_send.c_str());
    
    current_input = cmd_to_send;
    
    std::string display_text = "> " + current_input;
    if (input_label) {
        lv_label_set_text(input_label, display_text.c_str());
    }
    
    send_command(cmd_to_send.c_str());
    
    auto it = std::find(command_history.begin(), command_history.end(), current_input);
    if (it != command_history.end()) {
        command_history.erase(it);
    }
    command_history.push_back(current_input);
    history_needs_save = true;
    current_input.clear();
    history_index = -1;
    
    display_text = "> ";
    if (input_label) {
        lv_label_set_text(input_label, display_text.c_str());
    }
}

void SSHTerminal::load_history_from_nvs()
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for reading history: %s", esp_err_to_name(err));
        return;
    }
    
    uint32_t history_count = 0;
    err = nvs_get_u32(nvs_handle, "hist_count", &history_count);
    if (err != ESP_OK || history_count == 0) {
        ESP_LOGI(TAG, "No command history found in NVS");
        nvs_close(nvs_handle);
        return;
    }
    
    ESP_LOGI(TAG, "Loading %lu commands from NVS...", history_count);
    
    command_history.clear();
    for (uint32_t i = 0; i < history_count && i < 100; i++) {
        char key[16];
        snprintf(key, sizeof(key), "hist_%lu", i);
        
        size_t required_size = 0;
        err = nvs_get_str(nvs_handle, key, NULL, &required_size);
        if (err != ESP_OK) {
            continue;
        }
        
        char* cmd = (char*)malloc(required_size);
        if (cmd) {
            err = nvs_get_str(nvs_handle, key, cmd, &required_size);
            if (err == ESP_OK) {
                command_history.push_back(std::string(cmd));
            }
            free(cmd);
        }
    }
    
    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Loaded %d commands from NVS", (int)command_history.size());
}

void SSHTerminal::save_history_to_nvs()
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing history: %s", esp_err_to_name(err));
        return;
    }
    
    size_t start_idx = 0;
    if (command_history.size() > 100) {
        start_idx = command_history.size() - 100;
    }
    
    uint32_t history_count = command_history.size() - start_idx;
    err = nvs_set_u32(nvs_handle, "hist_count", history_count);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save history count: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return;
    }
    
    int saved_count = 0;
    for (size_t i = start_idx; i < command_history.size() && saved_count < 100; i++) {
        char key[16];
        snprintf(key, sizeof(key), "hist_%lu", (unsigned long)(i - start_idx));
        
        err = nvs_set_str(nvs_handle, key, command_history[i].c_str());
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to save command %zu: %s", i, esp_err_to_name(err));
        } else {
            saved_count++;
        }
        
        if (saved_count % 10 == 0) {
            vTaskDelay(1);
        }
    }
    
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS changes: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Saved %d commands to NVS", saved_count);
    }
    
    nvs_close(nvs_handle);
}

void SSHTerminal::clear_history_nvs()
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return;
    }
    
    uint32_t history_count = 0;
    nvs_get_u32(nvs_handle, "hist_count", &history_count);
    
    for (uint32_t i = 0; i < history_count && i < 100; i++) {
        char key[16];
        snprintf(key, sizeof(key), "hist_%lu", i);
        nvs_erase_key(nvs_handle, key);
    }
    
    nvs_erase_key(nvs_handle, "hist_count");
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
}

int SSHTerminal::waitsocket(int socket_fd, LIBSSH2_SESSION *session)
{
    struct timeval timeout;
    int rc;
    fd_set fd;
    fd_set *writefd = NULL;
    fd_set *readfd = NULL;
    int dir;

    timeout.tv_sec = 2;
    timeout.tv_usec = 0;

    FD_ZERO(&fd);
    FD_SET(socket_fd, &fd);

    dir = libssh2_session_block_directions(session);

    if(dir & LIBSSH2_SESSION_BLOCK_INBOUND)
        readfd = &fd;

    if(dir & LIBSSH2_SESSION_BLOCK_OUTBOUND)
        writefd = &fd;

    rc = select(socket_fd + 1, readfd, writefd, NULL, &timeout);

    return rc;
}

esp_err_t SSHTerminal::connect(const char* host, int port, const char* username, const char* password)
{
    if (!wifi_connected) {
        ESP_LOGE(TAG, "WiFi not connected");
        append_text("ERROR: WiFi not connected\n");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Connecting to %s:%d", host, port);
    append_text("Connecting to ");
    append_text(host);
    append_text("...\n");

    int rc = libssh2_init(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "libssh2 initialization failed (%d)", rc);
        append_text("ERROR: libssh2 init failed\n");
        return ESP_FAIL;
    }

    struct sockaddr_in sin;
    ssh_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (ssh_socket < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        append_text("ERROR: Failed to create socket\n");
        libssh2_exit();
        return ESP_FAIL;
    }

    if (!resolve_host_ipv4(host, port, &sin)) {
        ESP_LOGE(TAG, "Failed to resolve host: %s", host);
        append_text("ERROR: Failed to resolve host\n");
        append_text("Hint: .local uses mDNS; guest/VLAN networks often block it. IP/DNS hostname may still work.\n");
        close(ssh_socket);
        ssh_socket = -1;
        libssh2_exit();
        return ESP_FAIL;
    }
    char resolved_ip[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, &sin.sin_addr, resolved_ip, sizeof(resolved_ip));
    append_text("Resolved to ");
    append_text(resolved_ip);
    append_text("\n");
    ESP_LOGI(TAG, "Resolved %s -> %s", host, resolved_ip);

    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    setsockopt(ssh_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(ssh_socket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    if (::connect(ssh_socket, (struct sockaddr*)(&sin), sizeof(struct sockaddr_in)) != 0) {
        const int err = errno;
        ESP_LOGE(TAG, "Failed to connect socket errno=%d (%s)", err, strerror(err));
        append_text("ERROR: Failed to connect socket\n");
        char err_line[96];
        std::snprintf(err_line, sizeof(err_line), "errno=%d (%s)\n", err, strerror(err));
        append_text(err_line);
        if (err == EHOSTUNREACH || err == ECONNABORTED || err == ENETUNREACH || err == ETIMEDOUT) {
            append_text("Hint: target likely unreachable from current WiFi/network segment.\n");
        }
        close(ssh_socket);
        ssh_socket = -1;
        libssh2_exit();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Socket connected");
    append_text("Socket connected, initializing SSH session...\n");
    log_heap_snapshot("pre_session_init");

    session = libssh2_session_init();
    if (!session) {
        ESP_LOGW(TAG, "Failed to create SSH session, attempting low-memory recovery");
        append_text("WARN: session alloc failed, clearing terminal and retrying...\n");
        if (display_lock(50)) {
            if (terminal_output) {
                lv_textarea_set_text(terminal_output, "");
            }
            display_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
        log_heap_snapshot("post_recovery");
        session = libssh2_session_init();
    }
    if (!session) {
        ESP_LOGE(TAG, "Failed to create SSH session after recovery");
        append_text("ERROR: Failed to create SSH session\n");
        close(ssh_socket);
        ssh_socket = -1;
        libssh2_exit();
        return ESP_FAIL;
    }

    libssh2_session_set_blocking(session, 0);

    append_text("Performing SSH handshake...\n");
    while ((rc = libssh2_session_handshake(session, ssh_socket)) == LIBSSH2_ERROR_EAGAIN);
    
    if (rc) {
        ESP_LOGE(TAG, "SSH handshake failed: %d", rc);
        append_text("ERROR: SSH handshake failed\n");
        disconnect();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "SSH handshake successful");
    append_text("SSH handshake successful\n");

    if (ssh_authenticate(username, password) != ESP_OK) {
        append_text("ERROR: Authentication failed\n");
        disconnect();
        return ESP_FAIL;
    }

    append_text("Authentication successful\n");

    if (ssh_open_channel() != ESP_OK) {
        append_text("ERROR: Failed to open channel\n");
        disconnect();
        return ESP_FAIL;
    }

    append_text("SSH channel opened - connected!\n");
    ssh_connected = true;
    connected_ssh_host = host != nullptr ? host : "";
    if (port != 22 && !connected_ssh_host.empty()) {
        connected_ssh_host += ":" + std::to_string(port);
    }
    update_status_bar();

    BaseType_t rx_task_ok = xTaskCreate(ssh_receive_task, "ssh_rx", 6144, this, 5, NULL);
    if (rx_task_ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create SSH receive task");
        append_text("ERROR: SSH receive task failed to start\n");
        disconnect();
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t SSHTerminal::connect_with_key(const char* host, int port, const char* username, const char* privkey_data, size_t privkey_len)
{
    if (!wifi_connected) {
        ESP_LOGE(TAG, "WiFi not connected");
        append_text("ERROR: WiFi not connected\n");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Connecting to %s:%d with public key", host, port);
    append_text("Connecting to ");
    append_text(host);
    append_text(" with public key...\n");

    int rc = libssh2_init(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "libssh2 initialization failed (%d)", rc);
        append_text("ERROR: libssh2 init failed\n");
        return ESP_FAIL;
    }

    struct sockaddr_in sin;
    ssh_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (ssh_socket < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        append_text("ERROR: Failed to create socket\n");
        libssh2_exit();
        return ESP_FAIL;
    }

    if (!resolve_host_ipv4(host, port, &sin)) {
        ESP_LOGE(TAG, "Failed to resolve host: %s", host);
        append_text("ERROR: Failed to resolve host\n");
        append_text("Hint: .local uses mDNS; guest/VLAN networks often block it. IP/DNS hostname may still work.\n");
        close(ssh_socket);
        ssh_socket = -1;
        libssh2_exit();
        return ESP_FAIL;
    }
    char resolved_ip[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, &sin.sin_addr, resolved_ip, sizeof(resolved_ip));
    append_text("Resolved to ");
    append_text(resolved_ip);
    append_text("\n");
    ESP_LOGI(TAG, "Resolved %s -> %s", host, resolved_ip);

    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    setsockopt(ssh_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(ssh_socket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    if (::connect(ssh_socket, (struct sockaddr*)(&sin), sizeof(struct sockaddr_in)) != 0) {
        const int err = errno;
        ESP_LOGE(TAG, "Failed to connect socket errno=%d (%s)", err, strerror(err));
        append_text("ERROR: Failed to connect socket\n");
        char err_line[96];
        std::snprintf(err_line, sizeof(err_line), "errno=%d (%s)\n", err, strerror(err));
        append_text(err_line);
        if (err == EHOSTUNREACH || err == ECONNABORTED || err == ENETUNREACH || err == ETIMEDOUT) {
            append_text("Hint: target likely unreachable from current WiFi/network segment.\n");
        }
        close(ssh_socket);
        ssh_socket = -1;
        libssh2_exit();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Socket connected");
    append_text("Socket connected, initializing SSH session...\n");
    log_heap_snapshot("pre_session_init_key");

    session = libssh2_session_init();
    if (!session) {
        ESP_LOGW(TAG, "Failed to create SSH session, attempting low-memory recovery");
        append_text("WARN: session alloc failed, clearing terminal and retrying...\n");
        if (display_lock(50)) {
            if (terminal_output) {
                lv_textarea_set_text(terminal_output, "");
            }
            display_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
        log_heap_snapshot("post_recovery_key");
        session = libssh2_session_init();
    }
    if (!session) {
        ESP_LOGE(TAG, "Failed to create SSH session after recovery");
        append_text("ERROR: Failed to create SSH session\n");
        close(ssh_socket);
        ssh_socket = -1;
        libssh2_exit();
        return ESP_FAIL;
    }

    libssh2_session_set_blocking(session, 0);

    append_text("Performing SSH handshake...\n");
    while ((rc = libssh2_session_handshake(session, ssh_socket)) == LIBSSH2_ERROR_EAGAIN);
    
    if (rc) {
        ESP_LOGE(TAG, "SSH handshake failed: %d", rc);
        append_text("ERROR: SSH handshake failed\n");
        disconnect();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "SSH handshake successful");
    append_text("SSH handshake successful\n");

    if (ssh_authenticate_pubkey(username, privkey_data, privkey_len) != ESP_OK) {
        append_text("ERROR: Public key authentication failed\n");
        disconnect();
        return ESP_FAIL;
    }

    append_text("Public key authentication successful\n");

    if (ssh_open_channel() != ESP_OK) {
        append_text("ERROR: Failed to open channel\n");
        disconnect();
        return ESP_FAIL;
    }

    append_text("SSH channel opened - connected!\n");
    ssh_connected = true;
    connected_ssh_host = host != nullptr ? host : "";
    if (port != 22 && !connected_ssh_host.empty()) {
        connected_ssh_host += ":" + std::to_string(port);
    }
    update_status_bar();

    BaseType_t rx_task_ok = xTaskCreate(ssh_receive_task, "ssh_rx", 6144, this, 5, NULL);
    if (rx_task_ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create SSH receive task");
        append_text("ERROR: SSH receive task failed to start\n");
        disconnect();
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t SSHTerminal::ssh_authenticate(const char* username, const char* password)
{
    append_text("Authenticating as ");
    append_text(username);
    append_text("...\n");

    int rc;
    while ((rc = libssh2_userauth_password(session, username, password)) == LIBSSH2_ERROR_EAGAIN);
    
    if (rc) {
        char *err_msg;
        int err_len;
        libssh2_session_last_error(session, &err_msg, &err_len, 0);
        ESP_LOGE(TAG, "Authentication failed: %s", err_msg);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Authentication successful");
    return ESP_OK;
}

esp_err_t SSHTerminal::ssh_authenticate_pubkey(const char* username, const char* privkey_data, size_t privkey_len)
{
    append_text("Authenticating as ");
    append_text(username);
    append_text(" with public key...\n");

    int rc;
    // libssh2_userauth_publickey_frommemory expects the private key, public key (can be NULL), and passphrase
    while ((rc = libssh2_userauth_publickey_frommemory(session, username, strlen(username),
                                                         NULL, 0,  // public key (optional)
                                                         privkey_data, privkey_len,
                                                         NULL)) == LIBSSH2_ERROR_EAGAIN);  // no passphrase
    
    if (rc) {
        char *err_msg;
        int err_len;
        libssh2_session_last_error(session, &err_msg, &err_len, 0);
        ESP_LOGE(TAG, "Public key authentication failed: %s (error code: %d)", err_msg, rc);
        append_text("ERROR: Public key authentication failed\n");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Public key authentication successful");
    return ESP_OK;
}

esp_err_t SSHTerminal::ssh_open_channel()
{
    append_text("Opening SSH channel...\n");

    int rc;
    while ((channel = libssh2_channel_open_session(session)) == NULL &&
           libssh2_session_last_error(session, NULL, NULL, 0) == LIBSSH2_ERROR_EAGAIN) {
        waitsocket(ssh_socket, session);
    }

    if (channel == NULL) {
        ESP_LOGE(TAG, "Failed to open channel");
        return ESP_FAIL;
    }

    while ((rc = libssh2_channel_request_pty(channel, "vt100")) == LIBSSH2_ERROR_EAGAIN) {
        waitsocket(ssh_socket, session);
    }
    
    if (rc) {
        ESP_LOGE(TAG, "Failed to request PTY");
        return ESP_FAIL;
    }

    while ((rc = libssh2_channel_shell(channel)) == LIBSSH2_ERROR_EAGAIN) {
        waitsocket(ssh_socket, session);
    }
    
    if (rc) {
        ESP_LOGE(TAG, "Failed to start shell");
        return ESP_FAIL;
    }

    libssh2_channel_set_blocking(channel, 0);

    ESP_LOGI(TAG, "SSH channel opened successfully");
    return ESP_OK;
}

esp_err_t SSHTerminal::disconnect()
{
    ssh_connected = false;
    connected_ssh_host.clear();
    
    if (channel) {
        libssh2_channel_free(channel);
        channel = NULL;
    }

    if (session) {
        libssh2_session_disconnect(session, "Normal Shutdown");
        libssh2_session_free(session);
        session = NULL;
    }

    if (ssh_socket >= 0) {
        close(ssh_socket);
        ssh_socket = -1;
    }

    libssh2_exit();
    
    if (display_lock(0)) {
        update_status_bar();
        append_text("\nDisconnected\n");
        display_unlock();
    }
    
    ESP_LOGI(TAG, "Disconnected");
    
    return ESP_OK;
}

bool SSHTerminal::is_connected()
{
    return ssh_connected;
}

void SSHTerminal::send_command(const char* cmd)
{
    if (!channel) {
        return;
    }
    
    bytes_received = 0;

    std::string full_cmd = std::string(cmd) + "\n";
    ssize_t nwritten = 0;
    int retry_count = 0;
    const int MAX_RETRIES = 20;
    
    ESP_LOGI(TAG, "Sending command: %s", cmd);
    
    while (nwritten < (ssize_t)full_cmd.length() && retry_count < MAX_RETRIES) {
        ssize_t n = libssh2_channel_write(channel, full_cmd.c_str() + nwritten, 
                                          full_cmd.length() - nwritten);
        if (n == LIBSSH2_ERROR_EAGAIN) {
            retry_count++;
            vTaskDelay(1);
            continue;
        }
        if (n < 0) {
            ESP_LOGE(TAG, "Failed to write to channel: %d", (int)n);
            break;
        }
        nwritten += n;
        retry_count = 0;  // Forward progress reset.
    }
    
    if (nwritten < (ssize_t)full_cmd.length()) {
        ESP_LOGW(TAG, "Command partially sent (%d/%d bytes)", (int)nwritten, (int)full_cmd.length());
    }

    ESP_LOGI(TAG, "Command sent: %d bytes", (int)nwritten);
}

void SSHTerminal::ssh_receive_task(void* param)
{
    SSHTerminal* terminal = (SSHTerminal*)param;
    char buffer[1024];
    ssize_t rc;

    ESP_LOGI(TAG, "SSH receive task started");

    while (terminal->ssh_connected && terminal->channel) {
        rc = libssh2_channel_read(terminal->channel, buffer, sizeof(buffer) - 1);
        
        if (rc > 0) {
            buffer[rc] = '\0';
            terminal->process_received_data(buffer, rc);
            vTaskDelay(1);
        } else if (rc == LIBSSH2_ERROR_EAGAIN) {
            terminal->flush_display_buffer();
            vTaskDelay(pdMS_TO_TICKS(100));
        } else if (rc < 0) {
            ESP_LOGE(TAG, "Read error: %d", (int)rc);
            break;
        }

        if (libssh2_channel_eof(terminal->channel)) {
            ESP_LOGI(TAG, "Channel EOF");
            terminal->flush_display_buffer();
            break;
        }
        
        vTaskDelay(1);
    }

    ESP_LOGI(TAG, "SSH receive task ended");
    terminal->disconnect();
    vTaskDelete(NULL);
}

std::string SSHTerminal::strip_ansi_codes(const char* data, size_t len)
{
    std::string result;
    result.reserve(len);
    
    for (size_t i = 0; i < len; i++) {
        if (i > 0 && i % 1024 == 0) {
            vTaskDelay(1);
        }
        
        if (data[i] == '\x1B' || data[i] == '\033') {
            i++;
            if (i >= len) break;
            
            if (data[i] == '[') {
                i++;
                while (i < len && !((data[i] >= 'A' && data[i] <= 'Z') || 
                                    (data[i] >= 'a' && data[i] <= 'z'))) {
                    i++;
                }
            } else if (data[i] == ']') {
                i++;
                while (i < len) {
                    if (data[i] == '\007') break;
                    if (data[i] == '\x1B' && i + 1 < len && data[i + 1] == '\\') {
                        i++;
                        break;
                    }
                    i++;
                }
            } else if (data[i] == '(' || data[i] == ')') {
                i++;
            }
        } else if (data[i] == '\r') {
            continue;
        } else {
            result += data[i];
        }
    }
    
    return result;
}

void SSHTerminal::process_received_data(const char* data, size_t len)
{
    bytes_received += len;
    
    std::string cleaned = strip_ansi_codes(data, len);
    text_buffer += cleaned;
    
    int64_t current_time = esp_timer_get_time() / 1000;
    
    if (text_buffer.size() > kTerminalIngressMaxBytes) {
        text_buffer = text_buffer.substr(text_buffer.size() - kTerminalIngressKeepBytes);
    }
    
    if (current_time - last_display_update >= kTerminalFlushIntervalMs) {
        flush_display_buffer();
    }
    
    vTaskDelay(1);
}

void SSHTerminal::flush_display_buffer()
{
    if (text_buffer.empty() && bytes_received == 0) {
        return;
    }
    
    const size_t CHUNK_SIZE = 256;
    size_t offset = 0;
    
    while (offset < text_buffer.size()) {
        if (display_lock(0)) {
            size_t chunk_len = std::min(CHUNK_SIZE, text_buffer.size() - offset);
            std::string chunk = text_buffer.substr(offset, chunk_len);
            append_text(chunk.c_str());
            display_unlock();
            offset += chunk_len;
            
            vTaskDelay(1);
        } else {
            break;
        }
    }
    
    if (offset > 0) {
        text_buffer = text_buffer.substr(offset);
    }
    
    if (text_buffer.empty()) {
        last_display_update = esp_timer_get_time() / 1000;
    }
    
    vTaskDelay(1);
}

void SSHTerminal::load_default_terminal_font_mode_from_config()
{
    bool config_big = false;
    if (!read_default_fontsize_big_from_config(&config_big)) {
        terminal_font_big = false;
        return;
    }
    terminal_font_big = config_big;
    ESP_LOGI(TAG, "Default terminal font from ssh_config: %s", terminal_font_big ? "big" : "normal");
}

void SSHTerminal::apply_terminal_font_mode()
{
    if (!terminal_output) {
        return;
    }
    const lv_font_t *font = terminal_font_big ? ui_font_terminal_big() : ui_font_small();
    lv_obj_set_style_text_font(terminal_output, font, 0);
    update_input_display();
}

void SSHTerminal::set_terminal_font_mode(bool big_mode, bool announce)
{
    if (terminal_font_big == big_mode) {
        if (announce) {
            append_text(big_mode ? "fontsize already big\n" : "fontsize already normal\n");
        }
        return;
    }

    terminal_font_big = big_mode;
    if (display_lock(0)) {
        apply_terminal_font_mode();
        display_unlock();
    } else {
        apply_terminal_font_mode();
    }

    if (announce) {
        if (terminal_font_big) {
            append_text("fontsize set to big (~53x9)\n");
        } else {
            append_text("fontsize set to normal (~67x13)\n");
        }
    }
}

void SSHTerminal::update_debug_metrics()
{
    if (byte_counter_label == nullptr) {
        return;
    }

    const int sram_free_pct = free_percent_for_caps(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const int psram_free_pct = free_percent_for_caps(MALLOC_CAP_SPIRAM);
    if (flash_headroom_percent < 0) {
        flash_headroom_percent = app_flash_headroom_percent();
    }

    auto fmt_pct = [](int pct, char out[4]) {
        if (pct < 0) {
            std::strcpy(out, "--");
            return;
        }
        if (pct > 99) {
            std::strcpy(out, "99+");
            return;
        }
        std::snprintf(out, 4, "%02d", pct);
    };

    char sram_txt[4];
    char psram_txt[4];
    char flash_txt[4];
    fmt_pct(sram_free_pct, sram_txt);
    fmt_pct(psram_free_pct, psram_txt);
    fmt_pct(flash_headroom_percent, flash_txt);

    char line[32];
    std::snprintf(line, sizeof(line), "S%s P%s F%s", sram_txt, psram_txt, flash_txt);
    lv_label_set_text(byte_counter_label, line);
}

void SSHTerminal::update_status_bar()
{
    if (!status_bar) return;
    
    if (!display_lock(0)) {
        return;
    }

    std::string status;
    
    if (battery_initialized) {
        float voltage = battery.readBatteryVoltage();
        if (voltage > 0.1f) {
            char voltage_text[32];
            snprintf(voltage_text, sizeof(voltage_text), "%.2fV | ", voltage);
            status = voltage_text;
            ESP_LOGD(TAG, "Battery voltage displayed: %.2fV", voltage);
        } else {
            ESP_LOGW(TAG, "Battery voltage too low or invalid: %.2fV", voltage);
        }
    } else {
        ESP_LOGD(TAG, "Battery not initialized, skipping voltage display");
    }
    
    if (!wifi_connected) {
        status += LV_SYMBOL_WIFI " OFF";
        lv_obj_set_style_text_color(status_bar, lv_color_hex(0xFF0000), 0);
    } else {
        status += LV_SYMBOL_WIFI " ";
        if (connected_wifi_ssid.empty()) {
            status += "ON";
        } else {
            status += abbreviate_status_value(connected_wifi_ssid, 12);
        }

        if (!ssh_connected) {
            status += " | " LV_SYMBOL_CLOSE " SSH";
            lv_obj_set_style_text_color(status_bar, lv_color_hex(0xFFFF00), 0);
        } else {
            status += " | " LV_SYMBOL_OK " ";
            if (connected_ssh_host.empty()) {
                status += "SSH";
            } else {
                status += abbreviate_status_value(connected_ssh_host, 14);
            }
            lv_obj_set_style_text_color(status_bar, lv_color_hex(0x00FF00), 0);
        }
    }
    
    lv_label_set_text(status_bar, status.c_str());
    
    display_unlock();
}

void SSHTerminal::update_terminal_display()
{
}

void SSHTerminal::create_side_panel()
{
    side_panel = lv_obj_create(terminal_screen);
    lv_obj_set_size(side_panel, 100, lv_pct(100));
    lv_obj_set_style_bg_color(side_panel, lv_color_hex(0x101010), 0);
    lv_obj_set_style_bg_opa(side_panel, LV_OPA_80, 0);
    lv_obj_set_style_border_color(side_panel, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_border_width(side_panel, 2, 0);
    lv_obj_set_scrollbar_mode(side_panel, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(side_panel, LV_DIR_VER);
    lv_obj_align(side_panel, LV_ALIGN_TOP_RIGHT, 100, 0);
    lv_obj_add_flag(side_panel, LV_OBJ_FLAG_HIDDEN);
    
    lv_obj_t* title = lv_label_create(side_panel);
    lv_label_set_text(title, "Keys");
    lv_obj_set_style_text_color(title, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_text_font(title, ui_font_body(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);
    
    auto create_key_button = [this](const char* label, const char* key_seq, int y_offset) {
        lv_obj_t* btn = lv_btn_create(side_panel);
        lv_obj_set_size(btn, 85, 30);
        lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, y_offset);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x1A1A1A), 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x00CC00), LV_STATE_PRESSED);
        
        lv_obj_t* btn_label = lv_label_create(btn);
        lv_label_set_text(btn_label, label);
        lv_obj_set_style_text_color(btn_label, lv_color_hex(0x00FF00), 0);
        lv_obj_set_style_text_font(btn_label, ui_font_small(), 0);
        lv_obj_center(btn_label);
        
        lv_obj_set_user_data(btn, (void*)key_seq);
        lv_obj_add_event_cb(btn, special_key_event_cb, LV_EVENT_CLICKED, this);
        
        return btn;
    };

    create_key_button("<-", "LEFT", 35);
    create_key_button("->", "RIGHT", 70);
    create_key_button("Line <", "HOME", 105);
    create_key_button("> Line", "END", 140);
    create_key_button("Ctrl+C", "\x03", 175);
    create_key_button("Ctrl+Z", "\x1A", 210);
    create_key_button("Ctrl+D", "\x04", 245);
    create_key_button("Ctrl+L", "\x0C", 280);
    create_key_button("Tab", "\t", 315);
    create_key_button("Esc", "\x1B", 350);
    create_key_button("Exit SSH", "EXIT", 385);
    create_key_button("Clear", "CLEAR", 420);
}

void SSHTerminal::toggle_side_panel()
{
    if (!side_panel) return;
    
    if (lv_obj_has_flag(side_panel, LV_OBJ_FLAG_HIDDEN)) {
        lv_obj_clear_flag(side_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_align(side_panel, LV_ALIGN_TOP_RIGHT, 0, 0);
    } else {
        lv_obj_add_flag(side_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_align(side_panel, LV_ALIGN_TOP_RIGHT, 100, 0);
    }
}

void SSHTerminal::send_special_key(const char* sequence)
{
    if (!sequence || strlen(sequence) == 0) {
        toggle_side_panel();
        return;
    }
    
    if (strcmp(sequence, "EXIT") == 0) {
        disconnect();
        toggle_side_panel();
        return;
    }
    
    if (strcmp(sequence, "CLEAR") == 0) {
        clear_terminal();
        toggle_side_panel();
        return;
    }
    
    // Handle cursor movement keys
    if (strcmp(sequence, "LEFT") == 0) {
        move_cursor_left();
        return;
    }
    
    if (strcmp(sequence, "RIGHT") == 0) {
        move_cursor_right();
        return;
    }
    
    if (strcmp(sequence, "HOME") == 0) {
        move_cursor_home();
        return;
    }
    
    if (strcmp(sequence, "END") == 0) {
        move_cursor_end();
        return;
    }
    
    if (ssh_connected && channel) {
        libssh2_channel_write(channel, sequence, strlen(sequence));
        ESP_LOGI(TAG, "Sent special key sequence");
    } else {
        ESP_LOGW(TAG, "Cannot send special key - not connected");
    }
    
    toggle_side_panel();
}

void SSHTerminal::gesture_event_cb(lv_event_t* e)
{
    SSHTerminal* terminal = (SSHTerminal*)lv_event_get_user_data(e);
    lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
    
    if (dir == LV_DIR_LEFT) {
        ESP_LOGI(TAG, "Swipe left detected - showing special keys panel");
        if (terminal->side_panel && lv_obj_has_flag(terminal->side_panel, LV_OBJ_FLAG_HIDDEN)) {
            terminal->toggle_side_panel();
        }
    } else if (dir == LV_DIR_RIGHT) {
        ESP_LOGI(TAG, "Swipe right detected - hiding special keys panel");
        if (terminal->side_panel && !lv_obj_has_flag(terminal->side_panel, LV_OBJ_FLAG_HIDDEN)) {
            terminal->toggle_side_panel();
        }
    }
}

void SSHTerminal::special_key_event_cb(lv_event_t* e)
{
    SSHTerminal* terminal = (SSHTerminal*)lv_event_get_user_data(e);
    lv_obj_t* btn = (lv_obj_t* ) lv_event_get_target(e);
    const char* key_seq = (const char*)lv_obj_get_user_data(btn);
    
    if (key_seq) {
        terminal->send_special_key(key_seq);
    }
}

void SSHTerminal::load_key_from_memory(const char* keyname, const char* key_data, size_t key_len)
{
    if (!keyname || !key_data || key_len == 0) {
        ESP_LOGE(TAG, "Invalid key parameters");
        return;
    }
    
    // Convert keyname to lowercase for case-insensitive lookup
    std::string keyname_str(keyname);
    std::transform(keyname_str.begin(), keyname_str.end(), keyname_str.begin(), ::tolower);
    
    std::string key_str(key_data, key_len);
    
    loaded_keys[keyname_str] = key_str;
    
    ESP_LOGI(TAG, "Loaded SSH key: %s (%d bytes)", keyname, key_len);
}

const char* SSHTerminal::get_loaded_key(const char* keyname, size_t* len)
{
    if (!keyname) {
        return NULL;
    }
    
    // Convert keyname to lowercase for case-insensitive lookup
    std::string keyname_str(keyname);
    std::transform(keyname_str.begin(), keyname_str.end(), keyname_str.begin(), ::tolower);
    
    auto it = loaded_keys.find(keyname_str);
    
    if (it != loaded_keys.end()) {
        if (len) {
            *len = it->second.length();
        }
        return it->second.c_str();
    }
    
    return NULL;
}

std::vector<std::string> SSHTerminal::get_loaded_key_names()
{
    std::vector<std::string> key_names;
    
    for (const auto& pair : loaded_keys) {
        key_names.push_back(pair.first);
    }
    
    return key_names;
}
