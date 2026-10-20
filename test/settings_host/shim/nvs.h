// Host shim: the NVS calls of NvsStore compile; the host test never calls them (it uses the RAM store).
#pragma once
#include <cstddef>
#include <cstdint>
typedef int esp_err_t;
typedef uint32_t nvs_handle_t;
enum { ESP_OK = 0, ESP_FAIL = -1 };
enum nvs_open_mode_t { NVS_READONLY, NVS_READWRITE };
static inline esp_err_t nvs_open(const char*, nvs_open_mode_t, nvs_handle_t*) { return ESP_FAIL; }
static inline void nvs_close(nvs_handle_t) {}
static inline esp_err_t nvs_commit(nvs_handle_t) { return ESP_FAIL; }
static inline esp_err_t nvs_erase_all(nvs_handle_t) { return ESP_FAIL; }
#define NVS_SHIM_GET(name, T) static inline esp_err_t name(nvs_handle_t, const char*, T*) { return ESP_FAIL; }
NVS_SHIM_GET(nvs_get_i8, int8_t) NVS_SHIM_GET(nvs_get_u8, uint8_t) NVS_SHIM_GET(nvs_get_i16, int16_t)
NVS_SHIM_GET(nvs_get_u16, uint16_t) NVS_SHIM_GET(nvs_get_i32, int32_t) NVS_SHIM_GET(nvs_get_u32, uint32_t)
NVS_SHIM_GET(nvs_get_i64, int64_t) NVS_SHIM_GET(nvs_get_u64, uint64_t)
static inline esp_err_t nvs_get_str(nvs_handle_t, const char*, char*, size_t*) { return ESP_FAIL; }
static inline esp_err_t nvs_get_blob(nvs_handle_t, const char*, void*, size_t*) { return ESP_FAIL; }
static inline esp_err_t nvs_set_u8(nvs_handle_t, const char*, uint8_t) { return ESP_FAIL; }
static inline esp_err_t nvs_set_i32(nvs_handle_t, const char*, int32_t) { return ESP_FAIL; }
static inline esp_err_t nvs_set_str(nvs_handle_t, const char*, const char*) { return ESP_FAIL; }
static inline esp_err_t nvs_set_blob(nvs_handle_t, const char*, const void*, size_t) { return ESP_FAIL; }
