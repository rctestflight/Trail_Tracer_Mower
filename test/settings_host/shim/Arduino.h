// Host shim for the settings self-check (test/settings_host): the few Arduino names the settings code uses.
#pragma once
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

class String {
  public:
    String() {}
    String(const char* s) { if (s) s_ = s; }
    String(const String&) = default;
    String(int v) { s_ = std::to_string(v); }
    String(long v) { s_ = std::to_string(v); }
    String(unsigned v) { s_ = std::to_string(v); }
    String(double v, int dec = 2) { char b[64]; snprintf(b, sizeof(b), "%.*f", dec, v); s_ = b; }
    String& operator=(const String&) = default;
    String& operator=(const char* s) { s_ = s ? s : ""; return *this; }
    const char* c_str() const { return s_.c_str(); }
    unsigned length() const { return (unsigned)s_.size(); }
    bool reserve(unsigned n) { s_.reserve(n); return true; }
    bool concat(const char* s) { if (s) s_ += s; return true; }
    bool concat(const char* s, unsigned n) { s_.append(s, n); return true; }
    void clear() { s_.clear(); }
    String& operator+=(const String& o) { s_ += o.s_; return *this; }
    String& operator+=(const char* o) { if (o) s_ += o; return *this; }
    String& operator+=(char c) { s_ += c; return *this; }
    char operator[](unsigned i) const { return i < s_.size() ? s_[i] : 0; }
    bool operator==(const String& o) const { return s_ == o.s_; }
    bool operator!=(const String& o) const { return s_ != o.s_; }
    bool operator==(const char* o) const { return s_ == (o ? o : ""); }
    String substring(unsigned a, unsigned b) const { if (a > s_.size()) return String(); if (b > s_.size()) b = (unsigned)s_.size(); return String(s_.substr(a, b - a).c_str()); }
    String substring(unsigned a) const { return substring(a, (unsigned)s_.size()); }
    bool startsWith(const char* p) const { return s_.rfind(p, 0) == 0; }
    bool endsWith(const char* p) const { size_t n = strlen(p); return s_.size() >= n && s_.compare(s_.size() - n, n, p) == 0; }
    void trim() {}
    friend String operator+(const String& a, const String& b) { String r(a); r += b; return r; }
    friend String operator+(const String& a, const char* b) { String r(a); r += b; return r; }
    friend String operator+(const char* a, const String& b) { String r(a); r += b; return r; }
    friend String operator+(const String& a, int b) { String r(a); r += String(b); return r; }
    friend String operator+(const String& a, long b) { String r(a); r += String(b); return r; }
  private:
    std::string s_;
};

template <class T, class L, class H> static inline auto constrain(T v, L lo, H hi) -> decltype(v + lo + hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}
uint32_t millis();
static inline void delay(uint32_t) {}

#define ESP_LOGE(...) do {} while (0)
#define ESP_LOGW(...) do {} while (0)
#define ESP_LOGI(...) do {} while (0)
