// multipart: see multipart.h.
#include "multipart.h"

#include <string.h>
#include <strings.h>

namespace multipart {

bool boundary_from(const char* ct, char* out, size_t cap) {
  if (!ct || strncasecmp(ct, "multipart/form-data", 19)) return false;
  const char* p = ct;
  for (; *p; p++)
    if (!strncasecmp(p, "boundary=", 9)) break;
  if (!*p) return false;
  p += 9;
  const bool q = *p == '"';
  if (q) p++;
  size_t n = 0;
  while (p[n] && (q ? p[n] != '"' : p[n] != ';' && p[n] != ' ' && p[n] != '\t')) n++;
  if (n == 0 || n > 70 || n + 1 > cap) return false;
  memcpy(out, p, n);
  out[n] = 0;
  return true;
}

void Parser::begin(const char* boundary, const Callbacks& cb) {
  cb_ = cb;
  const size_t bl = strlen(boundary);
  memcpy(delim_, "\r\n--", 4);
  memcpy(delim_ + 4, boundary, bl);
  dl_ = (uint8_t)(4 + bl);
  fail_[0] = 0;
  for (uint8_t i = 1, k = 0; i < dl_; i++) {
    while (k && delim_[i] != delim_[k]) k = fail_[k - 1];
    if (delim_[i] == delim_[k]) k++;
    fail_[i] = k;
  }
  m_ = 2;  // the body starts with "--boundary": as if "\r\n" came before it
  st_ = S_PRE;
  res_ = MP_MORE;
  hn_ = on_ = 0;
  parts_ = 0;
  err_ = "";
}

bool Parser::flush() {
  if (on_ && st_ == S_DATA && !cb_.on_data(cb_.ctx, out_, on_)) { on_ = 0; return false; }
  on_ = 0;
  return true;
}

// One part header line in head_. Only Content-Disposition matters: name and filename.
bool Parser::header_line() {
  if (strncasecmp(head_, "content-disposition:", 20)) return true;
  for (int k = 0; k < 2; k++) {
    const char* key = k ? "filename=\"" : "name=\"";
    char* dst = k ? file_ : name_;
    const size_t cap = k ? sizeof(file_) : sizeof(name_);
    for (const char* p = head_ + 20; (p = strstr(p, key)); p++) {
      if (!k && p > head_ && p[-1] != ' ' && p[-1] != ';') continue;  // "filename=" also holds "name="
      p += strlen(key);
      size_t n = 0;
      while (p[n] && p[n] != '"') n++;
      if (n >= cap) n = cap - 1;
      memcpy(dst, p, n);
      dst[n] = 0;
      break;
    }
  }
  return true;
}

Result Parser::feed(const uint8_t* b, size_t n) {
  for (size_t i = 0; i < n && res_ == MP_MORE; i++) {
    const uint8_t c = b[i];
    switch (st_) {
      case S_PRE:  // preamble: search for the first delimiter, nothing emitted
      case S_DATA:
        while (m_ && c != (uint8_t)delim_[m_]) {
          const uint8_t k = fail_[m_ - 1];
          if (st_ == S_DATA)
            for (uint8_t j = 0; j < m_ - k; j++)
              if (!emit((uint8_t)delim_[j])) { res_ = MP_STOPPED; break; }
          m_ = k;
        }
        if (res_ != MP_MORE) break;
        if (c == (uint8_t)delim_[m_]) {
          if (++m_ == dl_) {  // a whole delimiter
            m_ = 0;
            if (st_ == S_DATA) {
              if (!flush() || !cb_.on_end(cb_.ctx)) { res_ = MP_STOPPED; break; }
            }
            st_ = S_AFTER;
          }
        } else if (st_ == S_DATA && !emit(c)) {
          res_ = MP_STOPPED;
        }
        break;
      case S_AFTER:  // "\r\n": a part follows; "--": the end
        after_[0] = (char)c;
        st_ = S_AFTER2;
        break;
      case S_AFTER2:
        if (after_[0] == '-' && c == '-') { st_ = S_EPI; res_ = MP_DONE; break; }
        if (after_[0] == '\r' && c == '\n') {
          st_ = S_HEAD;
          hn_ = 0;
          name_[0] = file_[0] = 0;
          break;
        }
        err_ = "bad bytes after a boundary";
        res_ = MP_ERROR;
        break;
      case S_HEAD:
        if (c == '\n') {
          if (hn_ && head_[hn_ - 1] == '\r') hn_--;
          head_[hn_] = 0;
          if (hn_ == 0) {  // empty line: the data starts
            parts_++;
            st_ = S_DATA;
            m_ = 0;
            on_ = 0;
            if (!cb_.on_part(cb_.ctx, name_, file_)) res_ = MP_STOPPED;
            break;
          }
          header_line();
          hn_ = 0;
        } else if (hn_ < sizeof(head_) - 1) {
          head_[hn_++] = (char)c;
        } else {
          err_ = "part header line too long";
          res_ = MP_ERROR;
        }
        break;
      default:  // epilogue after the closing delimiter: ignored
        break;
    }
  }
  if (res_ == MP_MORE && st_ == S_DATA && !flush()) res_ = MP_STOPPED;
  return (Result)res_;
}

}  // namespace multipart
