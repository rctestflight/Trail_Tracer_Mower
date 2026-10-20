// multipart: a streaming parser for an HTTP multipart/form-data body (RFC 7578), as a browser sends a form with file
// inputs. The body comes in pieces of any size; no part is kept in memory. Plain C++, no Arduino: it also builds on a PC.
// For each part: on_part(name, filename) once, on_data(bytes) zero or more times, on_end() once. A callback that
// returns false stops the parse (feed() then returns MP_STOPPED).
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace multipart {

enum Result : uint8_t { MP_MORE, MP_DONE, MP_ERROR, MP_STOPPED };

struct Callbacks {
  void* ctx;
  bool (*on_part)(void* ctx, const char* name, const char* filename);
  bool (*on_data)(void* ctx, const uint8_t* b, size_t n);
  bool (*on_end)(void* ctx);
};

// The boundary from a Content-Type value ("multipart/form-data; boundary=..."), quoted or not. False: none, or longer
// than 70 characters (RFC 2046).
bool boundary_from(const char* content_type, char* out, size_t cap);

class Parser {
 public:
  void begin(const char* boundary, const Callbacks& cb);
  Result feed(const uint8_t* b, size_t n);  // MP_MORE: wants more; MP_DONE: the closing delimiter came
  Result state() const { return (Result)res_; }
  const char* error() const { return err_; }
  uint32_t parts() const { return parts_; }

 private:
  enum : uint8_t { S_PRE, S_HEAD, S_DATA, S_AFTER, S_AFTER2, S_EPI };
  bool flush();
  bool emit(uint8_t c) { out_[on_++] = c; return on_ < sizeof(out_) || flush(); }
  bool header_line();
  Callbacks cb_ = {};
  char delim_[80];          // "\r\n--" + boundary
  uint8_t fail_[80];        // failure table of delim_ (Knuth-Morris-Pratt)
  uint8_t dl_ = 0, m_ = 0;  // delimiter length, bytes of it matched
  uint8_t st_ = S_PRE, res_ = MP_MORE;
  char after_[2];
  char head_[320];          // one header line
  uint16_t hn_ = 0;
  char name_[24], file_[64];
  uint8_t out_[256];
  uint16_t on_ = 0;
  uint32_t parts_ = 0;
  const char* err_ = "";
};

}  // namespace multipart
