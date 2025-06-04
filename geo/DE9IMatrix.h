// Copyright 2016, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Author: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#ifndef UTIL_GEO_DE9IMATRIX_H_
#define UTIL_GEO_DE9IMATRIX_H_

#include <iostream>

namespace util {
namespace geo {

const static uint16_t F = 0b00;
const static uint16_t D0 = 0b01;
const static uint16_t D1 = 0b10;
const static uint16_t D2 = 0b11;

class DE9IMatrix {
 public:
  DE9IMatrix() {}

  DE9IMatrix(const char* m) {
    if (!m) return;
    for (size_t i = 0; i < 8; i++) {
      if (m[i] == '0') _m |= (D0 << (i * 2));
      if (m[i] == '1') _m |= (D1 << (i * 2));
      if (m[i] == '2') _m |= (D2 << (i * 2));
    }
  }

  DE9IMatrix(const std::string& m) : DE9IMatrix(m.c_str()) {}

  std::string toString() const {
    std::string ret = "FFFFFFFF2";
    for (size_t i = 0; i < 8; i++) {
      uint8_t v = (_m & (3 << (i * 2))) >> (i * 2);
      if (v == F) ret[i] = 'F';
      if (v == D0) ret[i] = '0';
      if (v == D1) ret[i] = '1';
      if (v == D2) ret[i] = '2';
    }
    return ret;
  }

  void setTo(size_t i, uint16_t v) {
    _m &= ~(D2 << (i * 2));
    _m |= (v << (i * 2));
  }

  uint16_t get(size_t i) const {
    uint16_t ret = _m & (D2 << (i * 2));
    return ret >> (i * 2);
  }

  void II(uint16_t v) { setTo(0, v); }
  void IB(uint16_t v) { setTo(1, v); }
  void IE(uint16_t v) { setTo(2, v); }
  void BI(uint16_t v) { setTo(3, v); }
  void BB(uint16_t v) { setTo(4, v); }
  void BE(uint16_t v) { setTo(5, v); }
  void EI(uint16_t v) { setTo(6, v); }
  void EB(uint16_t v) { setTo(7, v); }

  uint16_t II() const { return get(0); }
  uint16_t IB() const { return get(1); }
  uint16_t IE() const { return get(2); }
  uint16_t BI() const { return get(3); }
  uint16_t BB() const { return get(4); }
  uint16_t BE() const { return get(5); }
  uint16_t EI() const { return get(6); }
  uint16_t EB() const { return get(7); }

  DE9IMatrix transpose() const {
    DE9IMatrix ret;
    ret.II(II());
    ret.IB(BI());
    ret.IE(EI());
    ret.BI(IB());
    ret.BB(BB());
    ret.BE(EB());
    ret.EI(IE());
    ret.EB(BE());
    return ret;
  }

  DE9IMatrix& operator+=(const DE9IMatrix& other) {
    for (size_t i = 0; i < 8; i++) {
      if (other.get(i) >= get(i))
        setTo(i, other.get(i));
      else
        setTo(i, get(i));
    }
    return *this;
  }

  bool disjoint() const { return !II() && !IB() && !BI() && !BB(); }

  uint16_t getMatrix() const { return _m; }

 private:
  uint16_t _m = 0;
};

inline DE9IMatrix operator+(const DE9IMatrix a, const DE9IMatrix b) {
  DE9IMatrix ret;

  for (size_t i = 0; i < 8; i++) {
    if (a.get(i) >= b.get(i))
      ret.setTo(i, a.get(i));
    else
      ret.setTo(i, b.get(i));
  }

  return ret;
}

inline std::ostream& operator<<(std::ostream& os, const DE9IMatrix a) {
  os << a.toString();
  return os;
}

inline bool operator==(const DE9IMatrix a, const DE9IMatrix b) {
  return a.getMatrix() == b.getMatrix();
}

inline bool operator!=(const std::string& a, const DE9IMatrix b) {
  return a != b.toString();
}

inline bool operator!=(const DE9IMatrix a, const std::string& b) {
  return b != a;
}

inline bool operator!=(const DE9IMatrix a, const DE9IMatrix b) {
  return a.getMatrix() != b.getMatrix();
}

}  // namespace geo
}  // namespace util

#endif  // UTIL_GEO_DE9IMATRIX_H_
