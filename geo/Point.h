// Copyright 2016, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Author: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#ifndef UTIL_GEO_POINT_H_
#define UTIL_GEO_POINT_H_

#include <vector>

namespace util {
namespace geo {

template <typename T>
class Point {
 public:
  Point() : _x(0), _y(0), _crs(1) {}
  Point(T x, T y, uint8_t crs = 1) : _x(x), _y(y), _crs(crs) {}
  const T& getX() const { return _x; }
  const T& getY() const { return _y; }
  const uint8_t& getCRS() const { return _crs; }

  void setX(T x) { _x = x; }
  void setY(T y) { _y = y; }
  void setCRS(uint8_t crs) { _crs = crs; }

  Point<T> operator+(const Point<T>& p) const {
    return Point<T>(_x + p.getX(), _y + p.getY());
  }

  Point<T> operator-(const Point<T>& p) const {
    return Point<T>(_x - p.getX(), _y - p.getY());
  }

  bool operator==(const Point<T>& p) const {
    return p.getX() == _x && p.getY() == _y;
  }

  bool operator!=(const Point<T>& p) const { return !(*this == p); }

  bool operator<(const Point<T>& p) const {
    return _x < p.getX() || (_x == p.getX() && _y < p.getY());
  }

 private:
  T _x, _y;
  // As defined in `Geo.h` as `CRSType` enum.
  // `_crs = 1` is default `CRS84`.
  uint8_t _crs; 
};

template <typename T>
using MultiPoint = std::vector<Point<T>>;

}  // namespace geo
}  // namespace util

#endif  // UTIL_GEO_POINT_H_
