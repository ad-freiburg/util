// Copyright 2016, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Author: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#ifndef UTIL_GEO_POINT_H_
#define UTIL_GEO_POINT_H_

#include <limits>
#include <vector>

namespace util {
namespace geo {

template <typename T>
class Point {
 public:
  Point()
      : _x(std::numeric_limits<T>::lowest()),
        _y(std::numeric_limits<T>::max()) {}
  Point(T x, T y) : _x(x), _y(y) {}
  const T& getX() const { return _x; }
  const T& getY() const { return _y; }

  void setX(T x) { _x = x; }
  void setY(T y) { _y = y; }

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

  // return point in full precision
  Point<double> asDPoint() const {
    return {static_cast<double>(_x), static_cast<double>(_y)};
  }

  bool empty() const {
    return _x == std::numeric_limits<T>::lowest() &&
           _y == std::numeric_limits<T>::max();
  }

  inline static bool representable(double x, double y) {
    const double lo = static_cast<double>(std::numeric_limits<T>::lowest());

    if (std::numeric_limits<T>::is_integer) {
      // ensure that the limit will not be rounded to sth that is
      // not representable as an int (notabtly max of int64)
      const double hi =
          static_cast<double>(std::numeric_limits<T>::max() / 2 + 1) * 2.0;
      if (!(x >= lo && x < hi && y >= lo && y < hi)) return false;
    } else {
      const double hi = static_cast<double>(std::numeric_limits<T>::max());
      if (!(x >= lo && x <= hi && y >= lo && y <= hi)) return false;
    }

    // value pair which is used to mark EMPTY are also not representable
    return !(static_cast<T>(x) == std::numeric_limits<T>::lowest() &&
             static_cast<T>(y) == std::numeric_limits<T>::max());
  }

 private:
  T _x, _y;
};

template <typename T>
using MultiPoint = std::vector<Point<T>>;

template <typename T>
std::ostream& operator<<(std::ostream& o, const Point<T>& p) {
  o << "(" << p.getX() << "," << p.getY() << ")";
  return o;
}

}  // namespace geo
}  // namespace util

#endif  // UTIL_GEO_POINT_H_
