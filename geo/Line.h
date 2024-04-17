// Copyright 2016, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Author: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#ifndef UTIL_GEO_LINE_H_
#define UTIL_GEO_LINE_H_

#include <vector>

#include "./Point.h"

namespace util {
namespace geo {

template <typename T>
class Line : public std::vector<Point<T>> {
  using std::vector<Point<T>>::vector;
};

template <typename T>
using LineSegment = std::pair<Point<T>, Point<T>>;

template <typename T>
struct AngledLineSegment {
  LineSegment<T> seg;
  int16_t ang;
};

// _____________________________________________________________________________
template <typename T>
inline double angBetween(const Point<T>& o, const Point<T>& p1,
                         const Point<T>& p2) {
  double dy1 = p1.getY() - o.getY();
  double dy2 = p2.getY() - o.getY();
  double dx1 = p1.getX() - o.getX();
  double dx2 = p2.getX() - o.getX();

  return atan2(dy1 * dx2 - dy2 * dx1, dx1*dx2 + dy1*dy2);
}

template <typename T>
struct XSortedTuple {
  // a tuple used in X-sorted linestrings and polygons.
  // we do not store the corresponding line segment explicitly, but only the
  // other point, with "vals" holding which of the two points comes first on
  // bit 1, and whether this element is an out event on bit 0. Bit 2 is 1 if
  // the line is completely empty.
  // bit 3 holds whether the line segment was reversed due to the x order
  // for 64bit points, we thus safe 16 bytes (~29%), which let's more
  // XSortedTuples fit into the CPU caches during tests for intersect / contains
  // (in particular the binary search during startup), and also makes for faster
  // reading from disk
  Point<T> p;
  Point<T> other;
  int16_t outAngle;
  uint8_t vals : 4;

  XSortedTuple() {}
  XSortedTuple(const Point<T>& p, bool out) : p(p), vals(0) {
    if (out) vals = 1;
    vals += 4;
  }
  XSortedTuple(const Point<T>& p, const LineSegment<T>& seg, bool rev,
               double angle, bool out)
      : p(p), vals(0) {
    if (out) vals = 1;
    if (rev) vals += 8;
    if (seg.first == p) {
      vals += 2;
      other = seg.second;
    } else if (seg.second == p) {
      other = seg.first;
    }
    outAngle = (angle / M_PI) * 32767;
  }

  double ang() const { return ((1.0 * outAngle) / 32767.0) * M_PI; }

  int16_t rawAng() const { return outAngle; }

  LineSegment<T> seg() const {
    if (vals & 4) return {{0, 0}, {0, 0}};
    if (vals & 2) return {p, other};
    return {other, p};
  }

  LineSegment<T> origSeg() const {
    if (vals & 4) {
      return {{0, 0}, {0, 0}};
    }
    if (vals & 2) {
      if (vals & 8) return {other, p};
      return {p, other};
    }

    if (vals & 8) return {p, other};
    return {other, p};
  }

  AngledLineSegment<T> origSegAng() const {
    if (vals & 4) {
      return {{{0, 0}, {0, 0}}, rawAng()};
    }
    if (vals & 2) {
      if (vals & 8) return {{other, p}, rawAng()};
      return {{p, other}, rawAng()};
    }

    if (vals & 8) return {{p, other}, rawAng()};
    return {{other, p}, rawAng()};
  }

  bool out() const { return vals & 1; }
};

template <typename T>
bool operator<(const XSortedTuple<T>& a, const XSortedTuple<T>& b) {
  return a.p.getX() < b.p.getX() ||
         (a.p.getX() == b.p.getX() && !a.out() && b.out());
}

template <typename T>
bool operator<(const LineSegment<T>& a, const LineSegment<T>& b) {
  auto af = a.first;
  auto as = a.second;

  auto bf = b.first;
  auto bs = b.second;

  if (as.getX() < af.getX()) std::swap(af, as);
  if (bs.getX() < bf.getX()) std::swap(bf, bs);

  // this should be implicitely catched by the comparisons below,
  // however, because of floating point imprecisions, we might decide
  // < for equivalent segments. This is a problem, as segments are only
  // identified via they coordinates, not by some ID, in the active sets
  if (af.getX() == bf.getX() && af.getY() == bf.getY() &&
      as.getX() == bs.getX() && as.getY() == bs.getY()) {
    return false;
  }

  if (af.getX() < bf.getX() || bf.getX() == bs.getX()) {
    // a was first in active set
    if (af.getX() != as.getX()) {
      // check whether first point of b is right of or left of a
      double d =
          -((1.0 * bf.getX() - af.getX()) * (1.0 * as.getY() - af.getY()) -
            (1.0 * bf.getY() - af.getY()) * (1.0 * as.getX() - af.getX()));
      if (d < 0) return false;
      if (d > 0) return true;

      // if we arrive here, first point of b was EXACTLY on a, we have to decide
      // based on the second point of b
      d = -((1.0 * bs.getX() - af.getX()) * (1.0 * as.getY() - af.getY()) -
            (1.0 * bs.getY() - af.getY()) * (1.0 * as.getX() - af.getX()));
      if (d < 0) return false;
      if (d > 0) return true;
    }
  } else {
    // b was first in active set
    if (bf.getX() != bs.getX()) {
      // check whether first point of a is right of or left of b
      double d =
          ((1.0 * af.getX() - bf.getX()) * (1.0 * bs.getY() - bf.getY()) -
           (1.0 * af.getY() - bf.getY()) * (1.0 * bs.getX() - bf.getX()));
      if (d < 0) return false;
      if (d > 0) return true;

      // if we arrive here, first point of a was EXACTLY on b, we have to decide
      // based on the second point of a
      d = ((1.0 * as.getX() - bf.getX()) * (1.0 * bs.getY() - bf.getY()) -
           (1.0 * as.getY() - bf.getY()) * (1.0 * bs.getX() - bf.getX()));
      if (d < 0) return false;
      if (d > 0) return true;
    }
  }

  // a and b are colinear
  return af.getY() < bf.getY() ||
         (af.getY() == bf.getY() && af.getX() < bf.getX()) ||
         (af.getY() == bf.getY() && af.getX() == bf.getX() &&
          as.getY() < bs.getY()) ||
         (af.getY() == bf.getY() && af.getX() == bf.getX() &&
          as.getY() == bs.getY() && as.getX() < bs.getX());
}

template <typename T>
bool operator>(const LineSegment<T>& a, const LineSegment<T>& b) {
  return b < a;
}

template <typename T>
bool operator<(const AngledLineSegment<T>& a, const AngledLineSegment<T>& b) {
  return a.seg < b.seg;
}

template <typename T>
bool operator>(const AngledLineSegment<T>& a, const AngledLineSegment<T>& b) {
  return b.seg < a.seg;
}

template <typename T>
class XSortedLine {
 public:
  XSortedLine() {}

  XSortedLine(const Line<T>& line) {
    if (line.size() < 2) return;

    _line.reserve(2 * line.size());
    for (size_t i = 1; i < line.size(); i++) {
      double len = fabs(line[i - 1].getX() - line[i].getX());
      if (len > _maxSegLen) _maxSegLen = len;

      double ang = M_PI;

      size_t next = (i + 1) % line.size();

      while (line[next].getX() == line[i].getX() && line[next].getY() == line[i].getY() && next < line.size()) {
        next = next + 1;
      }

      if (next < line.size()) {
        ang = util::geo::angBetween(
            line[i - 1], line[i],
            {line[next].getX() - (line[i].getX() - line[i - 1].getX()),
             line[next].getY() - (line[i].getY() - line[i - 1].getY())});
      }

      if (line[i - 1].getX() < line[i].getX()) {
        _line.push_back({line[i - 1], {line[i - 1], line[i]}, false, ang, false});
        _line.push_back({line[i], {line[i - 1], line[i]}, false, ang, true});
      } else {
        _line.push_back({line[i], {line[i], line[i - 1]}, true, ang, false});
        _line.push_back({line[i - 1], {line[i], line[i - 1]}, true, ang, true});
      }
    }

    // beginning closure segment
    if (line[0].getX() < line[1].getX()) {
        _line.push_back({line[0], {line[0], line[1]}, true, M_PI, false});
        _line.push_back({line[1], {line[0], line[1]}, true, M_PI, true});
    } else {
        _line.push_back({line[1], {line[1], line[0]}, false, M_PI, false});
        _line.push_back({line[0], {line[1], line[0]}, false, M_PI, true});
    }

    std::sort(_line.begin(), _line.end());
  }

  XSortedLine(const LineSegment<T>& line) {
    _line.resize(4);
    if (line.first.getX() < line.second.getX()) {
      _line[0] = {line.first, {line.first, line.second}, true, M_PI, false};
      _line[1] = {line.first, {line.first, line.second}, false, M_PI, false};
      _line[2] = {line.second, {line.first, line.second}, true, M_PI, true};
      _line[3] = {line.second, {line.first, line.second}, false, M_PI, true};
    } else {
      _line[0] = {line.second, {line.second, line.first}, false, M_PI, false};
      _line[1] = {line.second, {line.second, line.first}, true, M_PI, false};
      _line[2] = {line.first, {line.second, line.first}, false, M_PI, true};
      _line[3] = {line.first, {line.second, line.first}, true, M_PI, true};
    }
    _maxSegLen = fabs(line.first.getX() - line.second.getX());
  }

  double getMaxSegLen() const { return _maxSegLen; }
  void setMaxSegLen(double l) { _maxSegLen = l; }

  const std::vector<XSortedTuple<T>>& rawLine() const { return _line; }
  std::vector<XSortedTuple<T>>& rawLine() { return _line; }

 private:
  std::vector<XSortedTuple<T>> _line;
  double _maxSegLen = -1;
};

template <typename T>
using MultiLine = std::vector<Line<T>>;

}  // namespace geo
}  // namespace util

#endif  // UTIL_GEO_LINE_H_
