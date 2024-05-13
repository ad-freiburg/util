// Copyright 2016, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Author: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#ifndef UTIL_GEO_POLYGON_H_
#define UTIL_GEO_POLYGON_H_

#include <vector>

#include "./Box.h"
#include "./Line.h"
#include "./Point.h"

namespace util {
namespace geo {

template <typename T>
using Ring = Line<T>;

template <typename T>
class Polygon {
 public:
  Polygon() {}

  explicit Polygon(const Ring<T>& l) : _outer(l) {}
  explicit Polygon(const Box<T>& b)
      : _outer({b.getLowerLeft(),
                Point<T>(b.getLowerLeft().getX(), b.getUpperRight().getY()),
                b.getUpperRight(),
                Point<T>(b.getUpperRight().getX(), b.getLowerLeft().getY())}) {}

  const Ring<T>& getOuter() const { return _outer; }
  Ring<T>& getOuter() { return _outer; }

  const std::vector<Ring<T>>& getInners() const { return _inners; }
  std::vector<Ring<T>>& getInners() { return _inners; }

 private:
  Ring<T> _outer;
  std::vector<Ring<T>> _inners;
};

template <typename T>
using MultiPolygon = std::vector<Polygon<T>>;

// _____________________________________________________________________________
template <typename T>
inline double ringArea(const Ring<T>& b) {
  double ret = 0;
  size_t j = b.size() - 1;
  for (size_t i = 0; i < b.size(); i++) {
    ret += (1.0 * b[j].getX() + 1.0 * b[i].getX()) *
           (1.0 * b[j].getY() - 1.0 * b[i].getY());
    j = i;
  }

  return fabs(ret / 2.0);
}

// _____________________________________________________________________________
template <typename T>
inline double outerArea(const Polygon<T>& b) {
  return ringArea(b.getOuter());
}

// _____________________________________________________________________________
template <typename T>
inline double area(const Polygon<T>& b) {
  double ret = ringArea(b.getOuter());
  for (const auto& inner : b.getInners()) ret -= ringArea(inner);

  return ret;
}

// _____________________________________________________________________________
template <typename T>
inline double area(const MultiPolygon<T>& b) {
  double ret = 0;

  for (const auto& p : b) ret += area(p);

  return ret;
}

// _____________________________________________________________________________
template <typename T>
inline double signedRingArea(const Ring<T>& b) {
  double ret = 0;
  size_t j = b.size() - 1;
  for (size_t i = 0; i < b.size(); i++) {
    ret += (1.0 * b[j].getX() + 1.0 * b[i].getX()) *
           (1.0 * b[j].getY() - 1.0 * b[i].getY());
    j = i;
  }

  return ret / 2.0;
}

// _____________________________________________________________________________
template <typename T>
inline double signedArea(const Polygon<T>& b) {
  double ret = signedRingArea(b.getOuter());
  for (const auto& inner : b.getInners()) ret += signedRingArea(inner);

  return ret;
}

// _____________________________________________________________________________
template <typename T>
inline double signedArea(const MultiPolygon<T>& b) {
  double ret = 0;

  for (const auto& p : b) ret += signedArea(p);

  return ret;
}

// _____________________________________________________________________________
template <typename T>
class XSortedRing {
 public:
  XSortedRing() {}
  XSortedRing(const Box<T>& box) : _ring(8) {
    _maxSegLen = box.getUpperRight().getX() - box.getLowerLeft().getX();

    _ring[0] = {box.getLowerLeft(),
                {box.getLowerLeft(), box.getUpperLeft()},
                true,
                M_PI / 2,
                -M_PI / 2,
                false};
    _ring[1] = {box.getLowerLeft(),
                {box.getLowerLeft(), box.getLowerRight()},
                false,
                M_PI / 2,
                -M_PI / 2,
                false};
    _ring[2] = {box.getUpperLeft(),
                {box.getUpperLeft(), box.getUpperRight()},
                true,
                M_PI / 2,
                -M_PI / 2,
                false};
    _ring[3] = {box.getUpperLeft(),
                {box.getLowerLeft(), box.getUpperLeft()},
                true,
                M_PI / 2,
                -M_PI / 2,
                true};

    _ring[4] = {box.getLowerRight(),
                {box.getLowerRight(), box.getUpperRight()},
                false,
                M_PI / 2,
                -M_PI / 2,
                false};
    _ring[5] = {box.getLowerRight(),
                {box.getLowerLeft(), box.getLowerRight()},
                false,
                M_PI / 2,
                -M_PI / 2,
                true};
    _ring[6] = {box.getUpperRight(),
                {box.getUpperLeft(), box.getUpperRight()},
                true,
                M_PI / 2,
                -M_PI / 2,
                true};
    _ring[7] = {box.getUpperRight(),
                {box.getLowerRight(), box.getUpperRight()},
                false,
                M_PI / 2,
                -M_PI / 2,
                true};
  }

  XSortedRing(const Ring<T>& ring) {
    _ring.reserve(ring.size());

    for (size_t i = 1; i < ring.size(); i++) {
      if (ring[i - 1].getX() == ring[i].getX() &&
          ring[i - 1].getY() == ring[i].getY())
        continue;
      T len = fabs(ring[i - 1].getX() - ring[i].getX());
      if (len > _maxSegLen) _maxSegLen = len;

      double prevAng = 0;
      double nextAng = 0;

      size_t prev;

      if (i > 1)
        prev = i - 2;
      else
        prev = ring.size() - 1;

      while (ring[prev].getX() == ring[i - 1].getX() &&
             ring[prev].getY() == ring[i - 1].getY() && prev != i - 1) {
        if (prev > 0)
          prev = prev - 1;
        else
          prev = ring.size() - 1;
      }

      prevAng = util::geo::angBetween(
          ring[i], ring[i - 1],
          {ring[prev].getX() - (ring[i - 1].getX() - ring[i].getX()),
           ring[prev].getY() - (ring[i - 1].getY() - ring[i].getY())});

      size_t next = (i + 1) % ring.size();

      while (ring[next].getX() == ring[i].getX() &&
             ring[next].getY() == ring[i].getY() && next != i) {
        next = (next + 1) % ring.size();
      }

      nextAng = util::geo::angBetween(
          ring[i - 1], ring[i],
          {ring[next].getX() - (ring[i].getX() - ring[i - 1].getX()),
           ring[next].getY() - (ring[i].getY() - ring[i - 1].getY())});

      if (len > _maxSegLen) _maxSegLen = len;

      if (ring[i - 1].getX() < ring[i].getX()) {
        _ring.push_back({ring[i - 1],
                         {ring[i - 1], ring[i]},
                         false,
                         prevAng,
                         nextAng,
                         false});
        _ring.push_back(
            {ring[i], {ring[i - 1], ring[i]}, false, prevAng, nextAng, true});
      } else {
        _ring.push_back(
            {ring[i], {ring[i], ring[i - 1]}, true, prevAng, nextAng, false});
        _ring.push_back({ring[i - 1],
                         {ring[i], ring[i - 1]},
                         true,
                         prevAng,
                         nextAng,
                         true});
      }
    }

    if (ring.size() > 1 && !(ring[ring.size() - 1].getX() == ring[0].getX() &&
                             ring[ring.size() - 1].getY() == ring[0].getY())) {
      size_t i = ring.size();
      T len = fabs(ring[i - 1].getX() - ring[0].getX());
      if (len > _maxSegLen) _maxSegLen = len;

      size_t prev;

      if (i > 1)
        prev = i - 2;
      else
        prev = ring.size() - 1;

      while (ring[prev].getX() == ring[i - 1].getX() &&
             ring[prev].getY() == ring[i - 1].getY() && prev != i - 1) {
        if (prev > 0)
          prev = prev - 1;
        else
          prev = ring.size() - 1;
      }

      double prevAng = util::geo::angBetween(
          ring[0], ring[i - 1],
          {ring[prev].getX() - (ring[i - 1].getX() - ring[0].getX()),
           ring[prev].getY() - (ring[i - 1].getY() - ring[0].getY())});

      size_t next = 1;

      while (ring[next].getX() == ring[0].getX() &&
             ring[next].getY() == ring[0].getY() && next != 0) {
        next = (next + 1) % ring.size();
      }

      double nextAng = util::geo::angBetween(
          ring[i - 1], ring[0],
          {ring[next].getX() - (ring[0].getX() - ring[i - 1].getX()),
           ring[next].getY() - (ring[0].getY() - ring[i - 1].getY())});

      if (ring[i - 1].getX() < ring[0].getX()) {
        _ring.push_back({ring[i - 1],
                         {ring[i - 1], ring[0]},
                         false,
                         prevAng,
                         nextAng,
                         false});
        _ring.push_back(
            {ring[0], {ring[i - 1], ring[0]}, false, prevAng, nextAng, true});
      } else {
        _ring.push_back(
            {ring[0], {ring[0], ring[i - 1]}, true, prevAng, nextAng, false});
        _ring.push_back({ring[i - 1],
                         {ring[0], ring[i - 1]},
                         true,
                         prevAng,
                         nextAng,
                         true});
      }
    }

    std::sort(_ring.begin(), _ring.end());
  }

  T getMaxSegLen() const { return _maxSegLen; }
  void setMaxSegLen(T l) { _maxSegLen = l; }

  const std::vector<XSortedTuple<T>>& rawRing() const { return _ring; }
  std::vector<XSortedTuple<T>>& rawRing() { return _ring; }

 private:
  std::vector<XSortedTuple<T>> _ring;
  T _maxSegLen = -1;
};

template <typename T>
class XSortedPolygon {
 public:
  XSortedPolygon() {}
  XSortedPolygon(const Box<T>& box) : _outer(box) {}
  XSortedPolygon(const Polygon<T>& poly) {
    auto outer = poly.getOuter();

    // outer ring  must be oriented counter-clockwise
    if (signedRingArea(outer) > 0) std::reverse(outer.begin(), outer.end());

    _outer = outer;

    for (const auto& innerRaw : poly.getInners()) {
      // skip empty polygons
      if (innerRaw.size() < 2) continue;
      auto inner = innerRaw;

      // inner rings must be oriented clockwise
      if (signedRingArea(inner) < 0) std::reverse(inner.begin(), inner.end());

      _inners.push_back(inner);

      Box<T> box;
      for (const auto& p : inner) box = extendBox(p, box);
      _innerBoxes.push_back(box);
      _innerAreas.push_back(area(inner));
      _boxIdx.push_back({box.getLowerLeft().getX(), _innerAreas.size() - 1});
      if (box.getUpperRight().getX() - box.getLowerLeft().getX() >
          _innerMaxSegLen)
        _innerMaxSegLen =
            box.getUpperRight().getX() - box.getLowerLeft().getX();
    }

    std::sort(_boxIdx.begin(), _boxIdx.end());
  }

  const XSortedRing<T>& getOuter() const { return _outer; }
  XSortedRing<T>& getOuter() { return _outer; }

  const std::vector<XSortedRing<T>>& getInners() const { return _inners; }
  std::vector<XSortedRing<T>>& getInners() { return _inners; }

  const std::vector<util::geo::Box<T>>& getInnerBoxes() const {
    return _innerBoxes;
  }
  std::vector<util::geo::Box<T>>& getInnerBoxes() { return _innerBoxes; }

  const std::vector<std::pair<T, size_t>>& getInnerBoxIdx() const {
    return _boxIdx;
  }
  std::vector<std::pair<T, size_t>>& getInnerBoxIdx() { return _boxIdx; }

  const std::vector<double>& getInnerAreas() const { return _innerAreas; }
  std::vector<double>& getInnerAreas() { return _innerAreas; }

  T getInnerMaxSegLen() const { return _innerMaxSegLen; }
  void setInnerMaxSegLen(T len) { _innerMaxSegLen = len; }

 private:
  XSortedRing<T> _outer;
  std::vector<XSortedRing<T>> _inners;
  std::vector<double> _innerAreas;
  std::vector<util::geo::Box<T>> _innerBoxes;
  std::vector<std::pair<T, size_t>> _boxIdx;
  T _innerMaxSegLen = 0;
};

template <typename T>
using XSortedMultiPolygon = std::vector<XSortedPolygon<T>>;

}  // namespace geo
}  // namespace util

#endif  // UTIL_GEO_LINE_H_
