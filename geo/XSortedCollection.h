// Copyright 2026, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Author: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#ifndef UTIL_GEO_XSORTEDCOLLECTION_H_
#define UTIL_GEO_XSORTEDCOLLECTION_H_

namespace util {
namespace geo {

enum GeomType : uint8_t {
  SWEEP_POINT = 1,
  SWEEP_LINESTRING = 2,
  SWEEP_POLYGON = 3,
};

struct BoxVal {
  int32_t x;
  size_t id;
  int32_t loY;
  int32_t upY;
  bool out;
  double areaOrLen;
  GeomType type;
};

template <typename T>
class XSortedCollection {
 public:
  XSortedCollection(){};
  XSortedCollection(const MultiPolygon<T>& mp);

  const std::vector<BoxVal>& sweepEvents() const { return _sweepEvents; }
  const XSortedPolygon<T>& getPolygon(size_t i) const { return _polygons[i]; }

  const util::geo::Box<T>& boundingBox() const { return _bbox; }

 private:
  std::vector<XSortedPolygon<T>> _polygons;
  std::vector<BoxVal> _sweepEvents;

  util::geo::Box<T> _bbox;

  void add(const MultiPolygon<T>& mp);
  void add(const Polygon<T>& mp);

  void finalize();
};

inline bool operator<(const BoxVal& boxa, const BoxVal& boxb) {
  if (boxa.x < boxb.x) return true;
  if (boxa.x > boxb.x) return false;

  if (!boxa.out && boxb.out) return true;
  if (boxa.out && !boxb.out) return false;

  // everything before a polygon
  if (boxa.type != SWEEP_POLYGON && (boxb.type == SWEEP_POLYGON)) return true;
  if ((boxa.type == SWEEP_POLYGON) && boxb.type != SWEEP_POLYGON) return false;

  // points before lines
  if ((boxa.type == SWEEP_POINT) && (boxb.type == SWEEP_LINESTRING))
    return true;
  if ((boxb.type == SWEEP_POINT) && (boxa.type == SWEEP_LINESTRING))
    return false;

  // smaller polygons before larger
  if ((boxa.type == SWEEP_POLYGON) && (boxb.type == SWEEP_POLYGON) &&
      boxa.areaOrLen < boxb.areaOrLen)
    return true;
  if ((boxa.type == SWEEP_POLYGON) && (boxb.type == SWEEP_POLYGON) &&
      boxa.areaOrLen > boxb.areaOrLen)
    return false;

  // shorter lines before longer
  if ((boxa.type == SWEEP_LINESTRING) && (boxb.type == SWEEP_LINESTRING) &&
      boxa.areaOrLen < boxb.areaOrLen)
    return true;
  if ((boxa.type == SWEEP_LINESTRING) && (boxb.type == SWEEP_LINESTRING) &&
      boxa.areaOrLen > boxb.areaOrLen)
    return false;

  return false;
}

inline bool operator>(const BoxVal& boxa, const BoxVal& boxb) { return boxb < boxa; }

inline bool operator==(const BoxVal& boxa, const BoxVal& boxb) {
  return !(boxb < boxa) && !(boxa < boxb);
}

#include "util/geo/XSortedCollection.tpp"
}  // namespace geo
}  // namespace util

#endif  // UTIL_GEO_XSORTEDCOLLECTION_H_
