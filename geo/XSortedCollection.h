// Copyright 2026, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Author: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#ifndef UTIL_GEO_XSORTEDCOLLECTION_H_
#define UTIL_GEO_XSORTEDCOLLECTION_H_

namespace util {
namespace geo {
template <typename T>
class XSortedCollection {
 public:
  XSortedCollection() {};
  XSortedCollection(const MultiPolygon<T>& mp);

  void add(const MultiPolygon<T>& mp);
  void add(const Polygon<T>& mp);

 private:
  std::vector<XSortedPolygon<T>> _polygons;
};

#include "util/geo/XSortedCollection.tpp"
}  // namespace geo
}  // namespace util

#endif  // UTIL_GEO_XSORTEDCOLLECTION_H_
