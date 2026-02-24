// Copyright 2025, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

// _____________________________________________________________________________
template <typename T>
XSortedCollection<T>::XSortedCollection(const MultiPolygon<T>& mp) {
  add(mp);

  finalize();
}

// _____________________________________________________________________________
template <typename T>
void XSortedCollection<T>::add(const MultiPolygon<T>& mp) {
  _polygons.reserve(mp.size());
  for (const auto& p : mp) {
    add(p);
  }
}

// _____________________________________________________________________________
template <typename T>
void XSortedCollection<T>::add(const Polygon<T>& p) {
  _polygons.push_back(p);
  _bbox = util::geo::extendBox(_polygons.back().boundingBox(), _bbox);
  _sweepEvents.push_back({_polygons.back().boundingBox().getLowerLeft().getX(),
                          _polygons.size() - 1,
                          _polygons.back().boundingBox().getLowerLeft().getY(),
                          _polygons.back().boundingBox().getUpperRight().getY(),
                          false, _polygons.back().area(), SWEEP_POLYGON});
  _sweepEvents.push_back({_polygons.back().boundingBox().getUpperRight().getX(),
                          _polygons.size() - 1,
                          _polygons.back().boundingBox().getLowerLeft().getY(),
                          _polygons.back().boundingBox().getUpperRight().getY(),
                          true, _polygons.back().area(), SWEEP_POLYGON});
}

// _____________________________________________________________________________
template <typename T>
void XSortedCollection<T>::finalize() {
  std::sort(_sweepEvents.begin(), _sweepEvents.end());
}
