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
XSortedCollection<T>::XSortedCollection(const MultiLine<T>& ml) {
  add(ml);

  finalize();
}

// _____________________________________________________________________________
template <typename T>
XSortedCollection<T>::XSortedCollection(const MultiPoint<T>& mp) {
  add(mp);

  finalize();
}

// _____________________________________________________________________________
template <typename T>
XSortedCollection<T>::XSortedCollection(const Collection<T>& c) {
  add(c);

  finalize();
}

// _____________________________________________________________________________
template <typename T>
void XSortedCollection<T>::add(const Collection<T>& c) {
  for (const auto& a : c) {
    if (a.getType() == 0) add(a.getPoint());
    if (a.getType() == 1) add(a.getLine());
    if (a.getType() == 2) add(a.getPolygon());
    if (a.getType() == 3) {
      for (const auto& l : a.getMultiLine()) add(l);
    }
    if (a.getType() == 4) {
      for (const auto& p : a.getMultiPolygon()) add(p);
    }
    if (a.getType() == 5) {
      add(a.getCollection());
    }
    if (a.getType() == 6) {
      for (const auto& p : a.getMultiPoint()) add(p);
    }
  }
}

// _____________________________________________________________________________
template <typename T>
void XSortedCollection<T>::add(const MultiPolygon<T>& mp) {
  _polygons.reserve(mp.size());
  for (const auto& p : mp) add(p);
}

// _____________________________________________________________________________
template <typename T>
void XSortedCollection<T>::add(const MultiLine<T>& ml) {
  _lines.reserve(ml.size());
  for (const auto& l : ml) add(l);
}

// _____________________________________________________________________________
template <typename T>
void XSortedCollection<T>::add(const MultiPoint<T>& mp) {
  _points.reserve(mp.size());
  for (const auto& p : mp) add(p);
}

// _____________________________________________________________________________
template <typename T>
void XSortedCollection<T>::add(const Point<T>& p) {
  _points.push_back(p);
  _bbox = util::geo::extendBox(p, _bbox);
  _sweepEvents.push_back({p.getX(), _points.size() - 1, p.getY(), p.getY(),
                          false, 0, SWEEP_POINT});
  _sweepEvents.push_back(
      {p.getX(), _points.size() - 1, p.getY(), p.getY(), true, 0, SWEEP_POINT});
}

// _____________________________________________________________________________
template <typename T>
void XSortedCollection<T>::add(const Line<T>& l) {
  _lines.push_back(l);
  _bbox = util::geo::extendBox(_lines.back().boundingBox(), _bbox);
  _sweepEvents.push_back({_lines.back().boundingBox().getLowerLeft().getX(),
                          _lines.size() - 1,
                          _lines.back().boundingBox().getLowerLeft().getY(),
                          _lines.back().boundingBox().getUpperRight().getY(),
                          false, _lines.back().length(), SWEEP_LINESTRING});
  _sweepEvents.push_back({_lines.back().boundingBox().getUpperRight().getX(),
                          _lines.size() - 1,
                          _lines.back().boundingBox().getLowerLeft().getY(),
                          _lines.back().boundingBox().getUpperRight().getY(),
                          true, _lines.back().length(), SWEEP_LINESTRING});
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
