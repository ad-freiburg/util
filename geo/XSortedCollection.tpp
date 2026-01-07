// Copyright 2025, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

// _____________________________________________________________________________
template <typename T>
XSortedCollection<T>::XSortedCollection(const MultiPolygon<T>& mp) {
  _polygons.reserve(mp.size());
  add(mp);
}

// _____________________________________________________________________________
template <typename T>
void XSortedCollection<T>::add(const MultiPolygon<T>& mp) {
  for (const auto& p : mp) {
    _polygons.push_back(p);
  }
}

// _____________________________________________________________________________
template <typename T>
void XSortedCollection<T>::add(const Polygon<T>& p) {
  _polygons.push_back(p);
}
