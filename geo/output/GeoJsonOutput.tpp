// Copyright 2016, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

// _____________________________________________________________________________
template <typename T>
void GeoJsonOutput::print(const Point<T>& p, json::Val attrs) {
  _wr.obj();
  _wr.keyVal("type", "Feature");

  _wr.key("geometry");
  _wr.obj();
  _wr.keyVal("type", "Point");
  _wr.key("coordinates");
  _wr.arr();
  _wr.val(p.getX());
  _wr.val(p.getY());
  _wr.close();
  _wr.close();
  _wr.key("properties");
  _wr.val(attrs);
  _wr.close();
}

// _____________________________________________________________________________
template <typename T>
void GeoJsonOutput::print(const MultiPoint<T>& ps, json::Val attrs) {
  if (!ps.size()) return;
  _wr.obj();
  _wr.keyVal("type", "Feature");

  _wr.key("geometry");
  _wr.obj();
  _wr.keyVal("type", "MultiPoint");
  _wr.key("coordinates");
  _wr.arr();
  for (auto p : ps) {
    _wr.arr();
    _wr.val(p.getX());
    _wr.val(p.getY());
    _wr.close();
  }
  _wr.close();
  _wr.close();
  _wr.key("properties");
  _wr.val(attrs);
  _wr.close();
}

// _____________________________________________________________________________
template <typename T>
void GeoJsonOutput::print(const Line<T>& line, json::Val attrs) {
  if (!line.size()) return;
  _wr.obj();
  _wr.keyVal("type", "Feature");

  _wr.key("geometry");
  _wr.obj();
  _wr.keyVal("type", "LineString");
  _wr.key("coordinates");
  _wr.arr();
  for (auto p : line) {
    _wr.arr();
    _wr.val(p.getX());
    _wr.val(p.getY());
    _wr.close();
  }
  _wr.close();
  _wr.close();
  _wr.key("properties");
  _wr.val(attrs);
  _wr.close();
}

// _____________________________________________________________________________
template <typename T>
void GeoJsonOutput::print(const MultiLine<T>& lines, json::Val attrs) {
  if (!lines.size()) return;
  _wr.obj();
  _wr.keyVal("type", "Feature");

  _wr.key("geometry");
  _wr.obj();
  _wr.keyVal("type", "MultiLineString");
  _wr.key("coordinates");
  _wr.arr();
  for (auto l : lines) {
    _wr.arr();
    for (auto p : l) {
      _wr.arr();
      _wr.val(p.getX());
      _wr.val(p.getY());
      _wr.close();
    }
    _wr.close();
  }
  _wr.close();
  _wr.close();
  _wr.key("properties");
  _wr.val(attrs);
  _wr.close();
}

// _____________________________________________________________________________
template <typename T>
void GeoJsonOutput::print(const Polygon<T>& poly, json::Val attrs) {
  if (!poly.getOuter().size()) return;
  _wr.obj();
  _wr.keyVal("type", "Feature");

  _wr.key("geometry");
  _wr.obj();
  _wr.keyVal("type", "Polygon");
  _wr.key("coordinates");
  _wr.arr();

  _wr.arr();
  for (auto p : poly.getOuter()) {
    _wr.arr();
    _wr.val(p.getX());
    _wr.val(p.getY());
    _wr.close();
  }
  _wr.close();

  for (const auto& inner : poly.getInners()) {
    _wr.arr();
    for (auto p : inner) {
      _wr.arr();
      _wr.val(p.getX());
      _wr.val(p.getY());
      _wr.close();
    }
    _wr.close();
  }

  _wr.close();
  _wr.close();
  _wr.key("properties");
  _wr.val(attrs);
  _wr.close();
}

// _____________________________________________________________________________
template <typename T>
void GeoJsonOutput::print(const MultiPolygon<T>& mpoly, json::Val attrs) {
  if (!mpoly.size()) return;
  _wr.obj();
  _wr.keyVal("type", "Feature");

  _wr.key("geometry");
  _wr.obj();
  _wr.keyVal("type", "MultiPolygon");
  _wr.key("coordinates");
  _wr.arr();
  for (auto poly : mpoly) {
    _wr.arr();
    _wr.arr();
    for (auto p : poly.getOuter()) {
      _wr.arr();
      _wr.val(p.getX());
      _wr.val(p.getY());
      _wr.close();
    }
    _wr.close();

    for (const auto& inner : poly.getInners()) {
      _wr.arr();
      for (auto p : inner) {
        _wr.arr();
        _wr.val(p.getX());
        _wr.val(p.getY());
        _wr.close();
      }
      _wr.close();
    }
    _wr.close();
  }
  _wr.close();
  _wr.close();
  _wr.key("properties");
  _wr.val(attrs);
  _wr.close();
}

// _____________________________________________________________________________
template <typename T>
void GeoJsonOutput::printLatLng(const Point<T>& p, json::Val attrs) {
  auto projP = util::geo::webMercToLatLng<T>(p.getX(), p.getY());
  print(projP, attrs);
}

// _____________________________________________________________________________
template <typename T>
void GeoJsonOutput::printLatLng(const MultiPoint<T>& ps, json::Val attrs) {
  MultiPoint<T> projPs;
  for (auto p : ps)
    projPs.push_back(util::geo::webMercToLatLng<T>(p.getX(), p.getY()));

  print(projPs, attrs);
}

// _____________________________________________________________________________
template <typename T>
void GeoJsonOutput::printLatLng(const Line<T>& line, json::Val attrs) {
  Line<T> projL;
  for (auto p : line)
    projL.push_back(util::geo::webMercToLatLng<T>(p.getX(), p.getY()));

  print(projL, attrs);
}

// _____________________________________________________________________________
template <typename T>
void GeoJsonOutput::printLatLng(const MultiLine<T>& mline, json::Val attrs) {
  MultiLine<T> projLs;
  for (auto line : mline) {
    Line<T> projL;
    projL.reserve(line.size());
    for (auto p : line) {
      projL.push_back(util::geo::webMercToLatLng<T>(p.getX(), p.getY()));
    }
    projLs.push_back(projL);
  }

  print(projLs, attrs);
}

// _____________________________________________________________________________
template <typename T>
void GeoJsonOutput::printLatLng(const Polygon<T>& poly, json::Val attrs) {
  Polygon<T> projP;
  for (auto p : poly.getOuter())
    projP.getOuter().push_back(
        util::geo::webMercToLatLng<T>(p.getX(), p.getY()));

  for (auto pi : poly.getInners()) {
    Line<T> inner;
    for (auto p : pi) {
      inner.push_back(util::geo::webMercToLatLng<T>(p.getX(), p.getY()));
    }
    projP.getInners().push_back(inner);
  }

  print(projP, attrs);
}

// _____________________________________________________________________________
template <typename T>
void GeoJsonOutput::printLatLng(const MultiPolygon<T>& mpoly, json::Val attrs) {
  MultiPolygon<T> projPolys;
  for (auto poly : mpoly) {
    Polygon<T> projP;
    for (auto p : poly.getOuter()) {
      projP.getOuter().push_back(
          util::geo::webMercToLatLng<T>(p.getX(), p.getY()));
    }
    for (auto pi : poly.getInners()) {
      Line<T> inner;
      for (auto p : pi) {
        inner.push_back(util::geo::webMercToLatLng<T>(p.getX(), p.getY()));
      }
      projP.getInners().push_back(inner);
    }
    projPolys.push_back(projP);
  }

  print(projPolys, attrs);
}
