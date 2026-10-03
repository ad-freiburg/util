// Copyright 2025, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#include "./Geo.h"

// _____________________________________________________________________________
uint8_t util::geo::boolArrToInt8(const std::array<bool, 8> arr) {
  uint8_t ret = 0;
  for (size_t i = 0; i < 8; i++) ret |= (uint8_t)arr[i] << i;
  return ret;
}

// _____________________________________________________________________________
bool util::geo::doubleEq(double a, double b) { return fabs(a - b) < EPSILON; }

// _____________________________________________________________________________
double util::geo::innerProd(double x1, double y1, double x2, double y2,
                            double x3, double y3) {
  double dx21 = x2 - x1;
  double dx31 = x3 - x1;
  double dy21 = y2 - y1;
  double dy31 = y3 - y1;
  double m12 = sqrt(dx21 * dx21 + dy21 * dy21);
  double m13 = sqrt(dx31 * dx31 + dy31 * dy31);
  double theta = acos(std::min((dx21 * dx31 + dy21 * dy31) / (m12 * m13), 1.0));

  return theta * IRAD;
}

// _____________________________________________________________________________
double util::geo::crossProd(double x1, double y1, double x2, double y2) {
  return x1 * y2 - x2 * y1;
}

// _____________________________________________________________________________
util::geo::WKTType util::geo::getWKTType(const char* c, const char** endr) {
  bool measurement = false;
  while(true) {

    // TODO: how do we handle another <, is there need for further checks?

    // Check for possible IRI and skip it.
    if (*c == '<') {
      if (measurement) break; // Measurement M cannot be in front of IRI.
      while (*c && *c != '>') c++;
      if (*c == '>') c++; // Also skip '>'.
      continue;
    }
    if ((*c == ' ' || *c == '\n' || *c == '\t' || *c == '\r') ||
         ((*c) == '"') || ((*c) == '\'')) {
      c++; // skip possible whitespace
      continue;
    }
    if ((tolower(*c) == 'm' && ((*(c + 1) == ' ' || *(c + 1) == '\n' || *(c + 1) == '\t' || 
          *(c + 1) == '\r') || tolower(*(c + 1)) != 'u'))) {
      c++; // skip possible measurement M
      measurement = true;
      continue;
    }
    break;
  }
  if (strncicmp("POINT", c, 5) == 0) {
    if (endr) (*endr) = c + 5;
    return POINT;
  }
  if (strncicmp("LINESTRING", c, 10) == 0) {
    if (endr) (*endr) = c + 10;
    return LINESTRING;
  }
  if (strncicmp("POLYGON", c, 7) == 0) {
    if (endr) (*endr) = c + 7;
    return POLYGON;
  }
  if (strncicmp("MULTIPOINT", c, 10) == 0) {
    if (endr) (*endr) = c + 10;
    return MULTIPOINT;
  }
  if (strncicmp("MULTILINESTRING", c, 15) == 0) {
    if (endr) (*endr) = c + 15;
    return MULTILINESTRING;
  }
  if (strncicmp("MULTIPOLYGON", c, 12) == 0) {
    if (endr) (*endr) = c + 12;
    return MULTIPOLYGON;
  }
  if (strncicmp("GEOMETRYCOLLECTION", c, 18) == 0) {
    if (endr) (*endr) = c + 18;
    return COLLECTION;
  }

  if (endr) (*endr) = 0;
  return NONE;
}


// _____________________________________________________________________________
util::geo::CRSType util::geo::getCRSType(const char* c, const char** endr) {
  while (((*c == ' ' || *c == '\n' || *c == '\t' || *c == '\r') ||
         ((*c) == '"') || ((*c) == '\'')) && ((*c) != '\0'))
    c++; // Skip possible whitespace.

  // If 'endr == nullptr' this function should still update it, for this
  // 'endr' is replaced accordingly.
  const char* replacement = nullptr;
  endr = (endr != nullptr) ? endr : &replacement;

  if (*c != '<') {
    if (endr) (*endr) = c;
    return CRS84;  // Default.
  }

  if (strncicmp(crs84Iri, c, crs84IriLen) == 0) {
    if (endr) (*endr) = c + crs84IriLen;
    return CRS84;
  }
  if (strncicmp(wgs84Iri, c, wgs84IriLen) == 0) {
    if (endr) (*endr) = c + wgs84IriLen;
    return WGS84;
  }
  if (strncicmp(webMercIri, c, webMercIriLen) == 0) {
    if (endr) (*endr) = c + webMercIriLen;
    return WEB_MERCATOR;
  }

  if (endr) (*endr) = c;
  return UNSUPPORTED;
}

// _____________________________________________________________________________
std::string util::geo::getCrsIri(util::geo::CRSType targetCRS) {
  switch (targetCRS)
  {
  case CRS84:
    // Not attaching IRI as CRS84 is the default.
    return "";
  case WGS84:
    return std::string{wgs84Iri} + " ";
  case WEB_MERCATOR:
    return std::string{webMercIri} + " ";
  default:
    throw std::runtime_error("Trying to get CRS IRI for unsupported CRS type.");
  }
}

// _____________________________________________________________________________
util::geo::Point<double> util::geo::appendPoint(std::string& ret, const util::geo::Point<double>& point, uint16_t prec, CRSType currentCRS, CRSType targetCRS) {
  auto projected = projectToCRS(point, currentCRS, targetCRS);
  ret.append(formatFloat(projected.getX(), prec));
  ret.push_back(' ');
  ret.append(formatFloat(projected.getY(), prec));
  return projected;
}

// _____________________________________________________________________________
std::string util::geo::getWKT(const Point<double>& p, uint16_t prec, CRSType currentCRS, CRSType targetCRS, bool hideIri) {
  std::string ret;
  const std::string iri = hideIri ? "" : getCrsIri(targetCRS);
  ret.reserve(iri.size() + 6 + prec + 3 + prec + 3 + 1);
  ret += iri;
  ret += "POINT(";
  appendPoint(ret, p, prec, currentCRS, targetCRS);
  ret.push_back(')');
  return ret;
}

// _____________________________________________________________________________
std::string util::geo::getWKT(const Point<double>& p, CRSType currentCRS, CRSType targetCRS, bool hideIri) {
  return getWKT(p, 6, currentCRS, targetCRS, hideIri);
}

// _____________________________________________________________________________
std::string util::geo::getWKT(const std::vector<Point<double>>& p, uint16_t prec, CRSType currentCRS, CRSType targetCRS, bool hideIri) {
  std::string ret;
  const std::string iri = hideIri ? "" : getCrsIri(targetCRS);
  ret.reserve(iri.size() + 10 + 1 + p.size() * (prec + 3) * 2 + 1);
  ret += iri;
  ret += "MULTIPOINT(";
  for (size_t i = 0; i < p.size(); i++) {
    if (i) ret.push_back(',');
    appendPoint(ret, p[i], prec, currentCRS, targetCRS);
  }
  ret.push_back(')');
  return ret;
}

// _____________________________________________________________________________
std::string util::geo::getWKT(const std::vector<Point<double>>& p, CRSType currentCRS, CRSType targetCRS, bool hideIri) {
  return getWKT(p, 6, currentCRS, targetCRS, hideIri);
}

// _____________________________________________________________________________
std::string util::geo::getWKT(const Line<double>& l, uint16_t prec, CRSType currentCRS, CRSType targetCRS, bool hideIri) {
  std::string ret;
  const std::string iri = hideIri ? "" : getCrsIri(targetCRS);
  ret.reserve(iri.size() + 10 + 1 + l.size() * (prec + 3) * 2 + 1);
  ret += iri;
  ret += "LINESTRING(";
  for (size_t i = 0; i < l.size(); i++) {
    if (i) ret.push_back(',');
    appendPoint(ret, l[i], prec, currentCRS, targetCRS);
  }
  ret.push_back(')');
  return ret;
}

// _____________________________________________________________________________
std::string util::geo::getWKT(const Line<double>& l, CRSType currentCRS, CRSType targetCRS, bool hideIri) {
  return getWKT(l, 6, currentCRS, targetCRS, hideIri);
}

// _____________________________________________________________________________
std::string util::geo::getWKT(const std::vector<Line<double>>& ls, uint16_t prec, CRSType currentCRS, CRSType targetCRS, bool hideIri) {
  std::string ret;
  const std::string iri = hideIri ? "" : getCrsIri(targetCRS);

  if (ls.size()) ret.reserve(iri.size() + 15 + 2 + ls[0].size() * (prec + 3) * 2 + 2);
  ret += iri;
  ret += "MULTILINESTRING(";

  for (size_t j = 0; j < ls.size(); j++) {
    if (j) ret.push_back(',');
    ret.push_back('(');
    for (size_t i = 0; i < ls[j].size(); i++) {
      if (i) ret.push_back(',');
      appendPoint(ret, ls[j][i], prec, currentCRS, targetCRS);
    }
    ret.push_back(')');
  }

  ret.push_back(')');
  return ret;
}

// _____________________________________________________________________________
std::string util::geo::getWKT(const std::vector<Line<double>>& ls, CRSType currentCRS, CRSType targetCRS, bool hideIri) {
  return getWKT(ls, 6, currentCRS, targetCRS, hideIri);
}

// _____________________________________________________________________________
std::string util::geo::getWKT(const Polygon<double>& p, uint16_t prec, CRSType currentCRS, CRSType targetCRS, bool hideIri) {
  if (p.getOuter().size() == 0) return hideIri ? "POLYGON()" : getCrsIri(targetCRS) + "POLYGON()";

  std::string ret;
  const std::string iri = hideIri ? "" : getCrsIri(targetCRS);
  ret.reserve(iri.size() + 7 + 2 + p.getOuter().size() * (prec + 3) * 2 + 2);
  ret += iri;
  ret += "POLYGON((";

  util::geo::Point<double> front;
  for (size_t i = 0; i < p.getOuter().size(); i++) {
    if (i > 0) ret.push_back(',');
    auto point = appendPoint(ret, p.getOuter()[i], prec, currentCRS, targetCRS);
    if (i == 0) front = point;
  }

  if (p.getOuter().front() != p.getOuter().back()) {
    ret.push_back(',');
    ret.append(formatFloat(front.getX(), prec));
    ret.push_back(' ');
    ret.append(formatFloat(front.getY(), prec));
  }
  ret.push_back(')');

  for (const auto& inner : p.getInners()) {
    ret.append(",(");
    for (size_t i = 0; i < inner.size(); i++) {
      if (i > 0) ret.push_back(',');
      auto point = appendPoint(ret, inner[i], prec, currentCRS, targetCRS);
      if (i == 0) front = point;
    }

    if (inner.front() != inner.back()) {
      ret.push_back(',');
      ret.append(formatFloat(front.getX(), prec));
      ret.push_back(' ');
      ret.append(formatFloat(front.getY(), prec));
    }
    ret.push_back(')');
  }
  ret.push_back(')');
  return ret;
}

// _____________________________________________________________________________
std::string util::geo::getWKT(const Polygon<double>& p, CRSType currentCRS, CRSType targetCRS, bool hideIri) {
  return getWKT(p, 6, currentCRS, targetCRS, hideIri);
}

// _____________________________________________________________________________
std::string util::geo::getWKT(const std::vector<Polygon<double>>& ls, uint16_t prec, CRSType currentCRS, CRSType targetCRS, bool hideIri) {
  std::string ret;
  const std::string iri = hideIri ? "" : getCrsIri(targetCRS);
  if (ls.size())
    ret.reserve(iri.size() + 12 + 2 + ls[0].getOuter().size() * (prec + 3) * 2 + 2);
  
  ret += iri;
  ret += "MULTIPOLYGON(";

  for (size_t j = 0; j < ls.size(); j++) {
    if (j) ret.push_back(',');
    ret.push_back('(');
    ret.push_back('(');

    util::geo::Point<double> front;
    for (size_t i = 0; i < ls[j].getOuter().size(); i++) {
      if (i > 0) ret.push_back(',');
      auto point = appendPoint(ret, ls[j].getOuter()[i], prec, currentCRS, targetCRS);
      if (i == 0) front = point;
    }

    if (ls[j].getOuter().front() != ls[j].getOuter().back()) {
      ret.push_back(',');
      ret.append(formatFloat(front.getX(), prec));
      ret.push_back(' ');
      ret.append(formatFloat(front.getY(), prec));
    }
    ret.push_back(')');

    for (const auto& inner : ls[j].getInners()) {
      ret.push_back(',');
      ret.push_back('(');
      for (size_t i = 0; i < inner.size(); i++) {
        if (i > 0) ret.push_back(',');
        auto point = appendPoint(ret, inner[i], prec, currentCRS, targetCRS);
        if (i == 0) front = point;
      }
      if (inner.front() != inner.back()) {
        ret.push_back(',');
        ret.append(formatFloat(front.getX(), prec));
        ret.push_back(' ');
        ret.append(formatFloat(front.getY(), prec));
      }
      ret.push_back(')');
    }
    ret.push_back(')');
  }

  ret.push_back(')');
  return ret;
}

// _____________________________________________________________________________
std::string util::geo::getWKT(const std::vector<Polygon<double>>& ls, CRSType currentCRS, CRSType targetCRS, bool hideIri) {
  return getWKT(ls, 6, currentCRS, targetCRS, hideIri);
}

// _____________________________________________________________________________
std::string util::geo::getWKT(const Collection<double>& coll, uint16_t prec, CRSType currentCRS, CRSType targetCRS, bool hideIri) {
  std::string ret;
  const std::string iri = hideIri ? "" : getCrsIri(targetCRS);
  ret += iri;
  ret += "GEOMETRYCOLLECTION(";

  std::string delim = "";

  for (const auto& g : coll) {
    ret += delim;
    delim = ",";
    if (g.getType() == 0) ret += util::geo::getWKT(g.getPoint(), prec, currentCRS, targetCRS, true);
    if (g.getType() == 1) ret += util::geo::getWKT(g.getLine(), prec, currentCRS, targetCRS, true);
    if (g.getType() == 2) ret += util::geo::getWKT(g.getPolygon(), prec, currentCRS, targetCRS, true);
    if (g.getType() == 3) ret += util::geo::getWKT(g.getMultiLine(), prec, currentCRS, targetCRS, true);
    if (g.getType() == 4) ret += util::geo::getWKT(g.getMultiPolygon(), prec, currentCRS, targetCRS, true);
    if (g.getType() == 5) ret += util::geo::getWKT(g.getCollection(), prec, currentCRS, targetCRS, true);
    if (g.getType() == 6) ret += util::geo::getWKT(g.getMultiPoint(), prec, currentCRS, targetCRS, true);
  }

  return ret + ")";
}

// _____________________________________________________________________________
std::string util::geo::getWKT(const Collection<double>& coll, CRSType currentCRS, CRSType targetCRS, bool hideIri) {
  return getWKT(coll, 6, currentCRS, targetCRS, hideIri);
}

// _____________________________________________________________________________
double util::geo::distToSegment(double lax, double lay, double lbx, double lby,
                                double px, double py) {
  double dx = lbx - lax;
  double dy = lby - lay;
  double d = dx * dx + dy * dy;
  if (d == 0) return dist(px, py, lax, lay);

  double dot = (px - lax) * dx + (py - lay) * dy;
  if (dot <= 0) return dist(px, py, lax, lay);
  if (dot >= d) return dist(px, py, lbx, lby);

  double t = dot / d;
  return dist(px, py, lax + t * dx, lay + t * dy);
}

// _____________________________________________________________________________
double util::geo::distToSegmentSquared(double lax, double lay, double lbx,
                                       double lby, double px, double py) {
  double dx = lbx - lax;
  double dy = lby - lay;
  double d = dx * dx + dy * dy;
  if (d == 0) return distSquared(px, py, lax, lay);

  double dot = (px - lax) * dx + (py - lay) * dy;
  if (dot <= 0) return distSquared(px, py, lax, lay);
  if (dot >= d) return distSquared(px, py, lbx, lby);

  double t = dot / d;
  return distSquared(px, py, lax + t * dx, lay + t * dy);
}

// _____________________________________________________________________________
util::geo::WKTType util::geo::getWKTType(const char* c) {
  return util::geo::getWKTType(c, 0);
}

// _____________________________________________________________________________
util::geo::CRSType util::geo::getCRSType(const char* c) { return util::geo::getCRSType(c, 0); }

// _____________________________________________________________________________
util::geo::WKTType util::geo::getWKTType(const std::string& str) {
  return util::geo::getWKTType(str.c_str(), 0);
}

// _____________________________________________________________________________
util::geo::CRSType util::geo::getCRSType(const std::string& str) {
  return util::geo::getCRSType(str.c_str(), 0);
}

// _____________________________________________________________________________
double util::geo::distSquared(double x1, double y1, double x2, double y2) {
  return (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
}

// _____________________________________________________________________________
double util::geo::dist(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// _____________________________________________________________________________
double util::geo::angBetween(double p1x, double p1y) { return atan2(p1x, p1y); }
