// Copyright 2025, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#include "./Geo.h"

// _____________________________________________________________________________
util::geo::WKTType util::geo::getWKTType(const char* c, const char** endr) {
  bool measurement = false;
  while (true) {
    // TODO: how do we handle another <, is there need for further checks?

    // Check for possible IRI and skip it.
    if (*c == '<') {
      if (measurement) break;  // Measurement M cannot be in front of IRI.
      while (*c && *c != '>') c++;
      if (*c == '>') c++;  // Also skip '>'.
      continue;
    }
    if ((*c == ' ' || *c == '\n' || *c == '\t' || *c == '\r') ||
        ((*c) == '"') || ((*c) == '\'')) {
      c++;  // skip possible whitespace
      continue;
    }
    if ((tolower(*c) == 'm' && ((*(c + 1) == ' ' || *(c + 1) == '\n' ||
                                 *(c + 1) == '\t' || *(c + 1) == '\r') ||
                                tolower(*(c + 1)) != 'u'))) {
      c++;  // skip possible measurement M
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
          ((*c) == '"') || ((*c) == '\'')) &&
         ((*c) != '\0'))
    c++;  // Skip possible whitespace.

  // If 'endr == nullptr' this function should still update it, for this
  // 'endr' is replaced accordingly.
  const char* replacement = nullptr;
  endr = (endr != nullptr) ? endr : &replacement;

  if (*c != '<') {
    if (endr) (*endr) = c;
    return CRS84;  // Default.
  }

  if (strncicmp("<http://www.opengis.net/def/crs/OGC/1.3/CRS84>", c, 46) == 0) {
    if (endr) (*endr) = c + 46;
    return CRS84;
  }
  if (strncicmp("<http://www.opengis.net/def/crs/EPSG/0/4326>", c, 44) == 0) {
    if (endr) (*endr) = c + 44;
    return WGS84;
  }
  if (strncicmp("<http://www.opengis.net/def/crs/EPSG/0/3857>", c, 44) == 0) {
    if (endr) (*endr) = c + 44;
    return WEB_MERCATOR;
  }

  if (endr) (*endr) = c;
  return UNSUPPORTED;
}

// _____________________________________________________________________________
util::geo::WKTType util::geo::getWKTType(const char* c) {
  return util::geo::getWKTType(c, 0);
}

// _____________________________________________________________________________
util::geo::CRSType util::geo::getCRSType(const char* c) {
  return util::geo::getCRSType(c, 0);
}

// _____________________________________________________________________________
util::geo::WKTType util::geo::getWKTType(const std::string& str) {
  return util::geo::getWKTType(str.c_str(), 0);
}

// _____________________________________________________________________________
util::geo::CRSType util::geo::getCRSType(const std::string& str) {
  return util::geo::getCRSType(str.c_str(), 0);
}

