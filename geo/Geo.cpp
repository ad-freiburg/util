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
    return util::geo::CRSType::CRS84;  // Default.
  }

  if (strncicmp(crs84Iri, c, crs84IriLen) == 0) {
    if (endr) (*endr) = c + crs84IriLen;
    return util::geo::CRSType::CRS84;
  }
  if (strncicmp(wgs84Iri, c, wgs84IriLen) == 0) {
    if (endr) (*endr) = c + wgs84IriLen;
    return util::geo::CRSType::WGS84;
  }
  if (strncicmp(webMercIri, c, webMercIriLen) == 0) {
    if (endr) (*endr) = c + webMercIriLen;
    return util::geo::CRSType::WEB_MERCATOR;
  }

  if (endr) (*endr) = c;
  return util::geo::CRSType::UNSUPPORTED;
}

// _____________________________________________________________________________
const std::string& util::geo::getCrsIri(util::geo::CRSType targetCRS, bool hideIri) {
  static const std::string empty{""};
  static const std::string wgs84 = std::string{wgs84Iri} + " ";
  static const std::string webMerc = std::string{webMercIri} + " ";

  if (hideIri) return empty;

  switch (targetCRS)
  {
  case util::geo::CRSType::CRS84:
    // Not attaching IRI as CRS84 is the default.
    return empty;
  case util::geo::CRSType::WGS84:
    return wgs84;
  case util::geo::CRSType::WEB_MERCATOR:
    return webMerc;
  default:
    throw std::runtime_error("Trying to get CRS IRI for unsupported CRS type.");
  }
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

