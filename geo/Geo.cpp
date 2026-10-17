// Copyright 2025, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#include "./Geo.h"

// _____________________________________________________________________________
util::geo::WKTType util::geo::getWKTType(const char* c, const char** endr,
                                         bool strict) {
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
    if (*c == ' ' || *c == '\n' || *c == '\t' || *c == '\r' || *c == '"' ||
        *c == '\'') {
      c++;  // skip possible whitespace
      continue;
    }
    if ((*c == 'M' || *c == 'm') && c[1] != 'U' && c[1] != 'u') {
      c++;  // skip possible measurement M
      measurement = true;
      continue;
    }
    break;
  }

  util::geo::WKTType ret = NONE;

  // dispatch on the first character, so at most two literals are compared
  switch (*c) {
    case 'P':
    case 'p':
      if (util::ascii_letter_strnicmp_lower(c, "point", 5) == 0) {
        c += 5;
        ret = POINT;
      } else if (util::ascii_letter_strnicmp_lower(c, "polygon", 7) == 0) {
        c += 7;
        ret = POLYGON;
      }
      break;
    case 'L':
    case 'l':
      if (util::ascii_letter_strnicmp_lower(c, "linestring", 10) == 0) {
        c += 10;
        ret = LINESTRING;
      }
      break;
    case 'M':
    case 'm':
      if (util::ascii_letter_strnicmp_lower(c, "multi", 5) != 0) break;
      if (util::ascii_letter_strnicmp_lower(c + 5, "point", 5) == 0) {
        c += 10;
        ret = MULTIPOINT;
      } else if (util::ascii_letter_strnicmp_lower(c + 5, "linestring", 10) ==
                 0) {
        c += 15;
        ret = MULTILINESTRING;
      } else if (util::ascii_letter_strnicmp_lower(c + 5, "polygon", 7) == 0) {
        c += 12;
        ret = MULTIPOLYGON;
      }
      break;
    case 'G':
    case 'g':
      if (util::ascii_letter_strnicmp_lower(c, "geometrycollection", 18) == 0) {
        c += 18;
        ret = COLLECTION;
      }
      break;
  }

  if (ret == NONE) {
    if (endr) (*endr) = 0;
    return NONE;
  }

  if (!strict) {
    if (endr) (*endr) = c;
    return ret;
  }

  // skip whitespace and possible measurment markers
  while (*c == 'z' || *c == 'Z' || *c == 'm' || *c == 'M' || *c == ' ' ||
         *c == '\n' || *c == '\t' || *c == '\r' || *c == '"' || (*c) == '\'') {
    c++;
  }

  if (ret != NONE && strict &&
      (*c != 0 && *c != '(' && *c != 'e' && *c != 'E')) {
    // no trailing stuff allowed in strict mode, note that e and E may be the
    // begining of EMPTY
    if (endr) (*endr) = 0;
    return NONE;
  }

  if (*c == 'e' || *c == 'E') {
    const char* r = c;
    if ((c[-1] == ' ' || c[-1] == '\n' || c[-1] == '\t' || c[-1] == '\r' ||
         c[-1] == '"' || c[-1] == '\'') &&
        util::ascii_letter_strnicmp_lower(r, "empty", 5) == 0) {
      r += 5;
      while (*r == ' ' || *r == '\n' || *r == '\t' || *r == '\r' || *r == '"' ||
             *r == '\'')
        r++;
    }
    const char* afterDatatype = skipWKTDatatype(r);
    if (r == c || (afterDatatype != r && *afterDatatype) ||
        (afterDatatype == r && *r != 0 && *r != ',' && *r != ')')) {
      if (endr) (*endr) = 0;
      return NONE;
    }
  }

  if (endr) (*endr) = c;
  return ret;
}

// ___________________________k_________________________________________________
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

  if (ascii_strnicmp_lower(c, "<http://www.opengis.net/def/crs/ogc/1.3/crs84>",
                           46) == 0) {
    if (endr) (*endr) = c + 46;
    return CRS84;
  }
  if (ascii_strnicmp_lower(c, "<http://www.opengis.net/def/crs/epsg/0/4326>",
                           44) == 0) {
    if (endr) (*endr) = c + 44;
    return WGS84;
  }
  if (ascii_strnicmp_lower(c, "<http://www.opengis.net/def/crs/epsg/0/3857>",
                           44) == 0) {
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
