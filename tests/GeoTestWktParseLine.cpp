// Copyright 2016, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#include "util/Test.h"
#include "util/log/Log.h"
#include "util/geo/Geo.h"
#include "util/geo/Grid.h"
#include "util/geo/RTree.h"
#include "util/geo/output/GeoJsonOutput.h"
#include "util/tests/GeoTest.h"

using namespace util;
using namespace util::geo;

// _____________________________________________________________________________
void GeoTest::testWktParseLine() {
  // WKT PARSING
  {
    TEST(util::geo::getWKTType("LINESTRING(0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);

    TEST(util::geo::getWKTType("MLINESTRING(0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);
    TEST(util::geo::getWKTType("  MLINESTRING (0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);
    TEST(util::geo::getWKTType("  MLINESTRING Z(0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);
    TEST(util::geo::getWKTType("  LINESTRING Z(0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);
    TEST(util::geo::getWKTType("  LINESTRING M(0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);
    TEST(util::geo::getWKTType("  LINESTRING ZM(0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);
    TEST(util::geo::getWKTType("  LINESTRING (0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);
    TEST(util::geo::getWKTType("  LINESTRIN (0 0, 1 1)", 0), ==, WKTType::NONE);
    TEST(util::geo::getWKTType("  INESTRING (0 0, 1 1)", 0), ==, WKTType::NONE);

    TEST(util::geo::getWKTType("linestring(0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);
    TEST(util::geo::getWKTType("mlinestring(0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);
    TEST(util::geo::getWKTType("  mlinestring (0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);
    TEST(util::geo::getWKTType("  mlinestring z(0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);
    TEST(util::geo::getWKTType("  linestring z(0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);
    TEST(util::geo::getWKTType("  linestring (0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);

    TEST(util::geo::getWKTType("liNestRing(0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);
    TEST(util::geo::getWKTType("mlinestrIng(0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);
    TEST(util::geo::getWKTType("  mliNestring (0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);
    TEST(util::geo::getWKTType("  MlInestring z(0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);
    TEST(util::geo::getWKTType("  Linestring z(0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);
    TEST(util::geo::getWKTType("  Linestring ZM(0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);
    TEST(util::geo::getWKTType("  linestrinG (0 0, 1 1)", 0), ==,
         WKTType::LINESTRING);

    TEST(util::geo::getWKT(lineFromWKT<int>("LINESTRING(0 0, 1 1)", 0)), ==,
         "LINESTRING(0 0,1 1)");
    TEST(util::geo::getWKT(lineFromWKT<int>("LINESTRING (0 0, 1 1)", 0)), ==,
         "LINESTRING(0 0,1 1)");
    TEST(util::geo::getWKT(lineFromWKT<int>("LINESTRING   (0 0, 1 1)", 0)), ==,
         "LINESTRING(0 0,1 1)");
    TEST(util::geo::getWKT(
             lineFromWKT<int>("   LINESTRING   (  0    0  , 1  1   )", 0)),
         ==, "LINESTRING(0 0,1 1)");

    TEST(util::geo::getWKT(lineFromWKT<int>("(0 0, 1 1)", 0)), ==,
         "LINESTRING(0 0,1 1)");
    TEST(util::geo::getWKT(lineFromWKT<int>(" (0 0, 1 1)", 0)), ==,
         "LINESTRING(0 0,1 1)");
    TEST(util::geo::getWKT(lineFromWKT<int>("   (0 0, 1 1)", 0)), ==,
         "LINESTRING(0 0,1 1)");
    TEST(util::geo::getWKT(lineFromWKT<int>("      (  0    0  , 1  1   )", 0)),
         ==, "LINESTRING(0 0,1 1)");

    TEST(util::geo::getWKT(lineFromWKT<int>("LINESTRING Z(0 0, 1 1)", 0)), ==,
         "LINESTRING(0 0,1 1)");
    TEST(util::geo::getWKT(lineFromWKT<int>("LINESTRING Z (0 0, 1 1)", 0)), ==,
         "LINESTRING(0 0,1 1)");
    TEST(util::geo::getWKT(lineFromWKT<int>("LINESTRING Z  (0 0, 1 1)", 0)), ==,
         "LINESTRING(0 0,1 1)");
    TEST(util::geo::getWKT(
             lineFromWKT<int>("   LINESTRING Z   (  0    0  , 1  1   )", 0)),
         ==, "LINESTRING(0 0,1 1)");
    TEST(util::geo::getWKT(lineFromWKT<int>("MLINESTRING (0 0, 1 1)", 0)), ==,
         "LINESTRING(0 0,1 1)");
    TEST(util::geo::getWKT(lineFromWKT<int>("MLINESTRING  (0 0, 1 1)", 0)), ==,
         "LINESTRING(0 0,1 1)");
    TEST(util::geo::getWKT(lineFromWKT<int>("MLINESTRING   (0 0, 1 1)", 0)), ==,
         "LINESTRING(0 0,1 1)");
    TEST(util::geo::getWKT(lineFromWKT<int>(
             "   MLINESTRING    (  0    0  0.5 , 1  1 100  )", 0)),
         ==, "LINESTRING(0 0,1 1)");
  }
}
