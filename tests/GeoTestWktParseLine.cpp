// Copyright 2016, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#include "util/Test.h"
#include "util/geo/Geo.h"
#include "util/geo/Grid.h"
#include "util/geo/RTree.h"
#include "util/geo/output/GeoJsonOutput.h"
#include "util/log/Log.h"
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

  {
    // strict parsing
    TEST(getWKT(util::geo::lineFromWKT<double>("LINESTRING(0 0, 1 1)", true)),
         ==, "LINESTRING(0 0,1 1)");
    TEST(getWKT(util::geo::lineFromWKT<double>("LINESTRING(0 0, 1 1)", false)),
         ==, "LINESTRING(0 0,1 1)");

    TEST(getWKT(util::geo::lineFromWKT<double>(
             "   LINESTRING Z   (  0    0  , 1  1   )", true)),
         ==, "LINESTRING(0 0,1 1)");
    TEST(getWKT(util::geo::lineFromWKT<double>(
             "   LINESTRING Z   (  0    0  , 1  1   )", false)),
         ==, "LINESTRING(0 0,1 1)");

    TEST(getWKT(util::geo::lineFromWKT<double>("(0 0, 1 1)", true)), ==,
         "LINESTRING(0 0,1 1)");
    TEST(getWKT(util::geo::lineFromWKT<double>("(0 0, 1 1)", false)), ==,
         "LINESTRING(0 0,1 1)");

    TEST(getWKT(util::geo::lineFromWKT<double>("MLINESTRING (0 0 0.5, 1 1 100)",
                                               true)),
         ==, "LINESTRING(0 0,1 1)");
    TEST(getWKT(util::geo::lineFromWKT<double>("MLINESTRING (0 0 0.5, 1 1 100)",
                                               false)),
         ==, "LINESTRING(0 0,1 1)");

    TEST(getWKT(util::geo::lineFromWKT<double>(
             "<http://www.opengis.net/def/crs/OGC/1.3/CRS84> LINESTRING(0 0, 1 "
             "1)",
             true)),
         ==, "LINESTRING(0 0,1 1)");
    TEST(getWKT(util::geo::lineFromWKT<double>(
             "<http://www.opengis.net/def/crs/OGC/1.3/CRS84> LINESTRING(0 0, 1 "
             "1)",
             false)),
         ==, "LINESTRING(0 0,1 1)");

    TEST(getWKT(util::geo::lineFromWKT<double>("linestring(0 0, 1 1)", true)),
         ==, "LINESTRING(0 0,1 1)");
    TEST(getWKT(util::geo::lineFromWKT<double>("linestring(0 0, 1 1)", false)),
         ==, "LINESTRING(0 0,1 1)");

    TEST(util::geo::lineFromWKT<double>("LINESTRING EMPTY", true).size(), ==,
         0);
    TEST(util::geo::lineFromWKT<double>("LINESTRING EMPTY", false).size(), ==,
         0);

    TEST(util::geo::lineFromWKT<double>("LINESTRING Z EMPTY", true).size(), ==,
         0);
    TEST(util::geo::lineFromWKT<double>("LINESTRING Z EMPTY", false).size(), ==,
         0);

    TEST(util::geo::lineFromWKT<double>("linestring empty", true).size(), ==,
         0);
    TEST(util::geo::lineFromWKT<double>("linestring empty", false).size(), ==,
         0);

    TEST(util::geo::lineFromWKT<double>("  LINESTRING   EMPTY  ", true).size(),
         ==, 0);
    TEST(util::geo::lineFromWKT<double>("  LINESTRING   EMPTY  ", false).size(),
         ==, 0);

    TEST(util::geo::lineFromWKT<double>("EMPTY", true).size(), ==, 0);
    TEST(util::geo::lineFromWKT<double>("EMPTY", false).size(), ==, 0);

    TEST(util::geo::lineFromWKT<double>("   empty", true).size(), ==, 0);
    TEST(util::geo::lineFromWKT<double>("   empty", false).size(), ==, 0);

    TEST(util::geo::lineFromWKT<double>(
             "<http://www.opengis.net/def/crs/OGC/1.3/CRS84> LINESTRING EMPTY",
             true)
             .size(),
         ==, 0);
    TEST(util::geo::lineFromWKT<double>(
             "<http://www.opengis.net/def/crs/OGC/1.3/CRS84> LINESTRING EMPTY",
             false)
             .size(),
         ==, 0);

    TEST(util::geo::lineFromWKT<double>("linestring empty", true).size(), ==,
         0);
    TEST(util::geo::lineFromWKT<double>("linestring empty", false).size(), ==,
         0);

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::lineFromWKT<double>("", true));
    TEST(util::geo::lineFromWKT<double>("", false).size(), ==, 0);

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::lineFromWKT<double>("   ", true));
    TEST(util::geo::lineFromWKT<double>("   ", false).size(), ==, 0);

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::lineFromWKT<double>("LINESTRING", true));
    TEST(util::geo::lineFromWKT<double>("LINESTRING", false).size(), ==, 0);

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::lineFromWKT<double>("LINESTRING EMPTYX", true));
    TEST(util::geo::lineFromWKT<double>("LINESTRING EMPTYX", false).size(), ==,
         0);

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::lineFromWKT<double>("LINESTRING EMPT", true));
    TEST(util::geo::lineFromWKT<double>("LINESTRING EMPT", false).size(), ==,
         0);

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::lineFromWKT<double>("EMPTY LINESTRING", true));
    TEST(util::geo::lineFromWKT<double>("EMPTY LINESTRING", false).size(), ==,
         0);

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::lineFromWKT<double>("LINESTRING(0 0, 1 1", true));
    TEST(util::geo::lineFromWKT<double>("LINESTRING(0 0, 1 1", false).size(),
         ==, 0);

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::lineFromWKT<double>("LINESTRING 0 0, 1 1)", true));
    TEST(util::geo::lineFromWKT<double>("LINESTRING 0 0, 1 1)", false).size(),
         ==, 0);

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::lineFromWKT<double>("LINESTRING()", true));
    TEST(util::geo::lineFromWKT<double>("LINESTRING()", false).size(), ==, 0);

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::lineFromWKT<double>("LINESTRING(5)", true));
    TEST(util::geo::lineFromWKT<double>("LINESTRING(5)", false).size(), ==, 0);

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::lineFromWKT<double>("LINESTRING(0 0, 1)", true));
    TEST(util::geo::lineFromWKT<double>("LINESTRING(0 0, 1)", false).size(), ==,
         0);

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::lineFromWKT<double>("LINESTRING(0 0,)", true));
    TEST(util::geo::lineFromWKT<double>("LINESTRING(0 0,)", false).size(), ==,
         0);

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::lineFromWKT<double>("linestring emptay", true));
    TEST(util::geo::lineFromWKT<double>("linestring emptay", false).size(), ==,
         0);

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::lineFromWKT<double>("linestring(a b) ", true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::lineFromWKT<double>("linestring(1 1, a b) ", true));
  }

  {
    TEST(getWKT(util::geo::lineFromWKTProj<double>(
             std::string("LINESTRING(0 0, 1 1)"),
             util::geo::projectToCRS84<double>, true)),
         ==, "LINESTRING(0 0,1 1)");
    TEST(getWKT(util::geo::lineFromWKTProj<double>(
             "LINESTRING(0 0, 1 1)", 0, util::geo::projectToCRS84<double>,
             true)),
         ==, "LINESTRING(0 0,1 1)");
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::lineFromWKTProj<double>(
                    std::string("LINESTRING(0 0, 1)"),
                    util::geo::projectToCRS84<double>, true));
    TEST_THROWS(
        util::geo::WKTParseException,
        util::geo::lineFromWKTProj<double>(
            "LINESTRING(0 0, 1)", 0, util::geo::projectToCRS84<double>, true));
    TEST(getWKT(util::geo::lineFromWKTProj<double>(
             std::string("LINESTRING(0 0, 1)"),
             util::geo::projectToCRS84<double>)),
         ==, "LINESTRING()");
  }
}
