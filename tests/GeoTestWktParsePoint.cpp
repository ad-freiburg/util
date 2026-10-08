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
void GeoTest::testWktParsePoint() {
  {
    TEST(util::geo::getWKTType("POINT(1 1)", 0), ==, WKTType::POINT);
    TEST(util::geo::getWKTType("POIN (1 1)", 0), ==, WKTType::NONE);
    TEST(util::geo::getWKTType(" POINT (1 1)", 0), ==, WKTType::POINT);
    TEST(util::geo::getWKTType(" Point (1 1)", 0), ==, WKTType::POINT);
    TEST(util::geo::getWKTType("POINT Z(1 1)", 0), ==, WKTType::POINT);
    TEST(util::geo::getWKTType("   POINT Z (1 1)", 0), ==, WKTType::POINT);
    TEST(util::geo::getWKTType(" POINT ZM(1 1)", 0), ==, WKTType::POINT);
    TEST(util::geo::getWKTType(" Point MZ (1 1)", 0), ==, WKTType::POINT);
    TEST(util::geo::getWKTType("mPoint MZ (1 1)", 0), ==, WKTType::POINT);
    TEST(util::geo::getWKTType(" oint MZ (1 1)", 0), ==, WKTType::NONE);

    TEST(util::geo::getWKT(pointFromWKT<int>("POINT(1 1)")), ==, "POINT(1 1)");
    TEST(util::geo::getWKT(pointFromWKT<int>("POINT (1 1)")), ==, "POINT(1 1)");
    TEST(util::geo::getWKT(pointFromWKT<int>("POINT   ( 1 1)")), ==,
         "POINT(1 1)");
    TEST(util::geo::getWKT(pointFromWKT<int>("   POINT   (   1  1   )")), ==,
         "POINT(1 1)");

    TEST(util::geo::getWKT(pointFromWKT<int>("( 1 1)")), ==, "POINT(1 1)");
    TEST(util::geo::getWKT(pointFromWKT<int>(" ( 1 1)")), ==, "POINT(1 1)");
    TEST(util::geo::getWKT(pointFromWKT<int>("   ( 1 1)")), ==, "POINT(1 1)");
    TEST(util::geo::getWKT(pointFromWKT<int>("      (   1  1   )")), ==,
         "POINT(1 1)");

    TEST(util::geo::getWKT(pointFromWKT<int>("POINT Z( 1 1)")), ==,
         "POINT(1 1)");
    TEST(util::geo::getWKT(pointFromWKT<int>("POINT Z ( 1 1)")), ==,
         "POINT(1 1)");
    TEST(util::geo::getWKT(pointFromWKT<int>("POINT Z  ( 1 1)")), ==,
         "POINT(1 1)");
    TEST(util::geo::getWKT(pointFromWKT<int>("   POINT Z   (   1  1   )")), ==,
         "POINT(1 1)");
    TEST(util::geo::getWKT(pointFromWKT<int>("MPOINT ( 1 1 50)")), ==,
         "POINT(1 1)");
    TEST(util::geo::getWKT(pointFromWKT<int>("MPOINT  ( 1 1)")), ==,
         "POINT(1 1)");
    TEST(util::geo::getWKT(pointFromWKT<int>("MPOINT   ( 1 1)")), ==,
         "POINT(1 1)");
    TEST(util::geo::getWKT(pointFromWKT<int>("   MPOINT    (   1  1 0.5  )")),
         ==, "POINT(1 1)");
  }

  {
    // strict mode

    TEST(getWKT(util::geo::pointFromWKT<double>("POINT(1 2)", true)), ==,
         "POINT(1 2)");
    TEST(getWKT(util::geo::pointFromWKT<double>("POINT(1 2)", false)), ==,
         "POINT(1 2)");

    TEST(getWKT(util::geo::pointFromWKT<double>("POINT Z (1 2 3)", true)), ==,
         "POINT(1 2)");
    TEST(getWKT(util::geo::pointFromWKT<double>("POINT Z (1 2 3)", false)), ==,
         "POINT(1 2)");

    TEST(
        getWKT(util::geo::pointFromWKT<double>(
            "<http://www.opengis.net/def/crs/OGC/1.3/CRS84> POINT(1 2)", true)),
        ==, "POINT(1 2)");
    TEST(getWKT(util::geo::pointFromWKT<double>(
             "<http://www.opengis.net/def/crs/OGC/1.3/CRS84> POINT(1 2)",
             false)),
         ==, "POINT(1 2)");

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT EMPTY", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1 2", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(5)", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1 )", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT()", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(a b)", true));
  }

  {
    TEST(getWKT(util::geo::pointFromWKTProj<double>(
             std::string("POINT(1 2)"), util::geo::projectToCRS84<double>,
             true)),
         ==, "POINT(1 2)");
    TEST(getWKT(util::geo::pointFromWKTProj<double>(
             "POINT(1 2)", 0, util::geo::projectToCRS84<double>, true)),
         ==, "POINT(1 2)");
    TEST_THROWS(
        util::geo::WKTParseException,
        util::geo::pointFromWKTProj<double>(
            std::string("POINT(5)"), util::geo::projectToCRS84<double>, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKTProj<double>(
                    "POINT(5)", 0, util::geo::projectToCRS84<double>, true));
    TEST(getWKT(util::geo::pointFromWKTProj<double>(
             std::string("POINT(5)"), util::geo::projectToCRS84<double>)),
         ==, "POINT(0 0)");
  }
}
