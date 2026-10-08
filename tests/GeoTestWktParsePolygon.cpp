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
void GeoTest::testWktParsePolygon() {

  {
    TEST(util::geo::getWKTType("POLYGON(0 0, 1 1)"), ==, WKTType::POLYGON);
    TEST(util::geo::getWKTType("POLYGO(0 0, 1 1)"), ==, WKTType::NONE);
    TEST(util::geo::getWKTType("  Polygon (0 0, 1 1)"), ==, WKTType::POLYGON);
    TEST(util::geo::getWKTType("  PolygonZ (0 0, 1 1)"), ==, WKTType::POLYGON);
    TEST(util::geo::getWKTType("  PolygonM (0 0, 1 1)"), ==, WKTType::POLYGON);
    TEST(util::geo::getWKTType("  Polygon ZM (0 0, 1 1)"), ==,
         WKTType::POLYGON);
    TEST(util::geo::getWKTType("  polygon ZM (0 0, 1 1)"), ==,
         WKTType::POLYGON);
    TEST(util::geo::getWKTType("  mpolygon ZM (0 0, 1 1)"), ==,
         WKTType::POLYGON);

    TEST(util::geo::getWKT(polygonFromWKT<int>("POLYGON((0 0, 1 1))")), ==,
         "POLYGON((0 0,1 1,0 0))");
    TEST(util::geo::getWKT(polygonFromWKT<int>("POLYGON ((0 0, 1 1))")), ==,
         "POLYGON((0 0,1 1,0 0))");
    TEST(util::geo::getWKT(polygonFromWKT<int>("POLYGON (  (0 0, 1 1) )")), ==,
         "POLYGON((0 0,1 1,0 0))");
    TEST(util::geo::getWKT(
             polygonFromWKT<int>("   POLYGON   (  (0    0  , 1  1   )  ) ")),
         ==, "POLYGON((0 0,1 1,0 0))");

    TEST(util::geo::getWKT(polygonFromWKT<int>("((0 0, 1 1))")), ==,
         "POLYGON((0 0,1 1,0 0))");
    TEST(util::geo::getWKT(polygonFromWKT<int>("( (0 0, 1 1))")), ==,
         "POLYGON((0 0,1 1,0 0))");
    TEST(util::geo::getWKT(polygonFromWKT<int>(" (  (0 0, 1 1))")), ==,
         "POLYGON((0 0,1 1,0 0))");
    TEST(
        util::geo::getWKT(polygonFromWKT<int>(" (     (  0    0  , 1  1   ))")),
        ==, "POLYGON((0 0,1 1,0 0))");

    TEST(util::geo::getWKT(polygonFromWKT<int>("POLYGON Z((0 0, 1 1))")), ==,
         "POLYGON((0 0,1 1,0 0))");
    TEST(util::geo::getWKT(polygonFromWKT<int>("POLYGON Z( (0 0, 1 1))")), ==,
         "POLYGON((0 0,1 1,0 0))");
    TEST(util::geo::getWKT(polygonFromWKT<int>("POLYGON Z(  (0 0, 1 1) )")), ==,
         "POLYGON((0 0,1 1,0 0))");
    TEST(util::geo::getWKT(
             polygonFromWKT<int>("   POLYGON Z   (  (0    0  , 1  1 (  )")),
         ==, "POLYGON((0 0,1 1,0 0))");
    TEST(util::geo::getWKT(polygonFromWKT<int>("MPOLYGON ((0 0, 1 1))")), ==,
         "POLYGON((0 0,1 1,0 0))");
    TEST(util::geo::getWKT(polygonFromWKT<int>("MPOLYGON ( (0 0, 1 1))")), ==,
         "POLYGON((0 0,1 1,0 0))");
    TEST(util::geo::getWKT(polygonFromWKT<int>("MPOLYGON (  (0 0, 1 1))")), ==,
         "POLYGON((0 0,1 1,0 0))");
    TEST(util::geo::getWKT(polygonFromWKT<int>(
             "   MPOLYGON  (  (  0    0  0.5 , 1  1 100  ))")),
         ==, "POLYGON((0 0,1 1,0 0))");
  }

  {
    // strict parsing

    TEST(getWKT(util::geo::polygonFromWKT<double>(
             "POLYGON((0 0, 1 0, 1 1, 0 0))", true)),
         ==, "POLYGON((0 0,1 0,1 1,0 0))");
    TEST(getWKT(util::geo::polygonFromWKT<double>(
             "POLYGON((0 0, 1 0, 1 1, 0 0))", false)),
         ==, "POLYGON((0 0,1 0,1 1,0 0))");
    TEST(getWKT(util::geo::polygonFromWKT<double>(
             "POLYGON((0 0, 4 0, 4 4, 0 0), (1 1, 2 1, 2 2, 1 1))", true)),
         ==, "POLYGON((0 0,4 0,4 4,0 0),(2 2,2 1,1 1,2 2))");
    TEST(getWKT(util::geo::polygonFromWKT<double>(
             "POLYGON((0 0, 4 0, 4 4, 0 0), (1 1, 2 1, 2 2, 1 1))", false)),
         ==, "POLYGON((0 0,4 0,4 4,0 0),(2 2,2 1,1 1,2 2))");

    TEST(getWKT(util::geo::polygonFromWKT<double>("POLYGON EMPTY", true)), ==,
         "POLYGON()");
    TEST(getWKT(util::geo::polygonFromWKT<double>("POLYGON Z EMPTY", true)), ==,
         "POLYGON()");

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::polygonFromWKT<double>("", true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::polygonFromWKT<double>("POLYGON", true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::polygonFromWKT<double>("POLYGON()", true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::polygonFromWKT<double>(
                    "POLYGON((0 0, 1 0, 1 1, 0 0)", true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::polygonFromWKT<double>(
                    "POLYGON((0 0, 1 0, 1 1, 0 0), (1 1, 2 1)", true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::polygonFromWKT<double>("POLYGON((0 0, 1))", true));
    TEST_THROWS(
        util::geo::WKTParseException,
        util::geo::polygonFromWKT<double>("POLYGON(0 0, 1 0, 1 1, 0 0)", true));
    TEST_THROWS(
        util::geo::WKTParseException,
        util::geo::polygonFromWKT<double>("polygon((0 0,4 0,4 4,0 0", true));
    TEST_THROWS(
        util::geo::WKTParseException,
        util::geo::polygonFromWKT<double>("polygon((0 0,4 0,4 4,0 0)", true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::polygonFromWKT<double>(
                    "polygon((0 0,4 0,4 4,0 0),(2 2,2 1,1 1,2 2)", true));
  }

  {
    TEST(getWKT(util::geo::polygonFromWKTProj<double>(
             std::string("POLYGON((0 0, 1 0, 1 1, 0 0))"),
             util::geo::projectToCRS84<double>, true)),
         ==, "POLYGON((0 0,1 0,1 1,0 0))");
    TEST(getWKT(util::geo::polygonFromWKTProj<double>(
             "POLYGON((0 0, 1 0, 1 1, 0 0))", 0,
             util::geo::projectToCRS84<double>, true)),
         ==, "POLYGON((0 0,1 0,1 1,0 0))");
    TEST_THROWS(
        util::geo::WKTParseException,
        util::geo::polygonFromWKTProj<double>(
            std::string("POLYGON()"), util::geo::projectToCRS84<double>, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::polygonFromWKTProj<double>(
                    "POLYGON()", 0, util::geo::projectToCRS84<double>, true));
    TEST(getWKT(util::geo::polygonFromWKTProj<double>(
             std::string("POLYGON()"), util::geo::projectToCRS84<double>)),
         ==, "POLYGON()");
  }
}
