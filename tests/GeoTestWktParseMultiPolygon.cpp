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
void GeoTest::testWktParseMultiPolygon() {
  {
    TEST(util::geo::getWKTType(
             "MULTIPOLYGON(((1 1,3 3,1 1), (0 0,1 1,0 0)),((1 3,3 1, 1 3)))"),
         ==, WKTType::MULTIPOLYGON);
    TEST(util::geo::getWKTType(
             "MULTIPOLYGO (((1 1,3 3,1 1), (0 0,1 1,0 0)),((1 3,3 1, 1 3)))"),
         ==, WKTType::NONE);
    TEST(util::geo::getWKTType(
             " MULTIPOLYGON (((1 1,3 3,1 1), (0 0,1 1,0 0)),((1 3,3 1, 1 3)))"),
         ==, WKTType::MULTIPOLYGON);
    TEST(
        util::geo::getWKTType(
            " MMULTIPOLYGON (((1 1,3 3,1 1), (0 0,1 1,0 0)),((1 3,3 1, 1 3)))"),
        ==, WKTType::MULTIPOLYGON);
    TEST(util::geo::getWKTType(" MULTIPOLYGON Z (((1 1,3 3,1 1), (0 0,1 1,0 "
                               "0)),((1 3,3 1, 1 3)))"),
         ==, WKTType::MULTIPOLYGON);
    TEST(util::geo::getWKTType(" MULTIPOLYGON ZM (((1 1,3 3,1 1), (0 0,1 1,0 "
                               "0)),((1 3,3 1, 1 3)))"),
         ==, WKTType::MULTIPOLYGON);
    TEST(util::geo::getWKTType("  MultiPolygon ZM (((1 1,3 3,1 1), (0 0,1 1,0 "
                               "0)),((1 3,3 1, 1 3)))"),
         ==, WKTType::MULTIPOLYGON);
    TEST(util::geo::getWKTType("M MultiPolygon ZM (((1 1,3 3,1 1), (0 0,1 1,0 "
                               "0)),((1 3,3 1, 1 3)))"),
         ==, WKTType::MULTIPOLYGON);
    TEST(util::geo::getWKTType("\tM\tMultiPolygon ZM\t(((1 1,3 3,1 1), (0 0,1 "
                               "1,0 0)),((1 3,3 1, 1 3)))"),
         ==, WKTType::MULTIPOLYGON);

    TEST(util::geo::getWKTType("  MultiPolygon ZM (((1 1,3 3,1 1), (0 0,1 1,0 "
                               "0)),((1 3,3 1, 1 3)))", 0, true),
         ==, WKTType::MULTIPOLYGON);
    TEST(util::geo::getWKTType("M MultiPolygon ZM (((1 1,3 3,1 1), (0 0,1 1,0 "
                               "0)),((1 3,3 1, 1 3)))", 0, true),
         ==, WKTType::MULTIPOLYGON);
    TEST(util::geo::getWKTType("\t\tMMultipolygon Z((1 1,3 3),(1 3,3 1))", 0, true),
         ==, WKTType::MULTIPOLYGON);
    TEST(util::geo::getWKTType("\t\tMMultipolygon Z EMPTY((1 1,3 3),(1 3,3 1))", 0, true),
         ==, WKTType::NONE);
    TEST(util::geo::getWKTType("tMMultipolygon EMPTY((1 1,3 3),(1 3,3 1))", 0, true),
         ==, WKTType::NONE);
    TEST(util::geo::getWKTType("MULTIPOLYGON EMPTY((1 1,3 3),(1 3,3 1))", 0, true),
         ==, WKTType::NONE);

    TEST(util::geo::getWKT(multiPolygonFromWKT<int>(
             "MULTIPOLYGON(((1 1,3 3,1 1), (0 0,1 1,0 0)),((1 3,3 1, 1 3)))")),
         ==, "MULTIPOLYGON(((1 1,3 3,1 1),(0 0,1 1,0 0)),((1 3,3 1,1 3)))");
    TEST(util::geo::getWKT(
             multiPolygonFromWKT<int>("MULTIPOLYGON(((1 1 ,3  3, 1 1 ), (0 0,1 "
                                      "1,0 0)),((1 3,3 1, 1 3)))")),
         ==, "MULTIPOLYGON(((1 1,3 3,1 1),(0 0,1 1,0 0)),((1 3,3 1,1 3)))");
    TEST(util::geo::getWKT(
             multiPolygonFromWKT<int>("MULTIPOLYGON Z (  (   ( 1    1 ,3  3, 1 "
                                      "1 ), (0 0,1 1,0 0)),((1 3,3 1, 1 3)))")),
         ==, "MULTIPOLYGON(((1 1,3 3,1 1),(0 0,1 1,0 0)),((1 3,3 1,1 3)))");
    TEST(
        util::geo::getWKT(multiPolygonFromWKT<int>(
            " (  (   ( 1    1 ,3  3, 1 1 ), (0 0,1 1,0 0)),((1 3,3 1, 1 3)))")),
        ==, "MULTIPOLYGON(((1 1,3 3,1 1),(0 0,1 1,0 0)),((1 3,3 1,1 3)))");
  }

  {
    // strict
    TEST(getWKT(util::geo::multiPolygonFromWKT<double>(
             "MULTIPOLYGON(((0 0, 1 0, 1 1, 0 0)), ((5 5, 6 5, 6 6, 5 5)))",
             true)),
         ==, "MULTIPOLYGON(((0 0,1 0,1 1,0 0)),((5 5,6 5,6 6,5 5)))");
    TEST(getWKT(util::geo::multiPolygonFromWKT<double>(
             "MULTIPOLYGON(((0 0, 1 0, 1 1, 0 0)), ((5 5, 6 5, 6 6, 5 5)))",
             false)),
         ==, "MULTIPOLYGON(((0 0,1 0,1 1,0 0)),((5 5,6 5,6 6,5 5)))");

    TEST(getWKT(util::geo::multiPolygonFromWKT<double>("MULTIPOLYGON EMPTY",
                                                       true)),
         ==, "MULTIPOLYGON()");

    TEST(getWKT(util::geo::multiPolygonFromWKT<double>(
             "MULTIPOLYGON(EMPTY, ((0 0, 1 0, 1 1, 0 0)))", true)),
         ==, "MULTIPOLYGON(((0 0,1 0,1 1,0 0)))");
    TEST(getWKT(util::geo::multiPolygonFromWKT<double>(
             "MULTIPOLYGON(EMPTY, ((0 0, 1 0, 1 1, 0 0)))", false)),
         ==, "MULTIPOLYGON(((0 0,1 0,1 1,0 0)))");

    TEST(getWKT(util::geo::multiPolygonFromWKT<double>(
             "MULTIPOLYGON(((0 0, 1 0, 1 1, 0 0)), EMPTY)", true)),
         ==, "MULTIPOLYGON(((0 0,1 0,1 1,0 0)))");
    TEST(getWKT(util::geo::multiPolygonFromWKT<double>(
             "MULTIPOLYGON(((0 0, 1 0, 1 1, 0 0)), EMPTY)", false)),
         ==, "MULTIPOLYGON(((0 0,1 0,1 1,0 0)))");

    TEST(getWKT(util::geo::multiPolygonFromWKT<double>("MULTIPOLYGON(EMPTY)",
                                                       true)),
         ==, "MULTIPOLYGON()");
    TEST(getWKT(util::geo::multiPolygonFromWKT<double>("MULTIPOLYGON(EMPTY)",
                                                       false)),
         ==, "MULTIPOLYGON()");

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPolygonFromWKT<double>("", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPolygonFromWKT<double>("MULTIPOLYGON", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPolygonFromWKT<double>("MULTIPOLYGON()", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPolygonFromWKT<double>(
                    "MULTIPOLYGON(((0 0, 1 0, 1 1, 0 0))", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPolygonFromWKT<double>(
                    "MULTIPOLYGON(((0 0, 1 0, 1 1, 0 0)),)", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPolygonFromWKT<double>(
                    "MULTIPOLYGON(((0 0, 1)))", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPolygonFromWKT<double>(
                    "MULTIPOLYGON(((0 0, 1 b2)))", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPolygonFromWKT<double>(
                    "MULTIPOLYGON((foo(0 0, 1 0, 1 1, 0 0)))", true));

    // numbers glued together must not be split into several coordinates
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPolygonFromWKT<double>(
                    "MULTIPOLYGON(((0 0, 1 0, 1 1, 0 0)), ((0 0, 1-0, 1 1, 0 0)))",
                    true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPolygonFromWKT<double>(
                    "MULTIPOLYGON(((0 0, 1 0, 1 1, 0 0), (0.1.2 0, 1 0, 1 1, 0 0)))",
                    true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPolygonFromWKT<double>(
                    "MULTIPOLYGON(((0 0, 1 0, 1 1, 0 0)), (foo(0 0, 1 0, 1 1, 0 0)))",
                    true));

    TEST(getWKT(util::geo::multiPolygonFromWKT<double>(
             "MULTIPOLYGON(( \n(0 0, 1 0, 1 1, 0 0)))", true)),
         ==, "MULTIPOLYGON(((0 0,1 0,1 1,0 0)))");
  }

  {
    TEST(getWKT(util::geo::multiPolygonFromWKTProj<double>(
             std::string("MULTIPOLYGON(((0 0, 1 0, 1 1, 0 0)))"),
             util::geo::projectToCRS84<double>, true)),
         ==, "MULTIPOLYGON(((0 0,1 0,1 1,0 0)))");
    TEST(getWKT(util::geo::multiPolygonFromWKTProj<double>(
             std::string("MULTIPOLYGON(((0 0, 1e0 0, 1 1e-0, 0 0e+0)))"),
             util::geo::projectToCRS84<double>, true)),
         ==, "MULTIPOLYGON(((0 0,1 0,1 1,0 0)))");
    TEST(getWKT(util::geo::multiPolygonFromWKTProj<double>(
             std::string("MULTIPOLYGON(((0 100e-0002, 1e0 0, 1 1e-0, 0 0e+0, 0  1)))"),
             util::geo::projectToCRS84<double>, true)),
         ==, "MULTIPOLYGON(((0 1,1 0,1 1,0 0,0 1)))");
    TEST(getWKT(util::geo::multiPolygonFromWKTProj<double>(
             std::string("MULTIPOLYGON(((1 2, 3 3, 1 2e0)),((0 100e-0002, 1e0 0, 1 1e-0, 0 0e+0, 0  1), (+5 -2, 10e2 3, 5 -20e-1)))"),
             util::geo::projectToCRS84<double>, true)),
         ==, "MULTIPOLYGON(((1 2,3 3,1 2)),((0 1,1 0,1 1,0 0,0 1),(5 -2,1000 3,5 -2)))");
    TEST(getWKT(util::geo::multiPolygonFromWKTProj<double>(
             "MULTIPOLYGON(((0 0, 1 0, 1 1, 0 0)))", 0,
             util::geo::projectToCRS84<double>, true)),
         ==, "MULTIPOLYGON(((0 0,1 0,1 1,0 0)))");
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPolygonFromWKTProj<double>(
                    std::string("MULTIPOLYGON()"),
                    util::geo::projectToCRS84<double>, true));
    TEST_THROWS(
        util::geo::WKTParseException,
        util::geo::multiPolygonFromWKTProj<double>(
            "MULTIPOLYGON()", 0, util::geo::projectToCRS84<double>, true));
    TEST(getWKT(util::geo::multiPolygonFromWKTProj<double>(
             std::string("MULTIPOLYGON()"), util::geo::projectToCRS84<double>)),
         ==, "MULTIPOLYGON()");
  }

  {
    TEST(util::geo::getWKT(
             util::geo::multiPolygonFromWKT<double>("MULTIPOLYGON(((0 0, 1 0, 1 1, 0 0)), ((2 2, 3 2, 3 3, 2 2))) \n\t", 0, true)),
         ==, "MULTIPOLYGON(((0 0,1 0,1 1,0 0)),((2 2,3 2,3 3,2 2)))");
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPolygonFromWKT<double>("MULTIPOLYGON(((0 0, 1 0, 1 1, 0 0)), ((2 2, 3 2, 3 3, 2 2))) GARBAGE", 0, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPolygonFromWKT<double>("MULTIPOLYGON(((0 0, 1 0, 1 1, 0 0)), ((2 2, 3 2, 3 3, 2 2))))", 0, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPolygonFromWKT<double>("MULTIPOLYGON(((0 0, 1 0, 1 1, 0 0)), ((2 2, 3 2, 3 3, 2 2))), POINT(5 5)", 0, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPolygonFromWKT<double>("MULTIPOLYGON EMPTY GARBAGE", 0, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPolygonFromWKT<double>("MULTIPOLYGON EMPTY)", 0, true));

    const char* in = "MULTIPOLYGON(((0 0, 1 0, 1 1, 0 0)), ((2 2, 3 2, 3 3, 2 2))) GARBAGE";
    const char* end = 0;
    TEST(util::geo::getWKT(util::geo::multiPolygonFromWKT<double>(in, &end, true)), ==,
         "MULTIPOLYGON(((0 0,1 0,1 1,0 0)),((2 2,3 2,3 3,2 2)))");
    TEST(end - in, ==, 59);

    TEST(util::geo::getWKT(util::geo::multiPolygonFromWKT<double>(in, 0, false)), ==,
         "MULTIPOLYGON(((0 0,1 0,1 1,0 0)),((2 2,3 2,3 3,2 2)))");
  }
}
