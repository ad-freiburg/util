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
}
