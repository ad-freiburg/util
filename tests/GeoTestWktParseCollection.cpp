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
void GeoTest::testWktParseCollection() {
  {
    TEST(util::geo::getWKTType("GEOMETRYCOLLECTION(MULTIPOLYGON(((1 1,3 3,1 "
                               "1),(0 0,1 1,0 0)),((1 3,3 1,1 3))))"),
         ==, WKTType::COLLECTION);
    TEST(util::geo::getWKTType("GEOMETRYCOLLECTIO (MULTIPOLYGON(((1 1,3 3,1 "
                               "1),(0 0,1 1,0 0)),((1 3,3 1,1 3))))"),
         ==, WKTType::NONE);
    TEST(util::geo::getWKTType(" GEOMETRYCOLLECTION (MULTIPOLYGON(((1 1,3 3,1 "
                               "1),(0 0,1 1,0 0)),((1 3,3 1,1 3))))"),
         ==, WKTType::COLLECTION);
    TEST(util::geo::getWKTType(" GeometryCollection (MULTIPOLYGON(((1 1,3 3,1 "
                               "1),(0 0,1 1,0 0)),((1 3,3 1,1 3))))"),
         ==, WKTType::COLLECTION);
    TEST(util::geo::getWKTType("\t\tGeometryCollection (MULTIPOLYGON(((1 1,3 "
                               "3,1 1),(0 0,1 1,0 0)),((1 3,3 1,1 3))))"),
         ==, WKTType::COLLECTION);
    TEST(util::geo::getWKTType("\t\tmGeometryCollection (MULTIPOLYGON(((1 1,3 "
                               "3,1 1),(0 0,1 1,0 0)),((1 3,3 1,1 3))))"),
         ==, WKTType::COLLECTION);
    TEST(util::geo::getWKTType("\t\tmGeometryCollection M(MULTIPOLYGON(((1 1,3 "
                               "3,1 1),(0 0,1 1,0 0)),((1 3,3 1,1 3))))"),
         ==, WKTType::COLLECTION);
    TEST(util::geo::getWKTType("\t\tmGeometryCollection ZM(MULTIPOLYGON(((1 "
                               "1,3 3,1 1),(0 0,1 1,0 0)),((1 3,3 1,1 3))))"),
         ==, WKTType::COLLECTION);

    TEST(util::geo::getWKT(collectionFromWKT<int>(
             "GEOMETRYCOLLECTION(MULTIPOLYGON(((1 1,3 3,1 1), (0 0,1 1,0 "
             "0)),((1 3,3 1, 1 3))))")),
         ==,
         "GEOMETRYCOLLECTION(MULTIPOLYGON(((1 1,3 3,1 1),(0 0,1 1,0 0)),((1 "
         "3,3 1,1 3))))");

    TEST(util::geo::getWKT(collectionFromWKT<int>(
             "GEOMETRYCOLLECTION (MULTIPOLYGON (((1 1,3 3,1 1), (0 0,1 1,0 "
             "0)),((1 3,3 1, 1 3))))")),
         ==,
         "GEOMETRYCOLLECTION(MULTIPOLYGON(((1 1,3 3,1 1),(0 0,1 1,0 0)),((1 "
         "3,3 1,1 3))))");

    TEST(util::geo::getWKT(collectionFromWKT<int>(
             "GEOMETRYCOLLECTION (  MULTIPOLYGON Z (((1 1,3 3,1 1), (0 0,1 1,0 "
             "0)),((1 3,3 1, 1 3))))")),
         ==,
         "GEOMETRYCOLLECTION(MULTIPOLYGON(((1 1,3 3,1 1),(0 0,1 1,0 0)),((1 "
         "3,3 1,1 3))))");

    TEST(util::geo::getWKT(collectionFromWKT<int>(
             "GEOMETRYCOLLECTION (  MMULTIPOLYGON Z (((1 1,3 3,1 1), (0 0,1 "
             "1,0 0)),((1 3,3 1, 1 3))))")),
         ==,
         "GEOMETRYCOLLECTION(MULTIPOLYGON(((1 1,3 3,1 1),(0 0,1 1,0 0)),((1 "
         "3,3 1,1 3))))");
  }

  {
    // strict
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(POINT(1 2), LINESTRING(0 0, 1 1))", true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),LINESTRING(0 0,1 1))");
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(POINT(1 2), LINESTRING(0 0, 1 1))", false)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),LINESTRING(0 0,1 1))");

    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(POLYGON((0 0, 1 0, 1 1, 0 0)), MULTIPOINT(1 "
             "2, 3 4))",
             true)),
         ==,
         "GEOMETRYCOLLECTION(POLYGON((0 0,1 0,1 1,0 0)),MULTIPOINT(1 2,3 4))");
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(POLYGON((0 0, 1 0, 1 1, 0 0)), MULTIPOINT(1 "
             "2, 3 4))",
             false)),
         ==,
         "GEOMETRYCOLLECTION(POLYGON((0 0,1 0,1 1,0 0)),MULTIPOINT(1 2,3 4))");

    TEST(getWKT(util::geo::collectionFromWKT<double>("GEOMETRYCOLLECTION EMPTY",
                                                     true)),
         ==, "GEOMETRYCOLLECTION()");

    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(LINESTRING EMPTY, POINT(1 2))", true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2))");
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(LINESTRING EMPTY, POINT(1 2))", false)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2))");

    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(POINT(1 2), POINT EMPTY)", true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2))");
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(POINT(1 2), POINT EMPTY)", false)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2))");

    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(POINT Z EMPTY, POINT(1 2))", true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2))");
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(POINT Z EMPTY, POINT(1 2))", false)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2))");

    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(POLYGON EMPTY, MULTIPOINT EMPTY, "
             "LINESTRING(0 0, 1 1))",
             true)),
         ==, "GEOMETRYCOLLECTION(LINESTRING(0 0,1 1))");
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(POINT EMPTY, LINESTRING EMPTY)", true)),
         ==, "GEOMETRYCOLLECTION()");

    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(POLYGON EMPTY, MULTIPOINT EMPTY, "
             "LINESTRING(0 0, 1 1))",
             false)),
         ==, "GEOMETRYCOLLECTION(LINESTRING(0 0,1 1))");
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(POINT EMPTY, LINESTRING EMPTY)", false)),
         ==, "GEOMETRYCOLLECTION()");

    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(MULTILINESTRING(EMPTY), POINT(1 2))", true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2))");
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(MULTILINESTRING(EMPTY), POINT(1 2))", false)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2))");

    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(MULTIPOLYGON(EMPTY), LINESTRING(0 0, 1 1))",
             true)),
         ==, "GEOMETRYCOLLECTION(LINESTRING(0 0,1 1))");
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(MULTIPOLYGON(EMPTY), LINESTRING(0 0, 1 1))",
             false)),
         ==, "GEOMETRYCOLLECTION(LINESTRING(0 0,1 1))");
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(MULTIPOLYGON(EMPTY), LINESTRING(0 0, a b, 1 "
             "1))",
             false)),
         ==, "GEOMETRYCOLLECTION(LINESTRING(0 0,1 1))");

    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(MULTIPOINT(EMPTY), POINT(1 2))", true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2))");
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             "GEOMETRYCOLLECTION(MULTIPOINT(EMPTY), POINT(1 2))", false)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2))");

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>("", true));

    TEST_THROWS(
        util::geo::WKTParseException,
        util::geo::collectionFromWKT<double>("GEOMETRYCOLLECTION", true));

    TEST_THROWS(
        util::geo::WKTParseException,
        util::geo::collectionFromWKT<double>("GEOMETRYCOLLECTION()", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    "GEOMETRYCOLLECTION(FOO(1 2))", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    "GEOMETRYCOLLECTION(POINT(1 2)", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    "GEOMETRYCOLLECTION(POINT(1 2),)", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    "GEOMETRYCOLLECTION(POINT(1))", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    "GEOMETRYCOLLECTION(LINESTRING(0 0, 1))", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    "GEOMETRYCOLLECTION(LINESTRING(0 0, 1 1)", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    "GEOMETRYCOLLECTION(POLYGON((0 0, 1 1)", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    "GEOMETRYCOLLECTION(POLYGON((0 0, 1 1))", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    "GEOMETRYCOLLECTION(POLYGON(0 0, 1 1))", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    "GEOMETRYCOLLECTION(POINT)", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    "GEOMETRYCOLLECTION(POINT EMPTaY)", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    "GEOMETRYCOLLECTION(POLYGON((0 0, 1 1, b b)))", true));
  }

  {
    TEST(getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2))"),
             util::geo::projectToCRS84<double>, true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2))");
    TEST(getWKT(util::geo::collectionFromWKTProj<double>(
             "GEOMETRYCOLLECTION(POINT(1 2))", 0,
             util::geo::projectToCRS84<double>, true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2))");
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKTProj<double>(
                    std::string("GEOMETRYCOLLECTION(FOO(1 2))"),
                    util::geo::projectToCRS84<double>, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKTProj<double>(
                    "GEOMETRYCOLLECTION(FOO(1 2))", 0,
                    util::geo::projectToCRS84<double>, true));
    TEST(getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(FOO(1 2))"),
             util::geo::projectToCRS84<double>)),
         ==, "GEOMETRYCOLLECTION()");

    TEST(getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT(100e-2 55), MULTIPOLYGON(((1 2, 3 3, 1 2e0)),((0 100e-0002, 1e0 0, 1 1e-0, 0 0e+0, 0  1), (+5 -2, 10e2 3, 5 -20e-1))))"),
             util::geo::projectToCRS84<double>, true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 55),MULTIPOLYGON(((1 2,3 3,1 2)),((0 1,1 0,1 1,0 0,0 1),(5 -2,1000 3,5 -2))))");

    TEST_THROWS(util::geo::WKTParseException, getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT(100e-2 55), MULTIPOLYGON(((1 2, 3 3, 1 2e)),((0 100e-0002, 1e0 0, 1 1e-0, 0 0e+0, 0  1), (+5 -2, 10e2 3, 5 -20e-1))))"),
             util::geo::projectToCRS84<double>, true)));

    TEST_THROWS(util::geo::WKTParseException, getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT(100e-2 55), MULTIPOLYGON(((1 2, 3 3, 1 2) garbage),((0 100e-0002, 1e0 0, 1 1e-0, 0 0e+0, 0  1), (+5 -2, 10e2 3, 5 -20e-1))))"),
             util::geo::projectToCRS84<double>, true)));

    TEST_THROWS(util::geo::WKTParseException, getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT GARBAGE(100e-2 55), MULTIPOLYGON(((1 2, 3 3, 1 2)),((0 100e-0002, 1e0 0, 1 1e-0, 0 0e+0, 0  1), (+5 -2, 10e2 3, 5 -20e-1))))"),
             util::geo::projectToCRS84<double>, true)));

    TEST_THROWS(util::geo::WKTParseException, getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT     (100e-2 55), MULTILINESTRING(foo(1 2, 3 3, 1 2),(0 100e-0002, 1e0 0, 1 1e-0, 0 0e+0, 0  1), (+5 -2, 10e2 3, 5 -20e-1)))"),
             util::geo::projectToCRS84<double>, true)));
    TEST_THROWS(util::geo::WKTParseException, getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT     (100e-2 55), MULTILINESTRING((1 2, 3 3, 1 2),,(0 100e-0002, 1e0 0, 1 1e-0, 0 0e+0, 0  1), (+5 -2, 10e2 3, 5 -20e-1)))"),
             util::geo::projectToCRS84<double>, true)));
    TEST_THROWS(util::geo::WKTParseException, getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT     (100e-2 55), MULTILINESTRING((1 2, 3 3, 1 2),((0 100e-0002, 1e0 0, 1 1e-0, 0 0e+0, 0  1), (+5 -2, 10e2 3, 5 -20e-1)))"),
             util::geo::projectToCRS84<double>, true)));
    TEST_THROWS(util::geo::WKTParseException, getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT     (100e-2 55), MULTILINESTRING((1 2, 3 3, 1 2),(f0 100e-0002, 1e0 0, 1 1e-0, 0 0e+0, 0  1), (+5 -2, 10e2 3, 5 -20e-1)))"),
             util::geo::projectToCRS84<double>, true)));

    TEST_THROWS(util::geo::WKTParseException, getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(MULTIPOINT     (foo(100e-2 55)), MULTIPOLYGON(((1 2, 3 3, 1 2)),((0 100e-0002, 1e0 0, 1 1e-0, 0 0e+0, 0  1), (+5 -2, 10e2 3, 5 -20e-1))))"),
             util::geo::projectToCRS84<double>, true)));

    TEST_THROWS(util::geo::WKTParseException, getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT     (100e-2 55), MULTIPOLYGON((foo(1 2, 3 3, 1 2)),((0 100e-0002, 1e0 0, 1 1e-0, 0 0e+0, 0  1), (+5 -2, 10e2 3, 5 -20e-1))))"),
             util::geo::projectToCRS84<double>, true)));

    TEST_THROWS(util::geo::WKTParseException, getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT     (100e-2 55), MULTIPOLYGON(((1 2, 3 3, 1 2)),(foo(0 100e-0002, 1e0 0, 1 1e-0, 0 0e+0, 0  1), (+5 -2, 10e2 3, 5 -20e-1))))"),
             util::geo::projectToCRS84<double>, true)));

    TEST_THROWS(util::geo::WKTParseException, getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT     (100e-2 55), MULTIPOLYGON(((1 2, 3 3, 1 2)),((0 100e-0002, 1e0 0, 1 1e-0, 0 0e+0, 0  1),foo (+5 -2, 10e2 3, 5 -20e-1))))"),
             util::geo::projectToCRS84<double>, true)));

    TEST_THROWS(util::geo::WKTParseException, getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT     (100e-2 55), MULTIPOLYGON(((1 2, 3 3, 1 2)),((0 100e-0002, 1e0 0, 1 1e-0, 0 0e+0, 0  1),, (+5 -2, 10e2 3, 5 -20e-1))))"),
             util::geo::projectToCRS84<double>, true)));
    TEST_THROWS(util::geo::WKTParseException, getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT     (100e-2 55), MULTIPOLYGON(((1 2, 3 3, 1 2)),((0 100e-0002, 1e0 0, 1 1e-0, 0 0e+0, 0  1),( (+5 -2, 10e2 3, 5 -20e-1))))"),
             util::geo::projectToCRS84<double>, true)));
    TEST_THROWS(util::geo::WKTParseException, getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT     (100e-2 55), MULTIPOLYGON(((1 2, 3 3, 1 2)),((0 100e-0002, 1e0 0, 1 1e-0, 0 0e+0, 0  1)foo, (+5 -2, 10e2 3, 5 -20e-1))))"),
             util::geo::projectToCRS84<double>, true)));

    TEST_THROWS(util::geo::WKTParseException, getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT     (100e-2 55), MULTIPOLYGON MULTIPOLYGON(((1 2, 3 3, 1 2)),((0 100e-0002, 1e0 0, 1 1e-0, 0 0e+0, 0  1), (+5 -2, 10e2 3, 5 -20e-1))))"),
             util::geo::projectToCRS84<double>, true)));

    TEST_THROWS(util::geo::WKTParseException, getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT     (100e-2 55), MULTIPOLYGON EMPTY(((1 2, 3 3, 1 2)),((0 100e-0002, 1e0 0, 1 1e-0, 0 0e+0, 0  1), (+5 -2, 10e2 3, 5 -20e-1))))"),
             util::geo::projectToCRS84<double>, true)));

    TEST_THROWS(util::geo::WKTParseException, getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT     (100e-2 55), MULTIPOLYGON EMPTY EMPTY)"),
             util::geo::projectToCRS84<double>, true)));

    TEST_THROWS(util::geo::WKTParseException, getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT     (100e-2 55), MULTIPOLYGON EMPTYY)"),
             util::geo::projectToCRS84<double>, true)));

    TEST_THROWS(util::geo::WKTParseException, getWKT(util::geo::collectionFromWKTProj<double>(
             std::string("GEOMETRYCOLLECTION(POINT     (100e-2 55), MULTIPOLYGON EEMPTY)"),
             util::geo::projectToCRS84<double>, true)));
  }

  {
    TEST(util::geo::getWKT(
             util::geo::collectionFromWKT<double>("GEOMETRYCOLLECTION(POINT(1 2), LINESTRING(1 2, 3 4)) \n\t", 0, true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),LINESTRING(1 2,3 4))");
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>("GEOMETRYCOLLECTION(POINT(1 2), LINESTRING(1 2, 3 4)) GARBAGE", 0, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>("GEOMETRYCOLLECTION(POINT(1 2), LINESTRING(1 2, 3 4)))", 0, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>("GEOMETRYCOLLECTION(POINT(1 2), LINESTRING(1 2, 3 4)), POINT(5 5)", 0, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>("GEOMETRYCOLLECTION EMPTY GARBAGE", 0, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>("GEOMETRYCOLLECTION EMPTY)", 0, true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(POINT(1 2),POINT"), true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(POINT(1 2),LINESTRING"), true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(POINT(1 2),POLYGON"), true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTIPOINT"), true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTILINESTRING"), true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTIPOLYGON"), true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(POINT(1 2),POLYGON "), true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(POINT(1 2),POLYGON Z"), true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(POINT(1 2),LINESTRING \n"), true));
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),POINT"), false)),
         ==, "GEOMETRYCOLLECTION()");
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),LINESTRING"), false)),
         ==, "GEOMETRYCOLLECTION()");
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),POLYGON"), false)),
         ==, "GEOMETRYCOLLECTION()");
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTIPOINT"), false)),
         ==, "GEOMETRYCOLLECTION()");
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTILINESTRING"), false)),
         ==, "GEOMETRYCOLLECTION()");
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTIPOLYGON"), false)),
         ==, "GEOMETRYCOLLECTION()");
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),POLYGON "), false)),
         ==, "GEOMETRYCOLLECTION()");
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),POLYGON Z"), false)),
         ==, "GEOMETRYCOLLECTION()");
    TEST(getWKT(util::geo::collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),LINESTRING \n"), false)),
         ==, "GEOMETRYCOLLECTION()");

    // numbers glued together must not be split into several coordinates
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>("GEOMETRYCOLLECTION(POINT(1-2), LINESTRING(1 2, 3 4))", 0, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::collectionFromWKT<double>("GEOMETRYCOLLECTION(POINT(1 2), LINESTRING(1 2, 3.4.5))", 0, true));

    const char* in = "GEOMETRYCOLLECTION(POINT(1 2), LINESTRING(1 2, 3 4)) GARBAGE";
    const char* end = 0;
    TEST(util::geo::getWKT(util::geo::collectionFromWKT<double>(in, &end, true)), ==,
         "GEOMETRYCOLLECTION(POINT(1 2),LINESTRING(1 2,3 4))");
    TEST(end - in, ==, 51);

    TEST(util::geo::getWKT(util::geo::collectionFromWKT<double>(in, 0, false)), ==,
         "GEOMETRYCOLLECTION(POINT(1 2),LINESTRING(1 2,3 4))");
  }

  {
    // in non-strict mode, unparsable inner rings of members are dropped
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POLYGON((0 0,4 0,4 4,0 0),(1)),"
                         "POINT(1 2))"),
             false)),
         ==, "GEOMETRYCOLLECTION(POLYGON((0 0,4 0,4 4,0 0)),POINT(1 2))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(MULTIPOLYGON(((0 0,4 0,4 4,0 0),"
                         "(1))),POINT(1 2))"),
             false)),
         ==,
         "GEOMETRYCOLLECTION(MULTIPOLYGON(((0 0,4 0,4 4,0 0))),POINT(1 2))");
  }

  {
    // nested collections
    //
    auto nested = collectionFromWKT<double>(
        std::string("GEOMETRYCOLLECTION(GEOMETRYCOLLECTION(POINT(1 2),"
                    "POINT(5 6)),POINT(3 4))"),
        true);
    TEST(nested.size(), ==, 2);
    TEST(nested[0].getType(), ==, 5);
    TEST(nested[0].getCollection().size(), ==, 2);
    TEST(nested[1].getType(), ==, 0);
    TEST(getWKT(nested), ==,
         "GEOMETRYCOLLECTION(GEOMETRYCOLLECTION(POINT(1 2),POINT(5 6)),"
         "POINT(3 4))");

    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(3 4),"
                         "GEOMETRYCOLLECTION(POINT(1 2)))"),
             true)),
         ==, "GEOMETRYCOLLECTION(POINT(3 4),GEOMETRYCOLLECTION(POINT(1 2)))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(GEOMETRYCOLLECTION("
                         "GEOMETRYCOLLECTION(POINT(1 2))))"),
             true)),
         ==,
         "GEOMETRYCOLLECTION(GEOMETRYCOLLECTION(GEOMETRYCOLLECTION(POINT(1 "
         "2))))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(GEOMETRYCOLLECTION(POINT(1 2)),"
                         "GEOMETRYCOLLECTION(LINESTRING(1 2,3 4),"
                         "POLYGON((0 0,1 0,1 1,0 0))))"),
             true)),
         ==,
         "GEOMETRYCOLLECTION(GEOMETRYCOLLECTION(POINT(1 2)),"
         "GEOMETRYCOLLECTION(LINESTRING(1 2,3 4),POLYGON((0 0,1 0,1 1,0 0))))");

    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(GEOMETRYCOLLECTION (POINT(1 2)))"),
             true)),
         ==, "GEOMETRYCOLLECTION(GEOMETRYCOLLECTION(POINT(1 2)))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(GEOMETRYCOLLECTION (POINT(1 2)))"),
             false)),
         ==, "GEOMETRYCOLLECTION(GEOMETRYCOLLECTION(POINT(1 2)))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION ( GEOMETRYCOLLECTION Z ( POINT ( 1 "
                         "2 ) , LINESTRING(1 2,3 4) ) , POINT(3 4) )"),
             true)),
         ==,
         "GEOMETRYCOLLECTION(GEOMETRYCOLLECTION(POINT(1 2),LINESTRING(1 2,3 "
         "4)),POINT(3 4))");

    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(GEOMETRYCOLLECTION EMPTY,"
                         "POINT(3 4))"),
             true)),
         ==, "GEOMETRYCOLLECTION(POINT(3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(GEOMETRYCOLLECTION(POINT EMPTY),"
                         "POINT(3 4))"),
             true)),
         ==, "GEOMETRYCOLLECTION(POINT(3 4))");

    Collection<double> inner;
    inner.push_back(Point<double>(1, 2));
    Collection<double> outer;
    outer.push_back(inner);
    outer.push_back(Point<double>(3, 4));
    TEST(collectionFromWKT<double>(getWKT(outer), true) == outer);

    const char* in =
        "GEOMETRYCOLLECTION(GEOMETRYCOLLECTION(POINT(1 2)),POINT(3 4)) rest";
    const char* end = 0;
    collectionFromWKT<double>(in, &end, true);
    TEST(end - in, ==, 60);

    TEST_THROWS(WKTParseException,
                collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(GEOMETRYCOLLECTION(POINT(1 "
                                "2)) garbage,POINT(3 4))"),
                    true));
    TEST_THROWS(WKTParseException,
                collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(GEOMETRYCOLLECTION(POINT(1 "
                                "2),POINT(3 4))"),
                    true));
    TEST_THROWS(WKTParseException,
                collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(GEOMETRYCOLLECTION(POINT(1 "
                                "2)),POINT(3 4)))"),
                    true));
    TEST_THROWS(WKTParseException,
                collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(GEOMETRYCOLLECTION(POINT(1 "
                                "2),),POINT(3 4))"),
                    true));
    TEST_THROWS(WKTParseException,
                collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(GEOMETRYCOLLECTION(FOO(1 2)),"
                                "POINT(3 4))"),
                    true));
    TEST_THROWS(WKTParseException,
                collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(GEOMETRYCOLLECTION"), true));
  }

  {
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTIPOINT \"EMPTY,POINT(3 4))"),
             true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),POINT(3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTIPOINT ' EMPTY,POINT(3 4))"),
             true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),POINT(3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),LINESTRING \"EMPTY,POINT(3 4))"),
             true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),POINT(3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),POLYGON \"EMPTY,POINT(3 4))"),
             true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),POINT(3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTILINESTRING \"EMPTY,POINT(3 4))"),
             true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),POINT(3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTIPOLYGON \"EMPTY,POINT(3 4))"),
             true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),POINT(3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),GEOMETRYCOLLECTION \"EMPTY,POINT(3 4))"),
             true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),POINT(3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTIPOINT \"EMPTY)"),
             true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTIPOINT \"EMPTY,POINT(3 4))"),
             false)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),POINT(3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTIPOINT ' EMPTY,POINT(3 4))"),
             false)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),POINT(3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),LINESTRING \"EMPTY,POINT(3 4))"),
             false)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),POINT(3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),POLYGON \"EMPTY,POINT(3 4))"),
             false)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),POINT(3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTILINESTRING \"EMPTY,POINT(3 4))"),
             false)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),POINT(3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTIPOLYGON \"EMPTY,POINT(3 4))"),
             false)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),POINT(3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),GEOMETRYCOLLECTION \"EMPTY,POINT(3 4))"),
             false)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),POINT(3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTIPOINT \"EMPTY)"),
             false)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2))");
  }

  {
    // an RDF datatype ^^<...> may follow a collection, but not a member
    TEST(getWKT(collectionFromWKT<double>(
             std::string("\"GEOMETRYCOLLECTION(POINT(1 2),LINESTRING(1 2,3 4))\"^^<http://www.opengis.net/ont/geosparql#wktLiteral>"), true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),LINESTRING(1 2,3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("\"GEOMETRYCOLLECTION EMPTY\"^^<http://www.opengis.net/ont/geosparql#wktLiteral>"), true)),
         ==, "GEOMETRYCOLLECTION()");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("\"GEOMETRYCOLLECTION(POINT(1 2),LINESTRING(1 2,3 4))\"^^<http://www.opengis.net/ont/geosparql#wktLiteral>"), false)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),LINESTRING(1 2,3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("\"GEOMETRYCOLLECTION EMPTY\"^^<http://www.opengis.net/ont/geosparql#wktLiteral>"), false)),
         ==, "GEOMETRYCOLLECTION()");
    TEST_THROWS(WKTParseException,
                collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(\"POINT(1 2)\"^^<http://www.opengis.net/ont/geosparql#wktLiteral>)"), true));
    TEST_THROWS(WKTParseException,
                collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(\"POINT(1 2)\"^^<http://www.opengis.net/ont/geosparql#wktLiteral>,POINT(3 4))"), true));
    TEST_THROWS(WKTParseException,
                collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(\"LINESTRING EMPTY\"^^<http://www.opengis.net/ont/geosparql#wktLiteral>,POINT(1 2))"), true));
    TEST_THROWS(WKTParseException,
                collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(POINT(1 2))^^<http://www.opengis.net/ont/geosparql#wktLiteral>)"), true));
    TEST_THROWS(WKTParseException,
                multiPointFromWKT<double>(
                    std::string("MULTIPOINT((1 2)\"^^<http://www.opengis.net/ont/geosparql#wktLiteral>,(3 4))"), true));
  }

  {
    TEST_THROWS(WKTParseException,
                collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(POINT(1 2), LINESTRINGEMPTY)"), true));
    TEST_THROWS(WKTParseException,
                collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(POINT(1 2), POLYGONEMPTY)"), true));
    TEST_THROWS(WKTParseException,
                collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(POINT(1 2), MULTIPOINTEMPTY)"), true));
    TEST_THROWS(WKTParseException,
                collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(POINT(1 2), MULTILINESTRINGEMPTY)"), true));
    TEST_THROWS(WKTParseException,
                collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(POINT(1 2), MULTIPOLYGONEMPTY)"), true));
    TEST_THROWS(WKTParseException,
                collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(POINT(1 2), GEOMETRYCOLLECTIONEMPTY)"), true));
    TEST_THROWS(WKTParseException,
                collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(POINT(1 2), LINESTRING ZEMPTY)"), true));
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2), LINESTRING EMPTY, POINT(3 4))"),
             true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),POINT(3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2), LINESTRING Z EMPTY, POINT(3 4))"),
             true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),POINT(3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2), LINESTRING\tEMPTY, POINT(3 4))"),
             true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),POINT(3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2), LINESTRING \"EMPTY\", POINT(3 4))"),
             true)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),POINT(3 4))");
    // non-strict mode still accepts them as empty members
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2), LINESTRINGEMPTY)"),
             false)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2))");
  }
}
