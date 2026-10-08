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
  }
}
