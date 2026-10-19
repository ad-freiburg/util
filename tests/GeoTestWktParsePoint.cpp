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
    TEST(util::geo::getWKTType("POINT EMPTY", 0), ==, WKTType::POINT);
    TEST(util::geo::getWKTType("POIN (1 1)", 0), ==, WKTType::NONE);
    TEST(util::geo::getWKTType(" POINT (1 1)", 0), ==, WKTType::POINT);
    TEST(util::geo::getWKTType(" Point (1 1)", 0), ==, WKTType::POINT);
    TEST(util::geo::getWKTType("POINT Z(1 1)", 0), ==, WKTType::POINT);
    TEST(util::geo::getWKTType("   POINT Z (1 1)", 0), ==, WKTType::POINT);
    TEST(util::geo::getWKTType(" POINT ZM(1 1)", 0), ==, WKTType::POINT);
    TEST(util::geo::getWKTType(" Point MZ (1 1)", 0), ==, WKTType::POINT);
    TEST(util::geo::getWKTType("mPoint MZ (1 1)", 0), ==, WKTType::POINT);
    TEST(util::geo::getWKTType(" oint MZ (1 1)", 0), ==, WKTType::NONE);

    TEST(util::geo::getWKTType("POINT(1 1)", 0, true), ==, WKTType::POINT);
    TEST(util::geo::getWKTType("POINT EMPTY", 0, true), ==, WKTType::POINT);
    TEST(util::geo::getWKTType("POIN (1 1)", 0, true), ==, WKTType::NONE);
    TEST(util::geo::getWKTType(" POINT (1 1)", 0, true), ==, WKTType::POINT);
    TEST(util::geo::getWKTType(" Point (1 1)", 0, true), ==, WKTType::POINT);
    TEST(util::geo::getWKTType("POINT Z(1 1)", 0, true), ==, WKTType::POINT);
    TEST(util::geo::getWKTType("   POINT Z (1 1)", 0, true), ==, WKTType::POINT);
    TEST(util::geo::getWKTType(" POINT ZM(1 1)", 0, true), ==, WKTType::POINT);
    TEST(util::geo::getWKTType(" Point MZ (1 1)", 0, true), ==, WKTType::POINT);
    TEST(util::geo::getWKTType("mPoint MZ (1 1)", 0, true), ==, WKTType::POINT);
    TEST(util::geo::getWKTType(" oint MZ (1 1)", 0, true), ==, WKTType::NONE);

    TEST(util::geo::getWKTType("POINT foo(1 1)", 0, true), ==, WKTType::NONE);
    TEST(util::geo::getWKTType("POIN (1 1)", 0, true), ==, WKTType::NONE);
    TEST(util::geo::getWKTType(" POINT foo M(1 1)", 0, true), ==, WKTType::NONE);
    TEST(util::geo::getWKTType(" Point M foo(1 1)", 0, true), ==, WKTType::NONE);
    TEST(util::geo::getWKTType("POINT Zfoo(1 1)", 0, true), ==, WKTType::NONE);
    TEST(util::geo::getWKTType("   POINT Z M foo (1 1)", 0, true), ==, WKTType::NONE);
    TEST(util::geo::getWKTType(" POINT ZM    foo   (1 1)", 0, true), ==, WKTType::NONE);
    TEST(util::geo::getWKTType(" Point MZ foo(1 1)", 0, true), ==, WKTType::NONE);
    TEST(util::geo::getWKTType("mPoint MZ foo (1 1)", 0, true), ==, WKTType::NONE);
    TEST(util::geo::getWKTType(" oint MZ (1 1)", 0, true), ==, WKTType::NONE);


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

    TEST(util::geo::getWKT(
             pointFromWKT<double>("   MPOINT    (   1  .5 0.5  )")),
         ==, "POINT(1 0.5)");
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

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(.b 1)", true));
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

  {
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT()", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1 2", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1 2, 3 4)", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1 2 blubb)", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1 2e)", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1e 2)", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1 2 3e)", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1 2 3) GARBAGE", true));
  }

  {
    TEST(getWKT(util::geo::pointFromWKT<double>(
             std::string("POINT(+1 2)"),
             true)),
         ==, "POINT(1 2)");

    TEST(getWKT(util::geo::pointFromWKT<double>(
             std::string("POINT(1e1 2)"),
             true)),
         ==, "POINT(10 2)");

    TEST(getWKT(util::geo::pointFromWKT<double>(
             std::string("POINT(1e1 2e3)"),
             true)),
         ==, "POINT(10 2000)");

    TEST(getWKT(util::geo::pointFromWKT<double>(
             std::string("POINT(1e-1 2e3)"),
             true)),
         ==, "POINT(0.1 2000)");

    TEST(getWKT(util::geo::pointFromWKT<double>(
             std::string("POINT(1.5e-1 2e3)"),
             true)),
         ==, "POINT(0.15 2000)");

    TEST(getWKT(util::geo::pointFromWKT<double>(
             std::string("POINT(.5e-1 2e3)"),
             true)),
         ==, "POINT(0.05 2000)");

    TEST(getWKT(util::geo::pointFromWKT<double>(
             std::string("POINT(-.5e-1 2e3)"),
             true)),
         ==, "POINT(-0.05 2000)");

    TEST(getWKT(util::geo::pointFromWKT<double>(
             std::string("POINT(+.5e-1 2e3)"),
             true)),
         ==, "POINT(0.05 2000)");
  }

  {
    TEST(util::geo::getWKT(
             util::geo::pointFromWKT<double>("POINT(1 2) \n\t", 0, true)),
         ==, "POINT(1 2)");
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1 2) GARBAGE", 0, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1 2))", 0, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1 2), POINT(5 5)", 0, true));

    const char* in = "POINT(1 2) GARBAGE";
    const char* end = 0;
    TEST(util::geo::getWKT(util::geo::pointFromWKT<double>(in, &end, true)), ==,
         "POINT(1 2)");
    TEST(end - in, ==, 9);

    TEST(util::geo::getWKT(util::geo::pointFromWKT<double>(in, 0, false)), ==,
         "POINT(1 2)");
  }

  {
    // numbers glued together must not be split into several coordinates
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1.2.3 4)", true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1-2 3)", true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1+2)", true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1 2.3.4)", true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1 2-3)", true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1 2 3.4.5)", true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1a 2)", true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::pointFromWKT<double>("POINT(1 2a)", true));

    TEST(getWKT(util::geo::pointFromWKT<double>("POINT(1 2)", true)), ==,
         "POINT(1 2)");
    TEST(getWKT(util::geo::pointFromWKT<double>("POINT(1\t2\n)", true)), ==,
         "POINT(1 2)");
    TEST(getWKT(util::geo::pointFromWKT<double>("POINT(1e-1 -2E+1)", true)), ==,
         "POINT(0.1 -20)");
    TEST(getWKT(util::geo::pointFromWKT<double>("POINT(1 2 3)", true)), ==,
         "POINT(1 2)");

    // non-strict parsing is unchanged
    TEST(getWKT(util::geo::pointFromWKT<double>("POINT(1-2 3)", false)), ==,
         "POINT(1 -2)");
  }

  {
    // quotes around WKT are treated like whitespace
    TEST(getWKT(pointFromWKT<double>(
             std::string("\"POINT(1 2)\""), true)),
         ==, "POINT(1 2)");
    TEST(getWKT(pointFromWKT<double>(
             std::string("'POINT(1 2)'"), true)),
         ==, "POINT(1 2)");
    TEST(getWKT(pointFromWKT<double>(
             std::string("\"POINT(1 2)"), true)),
         ==, "POINT(1 2)");
    TEST(getWKT(pointFromWKT<double>(
             std::string("POINT(1 2)\""), true)),
         ==, "POINT(1 2)");
    TEST(getWKTType("POINT \"(1 2)\"", 0, true), ==, WKTType::NONE);
    TEST_THROWS(WKTParseException,
                pointFromWKT<double>(std::string("POINT(1 \"2)"), true));
    TEST_THROWS(WKTParseException,
                pointFromWKT<double>(std::string("POINT(\" 1 2 \")"), true));
    TEST(getWKT(pointFromWKT<double>(
             std::string("\"POINT(1 2)\""), false)),
         ==, "POINT(1 2)");
    TEST(getWKT(pointFromWKT<double>(
             std::string("'POINT(1 2)'"), false)),
         ==, "POINT(1 2)");
    TEST(getWKT(pointFromWKT<double>(
             std::string("\"POINT(1 2)"), false)),
         ==, "POINT(1 2)");
    TEST(getWKT(pointFromWKT<double>(
             std::string("POINT(1 2)\""), false)),
         ==, "POINT(1 2)");
    TEST(getWKT(pointFromWKT<double>(
             std::string("POINT \"(1 2)\""), false)),
         ==, "POINT(1 2)");
    TEST(getWKT(pointFromWKT<double>(
             std::string("POINT(1 \"2)"), false)),
         ==, "POINT(0 0)");
    TEST(getWKT(pointFromWKT<double>(
             std::string("POINT(\" 1 2 \")"), false)),
         ==, "POINT(0 0)");
    TEST(getWKTType("\"POINT(1 2)\"", 0, true), ==,
         WKTType::POINT);
    TEST(getWKTType("'LINESTRING EMPTY'", 0, true), ==,
         WKTType::LINESTRING);
    TEST(getWKTType("LINESTRING \"EMPTY\"", 0, true), ==,
         WKTType::NONE);
    TEST_THROWS(WKTParseException, getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(\"POINT(1 2)\", 'LINESTRING(1 2, 3 4)')"), true)));
    TEST_THROWS(WKTParseException,
                collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTIPOINT \"EMPTY,POINT(3 4))"), true));
    TEST_THROWS(WKTParseException,
                collectionFromWKT<double>(
                    std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTIPOINT \"EMPTY)"), true));
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(\"POINT(1 2)\", 'LINESTRING(1 2, 3 4)')"), false)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),LINESTRING(1 2,3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTIPOINT \"EMPTY,POINT(3 4))"), false)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2),POINT(3 4))");
    TEST(getWKT(collectionFromWKT<double>(
             std::string("GEOMETRYCOLLECTION(POINT(1 2),MULTIPOINT \"EMPTY)"), false)),
         ==, "GEOMETRYCOLLECTION(POINT(1 2))");
  }
}
