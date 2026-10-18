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
void GeoTest::testWktParseMultiPoint() {
  {
    TEST(util::geo::getWKTType("MULTIPOINT(0 0, 1 1)", 0), ==,
         WKTType::MULTIPOINT);
    TEST(util::geo::getWKTType("MMULTIPOINT(0 0, 1 1)", 0), ==,
         WKTType::MULTIPOINT);
    TEST(util::geo::getWKTType("  MMULTIPOINT (0 0, 1 1)", 0), ==,
         WKTType::MULTIPOINT);
    TEST(util::geo::getWKTType("  MMULTIPOINT Z(0 0, 1 1)", 0), ==,
         WKTType::MULTIPOINT);
    TEST(util::geo::getWKTType("  MULTIPOINT Z(0 0, 1 1)", 0), ==,
         WKTType::MULTIPOINT);
    TEST(util::geo::getWKTType("  MULTIPOINT M(0 0, 1 1)", 0), ==,
         WKTType::MULTIPOINT);
    TEST(util::geo::getWKTType("  MULTIPOINT ZM(0 0, 1 1)", 0), ==,
         WKTType::MULTIPOINT);
    TEST(util::geo::getWKTType("  MULTIPOINT (0 0, 1 1)", 0), ==,
         WKTType::MULTIPOINT);
    TEST(util::geo::getWKTType("  MULTIPOIN (0 0, 1 1)", 0), ==, WKTType::NONE);
    TEST(util::geo::getWKTType("  ULTIPOINT (0 0, 1 1)", 0), ==, WKTType::NONE);

    TEST(util::geo::getWKTType("multipoint(0 0, 1 1)", 0), ==,
         WKTType::MULTIPOINT);
    TEST(util::geo::getWKTType("mmultipoint(0 0, 1 1)", 0), ==,
         WKTType::MULTIPOINT);
    TEST(util::geo::getWKTType("  mmultipoint (0 0, 1 1)", 0), ==,
         WKTType::MULTIPOINT);
    TEST(util::geo::getWKTType("  mmultipoint z(0 0, 1 1)", 0), ==,
         WKTType::MULTIPOINT);
    TEST(util::geo::getWKTType("  mmultipoint z(0 0, 1 1)", 0), ==,
         WKTType::MULTIPOINT);
    TEST(util::geo::getWKTType("  mmultipoint (0 0, 1 1)", 0), ==,
         WKTType::MULTIPOINT);

    TEST(util::geo::getWKTType("mUltIpOint(0 0, 1 1)", 0), ==,
         WKTType::MULTIPOINT);
    TEST(util::geo::getWKTType("mMultiPoint(0 0, 1 1)", 0), ==,
         WKTType::MULTIPOINT);
    TEST(util::geo::getWKTType("  Multipoint z(0 0, 1 1)", 0), ==,
         WKTType::MULTIPOINT);
    TEST(util::geo::getWKTType("  Multipoint ZM(0 0, 1 1)", 0), ==,
         WKTType::MULTIPOINT);
    TEST(util::geo::getWKTType("  MultipoinT (0 0, 1 1)", 0), ==,
         WKTType::MULTIPOINT);

    TEST(util::geo::getWKT(multiPointFromWKT<int>("MULTIPOINT(0 0, 1 1)", 0)),
         ==, "MULTIPOINT(0 0,1 1)");
    TEST(util::geo::getWKT(
             multiPointFromWKT<int>("MULTIPOINT((0 0), (1 1))", 0)),
         ==, "MULTIPOINT(0 0,1 1)");
    TEST(util::geo::getWKT(multiPointFromWKT<int>("MULTIPOINT (0 0, 1 1)", 0)),
         ==, "MULTIPOINT(0 0,1 1)");
    TEST(
        util::geo::getWKT(multiPointFromWKT<int>("MULTIPOINT   (0 0, 1 1)", 0)),
        ==, "MULTIPOINT(0 0,1 1)");
    TEST(util::geo::getWKT(multiPointFromWKT<int>(
             "   MULTIPOINT   (  0    0  , 1  1   )", 0)),
         ==, "MULTIPOINT(0 0,1 1)");

    TEST(util::geo::getWKT(multiPointFromWKT<int>("(0 0, 1 1)", 0)), ==,
         "MULTIPOINT(0 0,1 1)");
    TEST(util::geo::getWKT(multiPointFromWKT<int>(" (0 0, 1 1)", 0)), ==,
         "MULTIPOINT(0 0,1 1)");
    TEST(util::geo::getWKT(multiPointFromWKT<int>("   (0 0, 1 1)", 0)), ==,
         "MULTIPOINT(0 0,1 1)");
    TEST(util::geo::getWKT(
             multiPointFromWKT<int>("      (  0    0  , 1  1   )", 0)),
         ==, "MULTIPOINT(0 0,1 1)");

    TEST(util::geo::getWKT(multiPointFromWKT<int>("MULTIPOINT Z(0 0, 1 1)", 0)),
         ==, "MULTIPOINT(0 0,1 1)");
    TEST(
        util::geo::getWKT(multiPointFromWKT<int>("MULTIPOINT Z (0 0, 1 1)", 0)),
        ==, "MULTIPOINT(0 0,1 1)");
    TEST(util::geo::getWKT(
             multiPointFromWKT<int>("MULTIPOINT Z  (0 0, 1 1)", 0)),
         ==, "MULTIPOINT(0 0,1 1)");
    TEST(util::geo::getWKT(multiPointFromWKT<int>(
             "   MULTIPOINT Z   (  0    0  , 1  1   )", 0)),
         ==, "MULTIPOINT(0 0,1 1)");
    TEST(util::geo::getWKT(multiPointFromWKT<int>("MMULTIPOINT (0 0, 1 1)", 0)),
         ==, "MULTIPOINT(0 0,1 1)");
    TEST(util::geo::getWKT(
             multiPointFromWKT<int>("MMULTIPOINT  Z  ( (0 0 ),  ( 1 1 ) )", 0)),
         ==, "MULTIPOINT(0 0,1 1)");
    TEST(
        util::geo::getWKT(multiPointFromWKT<int>("MMULTIPOINT  (0 0, 1 1)", 0)),
        ==, "MULTIPOINT(0 0,1 1)");
    TEST(util::geo::getWKT(
             multiPointFromWKT<int>("MMULTIPOINT   (0 0, 1 1)", 0)),
         ==, "MULTIPOINT(0 0,1 1)");
    TEST(util::geo::getWKT(multiPointFromWKT<int>(
             "   MMULTIPOINT    (  0    0  0.5 , 1  1 100  )", 0)),
         ==, "MULTIPOINT(0 0,1 1)");
  }

  {
    // strict parsing
    TEST(getWKT(util::geo::multiPointFromWKT<double>("MULTIPOINT(1 2, 3 4)",
                                                     true)),
         ==, "MULTIPOINT(1 2,3 4)");
    TEST(getWKT(util::geo::multiPointFromWKT<double>("MULTIPOINT(1 2, 3 4)",
                                                     false)),
         ==, "MULTIPOINT(1 2,3 4)");

    TEST(getWKT(util::geo::multiPointFromWKT<double>("MULTIPOINT((1 2), (3 4))",
                                                     true)),
         ==, "MULTIPOINT(1 2,3 4)");
    TEST(getWKT(util::geo::multiPointFromWKT<double>("MULTIPOINT((1 2), (3 4))",
                                                     false)),
         ==, "MULTIPOINT(1 2,3 4)");

    TEST(getWKT(util::geo::multiPointFromWKT<double>(
             "MULTIPOINT ( (1 2) , (3 4) )", true)),
         ==, "MULTIPOINT(1 2,3 4)");
    TEST(getWKT(util::geo::multiPointFromWKT<double>(
             "MULTIPOINT ( (1 2) , (3 4) )", false)),
         ==, "MULTIPOINT(1 2,3 4)");

    TEST(getWKT(util::geo::multiPointFromWKT<double>("MULTIPOINT EMPTY", true)),
         ==, "MULTIPOINT EMPTY");

    TEST(getWKT(util::geo::multiPointFromWKT<double>("multipoint empty", true)),
         ==, "MULTIPOINT EMPTY");

    TEST(getWKT(util::geo::multiPointFromWKT<double>("MULTIPOINT(EMPTY, (1 2))",
                                                     true)),
         ==, "MULTIPOINT(1 2)");
    TEST(getWKT(util::geo::multiPointFromWKT<double>("MULTIPOINT(EMPTY, (1 2))",
                                                     false)),
         ==, "MULTIPOINT(1 2)");

    TEST(getWKT(util::geo::multiPointFromWKT<double>("MULTIPOINT((1 2), EMPTY)",
                                                     true)),
         ==, "MULTIPOINT(1 2)");
    TEST(getWKT(util::geo::multiPointFromWKT<double>("MULTIPOINT((1 2), EMPTY)",
                                                     false)),
         ==, "MULTIPOINT(1 2)");
    TEST(getWKT(util::geo::multiPointFromWKT<double>(
             "MULTIPOINT((1 2), empty, EMPTY)", false)),
         ==, "MULTIPOINT(1 2)");

    TEST(
        getWKT(util::geo::multiPointFromWKT<double>("MULTIPOINT(EMPTY)", true)),
        ==, "MULTIPOINT EMPTY");
    TEST(getWKT(
             util::geo::multiPointFromWKT<double>("MULTIPOINT(EMPTY)", false)),
         ==, "MULTIPOINT EMPTY");

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPointFromWKT<double>("", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPointFromWKT<double>("MULTIPOINT", true));

    TEST_THROWS(
        util::geo::WKTParseException,
        util::geo::multiPointFromWKT<double>("MULTIPOINT(1 2, 3 4", true));

    TEST_THROWS(
        util::geo::WKTParseException,
        util::geo::multiPointFromWKT<double>("MULTIPOINT((1 2), (3 4)", true));

    TEST_THROWS(
        util::geo::WKTParseException,
        util::geo::multiPointFromWKT<double>("MULTIPOINT((1 2, 3 4))", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPointFromWKT<double>("MULTIPOINT(1)", true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPointFromWKT<double>(
                    "MULTIPOINT(2 2 3, 4 3 2 GARBAGE)", true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPointFromWKT<double>(
                    "MULTIPOINT((2 2 3), (4 3 2 GARBAGE))", true));
    TEST(getWKT(util::geo::multiPointFromWKT<double>(
             "MULTIPOINT(2 2 3, 4 3 2 GARBAGE)", false)),
         ==, "MULTIPOINT(2 2,4 3)");
    TEST(getWKT(util::geo::multiPointFromWKT<double>(
             "MULTIPOINT((2 2 3), (4 3 2 GARBAGE))", false)),
         ==, "MULTIPOINT(2 2,4 3)");

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPointFromWKT<double>("MULTIPOINT()", true));

    TEST_THROWS(
        util::geo::WKTParseException,
        util::geo::multiPointFromWKT<double>("MULTIPOINT((1 2),)", true));
  }

  {
    TEST(getWKT(util::geo::multiPointFromWKTProj<double>(
             std::string("MULTIPOINT((1 2), (3 4))"),
             util::geo::projectToCRS84<double>, true)),
         ==, "MULTIPOINT(1 2,3 4)");
    TEST(getWKT(util::geo::multiPointFromWKTProj<double>(
             "MULTIPOINT((1 2), (3 4))", 0, util::geo::projectToCRS84<double>,
             true)),
         ==, "MULTIPOINT(1 2,3 4)");
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPointFromWKTProj<double>(
                    std::string("MULTIPOINT((1 2, 3 4))"),
                    util::geo::projectToCRS84<double>, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPointFromWKTProj<double>(
                    "MULTIPOINT((1 2, 3 4))", 0,
                    util::geo::projectToCRS84<double>, true));
  }

  {
    TEST(getWKT(util::geo::multiPointFromWKT<double>(
             std::string("MULTIPOINT((+1 2), (2 +1), (+1 +2))"),
             true)),
         ==, "MULTIPOINT(1 2,2 1,1 2)");

    TEST(getWKT(util::geo::multiPointFromWKT<double>(
             std::string("MULTIPOINT((+1 2),( 2 +1.0),( +1 +2))"),
             true)),
         ==, "MULTIPOINT(1 2,2 1,1 2)");

    TEST(getWKT(util::geo::multiPointFromWKT<double>(
             std::string("MULTIPOINT((+1 2),( 2 +1.0e0),( +1 +2))"),
             true)),
         ==, "MULTIPOINT(1 2,2 1,1 2)");

    TEST(getWKT(util::geo::multiPointFromWKT<double>(
             std::string("MULTIPOINT(+1 2, 2 +1.0e1, +1 +2)"),
             true)),
         ==, "MULTIPOINT(1 2,2 10,1 2)");

    TEST(getWKT(util::geo::multiPointFromWKT<double>(
             std::string("MULTIPOINT(+1 2, 2 +1.0e-1, +1 +2)"),
             true)),
         ==, "MULTIPOINT(1 2,2 0.1,1 2)");

    TEST(getWKT(util::geo::multiPointFromWKT<double>(
             std::string("MULTIPOINT(+1 2, 2 -1.0e-1, +1 +2)"),
             true)),
         ==, "MULTIPOINT(1 2,2 -0.1,1 2)");

    TEST(getWKT(util::geo::multiPointFromWKT<double>(
             std::string("MULTIPOINT(+1 2, 2 -1e-1, +1 +2)"),
             true)),
         ==, "MULTIPOINT(1 2,2 -0.1,1 2)");

    TEST(getWKT(util::geo::multiPointFromWKT<double>(
             std::string("MULTIPOINT(+1 2, 2 -.1e-0, +1 +2)"),
             true)),
         ==, "MULTIPOINT(1 2,2 -0.1,1 2)");
  }

  {
    TEST(util::geo::getWKT(
             util::geo::multiPointFromWKT<double>("MULTIPOINT((1 2), (3 4)) \n\t", 0, true)),
         ==, "MULTIPOINT(1 2,3 4)");
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPointFromWKT<double>("MULTIPOINT((1 2), (3 4)) GARBAGE", 0, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPointFromWKT<double>("MULTIPOINT((1 2), (3 4)))", 0, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPointFromWKT<double>("MULTIPOINT((1 2), (3 4)), POINT(5 5)", 0, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPointFromWKT<double>("MULTIPOINT EMPTY GARBAGE", 0, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPointFromWKT<double>("MULTIPOINT EMPTY)", 0, true));

    const char* in = "MULTIPOINT((1 2), (3 4)) GARBAGE";
    const char* end = 0;
    TEST(util::geo::getWKT(util::geo::multiPointFromWKT<double>(in, &end, true)), ==,
         "MULTIPOINT(1 2,3 4)");
    TEST(end - in, ==, 23);

    TEST(util::geo::getWKT(util::geo::multiPointFromWKT<double>(in, 0, false)), ==,
         "MULTIPOINT(1 2,3 4)");
  }

  {
    // numbers glued together must not be split into several coordinates
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPointFromWKT<double>("MULTIPOINT((1 2),(3-4))", true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPointFromWKT<double>("MULTIPOINT(1 2, 3.4.5)", true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiPointFromWKT<double>("MULTIPOINT(1-2, 3 4)", true));
    TEST(getWKT(util::geo::multiPointFromWKT<double>("MULTIPOINT((1 -2),(-3 4))", true)),
         ==, "MULTIPOINT(1 -2,-3 4)");
    TEST(getWKT(util::geo::multiPointFromWKT<double>("MULTIPOINT(1 -2, -3 4)", true)),
         ==, "MULTIPOINT(1 -2,-3 4)");
  }
}
