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
void GeoTest::testWktParseMultiLine() {

  {
    TEST(util::geo::getWKTType("MULTILINESTRIN((1 1,3 3),(1 3,3 1))"), ==,
         WKTType::NONE);
    TEST(util::geo::getWKTType("MULTILINESTRING((1 1,3 3),(1 3,3 1))"), ==,
         WKTType::MULTILINESTRING);
    TEST(util::geo::getWKTType(" MULTILINESTRING  ((1 1,3 3),(1 3,3 1))"), ==,
         WKTType::MULTILINESTRING);
    TEST(util::geo::getWKTType("  MMULTILINESTRING  ((1 1,3 3),(1 3,3 1))"), ==,
         WKTType::MULTILINESTRING);
    TEST(util::geo::getWKTType(" MultiLineString  ((1 1,3 3),(1 3,3 1))"), ==,
         WKTType::MULTILINESTRING);
    TEST(util::geo::getWKTType("  MultiLineString\tM((1 1,3 3),(1 3,3 1))"), ==,
         WKTType::MULTILINESTRING);
    TEST(util::geo::getWKTType("  MMultiLineString Z((1 1,3 3),(1 3,3 1))"), ==,
         WKTType::MULTILINESTRING);
    TEST(util::geo::getWKTType("\t\tMMultiLineString Z((1 1,3 3),(1 3,3 1))"),
         ==, WKTType::MULTILINESTRING);

    TEST(util::geo::getWKT(
             multiLineFromWKT<int>("MULTILINESTRING((1 1,3 3),(1 3,3 1))")),
         ==, "MULTILINESTRING((1 1,3 3),(1 3,3 1))");
    TEST(util::geo::getWKT(multiLineFromWKT<int>(
             " MULTILINESTRING  ( (1 1,3 3) ,(1 3,3 1))")),
         ==, "MULTILINESTRING((1 1,3 3),(1 3,3 1))");
    TEST(util::geo::getWKT(multiLineFromWKT<int>(
             " MULTILINESTRING Z  ( (1 1,3 3) ,(1 3,3 1 ) )")),
         ==, "MULTILINESTRING((1 1,3 3),(1 3,3 1))");
    TEST(util::geo::getWKT(multiLineFromWKT<int>(
             " MULTILINESTRING Z  ( (1 1  ,3 3) ,(1   3 ,  3 1 ) )")),
         ==, "MULTILINESTRING((1 1,3 3),(1 3,3 1))");
  }

  {
    // strict

    TEST(getWKT(util::geo::multiLineFromWKT<double>(
             "MULTILINESTRING((0 0, 1 1), (2 2, 3 3))", true)),
         ==, "MULTILINESTRING((0 0,1 1),(2 2,3 3))");
    TEST(getWKT(util::geo::multiLineFromWKT<double>(
             "MULTILINESTRING((0 0, 1 1), (2 2, 3 3))", false)),
         ==, "MULTILINESTRING((0 0,1 1),(2 2,3 3))");

    TEST(getWKT(util::geo::multiLineFromWKT<double>("MULTILINESTRING EMPTY",
                                                    true)),
         ==, "MULTILINESTRING()");

    TEST(getWKT(util::geo::multiLineFromWKT<double>(
             "MULTILINESTRING(EMPTY, (0 0, 1 1))", true)),
         ==, "MULTILINESTRING((0 0,1 1))");
    TEST(getWKT(util::geo::multiLineFromWKT<double>(
             "MULTILINESTRING(EMPTY, (0 0, 1 1))", false)),
         ==, "MULTILINESTRING((0 0,1 1))");

    TEST(getWKT(util::geo::multiLineFromWKT<double>(
             "MULTILINESTRING((0 0, 1 1), EMPTY)", true)),
         ==, "MULTILINESTRING((0 0,1 1))");
    TEST(getWKT(util::geo::multiLineFromWKT<double>(
             "MULTILINESTRING((0 0, 1 1), EMPTY)", false)),
         ==, "MULTILINESTRING((0 0,1 1))");

    TEST(getWKT(util::geo::multiLineFromWKT<double>("MULTILINESTRING(EMPTY)",
                                                    true)),
         ==, "MULTILINESTRING()");
    TEST(getWKT(util::geo::multiLineFromWKT<double>("MULTILINESTRING(EMPTY)",
                                                    false)),
         ==, "MULTILINESTRING()");

    TEST(getWKT(util::geo::multiLineFromWKT<double>(
             "MULTILINESTRING(EMPTY, EMPTY)", true)),
         ==, "MULTILINESTRING()");
    TEST(getWKT(util::geo::multiLineFromWKT<double>(
             "MULTILINESTRING(EMPTY, EMPTY)", false)),
         ==, "MULTILINESTRING()");

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiLineFromWKT<double>("", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiLineFromWKT<double>("MULTILINESTRING", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiLineFromWKT<double>("MULTILINESTRING()", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiLineFromWKT<double>(
                    "MULTILINESTRING((0 0, 1 1), (2 2, 3 3)", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiLineFromWKT<double>(
                    "MULTILINESTRING((0 0, 1 1), emptay)", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiLineFromWKT<double>(
                    "MULTILINESTRING((0 0, 1 1), ())", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiLineFromWKT<double>(
                    "MULTILINESTRING((0 0, 1 1),)", true));

    TEST_THROWS(
        util::geo::WKTParseException,
        util::geo::multiLineFromWKT<double>("MULTILINESTRING((0 0, 1))", true));

    TEST_THROWS(
        util::geo::WKTParseException,
        util::geo::multiLineFromWKT<double>("MULTILINESTRING(0 0, 1 1)", true));
  }

  {
    TEST(getWKT(util::geo::multiLineFromWKTProj<double>(
             std::string("MULTILINESTRING((0 0, 1 1))"),
             util::geo::projectToCRS84<double>, true)),
         ==, "MULTILINESTRING((0 0,1 1))");
    TEST(getWKT(util::geo::multiLineFromWKTProj<double>(
             "MULTILINESTRING((0 0, 1 1))", 0,
             util::geo::projectToCRS84<double>, true)),
         ==, "MULTILINESTRING((0 0,1 1))");
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiLineFromWKTProj<double>(
                    std::string("MULTILINESTRING((0 0, 1 1),)"),
                    util::geo::projectToCRS84<double>, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiLineFromWKTProj<double>(
                    "MULTILINESTRING((0 0, 1 1),)", 0,
                    util::geo::projectToCRS84<double>, true));
    TEST(getWKT(util::geo::multiLineFromWKTProj<double>(
             std::string("MULTILINESTRING((0 0, 1 1),)"),
             util::geo::projectToCRS84<double>)),
         ==, "MULTILINESTRING((0 0,1 1))");
  }
}
