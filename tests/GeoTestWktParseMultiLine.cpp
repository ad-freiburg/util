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

    TEST(util::geo::getWKTType("\t\tMMultiLineString Z EMPTY((1 1,3 3),(1 3,3 1))", 0, true),
         ==, WKTType::NONE);
    TEST(util::geo::getWKTType("MultiLineString EMPTY((1 1,3 3),(1 3,3 1))", 0, true),
         ==, WKTType::NONE);
    TEST(util::geo::getWKTType("MULTILINESTRING EMPTY((1 1,3 3),(1 3,3 1))", 0, true),
         ==, WKTType::NONE);

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
         ==, "MULTILINESTRING EMPTY");

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
         ==, "MULTILINESTRING EMPTY");
    TEST(getWKT(util::geo::multiLineFromWKT<double>("MULTILINESTRING(EMPTY)",
                                                    false)),
         ==, "MULTILINESTRING EMPTY");

    TEST(getWKT(util::geo::multiLineFromWKT<double>(
             "MULTILINESTRING(EMPTY, EMPTY)", true)),
         ==, "MULTILINESTRING EMPTY");
    TEST(getWKT(util::geo::multiLineFromWKT<double>(
             "MULTILINESTRING(EMPTY, EMPTY)", false)),
         ==, "MULTILINESTRING EMPTY");

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

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiLineFromWKT<double>(
                    "MULTILINESTRING((0 0, 1 1)", true));

    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiLineFromWKT<double>(
                    "MULTILINESTRING((0 0, 1 c))", true));
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

  {
    TEST(getWKT(util::geo::multiLineFromWKTProj<double>(
             std::string("MULTILINESTRING((0 +0, 1.0e-0 +1))"),
             util::geo::projectToCRS84<double>, true)),
         ==, "MULTILINESTRING((0 0,1 1))");
    TEST(getWKT(util::geo::multiLineFromWKTProj<double>(
             "MULTILINESTRING((0 0,  100e-2 .1e+1))", 0,
             util::geo::projectToCRS84<double>, true)),
         ==, "MULTILINESTRING((0 0,1 1))");
    TEST(getWKT(util::geo::multiLineFromWKTProj<double>(
             "MULTILINESTRING((0 0, .1e+1 100.0e-2))", 0,
             util::geo::projectToCRS84<double>, true)),
         ==, "MULTILINESTRING((0 0,1 1))");

    TEST(getWKT(util::geo::multiLineFromWKTProj<double>(
             std::string("MULTILINESTRING((1 1, 2e-00000 2), (0 +0, 1.0e-0 +1))"),
             util::geo::projectToCRS84<double>, true)),
         ==, "MULTILINESTRING((1 1,2 2),(0 0,1 1))");

    TEST(getWKT(util::geo::multiLineFromWKTProj<double>(
             std::string("MULTILINESTRING((1 100e-002, 2e-00000 2), (0 +0, 1.0e-0 +1))"),
             util::geo::projectToCRS84<double>, true)),
         ==, "MULTILINESTRING((1 1,2 2),(0 0,1 1))");

    TEST(getWKT(util::geo::multiLineFromWKTProj<double>(
             std::string("MULTILINESTRING((1 0.010e+002, 2e-00000 2), (0 +0, 1.0e-0 +1))"),
             util::geo::projectToCRS84<double>, true)),
         ==, "MULTILINESTRING((1 1,2 2),(0 0,1 1))");
  }

  {
    TEST(util::geo::getWKT(
             util::geo::multiLineFromWKT<double>("MULTILINESTRING((1 2, 3 4), (5 6, 7 8)) \n\t", 0, true)),
         ==, "MULTILINESTRING((1 2,3 4),(5 6,7 8))");
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiLineFromWKT<double>("MULTILINESTRING((1 2, 3 4), (5 6, 7 8)) GARBAGE", 0, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiLineFromWKT<double>("MULTILINESTRING((1 2, 3 4), (5 6, 7 8)))", 0, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiLineFromWKT<double>("MULTILINESTRING((1 2, 3 4), (5 6, 7 8)), POINT(5 5)", 0, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiLineFromWKT<double>("MULTILINESTRING EMPTY GARBAGE", 0, true));
    TEST_THROWS(util::geo::WKTParseException,
                util::geo::multiLineFromWKT<double>("MULTILINESTRING EMPTY)", 0, true));

    const char* in = "MULTILINESTRING((1 2, 3 4), (5 6, 7 8)) GARBAGE";
    const char* end = 0;
    TEST(util::geo::getWKT(util::geo::multiLineFromWKT<double>(in, &end, true)), ==,
         "MULTILINESTRING((1 2,3 4),(5 6,7 8))");
    TEST(end - in, ==, 38);

    TEST(util::geo::getWKT(util::geo::multiLineFromWKT<double>(in, 0, false)), ==,
         "MULTILINESTRING((1 2,3 4),(5 6,7 8))");
  }

  {
    MultiLine<double> empty;
    TEST(getWKT(empty), ==, "MULTILINESTRING EMPTY");

    MultiLine<double> oneEmpty;
    oneEmpty.push_back(Line<double>());
    TEST(getWKT(oneEmpty), ==, "MULTILINESTRING EMPTY");

    MultiLine<double> twoEmpty;
    twoEmpty.push_back(Line<double>());
    twoEmpty.push_back(Line<double>());
    TEST(getWKT(twoEmpty), ==, "MULTILINESTRING EMPTY");

    MultiLine<double> emptyFirst;
    emptyFirst.push_back(Line<double>());
    emptyFirst.push_back(Line<double>{{0, 0}, {1, 1}});
    TEST(getWKT(emptyFirst), ==, "MULTILINESTRING((0 0,1 1))");

    MultiLine<double> emptyLast;
    emptyLast.push_back(Line<double>{{0, 0}, {1, 1}});
    emptyLast.push_back(Line<double>());
    TEST(getWKT(emptyLast), ==, "MULTILINESTRING((0 0,1 1))");

    MultiLine<double> emptyMiddle;
    emptyMiddle.push_back(Line<double>{{0, 0}, {1, 1}});
    emptyMiddle.push_back(Line<double>());
    emptyMiddle.push_back(Line<double>{{2, 2}, {3, 3}});
    TEST(getWKT(emptyMiddle), ==, "MULTILINESTRING((0 0,1 1),(2 2,3 3))");

    TEST(getWKT(multiLineFromWKT<double>(getWKT(oneEmpty), true)), ==,
         "MULTILINESTRING EMPTY");
    TEST(getWKT(multiLineFromWKT<double>(getWKT(twoEmpty), true)), ==,
         "MULTILINESTRING EMPTY");
    TEST(getWKT(multiLineFromWKT<double>(getWKT(emptyFirst), true)), ==,
         "MULTILINESTRING((0 0,1 1))");
    TEST(getWKT(multiLineFromWKT<double>(getWKT(emptyMiddle), true)), ==,
         "MULTILINESTRING((0 0,1 1),(2 2,3 3))");

    Collection<double> col;
    col.push_back(oneEmpty);
    TEST(getWKT(col), ==, "GEOMETRYCOLLECTION EMPTY");
    TEST(getWKT(collectionFromWKT<double>(getWKT(col), true)), ==,
         "GEOMETRYCOLLECTION EMPTY");
  }
}
