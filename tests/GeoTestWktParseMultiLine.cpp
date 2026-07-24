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
}
