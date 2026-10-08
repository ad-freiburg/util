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
void GeoTest::testLinePointPredicates() {
  {
    XSortedLine<double> line(lineFromWKT<double>("LINESTRING(0 0, 10 0)"));

    auto pInterior = pointFromWKT<double>("POINT(5 0)");
    auto pBoundary = pointFromWKT<double>("POINT(0 0)");
    auto pOutside = pointFromWKT<double>("POINT(5 5)");

    TEST(geo::DE9IM(pInterior, line), ==, "0FFFFF102");
    TEST(geo::DE9IM(line, pInterior), ==, "0F1FF0FF2");
    TEST(geo::DE9IM(pBoundary, line), ==, "F0FFFF102");
    TEST(geo::DE9IM(line, pBoundary), ==, "FF10F0FF2");
    TEST(geo::DE9IM(pOutside, line), ==, "FF0FFF102");
    TEST(geo::DE9IM(line, pOutside), ==, "FF1FF00F2");
  }

  {
    XSortedLine<double> emptyLine(lineFromWKT<double>("LINESTRING EMPTY"));
    auto p = pointFromWKT<double>("POINT(5 0)");

    TEST(geo::intersectsContains(p, emptyLine) ==
         std::make_tuple(false, false));
  }
}
