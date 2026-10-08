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
void GeoTest::testPolyPointPredicates() {
  {
    XSortedPolygon<double> poly(
        polygonFromWKT<double>("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0))"));

    auto pInterior = pointFromWKT<double>("POINT(5 5)");
    auto pBoundary = pointFromWKT<double>("POINT(0 5)");
    auto pOutside = pointFromWKT<double>("POINT(20 20)");

    // polygon / point is the transpose of point / polygon
    TEST(geo::DE9IM(pInterior, poly), ==, "0FFFFF212");
    TEST(geo::DE9IM(poly, pInterior), ==, "0F2FF1FF2");
    TEST(geo::DE9IM(pBoundary, poly), ==, "F0FFFF212");
    TEST(geo::DE9IM(poly, pBoundary), ==, "FF20F1FF2");
    TEST(geo::DE9IM(pOutside, poly), ==, "FF0FFF212");
    TEST(geo::DE9IM(poly, pOutside), ==, "FF2FF10F2");
  }

  {
    auto emptyPoly = polygonFromWKT<double>("POLYGON EMPTY");
    Polygon<double> emptyOuter(
        Ring<double>(),
        {lineFromWKT<double>("LINESTRING(4 4, 5 4, 5 5, 4 5, 4 4)")});
    XSortedPolygon<double> emptyPolyX(emptyPoly);
    XSortedPolygon<double> emptyOuterX(emptyOuter);
    auto pInner = pointFromWKT<double>("POINT(4.5 4.5)");
    auto pOuter = pointFromWKT<double>("POINT(2 2)");

    TEST(geo::containsCovers(pInner, emptyPolyX) ==
         std::make_pair(false, false));
    TEST(geo::containsCovers(pInner, emptyOuterX) ==
         std::make_pair(false, false));
    TEST(geo::containsCovers(pOuter, emptyOuterX) ==
         std::make_pair(false, false));
  }
}
