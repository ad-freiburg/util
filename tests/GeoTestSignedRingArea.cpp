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
void GeoTest::testSignedRingArea() {

  {
    auto a =
        polygonFromWKT<int>("POLYGON((0 0, 1 0, 1 3, 3 3, 3 5, 0 5, 0 0))");
    auto b = polygonFromWKT<int>("POLYGON((1 3, 2 3, 3 4, 1 4, 1 3))");
    auto c = polygonFromWKT<int>("POLYGON((0 0, 1 0, 1 1, 0 1, 0 0))");

    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);
    XSortedPolygon<int> cx(c);

    TEST(signedRingArea(c.getOuter()), ==, signedRingArea(cx.getOuter()));
    TEST(signedRingArea(a.getOuter()), ==, signedRingArea(ax.getOuter()));
    TEST(signedRingArea(b.getOuter()), ==, signedRingArea(bx.getOuter()));
  }
}
