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
void GeoTest::testAngles() {

  {
    TEST(angBetween(Point<int>{0, 0}, Point<int>{0, 1}, Point<int>{0, 1}) == 0);
    TEST(angBetween(Point<int>{0, 0}, Point<int>{0, 1}, Point<int>{1, 1}) > 0);
    TEST(angBetween(Point<int>{0, 0}, Point<int>{0, 1}, Point<int>{-1, 1}) < 0);
    TEST(angBetween(Point<int>{0, 0}, Point<int>{0, 1}, Point<int>{1, 0}) ==
         approx(M_PI / 2));
    TEST(angBetween(Point<int>{0, 0}, Point<int>{0, 1}, Point<int>{-1, 0}) ==
         approx(-M_PI / 2));
    TEST(angBetween(Point<int>{0, 0}, Point<int>{1, 0}, Point<int>{0, 1}) ==
         approx(-M_PI / 2));
  }

  {
    auto c = LineSegment<int>{{8830335, 62149579}, {8829909, 62149287}};
    auto d = LineSegment<int>{{8829947, 62149129}, {8829997, 62148988}};
    auto a = LineSegment<int>{{8829974, 62148972}, {8829997, 62148988}};
    auto b = LineSegment<int>{{8829997, 62148988}, {8830058, 62148937}};

    TEST(a > b);
    TEST(!(a < b));

    TEST(b < c);
    TEST(!(b > c));

    TEST(a < c);
    TEST(!(a > c));

    TEST(a < d);
    TEST(!(a > d));

    TEST(d < c);
    TEST(!(d > c));
  }
}
