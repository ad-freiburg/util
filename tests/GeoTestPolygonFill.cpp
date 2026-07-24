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
void GeoTest::testPolygonFill() {
  {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 2 0, 2 2, 0 2, 0 0))");
    auto fillPoints = fillPolygon(a, 1);

    //  4
    //
    //  3
    //
    //  2 ---------
    //    |       |
    //  1 x   x   x
    //    |       |
    //    x---x---x
    //        1   2   3   4

    // the intersection of the uppermost y line does not produce any points
    // because of (wanted) asymmetry, see code
    TEST(fillPoints.size(), ==, 3 * 2);
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 3 0, 3 3, 0 3, 0 0))");
    auto fillPoints = fillPolygon(a, 1);

    //  4
    //
    //  3 -------------
    //    |           |
    //  2 x   x   x   x
    //    |           |
    //  1 x   x   x   x
    //    |           |
    //    x---x---x---x
    //        1   2   3   4

    TEST(fillPoints.size(), ==, 4 * 3);
  }

  {
    auto a = polygonFromWKT<int>(
        "POLYGON((0 0, 3 0, 3 3, 0 3, 0 0), (1 1, 2 1, 2 2, 1 2, 1 1))");
    auto fillPoints = fillPolygon(a, 1);

    //  4
    //
    //  3 -------------
    //    |           |
    //  2 x   x---x   x
    //    |   |   |   |
    //  1 x   x---x   x
    //    |           |
    //    x---x---x---x
    //        1   2   3   4

    TEST(fillPoints.size(), ==, 4 * 3);
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 4 0, 4 4, 0 4, 0 0))");
    auto fillPoints = fillPolygon(a, 1);

    //  4 -----------------
    //    |               |
    //  3 x   x   x   x   x
    //    |               |
    //  2 x   x   x   x   x
    //    |               |
    //  1 x   x   x   x   x
    //    |               |
    //    x---x---x---x---x
    //        1   2   3   4

    TEST(fillPoints.size(), ==, 5 * 4);
  }

  {
    auto a = polygonFromWKT<int>(
        "POLYGON((0 0, 4 0, 4 4, 0 4, 0 0), (1 1, 3 1, 3 3, 1 3, 1 1))");
    auto fillPoints = fillPolygon(a, 1);

    //  4 -----------------
    //    |               |
    //  3 x   x---x---x   x
    //    |   |       |   |
    //  2 x   x       x   x
    //    |   |       |   |
    //  1 x   x-------x   x
    //    |               |
    //    x---x---x---x---x
    //        1   2   3   4

    TEST(fillPoints.size(), ==, 18);

    std::sort(fillPoints.begin(), fillPoints.end());

    TEST(fillPoints[0], ==, (Point<int>(0, 0)));
    TEST(fillPoints[1], ==, (Point<int>(0, 1)));
    TEST(fillPoints[2], ==, (Point<int>(0, 2)));
    TEST(fillPoints[3], ==, (Point<int>(0, 3)));

    TEST(fillPoints[4], ==, (Point<int>(1, 0)));
    TEST(fillPoints[5], ==, (Point<int>(1, 1)));
    TEST(fillPoints[6], ==, (Point<int>(1, 2)));
    TEST(fillPoints[7], ==, (Point<int>(1, 3)));

    TEST(fillPoints[8], ==, (Point<int>(2, 0)));
    TEST(fillPoints[9], ==, (Point<int>(2, 3)));

    TEST(fillPoints[10], ==, (Point<int>(3, 0)));
    TEST(fillPoints[11], ==, (Point<int>(3, 1)));
    TEST(fillPoints[12], ==, (Point<int>(3, 2)));
    TEST(fillPoints[13], ==, (Point<int>(3, 3)));

    TEST(fillPoints[14], ==, (Point<int>(4, 0)));
    TEST(fillPoints[15], ==, (Point<int>(4, 1)));
    TEST(fillPoints[16], ==, (Point<int>(4, 2)));
    TEST(fillPoints[17], ==, (Point<int>(4, 3)));
  }
}
