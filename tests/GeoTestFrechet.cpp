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
void GeoTest::testFrechet() {

  // ___________________________________________________________________________
  {
    TEST(geo::frechetDist(Line<double>{{0, 0}, {10, 10}},
                          Line<double>{{0, 0}, {10, 10}}, 1) == approx(0));

    TEST(geo::frechetDist(Line<double>{{0, 0}, {10, 10}},
                          Line<double>{{0, 0}, {10, 10}}, 0.1) == approx(0));

    TEST(geo::frechetDist(Line<double>{{0, 10}, {10, 10}},
                          Line<double>{{0, 0}, {10, 0}}, 0.1) == approx(10));

    TEST(geo::frechetDist(Line<double>{{0, 10}, {10, 10}},
                          Line<double>{{0, 0}, {0, 0}}, 0.1) ==
         approx(util::geo::dist(Point<double>{0, 0}, Point<double>{10, 10})));

    TEST(geo::frechetDist(Line<double>{{0, 10}, {0, 10}},
                          Line<double>{{0, 0}, {0, 5}}, 0.1) == approx(10));

    TEST(geo::frechetDist(Line<double>{{0, 10}, {0, 10}},
                          Line<double>{{0, 5}, {0, 5}}, 0.1) == approx(5));
  }
}
