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
void GeoTest::testSegmentOrdering() {
  {
    TEST(LineSegment<int>({1, 1}, {2, 1}) < LineSegment<int>({1, 2}, {2, 2}));
    TEST(LineSegment<int>({10000, 10000}, {20000, 10000}) <
         LineSegment<int>({10000, 20000}, {20000, 20000}));
    TEST(LineSegment<double>({1000000, 1000000}, {2000000, 1000000}) <
         LineSegment<double>({1000000, 2000000}, {2000000, 2000000}));
    TEST(LineSegment<int>({1000000, 1000000}, {2000000, 1000000}) <
         LineSegment<int>({1000000, 2000000}, {2000000, 2000000}));
  }
}
