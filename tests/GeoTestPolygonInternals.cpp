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
void GeoTest::testPolygonInternals() {
  {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 1 0, 1 1, 0 1, 0 0))");
    XSortedPolygon<int> ax(a);

    for (size_t i = 0; i < ax.getOuter().rawRing().size(); i++) {
      auto c = ax.getOuter().rawRing()[i];
      if (util::geo::dist(c.seg().first, c.seg().second) > 0) {
        TEST(fabs(c.nextAng() - (-M_PI / 2)) < 0.001);
      }
    }
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 1 0, 1 0, 1 1, 0 1, 0 0))");
    XSortedPolygon<int> ax(a);

    for (size_t i = 0; i < ax.getOuter().rawRing().size(); i++) {
      auto c = ax.getOuter().rawRing()[i];
      TEST(fabs(c.nextAng() - (-M_PI / 2)) < 0.001);
    }
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 0 0, 1 0, 1 1, 0 1, 0 0))");
    XSortedPolygon<int> ax(a);

    for (size_t i = 0; i < ax.getOuter().rawRing().size(); i++) {
      auto c = ax.getOuter().rawRing()[i];
      TEST(fabs(c.nextAng() - (-M_PI / 2)) < 0.001);
    }
  }

  {
    auto a = polygonFromWKT<int>(
        "POLYGON((0 0, 0 0, 1 0, 1 0, 1 0, 1 1, 1 1, 1 1, 1 1, 0 1, 0 0))");
    XSortedPolygon<int> ax(a);

    for (size_t i = 0; i < ax.getOuter().rawRing().size(); i++) {
      auto c = ax.getOuter().rawRing()[i];
      TEST(fabs(c.nextAng() - (-M_PI / 2)) < 0.001);
    }
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 0 1, 1 1, 1 0, 0 0))");
    XSortedPolygon<int> ax(util::geo::getBoundingBox(a));

    for (size_t i = 0; i < ax.getOuter().rawRing().size(); i++) {
      auto c = ax.getOuter().rawRing()[i];
      TEST(fabs(c.nextAng() - (-M_PI / 2)) < 0.001);
    }
  }

  {
    // wrong polygon direction, should be corrected automatically
    auto a = polygonFromWKT<int>(
        "POLYGON((0 0, 1 0, 1 1, 0 1, 0 0), (0 0, 0 1, 1 1, 1 0, 0 0))");
    XSortedPolygon<int> ax(a);

    for (size_t i = 0; i < ax.getOuter().rawRing().size(); i++) {
      auto c = ax.getOuter().rawRing()[i];
      TEST(fabs(c.nextAng() - (-M_PI / 2)) < 0.001);
    }
    for (size_t i = 0; i < ax.getInners()[0].rawRing().size(); i++) {
      auto c = ax.getInners()[0].rawRing()[i];
      TEST(fabs(c.nextAng() - (M_PI / 2)) < 0.001);
    }
  }

  {
  // just to test that it does not crash
  auto a = polygonFromWKT<int>("POLYGON((0 0, 0 0, 0 0 0 0))");
    (void)a;
  }
}
