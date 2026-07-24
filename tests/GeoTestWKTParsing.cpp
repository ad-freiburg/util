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
void GeoTest::testWKTParsing() {

  // ___________________________________________________________________________
  {
    auto p = pointFromWKT<double>("POINT(10 50)");
    TEST(p.getX(), ==, approx(10));
    TEST(p.getY(), ==, approx(50));

    p = pointFromWKT<double>("POINT( 10 50)");
    TEST(p.getX(), ==, approx(10));
    TEST(p.getY(), ==, approx(50));

    p = pointFromWKT<double>("POINT(10 50 30)");
    TEST(p.getX(), ==, approx(10));
    TEST(p.getY(), ==, approx(50));

    p = pointFromWKT<double>("POINT(10     50 30)");
    TEST(p.getX(), ==, approx(10));
    TEST(p.getY(), ==, approx(50));

    p = pointFromWKT<double>("POINT(10 50 30)");
    TEST(p.getX(), ==, approx(10));
    TEST(p.getY(), ==, approx(50));

    p = pointFromWKT<double>("POINT(10    50) ");
    TEST(p.getX(), ==, approx(10));
    TEST(p.getY(), ==, approx(50));

    p = pointFromWKT<double>("MPOINT(10 50 30)");
    TEST(p.getX(), ==, approx(10));
    TEST(p.getY(), ==, approx(50));

    p = pointFromWKT<double>("MPOINT(10 50)");
    TEST(p.getX(), ==, approx(10));
    TEST(p.getY(), ==, approx(50));

    p = pointFromWKT<double>("POINT(10.05 50.05)");
    TEST(p.getX(), ==, approx(10.05));
    TEST(p.getY(), ==, approx(50.05));

    auto wktl = lineFromWKT<double>("LINESTRING(0 0, 1 1,2 3, 0 1)");
    TEST(wktl.size(), ==, (size_t)4);
    TEST(wktl[0].getX(), ==, approx(0));
    TEST(wktl[0].getY(), ==, approx(0));
    TEST(wktl[1].getX(), ==, approx(1));
    TEST(wktl[1].getY(), ==, approx(1));
    TEST(wktl[2].getX(), ==, approx(2));
    TEST(wktl[2].getY(), ==, approx(3));
    TEST(wktl[3].getX(), ==, approx(0));
    TEST(wktl[3].getY(), ==, approx(1));

    wktl = lineFromWKT<double>("MLINESTRING(0 0, 1 1,2 3, 0 1)");
    TEST(wktl.size(), ==, (size_t)4);
    TEST(wktl[0].getX(), ==, approx(0));
    TEST(wktl[0].getY(), ==, approx(0));
    TEST(wktl[1].getX(), ==, approx(1));
    TEST(wktl[1].getY(), ==, approx(1));
    TEST(wktl[2].getX(), ==, approx(2));
    TEST(wktl[2].getY(), ==, approx(3));
    TEST(wktl[3].getX(), ==, approx(0));
    TEST(wktl[3].getY(), ==, approx(1));

    wktl = lineFromWKT<double>("MLINESTRING(0 0, 1  1,2   3, 0 1 )");
    TEST(wktl.size(), ==, (size_t)4);
    TEST(wktl[0].getX(), ==, approx(0));
    TEST(wktl[0].getY(), ==, approx(0));
    TEST(wktl[1].getX(), ==, approx(1));
    TEST(wktl[1].getY(), ==, approx(1));
    TEST(wktl[2].getX(), ==, approx(2));
    TEST(wktl[2].getY(), ==, approx(3));
    TEST(wktl[3].getX(), ==, approx(0));
    TEST(wktl[3].getY(), ==, approx(1));
  }
}
