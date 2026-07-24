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
void GeoTest::testLineOps() {

  // ___________________________________________________________________________
  {
    Line<double> a;
    a.push_back(Point<double>(1, 1));
    a.push_back(Point<double>(10, 1));

    auto dense = util::geo::densify(a, 1);

    TEST(dense.size(), ==, (size_t)10);

    for (int i = 0; i < 10; i++) {
      TEST(dense[i].getX(), ==, approx(i + 1.0));
    }

    dense = util::geo::simplify(dense, 0.1);
    TEST(dense.size(), ==, (size_t)2);

    Line<double> b;
    b.push_back(Point<double>(1, 1));
    b.push_back(Point<double>(5, 7));
    b.push_back(Point<double>(10, 3));

    dense = util::geo::densify(b, 1);

    dense = util::geo::simplify(dense, 0.1);
    TEST(dense.size(), ==, (size_t)3);
  }

  // ___________________________________________________________________________
  {
    Line<double> a;
    a.push_back(Point<double>(1, 1));
    a.push_back(Point<double>(2, 1));
    a.push_back(Point<double>(3, 1));
    a.push_back(Point<double>(3, 2));
    a.push_back(Point<double>(4, 2));
    a.push_back(Point<double>(4, 1));
    a.push_back(Point<double>(5, 1));
    a.push_back(Point<double>(6, 1));

    Line<double> b;
    b.push_back(Point<double>(1, 1));
    b.push_back(Point<double>(2, 1));
    b.push_back(Point<double>(3, 1));
    b.push_back(Point<double>(4, 1));
    b.push_back(Point<double>(5, 1));
    b.push_back(Point<double>(6, 1));

    double fd = util::geo::accFrechetDistC(a, b, 0.1);
    TEST(fd, ==, approx(2));
  }

  // ___________________________________________________________________________
  {
    Line<double> e;
    e.push_back(Point<double>(1, 1));
    e.push_back(Point<double>(1, 2));

    Line<double> f;
    f.push_back(Point<double>(1, 1));
    f.push_back(Point<double>(1, 2));

    double fd = util::geo::frechetDist(e, f, 0.1);

    TEST(fd, ==, approx(0));

    Line<double> a;
    a.push_back(Point<double>(1, 1));
    a.push_back(Point<double>(2, 1));
    a.push_back(Point<double>(3, 2));
    a.push_back(Point<double>(4, 2));
    a.push_back(Point<double>(5, 1));
    a.push_back(Point<double>(6, 1));

    Line<double> b;
    b.push_back(Point<double>(1, 1));
    b.push_back(Point<double>(2, 1));
    b.push_back(Point<double>(3, 1));
    b.push_back(Point<double>(4, 1));
    b.push_back(Point<double>(5, 1));
    b.push_back(Point<double>(6, 1));

    auto adense = util::geo::densify(a, 0.1);
    auto bdense = util::geo::densify(b, 0.1);

    fd = util::geo::frechetDist(a, b, 0.1);

    TEST(fd, ==, approx(1));

    Line<double> c;
    c.push_back(Point<double>(1, 1));
    c.push_back(Point<double>(2, 1));

    Line<double> d;
    d.push_back(Point<double>(3, 1));
    d.push_back(Point<double>(4, 1));

    fd = util::geo::frechetDist(c, d, 0.1);

    TEST(fd, ==, approx(2));

    Line<double> g;
    g.push_back(Point<double>(1, 1));
    g.push_back(Point<double>(10, 1));

    Line<double> h;
    h.push_back(Point<double>(1, 1));
    h.push_back(Point<double>(3, 2));
    h.push_back(Point<double>(3, 1));
    h.push_back(Point<double>(10, 1));

    fd = util::geo::frechetDist(g, h, 0.1);

    TEST(fd, ==, approx(1));
  }

  // ___________________________________________________________________________
  {
    Line<double> a;
    a.push_back(Point<double>(1, 1));
    a.push_back(Point<double>(1, 2));

    Line<double> b;
    b.push_back(Point<double>(1, 2));
    b.push_back(Point<double>(2, 2));

    Line<double> c;
    c.push_back(Point<double>(2, 2));
    c.push_back(Point<double>(2, 1));

    Line<double> d;
    d.push_back(Point<double>(2, 1));
    d.push_back(Point<double>(1, 1));

    Box<double> box(Point<double>(2, 3), Point<double>(5, 4));
    MultiLine<double> ml;
    ml.push_back(a);
    ml.push_back(b);
    ml.push_back(c);
    ml.push_back(d);

    TEST(parallelity(box, ml), ==, approx(1));
    ml = rotate(ml, 45);
    TEST(parallelity(box, ml), ==, approx(0));
    ml = rotate(ml, 45);
    TEST(parallelity(box, ml), ==, approx(1));
    ml = rotate(ml, 45);
    TEST(parallelity(box, ml), ==, approx(0));
    ml = rotate(ml, 45);
    TEST(parallelity(box, ml), ==, approx(1));
  }
}
