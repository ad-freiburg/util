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

  // ___________________________________________________________________________
  {
    Line<int32_t> l{{0, 0}, {10, 0}, {12, 5}};
    auto d = densifyX(l, 3);
    TEST(d.size(), ==, 6);
    TEST(d[0] == Point<int32_t>(0, 0));
    TEST(d[1] == Point<int32_t>(3, 0));
    TEST(d[2] == Point<int32_t>(5, 0));
    TEST(d[3] == Point<int32_t>(8, 0));
    TEST(d[4] == Point<int32_t>(10, 0));
    TEST(d[5] == Point<int32_t>(12, 5));
    for (size_t i = 1; i < d.size(); i++) {
      TEST(std::abs(d[i].getX() - d[i - 1].getX()), <=, 3);
    }

    // nothing to split
    TEST(densifyX(l, 10) == l);
    TEST(densifyY(l, 5) == l);

    // y direction
    d = densifyY(l, 2);
    TEST(d.size(), ==, 5);
    TEST(d[2] == Point<int32_t>(11, 2));
    TEST(d[3] == Point<int32_t>(11, 3));
    TEST(d[4] == Point<int32_t>(12, 5));

    Line<int32_t> r{{0, 0}, {2, 0}, {2, 2}, {-10, 2}};
    TEST(densifyY(r, 1).size(), ==, 5);
    TEST(densifyX(r, 4).size(), ==, 6);
    TEST(densifyRingX(r, 4).size(), ==, 8);
    TEST(densifyRingY(r, 1).size(), ==, 6);

    Ring<int32_t> r2{{0, 0}, {-10, 2}, {2, 2}, {2, 0}};
    d = densifyRingX(r2, 4);
    TEST(d.size(), ==, 8);
    TEST(d.front() == Point<int32_t>(0, 0));
    TEST(d.back() == Point<int32_t>(2, 0));
    for (size_t i = 0; i < d.size(); i++) {
      TEST(std::abs(d[(i + 1) % d.size()].getX() - d[i].getX()), <=, 4);
    }

    Ring<int32_t> r3{{0, 0}, {10, 0}, {10, 1}, {0, 0}};
    d = densifyRingX(r3, 5);
    TEST(d.size(), ==, 6);
    TEST(d.back() == Point<int32_t>(0, 0));

    // floating point coordinates are not rounded
    Line<double> ld{{0, 0}, {1, 1}};
    auto dd = densifyX(ld, 0.4);
    TEST(dd.size(), ==, 4);
    TEST(dd[1].getX(), ==, approx(1.0 / 3));
    TEST(dd[1].getY(), ==, approx(1.0 / 3));

    // polygon, outer and inner rings
    Polygon<int32_t> poly(Line<int32_t>{{0, 0}, {100, 0}, {100, 10}, {0, 10}},
                          {Line<int32_t>{{10, 2}, {10, 8}, {90, 8}, {90, 2}}});
    auto dp = densifyX(poly, 10);
    TEST(dp.getOuter().size(), ==, 4 + 2 * 9);
    TEST(dp.getInners().size(), ==, 1);
    TEST(dp.getInners()[0].size(), ==, 4 + 2 * 7);
    TEST(signedRingArea(dp.getOuter()), ==,
         approx(signedRingArea(poly.getOuter())));
    TEST(signedRingArea(dp.getInners()[0]), ==,
         approx(signedRingArea(poly.getInners()[0])));

    auto dpy = densifyY(poly, 2);
    TEST(dpy.getOuter().size(), ==, 4 + 2 * 4);
    TEST(dpy.getInners()[0].size(), ==, 4 + 2 * 2);
    TEST(signedRingArea(dpy.getOuter()), ==,
         approx(signedRingArea(poly.getOuter())));
    TEST(signedRingArea(dpy.getInners()[0]), ==,
         approx(signedRingArea(poly.getInners()[0])));
  }
}
