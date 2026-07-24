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
void GeoTest::testGrid() {

  // ___________________________________________________________________________
  {
    Grid<int, Line, double> g(
        .5, .5, Box<double>(Point<double>(0, 0), Point<double>(3, 3)));

    Line<double> l;
    l.push_back(Point<double>(0, 0));
    l.push_back(Point<double>(1.5, 2));

    Line<double> l2;
    l2.push_back(Point<double>(2.5, 1));
    l2.push_back(Point<double>(2.5, 2));

    g.add(l, 1);
    g.add(l2, 2);

    std::set<int> ret;

    Box<double> req(Point<double>(.5, 1), Point<double>(1, 1.5));
    g.get(req, &ret);
    TEST(ret.size(), ==, (size_t)1);

    ret.clear();
    g.getNeighbors(1, 0, &ret);
    TEST(ret.size(), ==, (size_t)1);

    ret.clear();
    g.getNeighbors(1, 0.55, &ret);
    TEST(ret.size(), ==, (size_t)2);

    g.remove(1);
    ret.clear();
    g.getNeighbors(1, 0, &ret);
    TEST(ret.size(), ==, (size_t)0);

    g.remove(2);
    ret.clear();
    g.getNeighbors(1, 10, &ret);
    TEST(ret.size(), ==, (size_t)0);

    // TODO: more test cases
  }
}
