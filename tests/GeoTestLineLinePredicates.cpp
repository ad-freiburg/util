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
void GeoTest::testLineLinePredicates() {
  {
    auto a = lineFromWKT<int>("LINESTRING(1 1, 2 2)");
    auto b = lineFromWKT<int>("LINESTRING(2 2, 1 1)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<1>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "1FFF0FFF2");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "1FFF0FFF2");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 2 2)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 1 2)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));

    TEST(std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "F01FF0102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF10F0102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 2 2)");
    auto b = lineFromWKT<int>("LINESTRING(2 2, 3 3)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));

    TEST(std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF1F00102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF1F00102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 2 2)");
    auto b = lineFromWKT<int>("LINESTRING(2 2, 3 3, 1 3, 1 1)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));

    TEST(std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    DE9IMatrix m("F01F001F2");
    TEST(de9im, ==, m);

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, m.transpose());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 2 2)");
    auto b = lineFromWKT<int>("LINESTRING(2 2, 3 3, 1 3, 1 1, 2 0)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));

    TEST(!std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    DE9IMatrix m("0F1F00102");
    TEST(de9im, ==, m);

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, m.transpose());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 2 2)");
    auto b = lineFromWKT<int>("LINESTRING(1 2, 1 1)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));

    TEST(std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    DE9IMatrix m("F01FF0102");
    TEST(de9im, ==, m);

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, m.transpose());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 2 2)");
    auto b = lineFromWKT<int>("LINESTRING(0 0, 1 2)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));

    TEST(std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    DE9IMatrix m("FF1F00102");
    TEST(de9im, ==, m);

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, m.transpose());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 2 2)");
    auto b = lineFromWKT<int>("LINESTRING(2 2, 1 2)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));

    TEST(std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    DE9IMatrix m("FF1F00102");
    TEST(de9im, ==, m);

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, m.transpose());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 2 2)");
    auto b = lineFromWKT<int>("LINESTRING(0 0, 1 1, 1 2)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));

    TEST(!std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    DE9IMatrix m("1F1F00102");
    TEST(de9im, ==, m);

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, m.transpose());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 2 2)");
    auto b = lineFromWKT<int>("LINESTRING(0 0, 1 1)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));

    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<1>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    DE9IMatrix m("101F00FF2");
    TEST(de9im, ==, m);

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, m.transpose());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 3 3)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 2 2)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    DE9IMatrix m("101FF0FF2");
    TEST(de9im, ==, m);

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, m.transpose());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 3 3, 0 3)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 4 4)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(std::get<3>(geo::intersectsCovers(bx, ax)));
    TEST(std::get<3>(geo::intersectsCovers(ax, bx)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    DE9IMatrix m("101FF0102");
    TEST(de9im, ==, m);

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, m.transpose());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 3 3)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 4 4)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<1>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    DE9IMatrix m("1010F0102");
    TEST(de9im, ==, m);

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, m.transpose());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 3 3)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 3 3)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(std::get<1>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    DE9IMatrix m("101F00FF2");
    TEST(de9im, ==, m);

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, m.transpose());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 3 3, 5 5)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 4 4)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(std::get<1>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    DE9IMatrix m("101FF0FF2");
    TEST(de9im, ==, m);

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, m.transpose());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 3 3, 5 5, 5 6)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 4 4, 5 5, 5 6)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(std::get<1>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    DE9IMatrix m("101F00FF2");
    TEST(de9im, ==, m);

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, m.transpose());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 3 3, 5 5, 5 7)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 4 4, 5 5, 5 6)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(std::get<1>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    DE9IMatrix m("101FF0FF2");
    TEST(de9im, ==, m);

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, m.transpose());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 3 3, 5 5, 5 7)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 4 4, 5 5, 6 6)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<1>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "101FF0102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "1F10F0102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 3 3, 5 5, 6 6)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 4 4, 5 5, 5 7)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<1>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "101FF0102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "1F10F0102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(1 1, 4 4, 5 5, 6 6)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 4 4, 5 5, 6 6)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(std::get<1>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "1FFF0FFF2");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "1FFF0FFF2");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 4 4, 5 5, 6 6)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 4 4, 5 5, 6 6)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(std::get<1>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "101F00FF2");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "1FF00F102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(2 0, 1 1)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 4 4, 5 5, 6 6)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<1>(geo::intersectsCovers(bx, ax)));
    TEST(std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF1F00102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF1F00102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(7 300, 6 6)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 4 4, 5 5, 6 6)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<1>(geo::intersectsCovers(bx, ax)));

    TEST(std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF1F00102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF1F00102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(10 0, 5 5)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 4 4, 5 5, 6 6)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<1>(geo::intersectsCovers(bx, ax)));

    TEST(std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF10F0102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "F01FF0102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(5 5, 10 0)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 4 4, 5 5, 6 6)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<1>(geo::intersectsCovers(bx, ax)));

    TEST(std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF10F0102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "F01FF0102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(6 6, 10 0)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 4 4, 5 5, 7 7)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<1>(geo::intersectsCovers(bx, ax)));

    TEST(std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF10F0102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "F01FF0102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(10 0, 6 6)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 4 4, 5 5, 7 7)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<1>(geo::intersectsCovers(bx, ax)));

    TEST(std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF10F0102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "F01FF0102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 2 2)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 2 1)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<1>(geo::intersectsCovers(bx, ax)));

    TEST(std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "F01FF0102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF10F0102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 2 2)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 0 1)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<1>(geo::intersectsCovers(bx, ax)));

    TEST(std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "F01FF0102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF10F0102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 1 1, 2 2)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 0 1)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<1>(geo::intersectsCovers(bx, ax)));

    TEST(std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "F01FF0102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF10F0102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 1 1, 2 2)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 2 1)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(!std::get<1>(geo::intersectsCovers(bx, ax)));

    TEST(std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "F01FF0102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF10F0102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 4 4)");
    auto b = lineFromWKT<int>("LINESTRING(1 1, 2 2)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<0>(geo::intersectsCovers(bx, ax)));
    TEST(std::get<1>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsCovers(bx, ax)));

    TEST(!std::get<4>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "101FF0FF2");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "1FF0FF102");
  }

  {
    auto line = lineFromWKT<int>("LINESTRING(0 1, 2 1)");
    auto point = pointFromWKT<int>("POINT(0 1)");
    auto point2 = pointFromWKT<int>("POINT(1 1)");
    auto point3 = pointFromWKT<int>("POINT(2 1)");
    auto point4 = pointFromWKT<int>("POINT(3 1)");

    XSortedLine<int> linex(line);

    auto de9im = util::geo::DE9IM(point, linex);
    TEST(de9im, ==, "F0FFFF102");

    de9im = util::geo::DE9IM(point3, linex);
    TEST(de9im, ==, "F0FFFF102");

    de9im = util::geo::DE9IM(point2, linex);
    TEST(de9im, ==, "0FFFFF102");

    de9im = util::geo::DE9IM(point4, linex);
    TEST(de9im, ==, "FF0FFF102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(1 0, 3 3)");
    auto b = lineFromWKT<int>("LINESTRING(2 0, 2 2)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(geo::intersects(util::geo::getBoundingBox(a),
                         util::geo::getBoundingBox(b)));
    TEST(geo::intersects(a, b));
    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<4>(geo::intersectsCovers(ax, bx)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "0F1FF0102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "0F1FF0102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(1 0, 2 1)");
    auto b = lineFromWKT<int>("LINESTRING(2 0, 2 2)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF10F0102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "F01FF0102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(1 0, 2 2)");
    auto b = lineFromWKT<int>("LINESTRING(2 0, 2 2)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF1F00102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF1F00102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(2 2, 1 0)");
    auto b = lineFromWKT<int>("LINESTRING(2 0, 2 2)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF1F00102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF1F00102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(2 2, 1 0)");
    auto b = lineFromWKT<int>("LINESTRING(2 2, 2 0)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF1F00102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF1F00102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(2 1, 1 0, 2 2)");
    auto b = lineFromWKT<int>("LINESTRING(2 2, 2 0)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF100F102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "F01F001F2");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(2 0, 1 0, 2 2)");
    auto b = lineFromWKT<int>("LINESTRING(2 2, 2 0)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF1F0F1F2");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF1F0F1F2");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 1 1)");
    auto b = lineFromWKT<int>("LINESTRING(0 0, 1 1)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "1FFF0FFF2");
    TEST(de9im.equals());

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "1FFF0FFF2");
    TEST(de9im.equals());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 1 1, 2 2)");
    auto b = lineFromWKT<int>("LINESTRING(0 0, 2 2)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "1FFF0FFF2");
    TEST(de9im.equals());

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "1FFF0FFF2");
    TEST(de9im.equals());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(2 2, 1 1, 0 0)");
    auto b = lineFromWKT<int>("LINESTRING(0 0, 2 2)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "1FFF0FFF2");
    TEST(de9im.equals());

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "1FFF0FFF2");
    TEST(de9im.equals());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 0 1, 1 1, 0 0)");
    auto b = lineFromWKT<int>("LINESTRING(0 1, 1 1, 0 0, 0 1)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "10F0FFFF2");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "10F0FFFF2");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 1 1, 2 1, 2 2, 1 1)");
    auto b = lineFromWKT<int>("LINESTRING(1 0, 1 1)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsCovers(ax, bx)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "F01F00102");

    de9im = geo::DE9IM(bx, ax);
    // TEST(de9im, ==, "FF0000101");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 1 1, 2 1, 2 2, 1 1, 1 2)");
    auto b = lineFromWKT<int>("LINESTRING(1 0, 1 1)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsCovers(ax, bx)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "F01FF0102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF10F0102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(8335603 60914025, 8736353 60914025)");
    auto b = lineFromWKT<int>("LINESTRING(8440562 60918151, 8441122 60913050)");

    auto c = lineFromWKT<int>("LINESTRING(8335603 60593425, 8736353 60593425)");

    TEST(LineSegment<int>(b[0], b[1]) > LineSegment<int>(a[0], a[1]));
    TEST(LineSegment<int>(b[0], b[1]) > LineSegment<int>(c[0], c[1]));

    TEST(!(LineSegment<int>(a[0], a[1]) > LineSegment<int>(b[0], b[1])));
    TEST(!(LineSegment<int>(c[0], c[1]) > LineSegment<int>(b[0], b[1])));

    TEST(LineSegment<int>(c[0], c[1]) < LineSegment<int>(a[0], a[1]));

    TEST(LineSegment<int>(a[0], a[1]) < LineSegment<int>(b[0], b[1]));
    TEST(LineSegment<int>(b[0], b[1]) > LineSegment<int>(a[0], a[1]));

    TEST(
        intersects(LineSegment<int>(a[0], a[1]), LineSegment<int>(b[0], b[1])));
    TEST(
        intersects(LineSegment<int>(a[1], a[0]), LineSegment<int>(b[0], b[1])));
    TEST(
        intersects(LineSegment<int>(a[1], a[0]), LineSegment<int>(b[1], b[0])));
    TEST(
        intersects(LineSegment<int>(a[0], a[1]), LineSegment<int>(b[1], b[0])));

    TEST(
        intersects(LineSegment<int>(b[0], b[1]), LineSegment<int>(a[0], a[1])));
    TEST(
        intersects(LineSegment<int>(b[1], b[0]), LineSegment<int>(a[0], a[1])));
    TEST(
        intersects(LineSegment<int>(b[1], b[0]), LineSegment<int>(a[1], a[0])));
    TEST(
        intersects(LineSegment<int>(b[0], b[1]), LineSegment<int>(a[1], a[0])));
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(1 1, 2 1, 3 1)");
    auto b = lineFromWKT<int>("LINESTRING(2 0, 2 1, 2 2)");
    auto c = lineFromWKT<int>("LINESTRING(1 0, 2 1, 3 2)");

    TEST(intersects(a, b));
    TEST(intersects(a, c));
  }

  {
    TEST(intersects(LineSegment<double>{{2, 1}, {3, 1}},
                    LineSegment<double>{{2, 0}, {2, 1}}));
    TEST(intersects(LineSegment<double>{{2, 0}, {2, 1}},
                    LineSegment<double>{{2, 1}, {3, 1}}));

    auto ls1 = LineSegment<int>{{2, 0}, {2, 1}};
    auto ls2 = LineSegment<int>{{2, 1}, {2, 2}};
    TEST(intersects(ls1, ls2));

    {
      auto ls1 = LineSegment<int>{{2, 0}, {2, 2}};
      auto ls2 = LineSegment<int>{{2, 1}, {2, 2}};
      TEST(!intersects(ls1, ls2));
      TEST(!intersects(ls2, ls1));
    }

    {
      auto ls1 = LineSegment<int>{{2, 2}, {2, 0}};
      auto ls2 = LineSegment<int>{{2, 1}, {2, 2}};
      TEST(!intersects(ls1, ls2));
      TEST(!intersects(ls2, ls1));
    }

    {
      auto ls1 = LineSegment<int>{{2, 2}, {2, 0}};
      auto ls2 = LineSegment<int>{{2, 2}, {2, 1}};
      TEST(!intersects(ls1, ls2));
      TEST(!intersects(ls2, ls1));
    }

    {
      auto ls1 = LineSegment<int>{{2, 0}, {2, 2}};
      auto ls2 = LineSegment<int>{{2, 2}, {2, 1}};
      TEST(!intersects(ls1, ls2));
      TEST(!intersects(ls2, ls1));
    }

    {
      auto ls1 = LineSegment<int>{{2, 0}, {2, 2}};
      auto ls2 = LineSegment<int>{{2, 2}, {2, 1}};
      TEST(!intersects(ls1, ls2));
      TEST(!intersects(ls2, ls1));
    }
  }

  {
    auto a = lineFromWKT<double>(
        "LINESTRING(5.816117 51.1096889,5.8162167 51.1091479)");
    auto b = lineFromWKT<double>(
        "LINESTRING(5.8099984 51.1096468,5.8288865 51.1070398)");

    TEST(intersects(getBoundingBox(a), getBoundingBox(b)));

    auto ls1 = LineSegment<double>{a[0], a[1]};
    auto ls2 = LineSegment<double>{b[0], b[1]};

    TEST(!contains(ls1.first, ls2));
    TEST(!contains(ls1.second, ls2));
    TEST(!contains(ls2.first, ls1));
    TEST(!contains(ls2.second, ls1));

    TEST(!(
        ((crossProd(ls1.first, ls2) < 0) ^ (crossProd(ls1.second, ls2) < 0)) &&
        ((crossProd(ls2.first, ls1) < 0) ^ (crossProd(ls2.second, ls1) < 0))));

    TEST(!geo::intersects(ls1, ls2));

    TEST(!geo::intersects(a, b));
    TEST(!geo::intersects(b, a));
  }
}
