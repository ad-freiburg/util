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
void GeoTest::testPolygonPolygonPredicates() {
  {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 2 0, 2 2, 0 2, 0 0))");
    auto aa =
        polygonFromWKT<int>("POLYGON((0 0, 2 0, 2 1, 1 1, 1 3, 0 3, 0 0))");
    auto b = pointFromWKT<int>("POINT(0 1)");
    auto c = pointFromWKT<int>("POINT(1 1)");
    auto d = pointFromWKT<int>("POINT(0 0)");
    auto e = pointFromWKT<int>("POINT(2 2)");
    auto f = pointFromWKT<int>("POINT(2 0)");
    auto g = pointFromWKT<int>("POINT(2 3)");
    auto h = pointFromWKT<int>("POINT(2 -1)");
    auto i = pointFromWKT<int>("POINT(0 3)");
    auto j = pointFromWKT<int>("POINT(2 3)");

    auto bb = pointFromWKT<int>("POINT(1 4)");
    auto cc = pointFromWKT<int>("POINT(1 3)");

    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> aax(aa);

    TEST(geo::contains(c, a));
    TEST(geo::contains(b, a));

    TEST(!geo::containsCovers(b, ax).first);
    TEST(geo::containsCovers(b, ax).second);

    TEST(!geo::containsCovers(d, ax).first);
    TEST(geo::containsCovers(d, ax).second);

    TEST(!geo::containsCovers(e, ax).first);
    TEST(geo::containsCovers(e, ax).second);

    TEST(!geo::containsCovers(f, ax).first);
    TEST(geo::containsCovers(f, ax).second);

    TEST(geo::containsCovers(c, ax).first);
    TEST(geo::containsCovers(c, ax).second);

    TEST(!geo::containsCovers(g, ax).first);
    TEST(!geo::containsCovers(g, ax).second);

    TEST(!geo::containsCovers(h, ax).first);
    TEST(!geo::containsCovers(h, ax).second);

    TEST(!geo::containsCovers(i, ax).first);
    TEST(!geo::containsCovers(i, ax).second);

    TEST(!geo::containsCovers(j, ax).first);
    TEST(!geo::containsCovers(j, ax).second);

    TEST(!geo::containsCovers(bb, aax).first);
    TEST(!geo::containsCovers(bb, aax).second);

    TEST(!geo::containsCovers(cc, aax).first);
    TEST(geo::containsCovers(cc, aax).second);
  }

  {
    auto a =
        polygonFromWKT<int>("POLYGON((0 0, 1 0, 1 3, 3 3, 3 5, 0 5, 0 0))");
    auto b = polygonFromWKT<int>("POLYGON((1 3, 2 3, 3 4, 1 4, 1 3))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(bx, ax)));

    TEST(std::get<0>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(!std::get<1>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(std::get<2>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(std::get<1>(geo::intersectsContainsCovers(bx, ax)));
    TEST(std::get<2>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "212F11FF2");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(de9im.contains());
    TEST(de9im.covers());
    TEST(!de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "2FF11F212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(!de9im.touches());
    TEST(!de9im.disjoint());
    TEST(de9im.covered());
    TEST(de9im.within());
  }

  {
    auto a =
        polygonFromWKT<int>("POLYGON((0 0, 1 0, 1 3, 3 3, 3 5, 0 5, 0 0))");
    auto b = polygonFromWKT<int>("POLYGON((1 2, 2 2, 2 3, 1 3, 1 2))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(bx, ax)));

    TEST(std::get<0>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(std::get<1>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(!std::get<2>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(!std::get<1>(geo::intersectsContainsCovers(bx, ax)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(bx, ax)));
    TEST(std::get<3>(geo::intersectsContainsCovers(bx, ax)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF2F11212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());

    de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF2F11212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());
  }

  {
    auto a = polygonFromWKT<int>(
        "POLYGON((842 645,849 614,857 575,973 603,940 746,854 726,825 719,842 "
        "645))");
    auto b = polygonFromWKT<int>(
        "POLYGON((843 646,851 610,857 575,973 603,940 746,852 725,825 719,843 "
        "646))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    auto de9im = geo::DE9IM(bx, ax);
    auto de9imT = geo::DE9IM(ax, bx);
    TEST(de9im, ==, de9imT.transpose());
  }

  {
    auto a =
        polygonFromWKT<int>("POLYGON((0 0, 1 0, 1 3, 3 3, 3 5, 0 5, 0 0))");
    auto b = polygonFromWKT<int>("POLYGON((1 2, 2 1, 2 2, 1 2))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(bx, ax)));

    TEST(std::get<0>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(std::get<1>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(!std::get<1>(geo::intersectsContainsCovers(bx, ax)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF2F01212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());

    de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF2F01212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());
  }

  {
    auto a =
        polygonFromWKT<int>("POLYGON((0 0, 1 0, 1 3, 3 3, 3 5, 0 5, 0 0))");
    auto b = polygonFromWKT<int>("POLYGON((1 3, 2 1, 2 2, 1 3))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(bx, ax)));

    TEST(std::get<0>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(std::get<1>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(!std::get<1>(geo::intersectsContainsCovers(bx, ax)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF2F01212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());

    de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF2F01212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());
  }

  {
    auto a =
        polygonFromWKT<int>("POLYGON((0 0, 1 0, 1 3, 3 3, 3 5, 0 5, 0 0))");
    auto b = polygonFromWKT<int>("POLYGON((1 3, 2 4, 1 4, 1 3))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(bx, ax)));

    TEST(std::get<0>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(!std::get<1>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(std::get<1>(geo::intersectsContainsCovers(bx, ax)));
    TEST(std::get<2>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "212F01FF2");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(de9im.contains());
    TEST(de9im.covers());
    TEST(!de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "2FF10F212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(!de9im.touches());
    TEST(!de9im.disjoint());
    TEST(de9im.covered());
    TEST(de9im.within());
  }

  {
    auto a =
        polygonFromWKT<int>("POLYGON((1 3, 3 3, 3 5, 0 5, 0 0, 1 0, 1 3))");
    auto b = polygonFromWKT<int>("POLYGON((1 3, 2 1, 2 2, 1 3))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(bx, ax)));

    TEST(std::get<0>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(std::get<1>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(!std::get<1>(geo::intersectsContainsCovers(bx, ax)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF2F01212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF2F01212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());
  }

  {
    auto a =
        polygonFromWKT<int>("POLYGON((0 0, 1 0, 1 3, 3 3, 3 5, 0 5, 0 0))");
    auto b =
        polygonFromWKT<int>("POLYGON((0 0, 1 0, 1 3, 3 3, 3 5, 0 5, 0 0))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(bx, ax)));

    TEST(std::get<0>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(!std::get<1>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(std::get<1>(geo::intersectsContainsCovers(bx, ax)));
    TEST(std::get<2>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "2FFF1FFF2");

    TEST(de9im.equals());
    TEST(de9im.intersects());
    TEST(de9im.contains());
    TEST(de9im.covers());
    TEST(!de9im.touches());
    TEST(!de9im.disjoint());
    TEST(de9im.covered());
    TEST(de9im.within());

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "2FFF1FFF2");

    TEST(de9im.equals());
    TEST(de9im.intersects());
    TEST(de9im.contains());
    TEST(de9im.covers());
    TEST(!de9im.touches());
    TEST(!de9im.disjoint());
    TEST(de9im.covered());
    TEST(de9im.within());
  }

  {
    auto a =
        polygonFromWKT<int>("POLYGON((0 0, 1 0, 1 3, 3 3, 3 5, 0 4, 0 0))");
    auto b =
        polygonFromWKT<int>("POLYGON((0 0, 1 0, 1 3, 2 3, 3 5, 0 4, 0 0))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(bx, ax)));

    TEST(std::get<0>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(!std::get<1>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(std::get<1>(geo::intersectsContainsCovers(bx, ax)));
    TEST(std::get<2>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "212F11FF2");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(de9im.contains());
    TEST(de9im.covers());
    TEST(!de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "2FF11F212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(!de9im.touches());
    TEST(!de9im.disjoint());
    TEST(de9im.covered());
    TEST(de9im.within());
  }

  {
    auto a = polygonFromWKT<int>(
        "POLYGON((0 0, 1 0, 1 3, 2 3, 3 3, 3 4, 3 5, 1 5, 0 5, 0 0))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 1 0, 1 3, 3 3, 3 5, 0 5, 0 3, 0 0))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(bx, ax)));

    TEST(std::get<0>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(!std::get<1>(util::geo::intersectsPoly(
        bx.getOuter(), ax.getOuter(), bx.getOuter().getMaxSegLen(),
        ax.getOuter().getMaxSegLen(), 0, 0)));

    TEST(std::get<1>(geo::intersectsContainsCovers(bx, ax)));
    TEST(std::get<2>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "2FFF1FFF2");

    TEST(de9im.equals());
    TEST(de9im.intersects());
    TEST(de9im.contains());
    TEST(de9im.covers());
    TEST(!de9im.touches());
    TEST(!de9im.disjoint());
    TEST(de9im.covered());
    TEST(de9im.within());

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "2FFF1FFF2");

    TEST(de9im.equals());
    TEST(de9im.intersects());
    TEST(de9im.contains());
    TEST(de9im.covers());
    TEST(!de9im.touches());
    TEST(!de9im.disjoint());
    TEST(de9im.covered());
    TEST(de9im.within());
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((7 5, 8 5, 8 6, 7 6, 7 5))");
    auto b = polygonFromWKT<int>("POLYGON((0 0, 7 0, 7 7, 0 7, 0 0))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsContainsCovers(bx, ax)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF2F11212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF2F11212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((1 1, 6 1, 6 6, 1 6, 1 1))");
    auto b = polygonFromWKT<int>("POLYGON((0 0, 7 0, 7 7, 0 7, 0 0))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "2FF1FF212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(!de9im.touches());
    TEST(!de9im.disjoint());
    TEST(de9im.covered());
    TEST(de9im.within());

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "212FF1FF2");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(de9im.contains());
    TEST(de9im.covers());
    TEST(!de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((1 1, 6 1, 6 6, 1 6, 1 1))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (3 3, 3 4, 4 4, 4 3, 3 3))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(bx, ax)));
    TEST(std::get<4>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<4>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "2121FF212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(!de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "2121F12F2");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(!de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((1 1, 6 1, 6 6, 1 6, 1 1))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (1 1, 1 4, 4 4, 4 3, 3 3))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(bx, ax)));
    TEST(std::get<4>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<4>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "21211F212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(!de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "2121112F2");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(!de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((1 1, 6 1, 6 6, 1 6, 1 1))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (1 1, 6 1, 6 6, 1 6, 1 1))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsContainsCovers(bx, ax)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF2F1F212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF2F112F2");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((1 1, 4 1, 4 4, 1 4, 1 1))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (1 1, 6 1, 6 6, 1 6, 1 1))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);
    TEST(bx.getInnerAreas()[0], >, 0)

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsContainsCovers(bx, ax)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF2F11212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF2F11212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());
  }

  {
    auto a = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (1 1, 1 2, 2 2, 2 1, 1 1), (3 3, 3 "
        "4, 4 4, 4 3, 3 3))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (1 1, 1 2, 2 2, 2 1, 1 1), (3 3, 3 "
        "4, 4 4, 4 3, 3 3))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(bx, ax)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "2FFF1FFF2");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "2FFF1FFF2");
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((3 3, 3 4, 4 4, 4 3, 3 3))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (1 1, 1 2, 2 2, 2 1, 1 1), (3 3, 3 "
        "4, 4 4, 4 3, 3 3))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsContainsCovers(bx, ax)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF2F1F212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF2F112F2");
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((3 3, 3 4, 4 4, 4 3, 3 3))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (3 3, 3 4, 4 4, 4 3, 3 3))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsContainsCovers(bx, ax)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF2F1F212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF2F112F2");
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((3 3, 3 4, 4 3, 3 3))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (3 3, 3 4, 4 4, 4 3, 3 3))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsContainsCovers(bx, ax)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF2F11212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF2F11212");
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((1 3, 3 4, 4 3, 3 3))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (2 2, 2 5, 5 5, 5 2, 2 2)))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "212101212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "212101212");
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((-1 3, 3 4, 4 3, 3 3))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (2 2, 2 5, 5 5, 5 2, 2 2)))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "212101212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "212101212");
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((2 3, 3 4, 4 3, 3 3))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (2 2, 2 5, 5 5, 5 2, 2 2)))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF2F01212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF2F01212");
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((2 3, 2 4, 4 3, 3 3))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (2 2, 2 5, 5 5, 5 2, 2 2)))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF2F11212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF2F11212");
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((0 3, 0 4, 4 3, 3 3))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (2 2, 2 5, 5 5, 5 2, 2 2)))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "212111212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "212111212");
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((0 3, 2 4, 2 3))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (2 2, 2 5, 5 5, 5 2, 2 2)))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "2FF11F212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "212F11FF2");
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((0 3, 2 4, 2 3))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (2 2, 2 5, 5 5, 5 2, 2 2), (1 5, 2 "
        "1, 2 1, 1 1)))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "212111212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "212111212");
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((-1 3, 2 4, 2 3))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (2 2, 2 5, 5 5, 5 2, 2 2)))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "212111212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "212111212");
  }

  {
    auto a = polygonFromWKT<int>(
        "POLYGON((1 1, 6 1, 6 6, 1 6, 1 1), (2 2, 2 5, 5 5, 5 2, 2 2))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (3 3, 3 4, 4 4, 4 3, 3 3))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(bx, ax)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "2FF1FF212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "212FF1FF2");
  }

  {
    auto a = polygonFromWKT<int>(
        "POLYGON((1 1, 6 1, 6 6, 1 6, 1 1), (3 3, 2 5, 5 5, 5 2, 2 2))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (3 3, 3 4, 4 4, 4 3, 3 3))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(bx, ax)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "2FF10F212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "212F01FF2");
  }

  {
    auto a = polygonFromWKT<int>(
        "POLYGON((1 1, 6 1, 6 6, 1 6, 1 1), (2 2, 2 5, 5 5, 5 2, 2 2))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (2 2, 2 5, 5 5, 5 2, 2 2))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(bx, ax)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "2FF11F212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "212F11FF2");
  }

  {
    auto a = polygonFromWKT<int>(
        "POLYGON((1 1, 6 1, 6 6, 1 6, 1 1), (2 2, 3 3, 2 5, 5 5, 5 2, 2 2))");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (2 2, 2 5, 5 5, 5 2, 2 2))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(bx, ax)));
    TEST(std::get<4>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<4>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "21211F212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "2121112F2");
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 0 2, 2 2, 2 0, 0 0))");
    auto b = polygonFromWKT<int>("POLYGON((1 1, 1 3, 3 3, 3 1, 1 1))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(bx, ax)));
    TEST(std::get<4>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<4>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "212101212");

    de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "212101212");
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 0 1, 1 1, 1 0, 0 0))");
    auto b = polygonFromWKT<int>("POLYGON((2 2, 2 3, 3 3, 3 2, 2 2))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(!std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(bx, ax)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF2FF1212");

    de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF2FF1212");
  }

  {
    auto a = polygonFromWKT<int>("POLYGON((0 0, 0 2, 2 2, 2 0, 0 0))");
    auto b = polygonFromWKT<int>("POLYGON((2 2, 2 3, 3 3, 3 2, 2 2))");
    XSortedPolygon<int> ax(a);
    XSortedPolygon<int> bx(b);

    auto de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF2F01212");

    de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF2F01212");
  }
}
