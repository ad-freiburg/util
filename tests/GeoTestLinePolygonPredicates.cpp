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
void GeoTest::testLinePolygonPredicates() {
  {
    auto a =
        polygonFromWKT<int>("POLYGON((0 0, 1 0, 1 3, 3 3, 3 5, 0 5, 0 0))");
    auto b = lineFromWKT<int>("LINESTRING(1 3, 2 3, 3 4, 1 4, 1 3)");
    XSortedPolygon<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(signedRingArea(a.getOuter()) < 0);

    TEST(std::get<0>(geo::intersectsContainsCovers(bx, ax)));

    TEST(std::get<1>(geo::intersectsContainsCovers(bx, ax)));
    TEST(std::get<2>(geo::intersectsContainsCovers(bx, ax)));

    TEST(!std::get<3>(geo::intersectsContainsCovers(bx, ax)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "1F2101FF2");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(de9im.contains());
    TEST(de9im.transpose().within());
    TEST(de9im.covers());
    TEST(de9im.transpose().covered());
    TEST(!de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());
  }

  {
    auto a =
        polygonFromWKT<int>("POLYGON((0 0, 1 0, 1 3, 3 3, 3 5, 0 5, 0 0))");
    auto b = lineFromWKT<int>("LINESTRING(0 0, -1 0)");
    XSortedPolygon<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(bx, ax)));

    TEST(!std::get<1>(geo::intersectsContainsCovers(bx, ax)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(bx, ax)));

    TEST(std::get<3>(geo::intersectsContainsCovers(bx, ax)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(bx, ax)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF2F01102");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(de9im.transpose().intersects());
    TEST(!de9im.contains());
    TEST(!de9im.transpose().within());
    TEST(!de9im.covers());
    TEST(!de9im.transpose().covered());
    TEST(de9im.touches());
    TEST(de9im.transpose().touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.transpose().disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 7 0, 7 7, 0 7, 0 0)");
    auto b = polygonFromWKT<int>("POLYGON((0 0, 7 0, 7 7, 0 7, 0 0))");
    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(ax, bx)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "F1FF0F2F2");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(de9im.touches());
    TEST(!de9im.disjoint());
    TEST(de9im.covered());
    TEST(!de9im.within());

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF210FFF2");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(de9im.covers());
    TEST(de9im.touches());
    TEST(!de9im.disjoint());
    TEST(!de9im.covered());
    TEST(!de9im.within());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 0, 7 0, 7 7, 0 7)");
    auto b = polygonFromWKT<int>("POLYGON((0 0, 7 0, 7 7, 0 7, 0 0))");
    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    // auto de9im = geo::DE9IM(ax, bx,
    // util::geo::getBoundingBox(b)); TEST(de9im, ==, "F1FF0F212");

    // TEST(!de9im.equals());
    // TEST(de9im.intersects());
    // TEST(!de9im.contains());
    // TEST(!de9im.covers());
    // TEST(de9im.touches());
    // TEST(!de9im.disjoint());
    // TEST(de9im.covered());
    // TEST(!de9im.within());
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(1 1, 6 1, 6 6, 1 6, 1 1)");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (3 3, 3 4, 4 4, 4 3, 3 3))");
    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(ax, bx)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "1FF0FF212");

    TEST(!de9im.equals());
    TEST(de9im.intersects());
    TEST(!de9im.contains());
    TEST(!de9im.covers());
    TEST(!de9im.touches());
    TEST(!de9im.disjoint());
    TEST(de9im.covered());
    TEST(de9im.within());

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "102FF1FF2");

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
    auto a = lineFromWKT<int>("LINESTRING(3 3, 3 4, 4 4, 4 3, 3 3)");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (3 3, 3 4, 4 4, 4 3, 3 3))");
    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(ax, bx)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "F1FF0F212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF2101FF2");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(3 4, 4 4)");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (3 3, 3 5, 5 5, 5 3, 3 3))");
    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(ax, bx)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "FF1F00212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF2F01102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(3 4, 3 5)");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (3 2, 3 6, 5 5, 5 3, 3 2))");
    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(ax, bx)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "F1FF0F212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF2101FF2");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(3 4, 3 5, 4 5)");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (3 2, 3 6, 5 5, 5 3, 3 2))");
    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(ax, bx)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "F1FF00212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "FF2101F02");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(3 4, 3 5, 2 5)");
    auto b = polygonFromWKT<int>(
        "POLYGON((0 0, 7 0, 7 7, 0 7, 0 0), (3 2, 3 6, 5 5, 5 3, 3 2))");
    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<4>(geo::intersectsContainsCovers(ax, bx)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "11F00F212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "102101FF2");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 1, 3 1)");
    auto b = polygonFromWKT<int>("POLYGON((1 0, 1 2, 2 2, 2 0, 1 0))");
    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<4>(geo::intersectsContainsCovers(ax, bx)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "101FF0212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "1F20F1102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 1, 2 1)");
    auto b = polygonFromWKT<int>("POLYGON((1 0, 1 3, 3 3, 3 0, 1 0))");
    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<4>(geo::intersectsContainsCovers(ax, bx)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "1010F0212");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "1020F1102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 2, 10 2)");
    auto b = polygonFromWKT<int>("POLYGON((1 0, 1 3, 3 3, 3 0, 1 0))");
    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    auto de9im = DE9IM(ax, bx);
    TEST(de9im, ==, "101FF0212");

    de9im = DE9IM(bx, ax);
    TEST(de9im, ==, "1F20F1102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(1 0, 0 0, 0 2, 10 2)");
    auto b = polygonFromWKT<int>("POLYGON((1 0, 1 3, 3 3, 3 0, 1 0))");
    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<4>(geo::intersectsContainsCovers(ax, bx)));

    auto de9im = DE9IM(ax, bx);
    TEST(de9im, ==, "101F00212");

    de9im = DE9IM(bx, ax);
    TEST(de9im, ==, "1F2001102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 1, 2 1)");
    auto b =
        polygonFromWKT<int>("POLYGON((-1 -1, -1 10, 10 10, 10 -1, -1 -1))");
    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    auto de9im = DE9IM(ax, bx);
    TEST(de9im, ==, "1FF0FF212");

    de9im = DE9IM(bx, ax);
    TEST(de9im, ==, "102FF1FF2");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 1, 2 1)");
    auto b = polygonFromWKT<int>(
        "POLYGON((-1 -1, -1 10, 10 10, 10 -1, -1 -1), (1 0, 1 3, 3 3, 3 0, 1 "
        "0))");
    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    TEST(std::get<0>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<1>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<2>(geo::intersectsContainsCovers(ax, bx)));
    TEST(!std::get<3>(geo::intersectsContainsCovers(ax, bx)));
    TEST(std::get<4>(geo::intersectsContainsCovers(ax, bx)));

    auto de9im = DE9IM(ax, bx);
    TEST(de9im, ==, "1010F0212");

    de9im = DE9IM(bx, ax);
    TEST(de9im, ==, "1020F1102");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 1, 9 1)");
    auto b = polygonFromWKT<int>(
        "POLYGON((-1 -1, -1 10, 10 10, 10 -1, -1 -1), (1 0, 1 3, 3 3, 3 0, 1 "
        "0), (4 0, 4 3,7 3, 7 0, 4 0))");
    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    auto de9im = DE9IM(ax, bx);
    TEST(de9im, ==, "1010FF212");

    de9im = DE9IM(bx, ax);
    TEST(de9im, ==, "1020F11F2");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(3 1, 4 1)");
    auto b = polygonFromWKT<int>(
        "POLYGON((-1 -1, -1 10, 10 10, 10 -1, -1 -1), (1 0, 1 3, 3 3, 3 0, 1 "
        "0), (4 0, 4 3,7 3, 7 0, 4 0))");
    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    auto de9im = DE9IM(ax, bx);
    TEST(de9im, ==, "1FFF0F212");

    de9im = DE9IM(bx, ax);
    TEST(de9im, ==, "1F2F01FF2");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(3 1, 4 1)");
    auto b = polygonFromWKT<int>(
        "POLYGON((-1 -1, -1 10, 10 10, 10 -1, -1 -1), (1 0, 1 3, 3 3, 3 0, 1 "
        "0))");
    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    auto de9im = DE9IM(ax, bx);
    TEST(de9im, ==, "1FF00F212");

    de9im = DE9IM(bx, ax);
    TEST(de9im, ==, "102F01FF2");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(0 1, 1 1)");
    auto b = polygonFromWKT<int>(
        "POLYGON((-1 -1, -1 10, 10 10, 10 -1, -1 -1), (1 0, 1 3, 3 3, 3 0, 1 "
        "0))");
    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    auto de9im = DE9IM(ax, bx);
    TEST(de9im, ==, "1FF00F212");

    de9im = DE9IM(bx, ax);
    TEST(de9im, ==, "102F01FF2");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(3 1, 1 1)");
    auto b = polygonFromWKT<int>(
        "POLYGON((-1 -1, -1 10, 10 10, 10 -1, -1 -1), (1 0, 1 3, 3 3, 3 0, 1 "
        "0))");
    XSortedLine<int> ax(a);
    XSortedPolygon<int> bx(b);

    auto de9im = DE9IM(ax, bx);
    TEST(de9im, ==, "FF1F0F212");

    de9im = DE9IM(bx, ax);
    TEST(de9im, ==, "FF2F011F2");
  }

  {
    XSortedPolygon<double> poly(
        polygonFromWKT<double>("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0))"));
    XSortedPolygon<double> polyWithInner(polygonFromWKT<double>(
        "POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (2 2, 8 2, 8 8, 2 8, 2 2))"));
    XSortedLine<double> farLine(lineFromWKT<double>("LINESTRING(20 0, 30 0)"));
    XSortedLine<double> lineInHole(lineFromWKT<double>("LINESTRING(4 5, 6 5)"));

    TEST(geo::DE9IM(farLine, poly), ==, "FF1FF0212");
    TEST(geo::DE9IM(poly, farLine), ==, "FF2FF1102");

    // a line completely inside a hole of the polygon is disjoint, too
    TEST(geo::DE9IM(lineInHole, polyWithInner), ==, "FF1FF0212");
    TEST(geo::DE9IM(polyWithInner, lineInHole), ==, "FF2FF1102");
  }

  {
    auto emptyLine = lineFromWKT<double>("LINESTRING EMPTY");
    auto emptyPoly = polygonFromWKT<double>("POLYGON EMPTY");
    Polygon<double> emptyOuter(
        Ring<double>(),
        {lineFromWKT<double>("LINESTRING(4 4, 5 4, 5 5, 4 5, 4 4)")});
    XSortedLine<double> line(
        lineFromWKT<double>("LINESTRING(4.2 4.5, 4.8 4.5)"));
    XSortedPolygon<double> poly(
        polygonFromWKT<double>("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0))"));
    XSortedLine<double> emptyLineX(emptyLine);
    XSortedPolygon<double> emptyPolyX(emptyPoly);
    XSortedPolygon<double> emptyOuterX(emptyOuter);
    const auto allFalse = std::make_tuple(false, false, false, false, false);

    TEST(geo::intersectsContainsCovers(line, emptyPolyX) == allFalse);
    TEST(geo::intersectsContainsCovers(line, emptyOuterX) == allFalse);
    TEST(geo::intersectsContainsCovers(emptyLineX, poly) == allFalse);
    TEST(geo::intersectsContainsCovers(emptyLineX, emptyPolyX) == allFalse);
    TEST(geo::intersectsContainsCovers(emptyLineX, emptyOuterX) == allFalse);
  }
}
