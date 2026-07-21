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
void GeoTest::testWktSerialize() {

  {
    std::stringstream ss;
    util::geo::output::GeoJsonOutput out(ss);

    Polygon<int> poly{{{1, 1}, {3, 2}, {4, 3}, {6, 3}, {5, 1}, {1, 1}}};
    Polygon<int> poly3{{{1, 3}, {3, 4}, {4, 5}, {6, 5}, {5, 3}, {1, 3}}};

    out.print(poly, {});
    out.flush();
    std::string a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"FeatureCollection\",\"features\":[{\"type\":\"Feature\","
         "\"geometry\":{\"type\":\"Polygon\",\"coordinates\":[[[5,1],[6,3],[4,"
         "3],[3,2],[1,1],[5,1]]]},\"properties\":{}}]}");

    Polygon<int> poly2{{{1, 1}, {3, 2}, {4, 3}, {6, 3}, {5, 1}, {1, 1}},
                       {{{1, 1}, {1, 2}, {2, 2}, {1, 1}}}};

    ss.str("");
    out.print(poly2, {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"Polygon\","
         "\"coordinates\":[[[5,1],[6,3],[4,3],[3,2],[1,1],[5,1]],[[1,1],[1,2],["
         "2,2],[1,1]]]},\"properties\":{}}");

    ss.str("");
    out.print(util::geo::MultiPolygon<int>{poly3, poly2}, {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"MultiPolygon\","
         "\"coordinates\":[[[[5,3],[6,5],[4,5],[3,4],[1,3],[5,3]]],[[[5,1],[6,"
         "3],[4,3],[3,2],[1,1],[5,1]],[[1,1],[1,2],[2,2],[1,1]]]]},"
         "\"properties\":{}}");

    ss.str("");
    out.print(geo::lineFromWKT<int>("LINESTRING(0 0,1 1)"), {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"LineString\","
         "\"coordinates\":[[0,0],[1,1]]},\"properties\":{}}");

    ss.str("");
    out.print(
        util::geo::MultiLine<int>{geo::lineFromWKT<int>("LINESTRING(0 0,1 1)"),
                                  geo::lineFromWKT<int>("LINESTRING(2 2,3 3)")},
        {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"MultiLineString\","
         "\"coordinates\":[[[0,0],[1,1]],[[2,2],[3,3]]]},\"properties\":{}}");

    ss.str("");
    out.print(
        geo::multiLineFromWKT<int>("MULTILINESTRING((0 0, 1 1), (2 2, 3 3))"),
        {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"MultiLineString\","
         "\"coordinates\":[[[0,0],[1,1]],[[2,2],[3,3]]]},\"properties\":{}}");

    ss.str("");
    out.print(geo::multiPointFromWKT<int>("MULTIPOINT(0 0,1 1,2 2,3 3)"), {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"MultiPoint\","
         "\"coordinates\":[[0,0],[1,1],[2,2],[3,3]]},\"properties\":{}}");

    ss.str("");
    out.print(
        geo::multiPointFromWKT<int>("MULTIPOINT((0 0),(1 1),(2 2),(3 3))"), {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"MultiPoint\","
         "\"coordinates\":[[0,0],[1,1],[2,2],[3,3]]},\"properties\":{}}");
  }

  {
    std::stringstream ss;
    util::geo::output::GeoJsonOutput out(ss);

    Polygon<int> poly{{{1, 1}, {3, 2}, {4, 3}, {6, 3}, {5, 1}, {1, 1}}};
    Polygon<int> poly3{{{1, 3}, {3, 4}, {4, 5}, {6, 5}, {5, 3}, {1, 3}}};

    out.print(poly, {});
    out.flush();
    std::string a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"FeatureCollection\",\"features\":[{\"type\":\"Feature\","
         "\"geometry\":{\"type\":\"Polygon\",\"coordinates\":[[[5,1],[6,3],[4,"
         "3],[3,2],[1,1],[5,1]]]},\"properties\":{}}]}");

    Polygon<int> poly2{{{1, 1}, {3, 2}, {4, 3}, {6, 3}, {5, 1}, {1, 1}},
                       {{{1, 1}, {1, 2}, {2, 2}, {1, 1}}}};

    ss.str("");
    out.print(poly2, {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"Polygon\","
         "\"coordinates\":[[[5,1],[6,3],[4,3],[3,2],[1,1],[5,1]],[[1,1],[1,2],["
         "2,2],[1,1]]]},\"properties\":{}}");

    ss.str("");
    out.print(util::geo::MultiPolygon<int>{poly3, poly2}, {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"MultiPolygon\","
         "\"coordinates\":[[[[5,3],[6,5],[4,5],[3,4],[1,3],[5,3]]],[[[5,1],[6,"
         "3],[4,3],[3,2],[1,1],[5,1]],[[1,1],[1,2],[2,2],[1,1]]]]},"
         "\"properties\":{}}");

    ss.str("");
    out.print(geo::lineFromWKT<int>("LINESTRING(0 0, 1 1)"), {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"LineString\","
         "\"coordinates\":[[0,0],[1,1]]},\"properties\":{}}");

    ss.str("");
    out.print(
        util::geo::MultiLine<int>{
            geo::lineFromWKT<int>("LINESTRING(0 0, 1 1)"),
            geo::lineFromWKT<int>("LINESTRING(2 2, 3 3)")},
        {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"MultiLineString\","
         "\"coordinates\":[[[0,0],[1,1]],[[2,2],[3,3]]]},\"properties\":{}}");

    ss.str("");
    out.print(
        geo::multiLineFromWKT<int>("MULTILINESTRING((0 0, 1 1), (2 2, 3 3))"),
        {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"MultiLineString\","
         "\"coordinates\":[[[0,0],[1,1]],[[2,2],[3,3]]]},\"properties\":{}}");

    ss.str("");
    out.print(geo::multiPointFromWKT<int>("MULTIPOINT(0 0, 1 1, 2 2, 3 3)"),
              {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"MultiPoint\","
         "\"coordinates\":[[0,0],[1,1],[2,2],[3,3]]},\"properties\":{}}");

    ss.str("");
    out.print(
        geo::multiPointFromWKT<int>("MULTIPOINT((0 0), (1 1), (2 2), (3 3))"),
        {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"MultiPoint\","
         "\"coordinates\":[[0,0],[1,1],[2,2],[3,3]]},\"properties\":{}}");
  }

  {
    std::stringstream ss;
    util::geo::output::GeoJsonOutput out(ss);

    Polygon<int> poly{{{1, 1}, {3, 2}, {4, 3}, {6, 3}, {5, 1}, {1, 1}}};
    Polygon<int> poly3{{{1, 3}, {3, 4}, {4, 5}, {6, 5}, {5, 3}, {1, 3}}};

    out.print(poly, {});
    out.flush();
    std::string a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"FeatureCollection\",\"features\":[{\"type\":\"Feature\","
         "\"geometry\":{\"type\":\"Polygon\",\"coordinates\":[[[5,1],[6,3],[4,"
         "3],[3,2],[1,1],[5,1]]]},\"properties\":{}}]}");

    Polygon<int> poly2{{{1, 1}, {3, 2}, {4, 3}, {6, 3}, {5, 1}, {1, 1}},
                       {{{1, 1}, {1, 2}, {2, 2}, {1, 1}}}};

    ss.str("");
    out.print(poly2, {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"Polygon\","
         "\"coordinates\":[[[5,1],[6,3],[4,3],[3,2],[1,1],[5,1]],[[1,1],[1,2],["
         "2,2],[1,1]]]},\"properties\":{}}");

    ss.str("");
    out.print(util::geo::MultiPolygon<int>{poly3, poly2}, {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"MultiPolygon\","
         "\"coordinates\":[[[[5,3],[6,5],[4,5],[3,4],[1,3],[5,3]]],[[[5,1],[6,"
         "3],[4,3],[3,2],[1,1],[5,1]],[[1,1],[1,2],[2,2],[1,1]]]]},"
         "\"properties\":{}}");

    ss.str("");
    out.print(geo::lineFromWKT<int>("LINESTRING(0 0, 1 1)"), {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"LineString\","
         "\"coordinates\":[[0,0],[1,1]]},\"properties\":{}}");

    ss.str("");
    out.print(
        util::geo::MultiLine<int>{
            geo::lineFromWKT<int>("LINESTRING(0 0, 1 1)"),
            geo::lineFromWKT<int>("LINESTRING(2 2, 3 3)")},
        {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"MultiLineString\","
         "\"coordinates\":[[[0,0],[1,1]],[[2,2],[3,3]]]},\"properties\":{}}");

    ss.str("");
    out.print(
        geo::multiLineFromWKT<int>("MULTILINESTRING((0 0, 1 1), (2 2, 3 3))"),
        {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"MultiLineString\","
         "\"coordinates\":[[[0,0],[1,1]],[[2,2],[3,3]]]},\"properties\":{}}");

    ss.str("");
    out.print(geo::multiPointFromWKT<int>("MULTIPOINT(0 0, 1 1, 2 2, 3 3)"),
              {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"MultiPoint\","
         "\"coordinates\":[[0,0],[1,1],[2,2],[3,3]]},\"properties\":{}}");

    ss.str("");
    out.print(
        geo::multiPointFromWKT<int>("MULTIPOINT((0 0), (1 1), (2 2), (3 3))"),
        {});
    out.flush();
    a = ss.str();
    replaceAll(a, " ", "");
    replaceAll(a, "\n", "");
    TEST(a, ==,
         "{\"type\":\"Feature\",\"geometry\":{\"type\":\"MultiPoint\","
         "\"coordinates\":[[0,0],[1,1],[2,2],[3,3]]},\"properties\":{}}");
  }

  // geometrycollections
  {
    util::geo::Collection<double> coll;
    util::geo::Collection<double> coll2;
    util::geo::Collection<double> coll3;
    util::geo::Collection<double> coll4;

    coll.push_back(util::geo::Point<double>{3, 2});
    TEST(util::geo::getWKT(coll), ==, "GEOMETRYCOLLECTION(POINT(3 2))");

    coll.push_back(util::geo::Line<double>{{3, 2}, {1, 1}});
    TEST(util::geo::getWKT(coll), ==,
         "GEOMETRYCOLLECTION(POINT(3 2),LINESTRING(3 2,1 1))");

    coll2.push_back(util::geo::Point<double>{3, 2});

    coll3.push_back(util::geo::Line<double>{{3, 2}, {1, 1}});

    coll4.push_back(util::geo::Point<double>{3, 2});
    coll4.push_back(util::geo::Point<double>{5, 4});

    TEST(util::geo::getWKT(util::geo::getBoundingBox(coll)), ==,
         "POLYGON((1 1,3 1,3 2,1 2,1 1))");
    TEST(util::geo::getWKT(util::geo::convexHull(coll)), ==,
         "POLYGON((1 1,3 2,1 1))");
    TEST(util::geo::dimension(coll), ==, 2);
    TEST(util::geo::getWKT(util::geo::centroid(coll)), ==, "POINT(2 1.5)");
    TEST(util::geo::getWKT(
             util::geo::convexHull(util::geo::getOrientedEnvelope(coll))),
         ==, "POLYGON((1 1,1 1,3 2,3 2,1 1))");

    TEST(util::geo::getWKT(util::geo::centroid(coll2)), ==, "POINT(3 2)");
    TEST(util::geo::getWKT(util::geo::centroid(coll3)), ==, "POINT(2 1.5)");
    TEST(util::geo::getWKT(util::geo::centroid(coll4)), ==, "POINT(4 3)");
  }

  // centroids
  {
    TEST(util::geo::getWKT(util::geo::centroid(
             util::geo::pointFromWKT<double>("POINT(3.4 4.6)"))),
         ==, "POINT(3.4 4.6)");
    TEST(util::geo::getWKT(util::geo::centroid(
             util::geo::lineFromWKT<double>("LINESTRING(0 0, 1 1)"))),
         ==, "POINT(0.5 0.5)");
    TEST(
        util::geo::getWKT(util::geo::centroid(util::geo::polygonFromWKT<double>(
            "POLYGON((0 0, 1 0, 1 1, 0 1, 0 0))"))),
        ==, "POINT(0.5 0.5)");
    TEST(util::geo::getWKT(
             util::geo::centroid(util::geo::multiPolygonFromWKT<double>(
                 "MULTIPOLYGON(POLYGON((0 0,1 0,1 1,0 1,0 0)),POLYGON((0 0,1 "
                 "0,1 1,0 1,0 0)))"))),
         ==, "POINT(0.5 0.5)");

    TEST(util::geo::getWKT(
             util::geo::ringCentroid(util::geo::polygonFromWKT<double>(
                                         "POLYGON((5 5,5 10,10 10,10 5,5 5))")
                                         .getOuter())),
         ==, "POINT(7.5 7.5)");

    TEST(util::geo::getWKT(util::geo::ringCentroid(
             util::geo::polygonFromWKT<double>(
                 "POLYGON((0.5 0.5,0.5 1,1 1,1 0.5,0.5 0.5))")
                 .getOuter())),
         ==, "POINT(0.75 0.75)");
    TEST(
        util::geo::getWKT(util::geo::centroid(util::geo::polygonFromWKT<double>(
            "POLYGON((0.5 0.5,0.5 1,1 1,1 0.5,0.5 0.5))"))),
        ==, "POINT(0.75 0.75)");
    TEST(util::geo::getWKT(util::geo::ringCentroid(
             util::geo::polygonFromWKT<double>(
                 "POLYGON((0.5 0.5,1 0.5,1 1,0.5 1,0.5 0.5))")
                 .getOuter())),
         ==, "POINT(0.75 0.75)");
    TEST(
        util::geo::getWKT(util::geo::centroid(util::geo::polygonFromWKT<double>(
            "POLYGON((0.5 0.5,1 0.5,1 1,0.5 1,0.5 0.5))"))),
        ==, "POINT(0.75 0.75)");

    TEST(
        util::geo::getWKT(util::geo::centroid(util::geo::polygonFromWKT<double>(
            "POLYGON((0 0,1 0,1 1,0 1,0 0),(0.5 0.5,1 0.5,1 1,0.5 1,0.5 "
            "0.5),(0.5 0.5,0.5 0,1 0,1 0.5,0.5 0.5),(0 0.5,0.5 0.5,0.5 1, 0 "
            "1,0 0.5))"))),
        ==, "POINT(0.25 0.25)");
    TEST(util::geo::getWKT(util::geo::centroid(
             util::geo::polygonFromWKT<double>("POLYGON((0 0,1 1,0 0)"))),
         ==, "POINT(0.5 0.5)");

    TEST(util::geo::getWKT(util::geo::ringCentroid(
             util::geo::polygonFromWKT<double>(
                 "POLYGON((7.8386229 48.0081521,7.8386718 48.0081234,7.8387136 "
                 "48.0081553,7.8386647 48.008184,7.8386229 48.0081521))")
                 .getOuter())),
         ==, "POINT(7.838668 48.008154)");
  }
}
