// Copyright 2016, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#include <random>

#include "util/Test.h"
#include "util/geo/Geo.h"
#include "util/geo/Grid.h"
#include "util/geo/RTree.h"
#include "util/geo/output/GeoJsonOutput.h"
#include "util/log/Log.h"
#include "util/tests/GeoTest.h"

using namespace util;
using namespace util::geo;

// _____________________________________________________________________________
std::string readTestDataset(const std::string& name) {
  std::ifstream f(std::string(TEST_DATASETS) + "/" + name, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(f)), {});
}

struct LargeTestGeoms {
  // unsorted variants
  MultiPolygon<double> germany, spain;
  Polygon<double> saimaa;
  Polygon<double> vauban;
  Collection<double> flixbus;

  // xsorted variants
  XSortedMultiPolygon<double> germanyX, spainX, saimaaX, vaubanX;
  XSortedCollection<double> flixbusX;

  // web mercator variants for the meter distance tests
  MultiPolygon<double> germanyM, spainM;
  Polygon<double> saimaaM;
  Polygon<double> vaubanM;
  Collection<double> flixbusM;

  XSortedMultiPolygon<double> germanyMX, spainMX, saimaaMX, vaubanMX;
  XSortedCollection<double> flixbusMX;

  LargeTestGeoms()
      : germany(multiPolygonFromWKT<double>(readTestDataset("germany.tsv"))),
        spain(multiPolygonFromWKT<double>(readTestDataset("spain.tsv"))),
        saimaa(polygonFromWKT<double>(readTestDataset("saimaa.tsv"))),
        vauban(polygonFromWKT<double>(readTestDataset("vauban.tsv"))),
        flixbus(collectionFromWKT<double>(readTestDataset("flixbus.tsv"))),
        germanyX(germany),
        spainX(spain),
        saimaaX(saimaa),
        vaubanX(vauban),
        flixbusX(flixbus),
        germanyM(multiPolygonFromWKTProj<double>(readTestDataset("germany.tsv"),
                                                util::geo::projectToWebMerc<double>)),
        spainM(multiPolygonFromWKTProj<double>(readTestDataset("spain.tsv"),
                                              util::geo::projectToWebMerc<double>)),
        saimaaM(polygonFromWKTProj<double>(readTestDataset("saimaa.tsv"),
                                          util::geo::projectToWebMerc<double>)),
        vaubanM(polygonFromWKTProj<double>(readTestDataset("vauban.tsv"),
                                          util::geo::projectToWebMerc<double>)),
        flixbusM(collectionFromWKTProj<double>(readTestDataset("flixbus.tsv"),
                                              util::geo::projectToWebMerc<double>)),
        germanyMX(germanyM),
        spainMX(spainM),
        saimaaMX(saimaaM),
        vaubanMX(vaubanM),
        flixbusMX(flixbusM) {}
};

// _____________________________________________________________________________
static void testDistCombinations() {
  // dist between all possible combinations, including multigeometries
  auto poly = polygonFromWKT<double>("POLYGON((1 1, 1 10, 10 10, 10 1, 1 1))");
  auto poly2 = polygonFromWKT<double>(
      "POLYGON((4.25 4.25, 4.75 4.25, 4.75 4.75, 4.25 4.75, 4.25 4.25))");
  auto poly3 = polygonFromWKT<double>(
      "POLYGON((3.25 4.25, 4.75 4.25, 4.75 4.75, 4.25 4.75, 4.25 4.25))");
  auto poly4 = polygonFromWKT<double>(
      "POLYGON((2.25 2.25, 2.75 2.25, 2.75 2.75, 2.25 2.75, 2.25 2.25))");
  auto polyWithInner = polygonFromWKT<double>(
      "POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (4 4, 5 4, 5 5, 4 5, 4 4))");
  auto multiPoly = multiPolygonFromWKT<double>(
      "MULTIPOLYGON(((4.25 4.25, 4.75 4.25, 4.75 4.75, 4.25 4.75, 4.25 "
      "4.25)), ((0 0, 10 0, 10 10, 0 10, 0 0), (4 4, 5 4, 5 5, 4 5, 4 4), (2 "
      "2, 3 2, 3 3, 2 3, 2 2)))");
  auto line = lineFromWKT<double>("LINESTRING(10 4.5, 12 4.5)");
  auto line2 = lineFromWKT<double>("LINESTRING(4.75 4.5, 4.27 4.5)");
  auto line3 = lineFromWKT<double>("LINESTRING(3.75 4.5, 4.27 4.5)");
  auto point = pointFromWKT<double>("POINT(4.5 4.5)");
  auto point2 = pointFromWKT<double>("POINT(11 11)");
  auto point3 = pointFromWKT<double>("POINT(19 19)");

  auto collection = collectionFromWKT<double>(
      "GEOMETRYCOLLECTION(MULTIPOLYGON(((4.25 4.25, 4.75 4.25, 4.75 4.75, "
      "4.25 4.75, 4.25 4.25)), ((0 0, 10 0, 10 10, 0 10, 0 0), (4 4, 5 4, 5 "
      "5, 4 5, 4 4), (2 2, 3 2, 3 3, 2 3, 2 2))), POINT(20 20))");

  auto collection2 = collectionFromWKT<double>(
      "GEOMETRYCOLLECTION(MULTIPOINT(0 0, 1 1), POINT(2 2))");

  auto collection3 =
      collectionFromWKT<double>("GEOMETRYCOLLECTION(POINT(4 4), POINT(3 3))");

  TEST(util::geo::dist(point, line), ==, approx(5.5));
  TEST(util::geo::dist(line, point), ==, approx(5.5));
  TEST(util::geo::dist(point2, poly), ==, approx(sqrt(2)));
  TEST(util::geo::dist(poly, point2), ==, approx(sqrt(2)));
  TEST(util::geo::dist(line, poly), ==, 0);
  TEST(util::geo::dist(poly, line), ==, 0);
  TEST(util::geo::dist(line, line2), ==, 5.25);
  TEST(util::geo::dist(line, line3), ==, 5.73);

  TEST(
      util::geo::dist(MultiLine<double>{line}, MultiLine<double>{line2, line3}),
      ==,
      approx(std::min(util::geo::dist(line, line2),
                      util::geo::dist(line, line3))));

  TEST(util::geo::dist(point, point), ==, 0);
  TEST(util::geo::dist(line, line), ==, 0);
  TEST(util::geo::dist(poly, poly), ==, 0);

  TEST(util::geo::dist(point, poly), ==, approx(0));
  TEST(util::geo::dist(poly, point), ==, approx(0));

  TEST(util::geo::dist(point, polyWithInner), ==, approx(0.5));
  TEST(util::geo::dist(polyWithInner, point), ==, approx(0.5));

  TEST(util::geo::dist(line2, polyWithInner), ==, approx(0.25));
  TEST(util::geo::dist(polyWithInner, line2), ==, approx(0.25));

  TEST(util::geo::dist(poly2, polyWithInner), ==, approx(0.25));
  TEST(util::geo::dist(polyWithInner, poly2), ==, approx(0.25));

  TEST(util::geo::dist(poly2, multiPoly), ==, approx(0));
  TEST(util::geo::dist(multiPoly, poly2), ==, approx(0));

  TEST(util::geo::dist(poly3, multiPoly), ==, approx(0));
  TEST(util::geo::dist(multiPoly, poly3), ==, approx(0));

  TEST(util::geo::dist(poly4, multiPoly), ==, approx(0.25));
  TEST(util::geo::dist(multiPoly, poly4), ==, approx(0.25));

  TEST(util::geo::dist(line2, multiPoly), ==, approx(0));
  TEST(util::geo::dist(multiPoly, line2), ==, approx(0));

  TEST(util::geo::dist(polyWithInner, poly3), ==, approx(0));
  TEST(util::geo::dist(polyWithInner, line3), ==, approx(0));

  TEST(util::geo::dist(collection, poly3), ==, approx(0));
  TEST(util::geo::dist(poly3, collection), ==, approx(0));

  TEST(util::geo::dist(collection, point3), ==, approx(sqrt(2)));
  TEST(util::geo::dist(point3, collection), ==, approx(sqrt(2)));

  TEST(util::geo::dist(collection, multiPoly), ==, approx(0));
  TEST(util::geo::dist(multiPoly, collection), ==, approx(0));

  TEST(util::geo::dist(collection, collection), ==, approx(0));

  TEST(util::geo::dist(collection2, collection3), ==, approx(sqrt(2)));
  TEST(util::geo::dist(collection3, collection2), ==, approx(sqrt(2)));
}

// _____________________________________________________________________________
static void testDistWithinRealisticMaxDist() {
  auto poly = polygonFromWKT<double>("POLYGON((1 1, 1 10, 10 10, 10 1, 1 1))");
  auto poly2 = polygonFromWKT<double>(
      "POLYGON((4.25 4.25, 4.75 4.25, 4.75 4.75, 4.25 4.75, 4.25 4.25))");
  auto polyWithInner = polygonFromWKT<double>(
      "POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (4 4, 5 4, 5 5, 4 5, 4 4))");
  auto line = lineFromWKT<double>("LINESTRING(10 4.5, 12 4.5)");
  auto line2 = lineFromWKT<double>("LINESTRING(4.75 4.5, 4.27 4.5)");
  auto point = pointFromWKT<double>("POINT(4.5 4.5)");
  auto point2 = pointFromWKT<double>("POINT(11 11)");

  TEST(util::geo::withinDist(point, XSortedLine<double>(line), 10.0), ==,
       approx(5.5));
  TEST(util::geo::withinDist(XSortedLine<double>(line), point, 10.0), ==,
       approx(5.5));
  TEST(util::geo::withinDist(XSortedLine<double>(line), point, 5.4), >,
       5.4);
  TEST(util::geo::withinDist(XSortedLine<double>(line), point, 5.0), >,
       5.0);
  TEST(util::geo::withinDist(XSortedLine<double>(line), point, 1.0), >,
       1.0);

  TEST(util::geo::withinDist(XSortedPolygon<double>(poly), point2, 10.0), ==,
       approx(sqrt(2)));
  TEST(util::geo::withinDist(XSortedPolygon<double>(poly), point2, 1.0), >,
       1);
  TEST(util::geo::withinDist(point2, XSortedPolygon<double>(poly), 10.0), ==,
       approx(sqrt(2)));

  TEST(util::geo::withinDist(XSortedLine<double>(line),
                             XSortedPolygon<double>(poly), 10.0),
       ==, 0);
  TEST(util::geo::withinDist(XSortedPolygon<double>(poly),
                             XSortedLine<double>(line), 10.0),
       ==, 0);

  TEST(util::geo::withinDist(XSortedLine<double>(line2),
                             XSortedPolygon<double>(polyWithInner), 10.0),
       ==, approx(0.25));
  TEST(util::geo::withinDist(XSortedPolygon<double>(polyWithInner),
                             XSortedLine<double>(line2), 10.0),
       ==, approx(0.25));
  TEST(util::geo::withinDist(XSortedPolygon<double>(polyWithInner),
                             point, 10.0),
       ==, approx(0.5));
  TEST(util::geo::withinDist(XSortedPolygon<double>(polyWithInner),
                             XSortedLine<double>(line2), .2),
       >, 0.2);
  TEST(util::geo::withinDist(XSortedPolygon<double>(polyWithInner),
                             point, .4),
       >, 0.4);

  TEST(util::geo::withinDist(XSortedPolygon<double>(poly2),
                             XSortedPolygon<double>(polyWithInner), 10.0),
       ==, approx(0.25));
  TEST(util::geo::withinDist(XSortedPolygon<double>(polyWithInner),
                             XSortedPolygon<double>(poly2), 10.0),
       ==, approx(0.25));
  TEST(util::geo::withinDist(XSortedPolygon<double>(polyWithInner),
                             XSortedPolygon<double>(poly2), .2),
       >, 0.2);
}

// _____________________________________________________________________________
static void testDistWithinInfinityMaxDist() {
  auto poly = polygonFromWKT<double>("POLYGON((1 1, 1 10, 10 10, 10 1, 1 1))");
  auto poly2 = polygonFromWKT<double>(
      "POLYGON((4.25 4.25, 4.75 4.25, 4.75 4.75, 4.25 4.75, 4.25 4.25))");
  auto polyWithInner = polygonFromWKT<double>(
      "POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (4 4, 5 4, 5 5, 4 5, 4 4))");
  auto line = lineFromWKT<double>("LINESTRING(10 4.5, 12 4.5)");
  auto line2 = lineFromWKT<double>("LINESTRING(4.75 4.5, 4.27 4.5)");
  auto point = pointFromWKT<double>("POINT(4.5 4.5)");
  auto point2 = pointFromWKT<double>("POINT(11 11)");

  TEST(util::geo::dist(point, XSortedLine<double>(line)), ==, approx(5.5));
  TEST(util::geo::dist(XSortedLine<double>(line), point), ==, approx(5.5));

  TEST(util::geo::dist(XSortedPolygon<double>(poly), point2), ==,
       approx(sqrt(2)));
  TEST(util::geo::dist(point2, XSortedPolygon<double>(poly)), ==,
       approx(sqrt(2)));

  TEST(util::geo::dist(XSortedLine<double>(line), XSortedPolygon<double>(poly)),
       ==, 0);
  TEST(util::geo::dist(XSortedPolygon<double>(poly), XSortedLine<double>(line)),
       ==, 0);

  TEST(util::geo::dist(XSortedLine<double>(line2),
                       XSortedPolygon<double>(polyWithInner)),
       ==, approx(0.25));
  TEST(util::geo::dist(XSortedPolygon<double>(polyWithInner),
                       XSortedLine<double>(line2)),
       ==, approx(0.25));

  TEST(util::geo::dist(XSortedPolygon<double>(poly2),
                       XSortedPolygon<double>(polyWithInner)),
       ==, approx(0.25));
  TEST(util::geo::dist(XSortedPolygon<double>(polyWithInner),
                       XSortedPolygon<double>(poly2)),
       ==, approx(0.25));
}

// _____________________________________________________________________________
static void testDistWithinExactMaxDist() {
  auto poly = polygonFromWKT<double>("POLYGON((1 1, 1 10, 10 10, 10 1, 1 1))");
  auto poly2 = polygonFromWKT<double>(
      "POLYGON((4.25 4.25, 4.75 4.25, 4.75 4.75, 4.25 4.75, 4.25 4.25))");
  auto polyWithInner = polygonFromWKT<double>(
      "POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (4 4, 5 4, 5 5, 4 5, 4 4))");
  auto line = lineFromWKT<double>("LINESTRING(10 4.5, 12 4.5)");
  auto line2 = lineFromWKT<double>("LINESTRING(4.75 4.5, 4.27 4.5)");
  auto line3 = lineFromWKT<double>("LINESTRING(3.75 4.5, 4.27 4.5)");
  auto point = pointFromWKT<double>("POINT(4.5 4.5)");
  auto point2 = pointFromWKT<double>("POINT(11 11)");
  auto point3 = pointFromWKT<double>("POINT(19 19)");

  TEST(util::geo::withinDist(point, XSortedLine<double>(line), 5.5), ==,
       approx(5.5));
  TEST(util::geo::withinDist(XSortedLine<double>(line), point, 5.5), ==,
       approx(5.5));

  TEST(util::geo::withinDist(XSortedPolygon<double>(poly), point2, sqrt(2)), ==,
       approx(sqrt(2)));
  TEST(util::geo::withinDist(point2, XSortedPolygon<double>(poly), sqrt(2)), ==,
       approx(sqrt(2)));

  TEST(util::geo::withinDist(XSortedLine<double>(line),
                             XSortedPolygon<double>(poly), 0),
       ==, 0);
  TEST(util::geo::withinDist(XSortedPolygon<double>(poly),
                             XSortedLine<double>(line), 0),
       ==, 0);

  TEST(util::geo::withinDist(XSortedLine<double>(line2),
                             XSortedPolygon<double>(polyWithInner), 0.25),
       ==, approx(0.25));
  TEST(util::geo::withinDist(XSortedPolygon<double>(polyWithInner),
                             XSortedLine<double>(line2), 0.25),
       ==, approx(0.25));

  TEST(util::geo::withinDist(XSortedPolygon<double>(poly2),
                             XSortedPolygon<double>(polyWithInner), 0.25),
       ==, approx(0.25));

  TEST(util::geo::withinDist(
           XSortedMultiPolygon<double>(MultiPolygon<double>{poly2}),
           XSortedMultiPolygon<double>(MultiPolygon<double>{polyWithInner}),
           0.25),
       ==, approx(0.25));

  TEST(util::geo::withinDist(XSortedPolygon<double>(polyWithInner),
                             XSortedPolygon<double>(poly2), 0.25),
       ==, approx(0.25));

  TEST(util::geo::withinDist(
           XSortedMultiPolygon<double>(MultiPolygon<double>{polyWithInner}),
           XSortedMultiPolygon<double>(MultiPolygon<double>{poly2}), 0.25),
       ==, approx(0.25));

  TEST(util::geo::withinDist(
           XSortedMultiPolygon<double>(MultiPolygon<double>{polyWithInner}),
           XSortedMultiPolygon<double>(MultiPolygon<double>{polyWithInner}),
           0.25),
       ==, approx(0));

  TEST(util::geo::withinDist(
           XSortedMultiLine<double>(MultiLine<double>{line}),
           XSortedMultiLine<double>(MultiLine<double>{line2, line3}), 10),
       ==,
       approx(std::min(util::geo::dist(line, line2),
                       util::geo::dist(line, line3))));
  TEST(util::geo::withinDist(
           XSortedMultiPolygon<double>(MultiPolygon<double>{polyWithInner}),
           XSortedMultiLine<double>(MultiLine<double>{line2, line3}), 10),
       ==,
       approx(std::min(util::geo::dist(polyWithInner, line2),
                       util::geo::dist(polyWithInner, line3))));
  TEST(
      util::geo::withinDist(
          XSortedMultiLine<double>(MultiLine<double>{line2, line3}),
          XSortedMultiPolygon<double>(MultiPolygon<double>{polyWithInner}), 10),
      ==,
      approx(std::min(util::geo::dist(polyWithInner, line2),
                      util::geo::dist(polyWithInner, line3))));
  TEST(util::geo::withinDist(
           XSortedCollection<double>(MultiPoint<double>{point, point2}),
           XSortedCollection<double>(MultiPoint<double>{point3}), 20),
       ==,
       approx(std::min(util::geo::dist(point, point3),
                       util::geo::dist(point2, point3))));
  TEST(util::geo::withinDist(
           XSortedCollection<double>(MultiLine<double>{line}),
           XSortedCollection<double>(MultiPoint<double>{point, point2}), 20),
       ==,
       approx(std::min(util::geo::dist(MultiLine<double>{line}, point),
                       util::geo::dist(MultiLine<double>{line}, point2))));
  TEST(util::geo::withinDist(
           XSortedMultiPolygon<double>(MultiPolygon<double>{polyWithInner}),
           XSortedCollection<double>(MultiPoint<double>{point, point2}), 20),
       ==,
       approx(std::min(
           util::geo::dist(MultiPolygon<double>{polyWithInner}, point),
           util::geo::dist(MultiPolygon<double>{polyWithInner}, point2))));
}

// _____________________________________________________________________________
static void testDistComplexGeoms(const LargeTestGeoms& g) {
  TEST(util::geo::withinDist(g.germanyX, g.spainX, 10), ==, approx(6.5434));
  TEST(util::geo::withinDist(g.germanyX, g.spainX, 6.54341), ==,
       approx(6.5434));
  TEST(util::geo::dist(g.germanyX, g.spainX), ==, approx(6.5434));
  TEST(util::geo::webMercMeterDist(g.germanyMX, g.spainMX), ==,
       approx(653276.57366));

  TEST(util::geo::withinDist(g.germany, g.spain, 10), ==, approx(6.5434));
  TEST(util::geo::withinDist(g.germany, g.spain, 6.54341), ==, approx(6.5434));
  TEST(util::geo::dist(g.germany, g.spain), ==, approx(6.5434));
  TEST(util::geo::webMercMeterDist(g.germanyM, g.spainM), ==,
       approx(653276.57366));

  TEST(util::geo::withinDist(g.germany, g.germany, 10), ==, approx(0));
  TEST(util::geo::withinDist(g.germany, g.germany, 0), ==, approx(0));
  TEST(util::geo::dist(g.germany, g.germany), ==, approx(0));
  TEST(util::geo::webMercMeterDist(g.germanyM, g.germanyM), ==, approx(0));

  TEST(util::geo::withinDist(g.spain, g.spain, 10), ==, approx(0));
  TEST(util::geo::withinDist(g.spain, g.spain, 0), ==, approx(0));
  TEST(util::geo::dist(g.spain, g.spain), ==, approx(0));
  TEST(util::geo::webMercMeterDist(g.spainM, g.spainM), ==, approx(0));

  TEST(util::geo::withinDist(g.germanyX, g.germanyX, 10), ==, approx(0));
  TEST(util::geo::withinDist(g.germanyX, g.germanyX, 0), ==, approx(0));
  TEST(util::geo::dist(g.germanyX, g.germanyX), ==, approx(0));
  TEST(util::geo::webMercMeterDist(g.germanyMX, g.germanyMX), ==, approx(0));

  TEST(util::geo::withinDist(g.spainX, g.spainX, 10), ==, approx(0));
  TEST(util::geo::withinDist(g.spainX, g.spainX, 0), ==, approx(0));
  TEST(util::geo::dist(g.spainX, g.spainX), ==, approx(0));
  TEST(util::geo::webMercMeterDist(g.spainMX, g.spainMX), ==, approx(0));

  TEST(util::geo::withinDist(g.germanyX, g.flixbusX, 10), ==, approx(0));
  TEST(util::geo::withinDist(g.germanyX, g.flixbusX, 0), ==, approx(0));
  TEST(util::geo::dist(g.germanyX, g.flixbusX), ==, approx(0));
  TEST(util::geo::webMercMeterDist(g.germanyMX, g.flixbusMX), ==, approx(0));

  TEST(util::geo::withinDist(g.germany, g.flixbus, 10), ==, approx(0));
  TEST(util::geo::withinDist(g.germany, g.flixbus, 0), ==, approx(0));
  TEST(util::geo::dist(g.germany, g.flixbus), ==, approx(0));
  TEST(util::geo::webMercMeterDist(g.germanyM, g.flixbusM), ==, approx(0));

  TEST(util::geo::withinDist(g.flixbus, g.flixbus, 10), ==, approx(0));
  TEST(util::geo::withinDist(g.flixbus, g.flixbus, 0), ==, approx(0));
  TEST(util::geo::dist(g.flixbus, g.flixbus), ==, approx(0));
  TEST(util::geo::webMercMeterDist(g.flixbusM, g.flixbusM), ==, approx(0));

  TEST(util::geo::withinDist(g.flixbusX, g.flixbusX, 10), ==, approx(0));
  TEST(util::geo::withinDist(g.flixbusX, g.flixbusX, 0), ==, approx(0));
  TEST(util::geo::dist(g.flixbusX, g.flixbusX), ==, approx(0));
  TEST(util::geo::webMercMeterDist(g.flixbusMX, g.flixbusMX), ==, approx(0));

  TEST(util::geo::withinDist(g.spainX, g.flixbusX, 10), ==, approx(7.00409));
  TEST(util::geo::withinDist(g.spainX, g.flixbusX, 7.004091), ==,
       approx(7.00409));
  TEST(util::geo::dist(g.spainX, g.flixbusX), ==, approx(7.00409));
  TEST(util::geo::webMercMeterDist(g.spainMX, g.flixbusMX), ==,
       approx(703461.25144));

  TEST(util::geo::withinDist(g.spain, g.flixbus, 10), ==, approx(7.00409));
  TEST(util::geo::withinDist(g.spain, g.flixbus, 7.004091), ==,
       approx(7.00409));
  TEST(util::geo::dist(g.spain, g.flixbus), ==, approx(7.00409));
  TEST(util::geo::webMercMeterDist(g.spainM, g.flixbusM), ==,
       approx(703461.25144));

  auto line = lineFromWKTProj<double>("LINESTRING(7.8824970  48.0228303,7.8823288 48.0227874,7.8820604 48.0227417,7.8819946 48.0227305)", util::geo::projectToWebMerc<double>);
  auto lineX = XSortedLine<double>(line);

  TEST(util::geo::withinDist(g.vaubanM, line, 10), ==, approx(9638.74057));
  TEST(util::geo::withinDist(g.vaubanM, line, 9638.74057), ==,
       approx(9638.74057));
  TEST(util::geo::dist(g.vaubanM, line), ==, approx(9638.74057));
  TEST(util::geo::webMercMeterDist(g.vaubanM, line), !=,
       approx(util::geo::dist(g.vaubanM, line)));

  TEST(util::geo::webMercMeterDist(g.vaubanM, line), ==,
       util::geo::webMercMeterDist(line, g.vaubanM));

  TEST(util::geo::webMercMeterDist(g.vaubanM, line), ==,
       util::geo::webMercMeterDist(lineX, g.vaubanMX));

  TEST(util::geo::webMercMeterDist(line, g.vaubanM), ==,
       util::geo::webMercMeterDist(lineX, g.vaubanMX));

  TEST(util::geo::webMercMeterDist(line, g.vaubanM), ==,
       util::geo::webMercMeterDist(g.vaubanMX, lineX));

  TEST(util::geo::webMercMeterDist(g.vaubanM, line), ==,
       approx(6449.59555));

  TEST(util::geo::webMercMeterDist(line, g.vaubanM), ==,
       approx(6449.59555));
}

// _____________________________________________________________________________
static void testDistHaversineNoPadding(const LargeTestGeoms& g) {
  // with haversine, but without any search padding

  TEST(std::round(util::geo::withinDist(
                      g.germanyMX, g.spainMX, 1000000,
                      defaultPaddingFunc<double>(), 1e8,
                      [](const Point<double> a, const Point<double> b,
                         double) -> double { return haversineWebMerc(a, b); }) *
                  10.0) /
           10.0,
       ==, approx(653276.6));
  TEST(std::round(util::geo::withinDist(
                      g.germanyMX, g.germanyMX, 1000000,
                      defaultPaddingFunc<double>(), 1e8,
                      [](const Point<double> a, const Point<double> b,
                         double) -> double { return haversineWebMerc(a, b); }) *
                  10.0) /
           10.0,
       ==, approx(0));
  TEST(std::round(util::geo::withinDist(
                      g.germanyMX, g.saimaaMX, 10000000,
                      defaultPaddingFunc<double>(), 1e8,
                      [](const Point<double> a, const Point<double> b,
                         double) -> double { return haversineWebMerc(a, b); }) *
                  10.0) /
           10.0,
       ==, approx(1082465.4));

  TEST(std::round(
           util::geo::withinDist(
               XSortedCollection<double>(MultiPolygon<double>{g.germanyM}),
               XSortedCollection<double>(Collection<double>{g.spainM, g.saimaaM}),
               10000000, defaultPaddingFunc<double>(), 1e8,
               [](const Point<double> a, const Point<double> b,
                  double) -> double { return haversineWebMerc(a, b); }) *
           10.0) /
           10.0,
       ==, approx(653276.6));

  TEST(std::round(
           util::geo::withinDist(
               g.germanyMX,
               XSortedCollection<double>(Collection<double>{g.spainM, g.saimaaM}),
               10000000, defaultPaddingFunc<double>(), 1e8,
               [](const Point<double> a, const Point<double> b,
                  double) -> double { return haversineWebMerc(a, b); }) *
           10.0) /
           10.0,
       ==, approx(653276.6));

  TEST(std::round(
           util::geo::withinDist(
               XSortedCollection<double>(Collection<double>{g.spainM, g.saimaaM}),
               g.germanyMX, 10000000, defaultPaddingFunc<double>(), 1e8,
               [](const Point<double> a, const Point<double> b,
                  double) -> double { return haversineWebMerc(a, b); }) *
           10.0) /
           10.0,
       ==, approx(653276.6));
}

// _____________________________________________________________________________
static void testDistHaversineSmallPadding(const LargeTestGeoms& g) {
  // with haversine, but with small search padding

  TEST(std::round(util::geo::withinDist(
                      g.germanyMX, g.spainMX, 1000000,
                      [](double d, double, const Box<double>&,
                         const Box<double>&) -> double { return 1.02 * d; },
                      1e8,
                      [](const Point<double> a, const Point<double> b,
                         double) -> double { return haversineWebMerc(a, b); }) *
                  10.0) /
           10.0,
       ==, approx(653276.6));
  TEST(std::round(util::geo::withinDist(
                      g.germanyMX, g.flixbusMX, 1000000,
                      [](double d, double, const Box<double>&,
                         const Box<double>&) -> double { return 1.02 * d; },
                      1e8,
                      [](const Point<double> a, const Point<double> b,
                         double) -> double { return haversineWebMerc(a, b); }) *
                  10.0) /
           10.0,
       ==, approx(0));

  TEST(std::round(util::geo::withinDist(
                      g.spainMX, g.flixbusMX, 1000000,
                      [](double d, double, const Box<double>&,
                         const Box<double>&) -> double { return 1.02 * d; },
                      1e8,
                      [](const Point<double> a, const Point<double> b,
                         double) -> double { return haversineWebMerc(a, b); }) *
                  10.0) /
           10.0,
       ==, approx(703461.3));
}

// _____________________________________________________________________________
static void testDistHaversineMeterDistPadding(const LargeTestGeoms& g) {
  // with haversine, automatic padding via meterDist

  TEST(std::round(util::geo::webMercMeterDist(g.germanyMX, g.spainMX) * 10.0) /
           10.0,
       ==, approx(653276.6));
  TEST(std::round(util::geo::webMercMeterDist(g.germanyMX, g.flixbusMX) * 10.0) /
           10.0,
       ==, approx(0));
  TEST(std::round(util::geo::webMercMeterDist(g.spainMX, g.flixbusMX) * 10.0) /
           10.0,
       ==, approx(703461.3));
  // takes too long
  // TEST(std::round(util::geo::meterDist(g.germanyMX,
  // XSortedCollection<double>(Collection<double>{g.spainM, g.saimaaM})) * 10.0)
  // / 10.0, ==, approx(653276.6));
  // TEST(std::round(util::geo::meterDist(XSortedCollection<double>(Collection<double>{g.spain,
  // g.saimaa}), g.germanyMX) * 10.0) / 10.0, ==, approx(653276.6));
}

// _____________________________________________________________________________
static void testDistCollections() {
  auto poly = polygonFromWKT<double>("POLYGON((1 1, 1 10, 10 10, 10 1, 1 1))");
  auto polyWithInner = polygonFromWKT<double>(
      "POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (4 4, 5 4, 5 5, 4 5, 4 4))");
  auto line = lineFromWKT<double>("LINESTRING(10 4.5, 12 4.5)");
  auto line2 = lineFromWKT<double>("LINESTRING(4.75 4.5, 4.27 4.5)");
  auto line3 = lineFromWKT<double>("LINESTRING(3.75 4.5, 4.27 4.5)");
  auto point = pointFromWKT<double>("POINT(4.5 4.5)");

  Collection<double> col, col2;
  col.push_back(line);
  col.push_back(point);
  col2.push_back(line3);
  TEST(util::geo::withinDist(XSortedCollection<double>(col),
                             XSortedCollection<double>(col2), 20),
       ==, approx(.23));

  Collection<double> col3, col4;
  col3.push_back(polyWithInner);
  col4.push_back(line2);
  TEST(util::geo::withinDist(XSortedCollection<double>(col3),
                             XSortedCollection<double>(col4), 20),
       ==, approx(.25));
  TEST(util::geo::dist(XSortedCollection<double>(col3),
                       XSortedCollection<double>(col4)),
       ==, approx(.25));

  col4.push_back(poly);

  TEST(util::geo::withinDist(XSortedCollection<double>(col3),
                             XSortedCollection<double>(col4), 20),
       ==, approx(0));
  TEST(util::geo::dist(XSortedCollection<double>(col3),
                       XSortedCollection<double>(col4)),
       ==, approx(0));

  TEST(util::geo::withinDist(XSortedCollection<double>(),
                             XSortedCollection<double>(col4), 20),
       >, 20);
  TEST(util::geo::dist(XSortedCollection<double>(),
                       XSortedCollection<double>(col4)),
       >, 20);

  TEST(util::geo::withinDist(XSortedCollection<double>(),
                             XSortedCollection<double>(), 20),
       >, 20);
  TEST(
      util::geo::dist(XSortedCollection<double>(), XSortedCollection<double>()),
      >, 20);

  TEST(util::geo::withinDist(XSortedCollection<double>(col),
                             XSortedCollection<double>(col), 20),
       ==, approx(0));
  TEST(util::geo::dist(XSortedCollection<double>(col),
                       XSortedCollection<double>(col)),
       ==, approx(0));

  TEST(util::geo::withinDist(XSortedCollection<double>(col2),
                             XSortedCollection<double>(col2), 20),
       ==, approx(0));
  TEST(util::geo::dist(XSortedCollection<double>(col2),
                       XSortedCollection<double>(col2)),
       ==, approx(0));

  TEST(util::geo::withinDist(XSortedCollection<double>(col3),
                             XSortedCollection<double>(col3), 20),
       ==, approx(0));
  TEST(util::geo::dist(XSortedCollection<double>(col3),
                       XSortedCollection<double>(col3)),
       ==, approx(0));

  TEST(util::geo::withinDist(XSortedCollection<double>(col4),
                             XSortedCollection<double>(col4), 20),
       ==, approx(0));
  TEST(util::geo::dist(XSortedCollection<double>(col4),
                       XSortedCollection<double>(col4)),
       ==, approx(0));

  TEST(util::geo::withinDist(col4, col4, 20), ==, approx(0));
  TEST(util::geo::dist(col4, col4), ==, approx(0));

  TEST(util::geo::withinDist(XSortedCollection<double>(col),
                             XSortedLine<double>(line3), 20),
       ==, approx(.23));
  TEST(util::geo::dist(XSortedCollection<double>(col),
                       XSortedLine<double>(line3)),
       ==, approx(.23));

  TEST(util::geo::withinDist(XSortedLine<double>(line3),
                             XSortedCollection<double>(col), 20),
       ==, approx(.23));
  TEST(util::geo::dist(XSortedLine<double>(line3),
                       XSortedCollection<double>(col)),
       ==, approx(.23));

  TEST(util::geo::withinDist(XSortedCollection<double>(col),
                             XSortedPolygon<double>(poly), 20),
       ==, approx(0));
  TEST(util::geo::dist(XSortedCollection<double>(col),
                       XSortedPolygon<double>(poly)),
       ==, approx(0));

  TEST(util::geo::withinDist(XSortedPolygon<double>(poly),
                             XSortedCollection<double>(col), 20),
       ==, approx(0));
  TEST(util::geo::dist(XSortedPolygon<double>(poly),
                       XSortedCollection<double>(col)),
       ==, approx(0));

  TEST(util::geo::withinDist(XSortedCollection<double>(col), point, 20), ==,
       approx(0));
  TEST(util::geo::dist(XSortedCollection<double>(col), point), ==, approx(0));

  TEST(util::geo::withinDist(point, XSortedCollection<double>(col), 20), ==,
       approx(0));
  TEST(util::geo::dist(point, XSortedCollection<double>(col)), ==, approx(0));
}

// _____________________________________________________________________________
static void testDistOther() {
  auto polyWithInner = polygonFromWKT<double>(
      "POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (4 4, 5 4, 5 5, 4 5, 4 4))");
  auto line = lineFromWKT<double>("LINESTRING(10 4.5, 12 4.5)");
  auto line2 = lineFromWKT<double>("LINESTRING(4.75 4.5, 4.27 4.5)");
  auto point = pointFromWKT<double>("POINT(4.5 4.5)");
  auto point2 = pointFromWKT<double>("POINT(11 11)");

  auto lineFreiburgHbf = lineFromWKT<double>("");
  auto polygonFreiburg = polygonFromWKT<double>("");

  // web mercator copies, for the meter distance assertions
  auto polyWithInnerM = polygonFromWKTProj<double>(
      "POLYGON((0 0, 10 0, 10 10, 0 10, 0 0), (4 4, 5 4, 5 5, 4 5, 4 4))",
      util::geo::projectToWebMerc<double>);
  auto lineM = lineFromWKTProj<double>("LINESTRING(10 4.5, 12 4.5)",
                                       util::geo::projectToWebMerc<double>);
  auto line2M = lineFromWKTProj<double>("LINESTRING(4.75 4.5, 4.27 4.5)",
                                        util::geo::projectToWebMerc<double>);
  auto pointM = pointFromWKTProj<double>("POINT(4.5 4.5)",
                                         util::geo::projectToWebMerc<double>);
  auto point2M = pointFromWKTProj<double>("POINT(11 11)",
                                          util::geo::projectToWebMerc<double>);

  // standard point/point
  TEST(util::geo::withinDist(point, point2, 1), >, 1);
  TEST(util::geo::withinDist(point2, point, 1), >, 1);

  TEST(util::geo::dist(point, point2), ==, approx(9.19239));
  TEST(util::geo::dist(point2, point), ==, approx(9.19239));

  TEST(util::geo::webMercMeterDist(pointM, point2M), ==,
       approx(haversineWebMerc(pointM, point2M)));
  TEST(util::geo::webMercMeterDist(point2M, pointM), ==,
       approx(haversineWebMerc(pointM, point2M)));

  // standard point/line
  TEST(util::geo::withinDist(point2, line2, 100), ==, approx(9.01734));
  TEST(util::geo::withinDist(line, point, 100), ==, approx(5.5));

  TEST(util::geo::dist(point2, line2), ==, approx(9.01734));
  TEST(util::geo::dist(line, point), ==, approx(5.5));

  TEST(util::geo::webMercMeterDist(point2M, line2M), ==, approx(999138.32522));
  TEST(util::geo::webMercMeterDist(lineM, pointM), ==, approx(610368.37082));

  // standard point/polygon
  TEST(util::geo::withinDist(point, polyWithInner, 1), ==, approx(0.5));
  TEST(util::geo::withinDist(polyWithInner, point, 1), ==, approx(0.5));

  TEST(util::geo::dist(point, polyWithInner), ==, approx(0.5));
  TEST(util::geo::dist(polyWithInner, point), ==, approx(0.5));

  TEST(util::geo::webMercMeterDist(pointM, polyWithInnerM), ==,
       approx(55488.16389));
  TEST(util::geo::webMercMeterDist(polyWithInnerM, pointM), ==,
       approx(55488.16389));

  // standard line/polygon
  TEST(util::geo::withinDist(line2, polyWithInner, 100), ==, approx(0.25));
  TEST(util::geo::withinDist(polyWithInner, line2, 100), ==, approx(0.25));

  TEST(util::geo::dist(line2, polyWithInner), ==, approx(0.25));
  TEST(util::geo::dist(polyWithInner, line2), ==, approx(0.25));

  TEST(util::geo::webMercMeterDist(line2M, polyWithInnerM), ==,
       approx(27744.08235));
  TEST(util::geo::webMercMeterDist(polyWithInnerM, line2M), ==,
       approx(27744.08235));

  // standard line/line
  auto segLineA = lineFromWKT<double>("LINESTRING(0 0, 1 0)");
  auto segLineB = lineFromWKT<double>("LINESTRING(0.5 0.001, 1.5 0.001)");
  auto segLineAM = lineFromWKTProj<double>(
      "LINESTRING(0 0, 1 0)",
      util::geo::projectToWebMerc<double>);
  auto segLineBM = lineFromWKTProj<double>(
      "LINESTRING(0.5 0.001, 1.5 0.001)",
      util::geo::projectToWebMerc<double>);

  TEST(util::geo::dist(segLineA, segLineB), ==, approx(0.001));
  TEST(util::geo::dist(segLineB, segLineA), ==, approx(0.001));

  TEST(util::geo::webMercMeterDist(segLineAM, segLineBM), ==,
       approx(haversineWebMerc(latLngToWebMerc(Point<double>{0.5, 0.0}),
                               latLngToWebMerc(Point<double>{0.5, 0.001}))));
  TEST(util::geo::webMercMeterDist(segLineBM, segLineAM), ==,
       approx(haversineWebMerc(latLngToWebMerc(Point<double>{0.5, 0.0}),
                               latLngToWebMerc(Point<double>{0.5, 0.001}))));

  auto projSeg = lineFromWKT<double>("LINESTRING(0.5 0.001, 1.5 0.001)");
  const Point<double> projP{1.0, 0.0};
  auto projSegM = lineFromWKTProj<double>(
      "LINESTRING(0.5 0.001, 1.5 0.001)",
      util::geo::projectToWebMerc<double>);
  const auto projPM = latLngToWebMerc(projP);

  TEST(util::geo::dist(projP, projSeg), ==, approx(0.001));
  TEST(util::geo::webMercMeterDist(projPM, projSegM), ==,
       approx(haversineWebMerc(
           projPM, latLngToWebMerc(Point<double>{1.0, 0.001}))));

  auto probeA = lineFromWKT<double>(
      "LINESTRING(0.007055 0.000076, 0.000170 0.005777, 0.007176 0.009889)");
  auto probeB =
      lineFromWKT<double>("LINESTRING(0.005821 0.003172, 0.004706 0.007698)");
  auto probeAM = lineFromWKTProj<double>(
      "LINESTRING(0.007055 0.000076, 0.000170 0.005777, 0.007176 0.009889)",
      util::geo::projectToWebMerc<double>);
  auto probeBM = lineFromWKTProj<double>(
      "LINESTRING(0.005821 0.003172, 0.004706 0.007698)",
      util::geo::projectToWebMerc<double>);

  TEST(
      util::geo::dist(XSortedLine<double>(probeA), XSortedLine<double>(probeB)),
      ==, approx(util::geo::dist(probeA, probeB)));
  TEST(util::geo::webMercWithinMeterDist(
           XSortedLine<double>(probeAM), XSortedLine<double>(probeBM),
           std::numeric_limits<double>::infinity()),
       <, 72.0);

  auto crossA =
      lineFromWKT<double>("LINESTRING(0.003543 0.007117, 0.008329 0.004223)");
  auto crossB =
      lineFromWKT<double>("LINESTRING(0.008663 0.000144, 0.007041 0.005866)");
  auto crossAM = lineFromWKTProj<double>(
      "LINESTRING(0.003543 0.007117, 0.008329 0.004223)",
      util::geo::projectToWebMerc<double>);
  auto crossBM = lineFromWKTProj<double>(
      "LINESTRING(0.008663 0.000144, 0.007041 0.005866)",
      util::geo::projectToWebMerc<double>);

  // regression test: previously, meterDist used the multipoint/multipoint
  // variant internally because of unintended template matching,
  // producing only the vertex-vertex distance!
  TEST(util::geo::dist(crossA, crossB), ==, approx(0.0));
  TEST(util::geo::webMercMeterDist(crossAM, crossBM), ==, approx(0.0));

  // same tests for XSorted variant
  auto xCrossA =
      lineFromWKT<double>("LINESTRING(0.008017 0.009147, 0.004189 0.000769)");
  auto xCrossB = lineFromWKT<double>(
      "LINESTRING(0.006176 0.002461, 0.004996 0.003114, 0.002804 0.009636)");
  auto xCrossAM = lineFromWKTProj<double>(
      "LINESTRING(0.008017 0.009147, 0.004189 0.000769)",
      util::geo::projectToWebMerc<double>);
  auto xCrossBM = lineFromWKTProj<double>(
      "LINESTRING(0.006176 0.002461, 0.004996 0.003114, 0.002804 "
      "0.009636)",
      util::geo::projectToWebMerc<double>);

  TEST(util::geo::dist(xCrossA, xCrossB), ==, approx(0.0));
  TEST(util::geo::dist(XSortedLine<double>(xCrossA),
                       XSortedLine<double>(xCrossB)),
       ==, approx(0.0));
  TEST(util::geo::webMercWithinMeterDist(
           XSortedLine<double>(xCrossAM), XSortedLine<double>(xCrossBM),
           std::numeric_limits<double>::infinity()),
       ==, approx(0.0));

  auto collA =
      lineFromWKT<double>("LINESTRING(0.003042 0.003894, 0.004151 0.009815)");
  auto collB = lineFromWKT<double>(
      "LINESTRING(0.003929 0.007946, 0.007529 0.009230, 0.004281 0.008896)");
  auto collAM = lineFromWKTProj<double>(
      "LINESTRING(0.003042 0.003894, 0.004151 0.009815)",
      util::geo::projectToWebMerc<double>);
  auto collBM = lineFromWKTProj<double>(
      "LINESTRING(0.003929 0.007946, 0.007529 0.009230, 0.004281 "
      "0.008896)",
      util::geo::projectToWebMerc<double>);

  TEST(util::geo::webMercWithinMeterDist(
           XSortedCollection<double>(collAM), XSortedCollection<double>(collBM),
           std::numeric_limits<double>::infinity()),
       ==,
       approx(haversineWebMerc(
           latLngToWebMerc(Point<double>{0.004151, 0.009815}),
           latLngToWebMerc(Point<double>{0.004281, 0.008896}))));

  // Regression test for webMercMeterDist
  auto webMercLineA = lineFromWKT<double>("LINESTRING(0 0, 10 0)");
  auto webMercLineB = lineFromWKT<double>("LINESTRING(20 0, 30 0)");
  TEST(util::geo::webMercMeterDist(webMercLineA, webMercLineB), ==,
       approx(10.0));
  TEST(util::geo::webMercMeterDist(webMercLineB, webMercLineA), ==,
       approx(10.0));

  // haversine on web mercator coordinates must match haversine on the
  // corresponding lat/lng coordinates
  std::vector<Point<double>> lngLats{
      {0, 0},          {10, 0},         {7.842, 47.998}, {-122.419, 37.775},
      {139.69, 35.69}, {-58.38, -34.6}, {24.94, 60.17},  {170, -80},
      {-179.9, 85.0},  {179.9, 85.0}};

  for (const auto& latLngA : lngLats) {
    for (const auto& latLngB : lngLats) {
      const auto a = latLngToWebMerc(latLngA);
      const auto b = latLngToWebMerc(latLngB);
      TEST(util::geo::haversineWebMerc(a, b), ==,
           approx(util::geo::haversine(latLngA, latLngB)));
    }
  }

  TEST(util::geo::haversineWebMerc(latLngToWebMerc(Point<double>{0, 0}),
                                   latLngToWebMerc(Point<double>{0, 0})),
       ==, approx(0.0));

  // symmetry
  TEST(
      util::geo::haversineWebMerc(latLngToWebMerc(Point<double>{7.842, 47.998}),
                                  latLngToWebMerc(Point<double>{9.18, 48.78})),
      ==,
      approx(util::geo::haversineWebMerc(
          latLngToWebMerc(Point<double>{9.18, 48.78}),
          latLngToWebMerc(Point<double>{7.842, 47.998}))));
}

// _____________________________________________________________________________
static void testDistLimitedPrecision() {
  // tests using geometries with very limited precision

  auto poly = polygonFromWKT<uint16_t>(
      "POLYGON((100 100, 100 1000, 1000 1000, 1000 100, 100 100))");
  auto poly2 = polygonFromWKT<uint16_t>(
      "POLYGON((425 425, 475 425, 475 475, 425 475, 425 425))");
  auto poly3 = polygonFromWKT<uint16_t>(
      "POLYGON((325 425, 475 425, 475 475, 425 475, 425 425))");
  auto poly4 = polygonFromWKT<uint16_t>(
      "POLYGON((225 225, 275 225, 275 275, 225 275, 225 225))");
  auto polyWithInner = polygonFromWKT<uint16_t>(
      "POLYGON((0 0, 1000 0, 1000 10, 0 1000, 0 0), (400 400, 500 400, 500 "
      "500, 400 500, 400 400))");
  auto multiPoly = multiPolygonFromWKT<uint16_t>(
      "MULTIPOLYGON(((425 425, 475 425, 475 475, 425 475, 425 "
      "425)), ((0 0, 1000 0, 1000 1000, 0 1000, 0 0), (400 400, 500 400, 500 "
      "500, 400 500, 400 400), (200 "
      "200, 300 200, 300 300, 200 300, 200 200)))");
  auto line = lineFromWKT<uint16_t>("LINESTRING(1000 450, 1200 450)");
  auto line2 = lineFromWKT<uint16_t>("LINESTRING(475 450, 427 450)");
  auto line3 = lineFromWKT<uint16_t>("LINESTRING(375 450, 427 450)");
  auto point = pointFromWKT<uint16_t>("POINT(450 450)");
  auto point2 = pointFromWKT<uint16_t>("POINT(1100 1100)");

  auto collection = collectionFromWKT<uint16_t>(
      "GEOMETRYCOLLECTION(MULTIPOLYGON(((425 425, 475 425, 475 475, "
      "425 475, 425 425)), ((0 0, 1000 0, 1000 1000, 0 1000, 0 0), (400 400, "
      "500 400, 500 "
      "500, 400 500, 400 400), (200 200, 300 200, 300 300, 200 300, 200 "
      "200))), POINT(2000 2000))");

  auto collection2 = collectionFromWKT<uint16_t>(
      "GEOMETRYCOLLECTION(MULTIPOINT(0 0, 100 100), POINT(200 200))");

  auto collection3 = collectionFromWKT<uint16_t>(
      "GEOMETRYCOLLECTION(POINT(400 400), POINT(300 300))");

  TEST(util::geo::dist(point, line), ==, approx(550));
  TEST(util::geo::dist(line, point), ==, approx(550));
  TEST(util::geo::dist(point2, poly), ==, approx(sqrt(2) * 100));
  TEST(util::geo::dist(poly, point2), ==, approx(sqrt(2) * 100));
  TEST(util::geo::dist(line, poly), ==, 0);
  TEST(util::geo::dist(poly, line), ==, 0);
  TEST(util::geo::dist(line, line2), ==, 525);
  TEST(util::geo::dist(line, line3), ==, 573);

  auto point_8 = pointFromWKT<uint8_t>("POINT(180 180)");
  auto point2_8 = pointFromWKT<uint8_t>("POINT(200 200)");
  TEST(util::geo::dist(point_8, point2_8), ==, approx(sqrt(20 * 20 * 2)));

  auto point_b = pointFromWKT<bool>("POINT(1 1)");
  auto point2_b = pointFromWKT<bool>("POINT(0 0)");
  TEST(util::geo::dist(point_b, point2_b), ==, approx(sqrt(2)));
}

// _____________________________________________________________________________
static void testDistSyntheticWebMerc() {
  double EXACT_TOL = 0.000000001;
  double SWEEP_TOL = 0.0002;

  std::mt19937 rnd(12345);
  std::vector<double> lats = {0, 20, 45, 60, 75, 84};
  std::vector<double> spans = {0.001, 0.05, 1.0, 8.0};
  std::vector<double> seps = {0.0, 0.02, 0.4, 3.0};

  for (double lat : lats) {
    for (double span : spans) {
      if (lat + span > 85.0) continue;

      for (double sep : seps) {
        for (int rep = 0; rep < 40; rep++) {
          std::uniform_real_distribution<double> distri(0.0, span);

          // vary the length of the lines
          size_t na = 2 + rnd() % 12, nb = 2 + rnd() % 12;

          Line<double> a, b;
          for (size_t i = 0; i < na; i++)
            a.push_back(latLngToWebMerc(Point<double>{distri(rnd), lat + distri(rnd)}));
          for (size_t i = 0; i < nb; i++)
            b.push_back(latLngToWebMerc(
                Point<double>{span + sep + distri(rnd), lat + distri(rnd)}));

          // note: the ref is not the real distance, as it only measures
          // distances at the vertexes. However, it is still a valid upper
          // bound, real distance cannot exceed it
          double ref = std::numeric_limits<double>::infinity();
          for (const auto& p : a)
            for (const auto& q : b)
              ref = std::min(ref, haversineWebMerc(p, q));

          TEST(util::geo::webMercMeterDist(a, b), <=, ref * (1.0 + EXACT_TOL));
          TEST(util::geo::webMercWithinMeterDist(
                   XSortedCollection<double>(a), XSortedCollection<double>(b),
                   std::numeric_limits<double>::infinity()),
               <=, ref * (1.0 + EXACT_TOL));
          TEST(util::geo::webMercWithinMeterDist(
                   XSortedLine<double>(a), XSortedLine<double>(b),
                   std::numeric_limits<double>::infinity()),
               <=, ref * (1.0 + SWEEP_TOL));
          TEST(util::geo::webMercWithinMeterDist(XSortedLine<double>(a),
                                                 XSortedLine<double>(b),
                                                 ref * 10.0 + 1000.0),
               <=, ref * (1.0 + SWEEP_TOL));
        }
      }
    }
  }
}

// _____________________________________________________________________________
static void testDistToSegmentExtreme() {
  {
    // true distance witness point is at the endpoint
    auto seg = lineFromWKTProj<double>(
        "LINESTRING(-60.0 85.0, -5.657479 -80.733430)",
        util::geo::projectToWebMerc<double>);
    auto p =
        latLngToWebMerc(Point<double>{60.0, 67.410787});

    double ref = haversineWebMerc(p, seg.front());
    TEST(ref, >, 2832051.0);
    TEST(ref, <, 2832052.0);

    // FAILS with a significant error!
    // TEST(util::geo::webMercMeterDist(p, seg), ==, approx(ref));
  }

  {
    // true distance witness point is neither endpoint not projection poitn
    auto seg = lineFromWKTProj<double>(
        "LINESTRING(0.094603 74.851933, 0.290393 77.089685)",
        util::geo::projectToWebMerc<double>);
    auto p = pointFromWKTProj<double>("POINT(7.902886 76.813064)",
                                            util::geo::projectToWebMerc<double>);

    double ref =
        haversineWebMerc(p, latLngToWebMerc(Point<double>{0.278391, 76.962378}));
    TEST(ref, >, 193127.0);
    TEST(ref, <, 193128.0);

    // FAILS with a small error
    // TEST(util::geo::webMercMeterDist(p, seg), ==, approx(ref));
  }

  {
    // another error class: lines spanning the antimeridian (datumsgrenze)
    auto seg = lineFromWKTProj<double>(
        "LINESTRING(-150.0 85.0, 98.805598 -72.825412)",
        util::geo::projectToWebMerc<double>);
    auto p = latLngToWebMerc(Point<double>{150.0, 84.994120});

    double ref = haversineWebMerc(p, seg.front());
    TEST(ref, >, 556393.0);
    TEST(ref, <, 556395.0);

    // FAILS with a huge error, as expected
    // TEST(util::geo::webMercMeterDist(p, seg), ==, approx(ref));
  }
}

// _____________________________________________________________________________
void GeoTest::testDist() {
  testDistCombinations();
  testDistWithinRealisticMaxDist();
  testDistWithinInfinityMaxDist();
  testDistWithinExactMaxDist();

  LargeTestGeoms large;

  testDistComplexGeoms(large);
  testDistHaversineNoPadding(large);
  testDistHaversineSmallPadding(large);
  testDistHaversineMeterDistPadding(large);

  testDistCollections();
  testDistOther();
  testDistLimitedPrecision();
  testDistSyntheticWebMerc();
  testDistToSegmentExtreme();
}
