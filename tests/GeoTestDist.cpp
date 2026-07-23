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
void GeoTest::testDist() {

  {
    // dist between all possible combinations, including multigeometries

    auto poly =
        polygonFromWKT<double>("POLYGON((1 1, 1 10, 10 10, 10 1, 1 1))");
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

    TEST(util::geo::dist(MultiLine<double>{line}, MultiLine<double>{line2, line3}), ==, approx(std::min(util::geo::dist(line, line2), util::geo::dist(line, line3))));

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

    // with realistic max dist, SPLIT HERE

    TEST(util::geo::withinDist(point, XSortedLine<double>(line), 10.0), ==,
         approx(5.5));
    TEST(util::geo::withinDist(XSortedLine<double>(line), point, 10.0), ==,
         approx(5.5));

    TEST(util::geo::withinDist(XSortedPolygon<double>(poly), point2, 10.0), ==,
         approx(sqrt(2)));
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

    TEST(util::geo::withinDist(XSortedPolygon<double>(poly2),
                               XSortedPolygon<double>(polyWithInner), 10.0),
         ==, approx(0.25));
    TEST(util::geo::withinDist(XSortedPolygon<double>(polyWithInner),
                               XSortedPolygon<double>(poly2), 10.0),
         ==, approx(0.25));

    // with infinity max dist, SPLIT HERE

    TEST(util::geo::dist(point, XSortedLine<double>(line)), ==,
         approx(5.5));
    TEST(util::geo::dist(XSortedLine<double>(line), point), ==,
         approx(5.5));

    TEST(util::geo::dist(XSortedPolygon<double>(poly), point2), ==,
         approx(sqrt(2)));
    TEST(util::geo::dist(point2, XSortedPolygon<double>(poly)), ==,
         approx(sqrt(2)));

    TEST(util::geo::dist(XSortedLine<double>(line),
                               XSortedPolygon<double>(poly)),
         ==, 0);
    TEST(util::geo::dist(XSortedPolygon<double>(poly),
                               XSortedLine<double>(line)),
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

    // with exact max dist, SPLIT HERE

    TEST(util::geo::withinDist(point, XSortedLine<double>(line), 5.5), ==,
         approx(5.5));
    TEST(util::geo::withinDist(XSortedLine<double>(line), point, 5.5), ==,
         approx(5.5));

    TEST(util::geo::withinDist(XSortedPolygon<double>(poly), point2, sqrt(2)),
         ==, approx(sqrt(2)));
    TEST(util::geo::withinDist(point2, XSortedPolygon<double>(poly), sqrt(2)),
         ==, approx(sqrt(2)));

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

    TEST(util::geo::withinDist(XSortedMultiPolygon<double>(MultiPolygon<double>{poly2}),
                               XSortedMultiPolygon<double>(MultiPolygon<double>{polyWithInner}), 0.25),
         ==, approx(0.25));

    TEST(util::geo::withinDist(XSortedPolygon<double>(polyWithInner),
                               XSortedPolygon<double>(poly2), 0.25),
         ==, approx(0.25));

    TEST(util::geo::withinDist(XSortedMultiPolygon<double>(MultiPolygon<double>{polyWithInner}),
                               XSortedMultiPolygon<double>(MultiPolygon<double>{poly2}), 0.25),
         ==, approx(0.25));

    TEST(util::geo::withinDist(XSortedMultiPolygon<double>(MultiPolygon<double>{polyWithInner}),
                               XSortedMultiPolygon<double>(MultiPolygon<double>{polyWithInner}), 0.25),
         ==, approx(0));

    TEST(util::geo::withinDist(XSortedMultiLine<double>(MultiLine<double>{line}), XSortedMultiLine<double>(MultiLine<double>{line2, line3}), 10), ==, approx(std::min(util::geo::dist(line, line2), util::geo::dist(line, line3))));
    TEST(util::geo::withinDist(XSortedMultiPolygon<double>(MultiPolygon<double>{polyWithInner}), XSortedMultiLine<double>(MultiLine<double>{line2, line3}), 10), ==, approx(std::min(util::geo::dist(polyWithInner, line2), util::geo::dist(polyWithInner, line3))));
    TEST(util::geo::withinDist(XSortedMultiLine<double>(MultiLine<double>{line2, line3}), XSortedMultiPolygon<double>(MultiPolygon<double>{polyWithInner}), 10), ==, approx(std::min(util::geo::dist(polyWithInner, line2), util::geo::dist(polyWithInner, line3))));
    TEST(util::geo::withinDist(XSortedCollection<double>(MultiPoint<double>{point, point2}), XSortedCollection<double>(MultiPoint<double>{point3}), 20), ==, approx(std::min(util::geo::dist(point, point3), util::geo::dist(point2, point3))));
    TEST(util::geo::withinDist(XSortedCollection<double>(MultiLine<double>{line}), XSortedCollection<double>(MultiPoint<double>{point, point2}), 20), ==, approx(std::min(util::geo::dist(MultiLine<double>{line}, point), util::geo::dist(MultiLine<double>{line}, point2))));
    TEST(util::geo::withinDist(XSortedMultiPolygon<double>(MultiPolygon<double>{polyWithInner}), XSortedCollection<double>(MultiPoint<double>{point, point2}), 20), ==, approx(std::min(util::geo::dist(MultiPolygon<double>{polyWithInner}, point), util::geo::dist(MultiPolygon<double>{polyWithInner}, point2))));

    // complex geoms, SPLIT HERE

    std::ifstream germanyF(std::string(TEST_DATASETS) + "/germany.tsv", std::ios::binary);
    std::string germanyWKT((std::istreambuf_iterator<char>(germanyF)), {});
    auto germany = util::geo::multiPolygonFromWKT<float>(germanyWKT);

    std::ifstream spainF(std::string(TEST_DATASETS) + "/spain.tsv", std::ios::binary);
    std::string spainWKT((std::istreambuf_iterator<char>(spainF)), {});
    auto spain = util::geo::multiPolygonFromWKT<float>(spainWKT);

    std::ifstream saimaaF(std::string(TEST_DATASETS) + "/saimaa.tsv", std::ios::binary);
    std::string saimaaWKT((std::istreambuf_iterator<char>(saimaaF)), {});
    // careful, saimaa is not a MULTIPOLYGON
    auto saimaa = util::geo::polygonFromWKT<float>(saimaaWKT);

    std::ifstream flixbusF(std::string(TEST_DATASETS) + "/flixbus.tsv", std::ios::binary);
    std::string flixbusWKT((std::istreambuf_iterator<char>(flixbusF)), {});
    auto flixbus = util::geo::collectionFromWKT<float>(flixbusWKT);

    auto spainX = XSortedMultiPolygon<float>(spain);
    auto germanyX = XSortedMultiPolygon<float>(germany);
    auto saimaaX = XSortedMultiPolygon<float>(saimaa);
    auto flixbusX = XSortedCollection<float>(flixbus);

    TEST(util::geo::withinDist(germanyX, spainX, 10),
         ==, approx(6.5434));
    TEST(util::geo::withinDist(germanyX, spainX, 6.54341),
         ==, approx(6.5434));
    TEST(util::geo::dist(germanyX, spainX),
         ==, approx(6.5434));
    TEST(util::geo::meterDist(germanyX, spainX),
         ==, approx(653276.48232));

    TEST(util::geo::withinDist(germany, spain, 10),
         ==, approx(6.5434));
    TEST(util::geo::withinDist(germany, spain, 6.54341),
         ==, approx(6.5434));
    TEST(util::geo::dist(germany, spain),
         ==, approx(6.5434));
    TEST(util::geo::meterDist(germany, spain),
         ==, approx(653276.48232));

    TEST(util::geo::withinDist(germany, germany, 10),
         ==, approx(0));
    TEST(util::geo::withinDist(germany, germany, 0),
         ==, approx(0));
    TEST(util::geo::dist(germany, germany),
         ==, approx(0));
    TEST(util::geo::meterDist(germany, germany),
         ==, approx(0));

    TEST(util::geo::withinDist(spain, spain, 10),
         ==, approx(0));
    TEST(util::geo::withinDist(spain, spain, 0),
         ==, approx(0));
    TEST(util::geo::dist(spain, spain),
         ==, approx(0));
    TEST(util::geo::meterDist(spain, spain),
         ==, approx(0));

    TEST(util::geo::withinDist(germanyX, germanyX, 10),
         ==, approx(0));
    TEST(util::geo::withinDist(germanyX, germanyX, 0),
         ==, approx(0));
    TEST(util::geo::dist(germanyX, germanyX),
         ==, approx(0));
    TEST(util::geo::meterDist(germanyX, germanyX),
         ==, approx(0));

    TEST(util::geo::withinDist(spainX, spainX, 10),
         ==, approx(0));
    TEST(util::geo::withinDist(spainX, spainX, 0),
         ==, approx(0));
    TEST(util::geo::dist(spainX, spainX),
         ==, approx(0));
    TEST(util::geo::meterDist(spainX, spainX),
         ==, approx(0));

    TEST(util::geo::withinDist(germanyX, flixbusX, 10),
         ==, approx(0));
    TEST(util::geo::withinDist(germanyX, flixbusX, 0),
         ==, approx(0));
    TEST(util::geo::dist(germanyX, flixbusX),
         ==, approx(0));
    TEST(util::geo::meterDist(germanyX, flixbusX),
         ==, approx(0));

    TEST(util::geo::withinDist(germany, flixbus, 10),
         ==, approx(0));
    TEST(util::geo::withinDist(germany, flixbus, 0),
         ==, approx(0));
    TEST(util::geo::dist(germany, flixbus),
         ==, approx(0));
    TEST(util::geo::meterDist(germany, flixbus),
         ==, approx(0));

    TEST(util::geo::withinDist(flixbus, flixbus, 10),
         ==, approx(0));
    TEST(util::geo::withinDist(flixbus, flixbus, 0),
         ==, approx(0));
    TEST(util::geo::dist(flixbus, flixbus),
         ==, approx(0));
    TEST(util::geo::meterDist(flixbus, flixbus),
         ==, approx(0));

    TEST(util::geo::withinDist(flixbusX, flixbusX, 10),
         ==, approx(0));
    TEST(util::geo::withinDist(flixbusX, flixbusX, 0),
         ==, approx(0));
    TEST(util::geo::dist(flixbusX, flixbusX),
         ==, approx(0));
    TEST(util::geo::meterDist(flixbusX, flixbusX),
         ==, approx(0));

    TEST(util::geo::withinDist(spainX, flixbusX, 10),
         ==, approx(7.00409));
    TEST(util::geo::withinDist(spainX, flixbusX, 7.004091),
         ==, approx(7.00409));
    TEST(util::geo::dist(spainX, flixbusX),
         ==, approx(7.00409));
    TEST(util::geo::meterDist(spainX, flixbusX),
         ==, approx(703461.43896));

    TEST(util::geo::withinDist(spain, flixbus, 10),
         ==, approx(7.00409));
    TEST(util::geo::withinDist(spain, flixbus, 7.004091),
         ==, approx(7.00409));
    TEST(util::geo::dist(spain, flixbus),
         ==, approx(7.00409));
    TEST(util::geo::meterDist(spain, flixbus),
         ==, approx(703461.43896));

    // with haversine, but without any search padding, SPLIT HERE
    TEST(std::round(util::geo::withinDist(germanyX, spainX, 1000000, defaultPaddingFunc<float>(), 100, [](const Point<float> a, const Point<float> b, double) -> double { return haversine(a, b); }) * 10.0) / 10.0,
         ==, approx(655421.6));
    TEST(std::round(util::geo::withinDist(germanyX, germanyX, 1000000, defaultPaddingFunc<float>(), 100, [](const Point<float> a, const Point<float> b, double) -> double { return haversine(a, b); }) * 10.0) / 10.0,
         ==, approx(0));
    TEST(std::round(util::geo::withinDist(germanyX, saimaaX, 10000000, defaultPaddingFunc<float>(), 100, [](const Point<float> a, const Point<float> b, double) -> double { return haversine(a, b); }) * 10.0) / 10.0,
         ==, approx(1087168.4));

    TEST(std::round(util::geo::withinDist(XSortedCollection<float>(MultiPolygon<float>{germany}), XSortedCollection<float>(Collection<float>{spain, saimaa}), 10000000, defaultPaddingFunc<float>(), 100, [](const Point<float> a, const Point<float> b, double) -> double { return haversine(a, b); }) * 10.0) / 10.0,
         ==, approx(655421.6));

    TEST(std::round(util::geo::withinDist(germanyX, XSortedCollection<float>(Collection<float>{spain, saimaa}), 10000000, defaultPaddingFunc<float>(), 100, [](const Point<float> a, const Point<float> b, double) -> double { return haversine(a, b); }) * 10.0) / 10.0,
         ==, approx(655421.6));

    TEST(std::round(util::geo::withinDist(XSortedCollection<float>(Collection<float>{spain, saimaa}), germanyX, 10000000, defaultPaddingFunc<float>(), 100, [](const Point<float> a, const Point<float> b, double) -> double { return haversine(a, b); }) * 10.0) / 10.0,
         ==, approx(655421.6));

    // with haversine, but with small search padding, SPLIT HERE
    TEST(std::round(util::geo::withinDist(germanyX, spainX, 1000000, [](double d, double, const Box<float>&,
                            const Box<float>&) -> double { return 1.02 * d; }, 100, [](const Point<float> a, const Point<float> b, double) -> double { return haversine(a, b); }) * 10.0) / 10.0,
         ==, approx(653276.5));
    TEST(std::round(util::geo::withinDist(germanyX, flixbusX, 1000000, [](double d, double, const Box<float>&,
                            const Box<float>&) -> double { return 1.02 * d; }, 100, [](const Point<float> a, const Point<float> b, double) -> double { return haversine(a, b); }) * 10.0) / 10.0,
         ==, approx(0));

    TEST(std::round(util::geo::withinDist(spainX, flixbusX, 1000000, [](double d, double, const Box<float>&,
                            const Box<float>&) -> double { return 1.02 * d; }, 100, [](const Point<float> a, const Point<float> b, double) -> double { return haversine(a, b); }) * 10.0) / 10.0,
         ==, approx(703461.4));

    // with haversine, automatic padding via meterDist, SPLIT HERE
    TEST(std::round(util::geo::meterDist(germanyX, spainX) * 10.0) / 10.0, ==, approx(653276.5));
    TEST(std::round(util::geo::meterDist(germanyX, flixbusX) * 10.0) / 10.0, ==, approx(0));
    TEST(std::round(util::geo::meterDist(spainX, flixbusX) * 10.0) / 10.0,  ==, approx(703461.4));
    // takes too long
    // TEST(std::round(util::geo::meterDist(germanyX, XSortedCollection<float>(Collection<float>{spain, saimaa})) * 10.0) / 10.0, ==, approx(653276.5));
    // TEST(std::round(util::geo::meterDist(XSortedCollection<float>(Collection<float>{spain, saimaa}), germanyX) * 10.0) / 10.0, ==, approx(653276.5));

    // collections, SPLIT HERE
    Collection<double> col, col2;
    col.push_back(line);
    col.push_back(point);
    col2.push_back(line3);
    TEST(util::geo::withinDist(XSortedCollection<double>(col), XSortedCollection<double>(col2), 20), ==, approx(.23));

    Collection<double> col3, col4;
    col3.push_back(polyWithInner);
    col4.push_back(line2);
    TEST(util::geo::withinDist(XSortedCollection<double>(col3), XSortedCollection<double>(col4), 20), ==, approx(.25));
    TEST(util::geo::dist(XSortedCollection<double>(col3), XSortedCollection<double>(col4)), ==, approx(.25));

    col4.push_back(poly);

    TEST(util::geo::withinDist(XSortedCollection<double>(col3), XSortedCollection<double>(col4), 20), ==, approx(0));
    TEST(util::geo::dist(XSortedCollection<double>(col3), XSortedCollection<double>(col4)), ==, approx(0));

    TEST(util::geo::withinDist(XSortedCollection<double>(), XSortedCollection<double>(col4), 20), >, 20);
    TEST(util::geo::dist(XSortedCollection<double>(), XSortedCollection<double>(col4)), >, 20);

    TEST(util::geo::withinDist(XSortedCollection<double>(), XSortedCollection<double>(), 20), >, 20);
    TEST(util::geo::dist(XSortedCollection<double>(), XSortedCollection<double>()), >, 20);

    TEST(util::geo::withinDist(XSortedCollection<double>(col), XSortedCollection<double>(col), 20), ==, approx(0));
    TEST(util::geo::dist(XSortedCollection<double>(col), XSortedCollection<double>(col)), ==, approx(0));

    TEST(util::geo::withinDist(XSortedCollection<double>(col2), XSortedCollection<double>(col2), 20), ==, approx(0));
    TEST(util::geo::dist(XSortedCollection<double>(col2), XSortedCollection<double>(col2)), ==, approx(0));

    TEST(util::geo::withinDist(XSortedCollection<double>(col3), XSortedCollection<double>(col3), 20), ==, approx(0));
    TEST(util::geo::dist(XSortedCollection<double>(col3), XSortedCollection<double>(col3)), ==, approx(0));

    TEST(util::geo::withinDist(XSortedCollection<double>(col4), XSortedCollection<double>(col4), 20), ==, approx(0));
    TEST(util::geo::dist(XSortedCollection<double>(col4), XSortedCollection<double>(col4)), ==, approx(0));

    TEST(util::geo::withinDist(col4, col4, 20), ==, approx(0));
    TEST(util::geo::dist(col4, col4), ==, approx(0));

    TEST(util::geo::withinDist(XSortedCollection<double>(col), XSortedLine<double>(line3), 20), ==, approx(.23));
    TEST(util::geo::dist(XSortedCollection<double>(col), XSortedLine<double>(line3)), ==, approx(.23));

    TEST(util::geo::withinDist(XSortedLine<double>(line3), XSortedCollection<double>(col), 20), ==, approx(.23));
    TEST(util::geo::dist(XSortedLine<double>(line3), XSortedCollection<double>(col)), ==, approx(.23));

    TEST(util::geo::withinDist(XSortedCollection<double>(col), XSortedPolygon<double>(poly), 20), ==, approx(0));
    TEST(util::geo::dist(XSortedCollection<double>(col), XSortedPolygon<double>(poly)), ==, approx(0));

    TEST(util::geo::withinDist(XSortedPolygon<double>(poly), XSortedCollection<double>(col), 20), ==, approx(0));
    TEST(util::geo::dist(XSortedPolygon<double>(poly), XSortedCollection<double>(col)), ==, approx(0));

    TEST(util::geo::withinDist(XSortedCollection<double>(col), point, 20), ==, approx(0));
    TEST(util::geo::dist(XSortedCollection<double>(col), point), ==, approx(0));

    TEST(util::geo::withinDist(point, XSortedCollection<double>(col), 20), ==, approx(0));
    TEST(util::geo::dist(point, XSortedCollection<double>(col)), ==, approx(0));

    // other tests, SPLIT HERE

    // standard point/point
    TEST(util::geo::withinDist(point, point2, 1), >, 1);
    TEST(util::geo::withinDist(point2, point, 1), >, 1);

    TEST(util::geo::dist(point, point2), ==, approx(9.19239));
    TEST(util::geo::dist(point2, point), ==, approx(9.19239));

    TEST(util::geo::meterDist(point, point2), ==, approx(haversine(point, point2)));
    TEST(util::geo::meterDist(point2, point), ==, approx(haversine(point, point2)));

    // standard point/line
    TEST(util::geo::withinDist(point2, line2, 100), ==, approx(9.01734));
    TEST(util::geo::withinDist(line, point, 100), ==, approx(5.5));

    TEST(util::geo::dist(point2, line2), ==, approx(9.01734));
    TEST(util::geo::dist(line, point), ==, approx(5.5));

    TEST(util::geo::meterDist(point2, line2), ==, approx(999138.32522));
    TEST(util::geo::meterDist(line, point), ==, approx(610368.37082));

    // standard point/polygon
    TEST(util::geo::withinDist(point, polyWithInner, 1), ==, approx(0.5));
    TEST(util::geo::withinDist(polyWithInner, point, 1), ==, approx(0.5));

    TEST(util::geo::dist(point, polyWithInner), ==, approx(0.5));
    TEST(util::geo::dist(polyWithInner, point), ==, approx(0.5));

    TEST(util::geo::meterDist(point, polyWithInner), ==, approx(78579.58257));
    TEST(util::geo::meterDist(polyWithInner, point), ==, approx(78579.58257));

    // standard line/polygon
    TEST(util::geo::withinDist(line2, polyWithInner, 100), ==, approx(0.25));
    TEST(util::geo::withinDist(polyWithInner, line2, 100), ==, approx(0.25));

    TEST(util::geo::dist(line2, polyWithInner), ==, approx(0.25));
    TEST(util::geo::dist(polyWithInner, line2), ==, approx(0.25));

    TEST(util::geo::meterDist(line2, polyWithInner), ==, approx(4.5));
    TEST(util::geo::meterDist(polyWithInner, line2), ==, approx(4.5));

    {
    // tests using geometries with very limited precision, SPLIT HERE
    auto poly =
        polygonFromWKT<uint16_t>("POLYGON((100 100, 100 1000, 1000 1000, 1000 100, 100 100))");
    auto poly2 = polygonFromWKT<uint16_t>(
        "POLYGON((425 425, 475 425, 475 475, 425 475, 425 425))");
    auto poly3 = polygonFromWKT<uint16_t>(
        "POLYGON((325 425, 475 425, 475 475, 425 475, 425 425))");
    auto poly4 = polygonFromWKT<uint16_t>(
        "POLYGON((225 225, 275 225, 275 275, 225 275, 225 225))");
    auto polyWithInner = polygonFromWKT<uint16_t>(
        "POLYGON((0 0, 1000 0, 1000 10, 0 1000, 0 0), (400 400, 500 400, 500 500, 400 500, 400 400))");
    auto multiPoly = multiPolygonFromWKT<uint16_t>(
        "MULTIPOLYGON(((425 425, 475 425, 475 475, 425 475, 425 "
        "425)), ((0 0, 1000 0, 1000 1000, 0 1000, 0 0), (400 400, 500 400, 500 500, 400 500, 400 400), (200 "
        "200, 300 200, 300 300, 200 300, 200 200)))");
    auto line = lineFromWKT<uint16_t>("LINESTRING(1000 450, 1200 450)");
    auto line2 = lineFromWKT<uint16_t>("LINESTRING(475 450, 427 450)");
    auto line3 = lineFromWKT<uint16_t>("LINESTRING(375 450, 427 450)");
    auto point = pointFromWKT<uint16_t>("POINT(450 450)");
    auto point2 = pointFromWKT<uint16_t>("POINT(1100 1100)");

    auto collection = collectionFromWKT<uint16_t>(
        "GEOMETRYCOLLECTION(MULTIPOLYGON(((425 425, 475 425, 475 475, "
        "425 475, 425 425)), ((0 0, 1000 0, 1000 1000, 0 1000, 0 0), (400 400, 500 400, 500 "
        "500, 400 500, 400 400), (200 200, 300 200, 300 300, 200 300, 200 200))), POINT(2000 2000))");

    auto collection2 = collectionFromWKT<uint16_t>(
        "GEOMETRYCOLLECTION(MULTIPOINT(0 0, 100 100), POINT(200 200))");

    auto collection3 =
        collectionFromWKT<uint16_t>("GEOMETRYCOLLECTION(POINT(400 400), POINT(300 300))");

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
  }
}
