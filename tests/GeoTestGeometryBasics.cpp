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
void GeoTest::testGeometryBasics() {
  // ___________________________________________________________________________
  {
    geo::Point<double> a(1, 2);
    geo::Point<double> b(2, 3);
    geo::Point<double> c(4, 5);
    TEST(a.getX(), ==, approx(1));
    TEST(a.getY(), ==, approx(2));

    a.setX(3);
    TEST(a.getX(), ==, approx(3));
    TEST(a.getY(), ==, approx(2));

    a.setY(4);
    TEST(a.getX(), ==, approx(3));
    TEST(a.getY(), ==, approx(4));

    auto d = a + b;
    TEST(d.getX(), ==, approx(5));
    TEST(d.getY(), ==, approx(7));

    a.setX(1);
    a.setY(2);

    TEST(geo::dist(a, a), ==, approx(0));
    TEST(geo::dist(a, b), ==, approx(sqrt(2)));

    d = d + d;

    geo::Box<double> box(a, c);
    TEST(geo::contains(a, box));
    TEST(geo::contains(b, box));
    TEST(geo::contains(c, box));
    TEST(!geo::contains(d, box));

    geo::Line<double> line{a, b, c};

    TEST(geo::contains(line, box));
    line.push_back(d);
    TEST(!geo::contains(line, box));

    geo::LineSegment<double> ls{a, b};
    TEST(geo::contains(a, ls));
    TEST(geo::contains(b, ls));
    TEST(!geo::contains(c, ls));
    TEST(geo::contains(a + geo::Point<double>(.5, .5), ls));
    TEST(!geo::contains(a + geo::Point<double>(1.5, 1.5), ls));

    geo::LineSegment<double> lsa{geo::Point<double>(1, 1),
                                 geo::Point<double>(2, 2)};
    geo::LineSegment<double> lsb{geo::Point<double>(1, 2),
                                 geo::Point<double>(2, 1)};
    geo::LineSegment<double> lsc{geo::Point<double>(2.1, 2),
                                 geo::Point<double>(3, 3)};

    TEST(geo::crossProd(lsa.first, lsb), ==, approx(-1));
    TEST(geo::crossProd(lsa.second, lsb), ==, approx(1));

    TEST(geo::intersects(lsa, lsb));

    TEST(!geo::intersects(lsa, lsa));
    TEST(!geo::intersects(lsb, lsb));
    TEST(!geo::intersects(lsa, lsc));

    TEST(!geo::intersects(geo::Point<double>(871569.2, 6104550.4),
                          geo::Point<double>(871581.2, 6104536),
                          geo::Point<double>(871580.3, 6104541.3),
                          geo::Point<double>(871625.7, 6104510.1)));

    TEST(!geo::intersects(geo::Point<double>(0, 0), geo::Point<double>(1, 1),
                          geo::Point<double>(0.5, 0.5),
                          geo::Point<double>(1.5, 1.5)));

    geo::Line<double> l{geo::Point<double>(1, 1), geo::Point<double>(2, 2),
                        geo::Point<double>(2, 4)};
    TEST(!geo::contains(geo::Point<double>(1, 2), l));
    TEST(geo::contains(geo::Point<double>(2, 2), l));
    TEST(geo::contains(geo::Point<double>(2, 3), l));

    geo::Box<double> bbox(geo::Point<double>(1, 1), geo::Point<double>(3, 3));
    TEST(geo::intersects(l, bbox));
    geo::Line<double> ll{geo::Point<double>(0, 0), geo::Point<double>(4, 4)};
    TEST(geo::intersects(ll, bbox));
    geo::Line<double> lll{geo::Point<double>(0, 0), geo::Point<double>(0, 4)};
    TEST(!geo::intersects(lll, bbox));
    geo::Line<double> llll{geo::Point<double>(1.2, 0),
                           geo::Point<double>(1, 2)};
    TEST(geo::intersects(llll, bbox));

    Line<double> l5new;
    l5new.push_back(Point<double>(-10, -5));
    l5new.push_back(Point<double>(-8, -4));
    TEST(geo::getBoundingBox(l5new).getUpperRight().getX(), ==, approx(-8));
    TEST(geo::getBoundingBox(l5new).getUpperRight().getY(), ==, approx(-4));

    Line<double> l5;
    l5.push_back(Point<double>(0, 0));
    l5.push_back(Point<double>(1.5, 2));
    Box<double> req(Point<double>(.5, 1), Point<double>(1, 1.5));

    TEST(geo::getBoundingBox(l5[0]).getLowerLeft().getX(), ==, approx(0));
    TEST(geo::getBoundingBox(l5[0]).getLowerLeft().getY(), ==, approx(0));

    TEST(geo::getBoundingBox(l5).getLowerLeft().getX(), ==, approx(0));
    TEST(geo::getBoundingBox(l5).getLowerLeft().getY(), ==, approx(0));
    TEST(geo::getBoundingBox(l5).getUpperRight().getX(), ==, approx(1.5));
    TEST(geo::getBoundingBox(l5).getUpperRight().getY(), ==, approx(2));
    TEST(geo::intersects(geo::getBoundingBox(l5),
                         geo::getBoundingBox(Line<double>{
                             Point<double>(.5, 1), Point<double>(1, 1)})));
    TEST(geo::intersects(
        l5, Line<double>{Point<double>(.5, 1), Point<double>(1, 1)}));
    TEST(geo::intersects(l5, req));

    Box<double> boxa(Point<double>(1, 1), Point<double>(2, 2));
    TEST(geo::intersects(
        boxa, Box<double>(Point<double>(1.5, 1.5), Point<double>(1.7, 1.7))));
    TEST(geo::intersects(
        boxa, Box<double>(Point<double>(0, 0), Point<double>(3, 3))));
    TEST(geo::intersects(
        boxa, Box<double>(Point<double>(1.5, 1.5), Point<double>(3, 3))));
    TEST(geo::intersects(
        boxa, Box<double>(Point<double>(0, 0), Point<double>(1.5, 1.5))));

    TEST(geo::intersects(
        Box<double>(Point<double>(1.5, 1.5), Point<double>(1.7, 1.7)), boxa));
    TEST(geo::intersects(Box<double>(Point<double>(0, 0), Point<double>(3, 3)),
                         boxa));
    TEST(geo::intersects(
        Box<double>(Point<double>(1.5, 1.5), Point<double>(3, 3)), boxa));
    TEST(geo::intersects(
        Box<double>(Point<double>(0, 0), Point<double>(1.5, 1.5)), boxa));

    Polygon<double> poly({Point<double>(1, 1), Point<double>(3, 2),
                          Point<double>(4, 3), Point<double>(6, 3),
                          Point<double>(5, 1)});
    TEST(geo::getWKT(poly), ==, "POLYGON((5 1,6 3,4 3,3 2,1 1,5 1))");
    TEST(geo::contains(Point<double>(4, 2), poly));
    TEST(!geo::contains(Point<double>(3, 3), poly));
    TEST(geo::contains(Point<double>(1, 1), poly));
    TEST(geo::contains(Point<double>(3, 2), poly));
    TEST(geo::contains(Point<double>(4, 3), poly));
    TEST(geo::contains(Point<double>(6, 3), poly));
    TEST(geo::contains(Point<double>(5, 1), poly));

    TEST(geo::contains(Line<double>{Point<double>(6, 3), Point<double>(5, 1)},
                       poly));
    TEST(!geo::contains(Line<double>{Point<double>(6, 3), Point<double>(50, 1)},
                        poly));
    TEST(geo::contains(Line<double>{Point<double>(4, 2), Point<double>(4.5, 2)},
                       poly));
    TEST(geo::contains(Line<double>{Point<double>(4, 2), Point<double>(5, 1)},
                       poly));

    Box<double> polybox(Point<double>(1, 1), Point<double>(6, 4));
    TEST(geo::centroid(polybox).getX(), ==, approx(3.5));
    TEST(geo::centroid(polybox).getY(), ==, approx(2.5));
    TEST(geo::contains(poly, polybox));
    TEST(!geo::contains(polybox, poly));
    Box<double> polybox2(Point<double>(4, 1), Point<double>(5, 2));
    TEST(geo::contains(polybox2, poly));
    TEST(geo::contains(poly, getBoundingBox(poly)));

    Point<double> rotP(2, 2);
    TEST(geo::dist(geo::rotate(rotP, 180, Point<double>(1, 1)),
                   Point<double>(0, 0)) == approx(0));
    TEST(geo::dist(geo::rotate(rotP, 360, Point<double>(1, 1)), rotP) ==
         approx(0));

    Line<double> rotLine({{1, 1}, {3, 3}});
    TEST(geo::rotate(rotLine, 90, Point<double>(2, 2))[0].getX(), ==,
         approx(1));
    TEST(geo::rotate(rotLine, 90, Point<double>(2, 2))[0].getY(), ==,
         approx(3));
    TEST(geo::rotate(rotLine, 90, Point<double>(2, 2))[1].getX(), ==,
         approx(3));
    TEST(geo::rotate(rotLine, 90, Point<double>(2, 2))[1].getY(), ==,
         approx(1));

    MultiLine<double> multiRotLine({{{1, 1}, {3, 3}}, {{1, 3}, {3, 1}}});
    TEST(geo::rotate(multiRotLine, 90, Point<double>(2, 2))[0][0].getX() ==
         approx(1));
    TEST(geo::rotate(multiRotLine, 90, Point<double>(2, 2))[0][0].getY() ==
         approx(3));
    TEST(geo::rotate(multiRotLine, 90, Point<double>(2, 2))[0][1].getX() ==
         approx(3));
    TEST(geo::rotate(multiRotLine, 90, Point<double>(2, 2))[0][1].getY() ==
         approx(1));
    TEST(geo::rotate(multiRotLine, 90, Point<double>(2, 2))[1][0].getX() ==
         approx(3));
    TEST(geo::rotate(multiRotLine, 90, Point<double>(2, 2))[1][0].getY() ==
         approx(3));
    TEST(geo::rotate(multiRotLine, 90, Point<double>(2, 2))[1][1].getX() ==
         approx(1));
    TEST(geo::rotate(multiRotLine, 90, Point<double>(2, 2))[1][1].getY() ==
         approx(1));

    TEST(geo::getWKT(multiRotLine) == "MULTILINESTRING((1 1,3 3),(1 3,3 1))");

    TEST(geo::contains(
        multiRotLine[0],
        geo::move(geo::move(multiRotLine, 1.0, 2.0), -1.0, -2.0)[0]));
    TEST(geo::contains(multiRotLine, geo::getBoundingBox(Line<double>{
                                         {1, 1}, {3, 3}, {1, 3}, {3, 1}})));

    TEST(geo::contains(
        getBoundingBox(multiRotLine),
        geo::getBoundingBox(Line<double>{{1, 1}, {3, 3}, {1, 3}, {3, 1}})));
    TEST(geo::contains(
        geo::getBoundingBox(Line<double>{{1, 1}, {3, 3}, {1, 3}, {3, 1}}),
        getBoundingBox(multiRotLine)));

    TEST(geo::dist(geo::centroid(rotP), rotP), ==, approx(0));
    TEST(geo::dist(geo::centroid(rotLine), rotP), ==, approx(0));
    TEST(geo::dist(geo::centroid(polybox), Point<double>(3.5, 2.5)) ==
         approx(0));
    TEST(geo::dist(geo::centroid(Polygon<double>({{0, 0}, {3, 4}, {4, 3}})),
                   Point<double>(7.0 / 3.0, 7.0 / 3.0)) == approx(0));

    auto polyy = Polygon<double>({{0, 0}, {3, 4}, {4, 3}});
    MultiPolygon<double> mpoly{polyy, polyy};

    TEST(geo::getWKT(polyy), ==, "POLYGON((4 3,3 4,0 0,4 3))");
    TEST(geo::getWKT(mpoly) ==
         "MULTIPOLYGON(((4 3,3 4,0 0,4 3)),((4 3,3 4,0 0,4 3)))");

    TEST(geo::getWKT(geo::centroid(mpoly)) ==
         geo::getWKT(geo::centroid(polyy)));

    auto hull = geo::convexHull(Line<double>{
        {0.1, 3}, {1, 1}, {2, 2}, {4, 4}, {0, 0}, {1, 2}, {3, 1}, {3, 3}});
    TEST(hull.getOuter().size(), ==, size_t(4));
    TEST(geo::contains(geo::convexHull(geo::getBoundingBox(poly)),
                       geo::getBoundingBox(poly)));
    TEST(geo::contains(geo::getBoundingBox(poly),
                       geo::convexHull(geo::getBoundingBox(poly))));

    auto hull2 = geo::convexHull(Line<double>{{0.1, 3},
                                              {1, 1},
                                              {2, 2},
                                              {4, 4},
                                              {0, 0},
                                              {1, 2},
                                              {3, 1},
                                              {3, 3},
                                              {-0.1, 1}});
    TEST(hull2.getOuter().size(), ==, size_t(5));
    TEST(hull2.getOuter()[4].getX(), ==, approx(-.1));
    TEST(hull2.getOuter()[4].getY(), ==, approx(1));
    TEST(hull2.getOuter()[3].getX(), ==, approx(0.1));
    TEST(hull2.getOuter()[3].getY(), ==, approx(3));
    TEST(hull2.getOuter()[2].getX(), ==, approx(4));
    TEST(hull2.getOuter()[2].getY(), ==, approx(4));
    TEST(hull2.getOuter()[1].getX(), ==, approx(3));
    TEST(hull2.getOuter()[1].getY(), ==, approx(1));
    TEST(hull2.getOuter()[0].getX(), ==, approx(0));
    TEST(hull2.getOuter()[0].getY(), ==, approx(0));

    auto hull3 =
        geo::convexHull(Line<double>{{0.1, 3}, {4, 4}, {0, 0}, {1, 2}, {3, 1}});
    TEST(hull3.getOuter().size(), ==, size_t(4));
    TEST(hull3.getOuter()[0].getX(), ==, approx(3));
    TEST(hull3.getOuter()[0].getY(), ==, approx(1));
    TEST(hull3.getOuter()[1].getX(), ==, approx(4));
    TEST(hull3.getOuter()[1].getY(), ==, approx(4));
    TEST(hull3.getOuter()[2].getX(), ==, approx(0.1));
    TEST(hull3.getOuter()[2].getY(), ==, approx(3));
    TEST(hull3.getOuter()[3].getX(), ==, approx(0));
    TEST(hull3.getOuter()[3].getY(), ==, approx(0));

    hull3 = geo::convexHull(
        Line<double>{{0.1, 3}, {4, 4}, {2, 1}, {3, 2}, {0, 0}, {1, 2}, {3, 1}});
    TEST(hull3.getOuter().size(), ==, size_t(4));
    TEST(hull3.getOuter()[0].getX(), ==, approx(3));
    TEST(hull3.getOuter()[0].getY(), ==, approx(1));
    TEST(hull3.getOuter()[1].getX(), ==, approx(4));
    TEST(hull3.getOuter()[1].getY(), ==, approx(4));
    TEST(hull3.getOuter()[2].getX(), ==, approx(0.1));
    TEST(hull3.getOuter()[2].getY(), ==, approx(3));
    TEST(hull3.getOuter()[3].getX(), ==, approx(0));
    TEST(hull3.getOuter()[3].getY(), ==, approx(0));

    hull3 = geo::convexHull(Line<double>{
        {4, 4}, {1, 2}, {2, 1}, {3, 2}, {0.1, 3}, {0, 0}, {1, 2}, {3, 1}});
    TEST(hull3.getOuter().size(), ==, size_t(4));
    TEST(hull3.getOuter()[0].getX(), ==, approx(3));
    TEST(hull3.getOuter()[0].getY(), ==, approx(1));
    TEST(hull3.getOuter()[1].getX(), ==, approx(4));
    TEST(hull3.getOuter()[1].getY(), ==, approx(4));
    TEST(hull3.getOuter()[2].getX(), ==, approx(0.1));
    TEST(hull3.getOuter()[2].getY(), ==, approx(3));
    TEST(hull3.getOuter()[3].getX(), ==, approx(0));
    TEST(hull3.getOuter()[3].getY(), ==, approx(0));

    hull3 = geo::convexHull(Line<double>{{4, 4}, {1, 2}, {3, 1}});
    TEST(hull3.getOuter().size(), ==, size_t(3));
    TEST(hull3.getOuter()[0].getX(), ==, approx(3));
    TEST(hull3.getOuter()[0].getY(), ==, approx(1));
    TEST(hull3.getOuter()[1].getX(), ==, approx(4));
    TEST(hull3.getOuter()[1].getY(), ==, approx(4));
    TEST(hull3.getOuter()[2].getX(), ==, approx(1));
    TEST(hull3.getOuter()[2].getY(), ==, approx(2));

    hull3 = geo::convexHull(Line<double>{{4, 4}, {1, 2}, {3, 10}});
    TEST(hull3.getOuter().size(), ==, size_t(3));
    TEST(hull3.getOuter()[0].getX(), ==, approx(4));
    TEST(hull3.getOuter()[0].getY(), ==, approx(4));
    TEST(hull3.getOuter()[1].getX(), ==, approx(3));
    TEST(hull3.getOuter()[1].getY(), ==, approx(10));
    TEST(hull3.getOuter()[2].getX(), ==, approx(1));
    TEST(hull3.getOuter()[2].getY(), ==, approx(2));

    Line<double> test{{0.3215348546593775, 0.03629583077160248},
                      {0.02402358131857918, -0.2356728797179394},
                      {0.04590851212470659, -0.4156409924995536},
                      {0.3218384001607433, 0.1379850698988746},
                      {0.11506479756447, -0.1059521474930943},
                      {0.2622539999543261, -0.29702873322836},
                      {-0.161920957418085, -0.4055339716426413},
                      {0.1905378631228002, 0.3698601009043493},
                      {0.2387090918968516, -0.01629827079949742},
                      {0.07495888748668034, -0.1659825110491202},
                      {0.3319341836794598, -0.1821814101954749},
                      {0.07703635755650362, -0.2499430638271785},
                      {0.2069242999022122, -0.2232970760420869},
                      {0.04604079532068295, -0.1923573186549892},
                      {0.05054295812784038, 0.4754929463150845},
                      {-0.3900589168910486, 0.2797829520700341},
                      {0.3120693385713448, -0.0506329867529059},
                      {0.01138812723698857, 0.4002504701728471},
                      {0.009645149586391732, 0.1060251100976254},
                      {-0.03597933197019559, 0.2953639456959105},
                      {0.1818290866742182, 0.001454397571696298},
                      {0.444056063372694, 0.2502497166863175},
                      {-0.05301752458607545, -0.06553921621808712},
                      {0.4823896228171788, -0.4776170002088109},
                      {-0.3089226845734964, -0.06356112199235814},
                      {-0.271780741188471, 0.1810810595574612},
                      {0.4293626522918815, 0.2980897964891882},
                      {-0.004796652127799228, 0.382663812844701},
                      {0.430695573269106, -0.2995073500084759},
                      {0.1799668387323309, -0.2973467472915973},
                      {0.4932166845474547, 0.4928094162538735},
                      {-0.3521487911717489, 0.4352656197131292},
                      {-0.4907368011686362, 0.1865826865533206},
                      {-0.1047924716070224, -0.247073392148198},
                      {0.4374961861758457, -0.001606279519951237},
                      {0.003256207800708899, -0.2729194320486108},
                      {0.04310378203457577, 0.4452604050238248},
                      {0.4916198379282093, -0.345391701297268},
                      {0.001675087028811806, 0.1531837672490476},
                      {-0.4404289572876217, -0.2894855991839297}

    };
    hull3 = geo::convexHull(test);
    TEST(geo::contains(test, hull3));
    TEST(hull3.getOuter().size(), ==, size_t(8));
    TEST(geo::contains(
        Polygon<double>({{-0.161920957418085, -0.4055339716426413},
                         {0.05054295812784038, 0.4754929463150845},
                         {0.4823896228171788, -0.4776170002088109},
                         {0.4932166845474547, 0.4928094162538735},
                         {-0.3521487911717489, 0.4352656197131292},
                         {-0.4907368011686362, 0.1865826865533206},
                         {0.4916198379282093, -0.345391701297268},
                         {-0.4404289572876217, -0.2894855991839297}}),
        hull3));
    TEST(geo::contains(
        hull3, Polygon<double>({{-0.161920957418085, -0.4055339716426413},
                                {0.05054295812784038, 0.4754929463150845},
                                {0.4823896228171788, -0.4776170002088109},
                                {0.4932166845474547, 0.4928094162538735},
                                {-0.3521487911717489, 0.4352656197131292},
                                {-0.4907368011686362, 0.1865826865533206},
                                {0.4916198379282093, -0.345391701297268},
                                {-0.4404289572876217, -0.2894855991839297}})));

    hull3 = geo::convexHull(Line<double>{{3, 6},
                                         {8, 10},
                                         {3, 5},
                                         {20, -10},
                                         {-4, 5},
                                         {10, 2},
                                         {5, 1},
                                         {45, 1},
                                         {30, -9},
                                         {3, 14},
                                         {25, -5.5}});
    TEST(hull3.getOuter().size(), ==, size_t(5));
    TEST(hull3.getOuter()[0].getX(), ==, approx(20));
    TEST(hull3.getOuter()[0].getY(), ==, approx(-10));
    TEST(hull3.getOuter()[1].getX(), ==, approx(30));
    TEST(hull3.getOuter()[1].getY(), ==, approx(-9));
    TEST(hull3.getOuter()[2].getX(), ==, approx(45));
    TEST(hull3.getOuter()[2].getY(), ==, approx(1));
    TEST(hull3.getOuter()[3].getX(), ==, approx(3));
    TEST(hull3.getOuter()[3].getY(), ==, approx(14));
    TEST(hull3.getOuter()[4].getX(), ==, approx(-4));
    TEST(hull3.getOuter()[4].getY(), ==, approx(5));

    hull3 = geo::convexHull(Line<double>{
        {7, 7}, {7, -7}, {-7, -7}, {-7, 7}, {9, 0}, {-9, 0}, {0, 9}, {0, -9}});
    TEST(hull3.getOuter().size(), ==, size_t(8));
    TEST(geo::contains(geo::Polygon<double>({{-9, 0},
                                             {-7, -7},
                                             {0, -9},
                                             {7, -7},
                                             {9, 0},
                                             {7, 7},
                                             {0, 9},
                                             {-7, 7}}),
                       hull3));
    TEST(geo::contains(hull3, geo::Polygon<double>({{-9, 0},
                                                    {-7, -7},
                                                    {0, -9},
                                                    {7, -7},
                                                    {9, 0},
                                                    {7, 7},
                                                    {0, 9},
                                                    {-7, 7}})));

    hull3 = geo::convexHull(Line<double>{{7, 7},
                                         {7, -7},
                                         {-7, -7},
                                         {-7, 7},
                                         {9, 0},
                                         {-9, 0},
                                         {0, 9},
                                         {0, -9},
                                         {0, 0},
                                         {1, 2},
                                         {-2, 1},
                                         {-1, -1},
                                         {3, 4},
                                         {4, 3},
                                         {-5, 4},
                                         {6, 5}});
    TEST(hull3.getOuter().size(), ==, size_t(8));
    TEST(geo::contains(geo::Polygon<double>({{-9, 0},
                                             {-7, -7},
                                             {0, -9},
                                             {7, -7},
                                             {9, 0},
                                             {7, 7},
                                             {0, 9},
                                             {-7, 7}}),
                       hull3));
    TEST(geo::contains(hull3, geo::Polygon<double>({{-9, 0},
                                                    {-7, -7},
                                                    {0, -9},
                                                    {7, -7},
                                                    {9, 0},
                                                    {7, 7},
                                                    {0, 9},
                                                    {-7, 7}})));

    hull3 = geo::convexHull(Line<double>{
        {0, 0},   {1, 2},  {-2, 1}, {-1, -1}, {3, 4},   {4, 3},   {-5, 4},
        {6, 5},   {7, 7},  {7, -7}, {-7, -7}, {-7, 7},  {9, 0},   {-9, 0},
        {0, 9},   {0, -9}, {-8, 0}, {8, 0},   {-7, 0},  {7, 0},   {-6, 0},
        {6, 0},   {-5, 0}, {5, 0},  {-4, 0},  {4, 0},   {-3, 0},  {3, 0},
        {-2, 0},  {2, 0},  {-1, 0}, {1, 0},   {0, -8},  {0, 8},   {0, -7},
        {0, 7},   {0, -6}, {0, 6},  {0, -5},  {0, 5},   {0, -4},  {0, 4},
        {0, -3},  {0, 3},  {0, -2}, {0, 2},   {0, -1},  {0, 1},   {1, 1},
        {2, 2},   {3, 3},  {4, 4},  {5, 5},   {6, 6},   {1, -1},  {2, -2},
        {3, -3},  {4, -4}, {5, -5}, {6, -6},  {-1, 1},  {-2, 2},  {-3, 3},
        {-4, 4},  {-5, 5}, {-6, 6}, {-1, -1}, {-2, -2}, {-3, -3}, {-4, -4},
        {-5, -5}, {-6, -6}});
    TEST(hull3.getOuter().size(), ==, size_t(8));
    TEST(geo::contains(geo::Polygon<double>({{-9, 0},
                                             {-7, -7},
                                             {0, -9},
                                             {7, -7},
                                             {9, 0},
                                             {7, 7},
                                             {0, 9},
                                             {-7, 7}}),
                       hull3));
    TEST(geo::contains(hull3, geo::Polygon<double>({{-9, 0},
                                                    {-7, -7},
                                                    {0, -9},
                                                    {7, -7},
                                                    {9, 0},
                                                    {7, 7},
                                                    {0, 9},
                                                    {-7, 7}})));

    TEST(geo::area(geo::Point<double>(1, 2)), ==, approx(0));
    TEST(geo::area(geo::Line<double>{{1, 2}, {2, 5}}), ==, approx(0));
    TEST(geo::area(geo::Box<double>({0, 0}, {1, 1})), ==, approx(1));
    TEST(geo::area(geo::Box<double>({1, 1}, {1, 1})), ==, approx(0));
    TEST(geo::area(geo::Box<double>({0, 0}, {2, 2})), ==, approx(4));
    TEST(geo::area(geo::Polygon<double>({{0, 0}, {1, 0}, {1, 1}, {0, 1}})) ==
         approx(1));
    TEST(geo::area(geo::Polygon<double>({{0, 0}, {1, 0}, {1, 1}})) ==
         approx(0.5));

    auto obox =
        geo::getOrientedEnvelope(geo::Line<double>{{0, 0}, {1, 1}, {1.5, 0.5}});
    // TEST(geo::contains(
    // geo::Line<double>{{0, 0}, {1, 1}, {1.5, 0.5}}, geo::convexHull(obox)));
    TEST(geo::area(geo::convexHull(obox)), ==, approx(1));

    obox = geo::getOrientedEnvelope(geo::Polygon<double>({{0.0, 1.0},
                                                          {0.9, 1.1},
                                                          {1.0, 2.0},
                                                          {1.1, 1.1},
                                                          {2.0, 1.0},
                                                          {1.1, 0.9},
                                                          {1.0, 0.0},
                                                          {0.9, 0.9}}));
    // TEST(geo::contains(
    // geo::Polygon<double>({{0.0, 1.0}, {0.9, 1.1},
    // {1.0, 2.0}, {1.1, 1.1},
    // {2.0, 1.0}, {1.1, 0.9},
    // {1.0, 0.0}, {0.9, 0.9}}), geo::convexHull(obox)));
    TEST(geo::area(geo::convexHull(obox)), ==, approx(2));

    obox = geo::getOrientedEnvelope(std::vector<geo::Polygon<double>>(
        {geo::Polygon<double>({{0.0, 1.0},
                               {0.9, 1.1},
                               {1.0, 2.0},
                               {1.1, 1.1},
                               {2.0, 1.0},
                               {1.1, 0.9},
                               {1.0, 0.0},
                               {0.9, 0.9}})}));

    TEST(geo::dist(geo::LineSegment<double>{{1, 1}, {3, 1}},
                   geo::LineSegment<double>{{2, 2}, {2, 0}}) == approx(0));
    TEST(geo::dist(geo::LineSegment<double>{{1, 1}, {3, 1}},
                   geo::LineSegment<double>{{2, 4}, {2, 2}}) == approx(1));
    TEST(geo::dist(geo::LineSegment<double>{{1, 1}, {3, 1}},
                   geo::LineSegment<double>{{1, 1}, {3, 1}}) == approx(0));
    TEST(geo::dist(geo::LineSegment<double>{{1, 1}, {3, 1}},
                   geo::LineSegment<double>{{1, 2}, {3, 2}}) == approx(1));
    TEST(geo::dist(geo::LineSegment<double>{{1, 1}, {3, 1}},
                   geo::LineSegment<double>{{1, 2}, {3, 5}}) == approx(1));

    TEST(geo::dist(geo::Line<double>{{1, 1}, {3, 1}},
                   geo::Point<double>{2, 1}) == approx(0));
    TEST(geo::dist(geo::Line<double>{{1, 1}, {3, 1}},
                   geo::Point<double>{2, 2}) == approx(1));
    TEST(geo::dist(geo::Line<double>{{1, 1}, {3, 1}},
                   geo::Point<double>{3, 1}) == approx(0));
    TEST(geo::dist(geo::Line<double>{{1, 1}, {3, 1}},
                   geo::Point<double>{1, 1}) == approx(0));

    TEST(geo::dist(Line<double>{{7, 7},
                                {7, -7},
                                {-7, -7},
                                {-7, 7},
                                {9, 0},
                                {-9, 0},
                                {0, 9},
                                {0, -9}},
                   Line<double>{{7, 7},
                                {7, -7},
                                {-7, -7},
                                {-7, 7},
                                {9, 0},
                                {-9, 0},
                                {0, 9},
                                {0, -9}}) == approx(0));
    TEST(geo::dist(Line<double>{{7, 7},
                                {7, -7},
                                {-7, -7},
                                {-7, 7},
                                {9, 0},
                                {-9, 0},
                                {0, 9},
                                {0, -9}},
                   LineSegment<double>{{6, 7}, {8, -7}}) == approx(0));
    TEST(geo::dist(Line<double>{{7, 7},
                                {7, -7},
                                {-7, -7},
                                {-7, 7},
                                {9, 0},
                                {-9, 0},
                                {0, 9},
                                {0, -9}},
                   Point<double>{7, 4}) == approx(0));
    TEST(geo::dist(Line<double>{{0, 0}, {1, 1}, {2, 0}},
                   Line<double>{{1.5, 0.5}, {1.5, 100}}) == approx(0));
    TEST(geo::dist(Line<double>{{0, 0}, {1, 1}, {2, 0}},
                   Line<double>{{2, 0.5}, {2, 100}}) == approx(0.353553));

    TEST(geo::contains(util::geo::Point<double>{1.5, 0.5},
                       util::geo::LineSegment<double>{{1, 1}, {1.5, 0.5}}));
    TEST(geo::contains(util::geo::Point<double>{1.5, 0.5},
                       util::geo::LineSegment<double>{{1, 1}, {1.5, 0.5}}));

    auto polyTest =
        geo::Polygon<double>({{1, 1}, {3, 1}, {2, 2}, {3, 3}, {1, 3}});
    TEST(!geo::contains(util::geo::LineSegment<double>({2.5, 1.3}, {2.5, 2.6}),
                        polyTest));

    TEST(!geo::contains(util::geo::LineSegment<double>{{2.5, 1.3}, {2.5, 2.6}},
                        polyTest));
    TEST(geo::contains(util::geo::LineSegment<double>{{2.5, 2.6}, {1.5, 2}},
                       polyTest));
    TEST(!geo::contains(
        util::geo::Line<double>{{2.5, 1.3}, {2.5, 2.6}, {1.5, 2}}, polyTest));
    TEST(geo::contains(
        util::geo::Line<double>{{2.5, 1.3}, {1.5, 2}, {2.5, 2.6}}, polyTest));

    TEST(!geo::contains(util::geo::Box<double>{{1, 1}, {2.5, 2.6}}, polyTest));

    TEST(
        geo::intersects(Box<double>(Point<double>(0, 0), Point<double>(10, 10)),
                        Box<double>(Point<double>(2, 2), Point<double>(8, 8))));
    TEST(geo::intersects(
        Box<double>(Point<double>(0, 0), Point<double>(10, 10)),
        Box<double>(Point<double>(-2, -2), Point<double>(8, 8))));
    TEST(geo::intersects(
        Box<double>(Point<double>(0, 0), Point<double>(10, 10)),
        Box<double>(Point<double>(-2, -2), Point<double>(12, 12))));
    TEST(geo::intersects(
        Box<double>(Point<double>(0, 0), Point<double>(10, 10)),
        Box<double>(Point<double>(5, 5), Point<double>(12, 12))));

    TEST(!geo::intersects(
        Box<double>(Point<double>(0, 0), Point<double>(10, 10)),
        Box<double>(Point<double>(15, 15), Point<double>(12, 12))));

    double rad = 10.0;
    int n = 20;
    util::geo::MultiPoint<double> mp;

    for (int i = 0; i < n; i++) {
      double x = rad * cos((2.0 * M_PI / static_cast<double>(n)) *
                           static_cast<double>(i));
      double y = rad * sin((2.0 * M_PI / static_cast<double>(n)) *
                           static_cast<double>(i));

      mp.push_back(util::geo::DPoint(x, y));
    }

    auto h = util::geo::convexHull(mp);

    TEST(geo::contains(mp, h));

    TEST(geo::dist(geo::interpolate(DPoint(1.0, 1.0), DPoint(1.0, 3.0), 0.5),
                   DPoint(1.0, 2.0)),
         ==, approx(0));
    TEST(geo::dist(geo::interpolate(DPoint(1.0, 1.0), DPoint(2.0, 2.0), 0),
                   DPoint(1.0, 1.0)),
         ==, approx(0));
    TEST(geo::dist(geo::interpolate(DPoint(1.0, 1.0), DPoint(2.0, 2.0), 0.5),
                   DPoint(1.5, 1.5)),
         ==, approx(0));
    TEST(geo::dist(geo::interpolate(DPoint(1.0, 1.0), DPoint(2.0, 2.0), 1),
                   DPoint(2, 2)),
         ==, approx(0));
    TEST(geo::dist(geo::pointAtDist(DLine{{0, 0}, {0, 1}, {0, 2}}, 1),
                   DPoint{0, 1}),
         ==, approx(0));
    TEST(geo::dist(geo::pointAtDist(DLine{{0, 0}, {0, 1}, {0, 2}}, 2),
                   DPoint{0, 2}),
         ==, approx(0));
    TEST(geo::dist(geo::pointAtDist(DLine{{0, 0}, {0, 1}, {0, 2}}, 0),
                   DPoint{0, 0}),
         ==, approx(0));

    TEST(geo::getWKT(geo::orthoLineAtDist(DLine{{0, 0}, {0, 1}, {0, 2}}, 1, 1)),
         ==, "LINESTRING(-0.5 1,0.5 1)");

    TEST(geo::getWKT(geo::segment(DLine{{0, 0}, {0, 1}, {0, 2}}, 0, 0.5)), ==,
         "LINESTRING(0 0,0 1)");
    TEST(geo::getWKT(geo::segment(DLine{{0, 0}, {0, 1}, {0, 2}}, 0.5, 1)), ==,
         "LINESTRING(0 1,0 2)");
  }
}
