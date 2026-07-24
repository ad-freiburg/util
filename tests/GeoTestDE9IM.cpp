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
void GeoTest::testDE9IM() {

  {
    util::geo::DE9IMatrix de9im1 = "FFFFFFFF2";
    util::geo::DE9IMatrix de9im2 = "FFFFFFFF2";
    util::geo::DE9IMatrix de9im3 = "F1FFFFFF2";

    TEST(de9im1, ==, de9im2);
    TEST(de9im1, !=, de9im3);

    TEST(de9im1.toString(), ==, "FFFFFFFF2");
    TEST(de9im2.toString(), ==, "FFFFFFFF2");
    TEST(de9im3.toString(), ==, "F1FFFFFF2");

    de9im3.setTo(2, D2);
    TEST(de9im3.toString(), ==, "F12FFFFF2");

    de9im3.setTo(7, D1);
    TEST(de9im3.toString(), ==, "F12FFFF12");

    de9im3.setTo(4, D0);
    TEST(de9im3.toString(), ==, "F12F0FF12");

    de9im3.II(D0);
    TEST(de9im3.II(), ==, D0);
    TEST(de9im3.toString(), ==, "012F0FF12");

    de9im3.BB(D2);
    TEST(de9im3.BB(), ==, D2);
    TEST(de9im3.toString(), ==, "012F2FF12");

    util::geo::DE9IMatrix de9im4 = "FFFFFFFF2";
    util::geo::DE9IMatrix de9im5 = "F0FFFFFF2";
    auto a = de9im4 + de9im5;
    TEST(a.toString(), ==, "F0FFFFFF2");

    util::geo::DE9IMatrix de9im6 = "000200002";

    auto c = de9im3 + de9im5;
    TEST(c.toString(), ==, "012F2FF12");

    auto d = de9im3 + de9im5 + de9im6;
    TEST(d.toString(), ==, "012220012");
  }

  {
    util::geo::DE9IMFilter f1 = "F12******";
    util::geo::DE9IMFilter f2 = "012******";
    util::geo::DE9IMFilter f3 = "012*****1";
    util::geo::DE9IMFilter f4 = "012**T**T";
    util::geo::DE9IMFilter f5 = "012TTTTTT";
    util::geo::DE9IMFilter f6 = "TTTTTTTTT";
    util::geo::DE9IMatrix de9im1 = "F12222222";
    util::geo::DE9IMatrix de9im2 = "012222222";

    TEST(de9im1 & f1);
    TEST(!(de9im2 & f1));
    TEST(!(de9im1 & f2));
    TEST(!(de9im1 & f3));
    TEST(!(de9im2 & f3));
    TEST(!(de9im1 & f4));
    TEST(!(de9im1 & f5));
    TEST(!(de9im1 & f6));
    TEST(de9im2 & f6);
    TEST(de9im2 & f2);
    TEST(de9im2 & f4);
    TEST(de9im2 & f5);

    TEST(util::geo::DE9IMatrix("FFFF1FFF2") &
         util::geo::DE9IMFilter("****T****"));
    TEST(!(util::geo::DE9IMatrix("FFFF1FFF2") &
           util::geo::DE9IMFilter("***T*****")));
    TEST(!(util::geo::DE9IMatrix("FFFF1FFF2") &
           util::geo::DE9IMFilter("***TT*****")));
    TEST((util::geo::DE9IMatrix("FFFF1FFF2") &
          util::geo::DE9IMFilter("****T****T")));
    TEST((util::geo::DE9IMatrix("FFFF1FFF2") &
          util::geo::DE9IMFilter("****T***2")));

    TEST((util::geo::DE9IMatrix("2FFF1FFF2") &
          util::geo::DE9IMFilter("T***T***2")));
    TEST(!(util::geo::DE9IMatrix("2FFF1FFF2") &
           util::geo::DE9IMFilter("TT**T***2")));
    TEST((util::geo::DE9IMatrix("20FF1FFF2") &
          util::geo::DE9IMFilter("TT**T***2")));
    TEST(!(util::geo::DE9IMatrix("20FF1F1F2") &
           util::geo::DE9IMFilter("TT**T*2*2")));
    TEST(!(util::geo::DE9IMatrix("20FF1F1F2") &
           util::geo::DE9IMFilter("TT**T*0*2")));
    TEST((util::geo::DE9IMatrix("20FF1F1F2") &
          util::geo::DE9IMFilter("TT**T*T*2")));
    TEST((util::geo::DE9IMatrix("20FF1F1F2") &
          util::geo::DE9IMFilter("TT**T*1*2")));

    TEST((util::geo::DE9IMatrix("20FF1F1F2") &
          util::geo::DE9IMFilter("20FF1F1F2")));
    TEST((util::geo::DE9IMatrix("20FF1F1F2") &
          util::geo::DE9IMFilter("2*FF1F1F2")));
    TEST((util::geo::DE9IMatrix("20FF1F1F2") &
          util::geo::DE9IMFilter("2*FF1F1*2")));

    TEST(!(util::geo::DE9IMatrix("20FF1F1F2") &
           util::geo::DE9IMFilter("20FF0F1F2")));
    TEST((util::geo::DE9IMatrix("222222222") &
          util::geo::DE9IMFilter("222222222")));
    TEST((util::geo::DE9IMatrix("222222222") &
          util::geo::DE9IMFilter("*********")));
    TEST((util::geo::DE9IMatrix("222222222") &
          util::geo::DE9IMFilter("TTTTTTTTT")));
    TEST((util::geo::DE9IMatrix("111111111") &
          util::geo::DE9IMFilter("TTTTTTTTT")));
    TEST((util::geo::DE9IMatrix("000000000") &
          util::geo::DE9IMFilter("TTTTTTTTT")));
    TEST(!(util::geo::DE9IMatrix("0000F0000") &
           util::geo::DE9IMFilter("TTTTTTTTT")));
    TEST(!(util::geo::DE9IMatrix("0F0F0F0F0") &
           util::geo::DE9IMFilter("TTTTTTTTT")));
    TEST((util::geo::DE9IMatrix("0F0F0F0F0") &
          util::geo::DE9IMFilter("TFTFTFTFT")));
    TEST(!(util::geo::DE9IMatrix("0F0F0F0F0") &
           util::geo::DE9IMFilter("FTFTFTFTF")));
    TEST(!(util::geo::DE9IMatrix("1F1F1F1F1") &
           util::geo::DE9IMFilter("TTTTTTTTT")));
    TEST((util::geo::DE9IMatrix("1F1F1F1F1") &
          util::geo::DE9IMFilter("TFTFTFTFT")));
    TEST(!(util::geo::DE9IMatrix("1F1F1F1F1") &
           util::geo::DE9IMFilter("FTFTFTFTF")));
    TEST(!(util::geo::DE9IMatrix("2F2F2F2F2") &
           util::geo::DE9IMFilter("TTTTTTTTT")));
    TEST((util::geo::DE9IMatrix("2F2F2F2F2") &
          util::geo::DE9IMFilter("TFTFTFTFT")));
    TEST(!(util::geo::DE9IMatrix("2F2F2F2F2") &
           util::geo::DE9IMFilter("FTFTFTFTF")));

    TEST(util::geo::DE9IMFilter("FTFTFTFT*").toString(), ==, "FTFTFTFT*");
    TEST(util::geo::DE9IMFilter("FTFTFTFT2").toString(), ==, "FTFTFTFT2");
    TEST(util::geo::DE9IMFilter("FTFTFTFT1").toString(), ==, "FTFTFTFT1");
    TEST(util::geo::DE9IMFilter("FTFTFTFT0").toString(), ==, "FTFTFTFT0");
    TEST(util::geo::DE9IMFilter("TTTTTTTTT").toString(), ==, "TTTTTTTTT");
    TEST(util::geo::DE9IMFilter("*********").toString(), ==, "*********");
    TEST(util::geo::DE9IMFilter("****1****").toString(), ==, "****1****");
    TEST(util::geo::DE9IMFilter("T*T*T*T*T").toString(), ==, "T*T*T*T*T");
  }

  {
    auto a = lineFromWKT<int>("LINESTRING(1 1, 2 2)");
    auto b = lineFromWKT<int>("LINESTRING(0 0, 4 4)");
    XSortedLine<int> ax(a);
    XSortedLine<int> bx(b);

    TEST(std::get<0>(geo::intersectsCovers(ax, bx)));
    TEST(std::get<1>(geo::intersectsCovers(ax, bx)));

    auto de9im = geo::DE9IM(ax, bx);
    TEST(de9im, ==, "1FF0FF102");

    de9im = geo::DE9IM(bx, ax);
    TEST(de9im, ==, "101FF0FF2");
  }
}
