// Copyright 2026, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

#include <algorithm>
#include <fstream>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "util/BlockReader.h"
#include "util/Misc.h"
#include "util/Test.h"
#include "util/tests/BlockReaderTest.h"

using util::BlockReader;

namespace {

struct Elem {
  uint64_t key;
  uint32_t id;
};

bool operator<(const Elem& e, uint64_t key) { return e.key < key; }

template <typename T>
std::string writeTmp(const std::vector<T>& elems, size_t prefix) {
  char fname[] = "/tmp/blockreadertestXXXXXX";
  int fd = mkstemp(fname);
  if (fd < 0) throw std::runtime_error("Could not create temporary file");
  close(fd);

  std::ofstream f(fname, std::ios::binary);
  std::string garbage(prefix, 'x');
  f.write(garbage.data(), garbage.size());
  if (!elems.empty()) {
    f.write(reinterpret_cast<const char*>(elems.data()),
            elems.size() * sizeof(T));
  }
  return fname;
}

// returns the result from std::lower_bound for testing
size_t refLowerBound(const std::vector<Elem>& elems, size_t i, uint64_t k) {
  if (i >= elems.size()) return elems.size();
  return std::lower_bound(elems.begin() + i, elems.end(), k) - elems.begin();
}

}  // namespace

// _____________________________________________________________________________
void BlockReaderTest::run() {
  {
    std::mt19937_64 rng(1);
    size_t sizes[] = {0, 1, 2, 7, 8, 9, 100, 1000};
    size_t blockSizes[] = {1, 2, 3, 8, 64, 4096};

    for (size_t n : sizes) {
      std::vector<Elem> elems;
      uint64_t key = 5;
      while (elems.size() < n) {
        key += rng() % 3;  // also 0, equal keys
        size_t run = rng() % 10 == 0 ? 1 + rng() % 20 : 1;
        for (size_t k = 0; k < run && elems.size() < n; k++) {
          Elem e;
          e.key = key;
          e.id = static_cast<uint32_t>(elems.size());
          elems.push_back(e);
        }
      }

      std::string fname = writeTmp(elems, 13);

      for (size_t bs : blockSizes) {
        BlockReader<Elem> r(bs);
        r.open(fname, 13, n);
        TEST(r.isOpen());
        TEST(r.size(), ==, n);

        BlockReader<Elem>::Cursor c = r.cursor();

        for (size_t i = 0; i < n; i++) TEST(c.at(i).id, ==, i);
        for (size_t i = n; i > 0; i--) TEST(c.at(i - 1).id, ==, i - 1);
        for (size_t k = 0; k < n; k++) {
          size_t i = rng() % n;
          TEST(c.at(i).key, ==, elems[i].key);
        }

        uint64_t maxKey = n ? elems.back().key + 2 : 3;
        for (size_t i = 0; i <= n + 1; i++) {
          for (uint64_t k = 0; k <= maxKey; k++) {
            TEST(c.lowerBound(i, k), ==, refLowerBound(elems, i, k));
            TEST(util::lowerBound(elems, i, k), ==, refLowerBound(elems, i, k));
          }
        }

        BlockReader<Elem>::Cursor c2 = r.cursor();
        size_t j = 0;
        for (uint64_t k = 0; k <= maxKey; k += 1 + rng() % 7) {
          j = c2.lowerBound(j, k);
          TEST(j, ==, refLowerBound(elems, 0, k));
          if (j < n) TEST(c2.at(j).key, >=, k);
        }
      }

      unlink(fname.c_str());
    }
  }

  {
    std::vector<Elem> elems(10);
    for (size_t i = 0; i < elems.size(); i++) elems[i].key = i;
    std::string fname = writeTmp(elems, 0);

    {
      BlockReader<Elem> r;
      TEST_THROWS(std::runtime_error,
                  r.open("/tmp/blockreadertest-nirvana", 0, 1));
      TEST(!r.isOpen());
    }

    {
      BlockReader<Elem> r(4);
      TEST_THROWS(std::runtime_error, r.open(fname, 0, 11));
      TEST(!r.isOpen());
      TEST(r.size(), ==, 0);
    }

    {
      BlockReader<Elem> r(4);
      r.open(fname, 0, elems.size());
      BlockReader<Elem>::Cursor c = r.cursor();
      TEST_THROWS(std::out_of_range, c.at(10));
      TEST(c.at(9).key, ==, 9);
    }

    unlink(fname.c_str());
  }
}
