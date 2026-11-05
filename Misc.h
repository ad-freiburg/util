// Copyright 2017, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#ifndef UTIL_MISC_H_
#define UTIL_MISC_H_

#define FMT_HEADER_ONLY

#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <type_traits>
#include <vector>
#ifdef PBUTIL_ZLIB_FOUND
#include <zlib.h>
#endif
#ifdef PBUTIL_BZLIB_FOUND
#include <bzlib.h>
#endif
#include <sys/types.h>

#include "JobQueue.h"

static const size_t SORT_BUFFER_S = 64 * 128 * 1024;

#define UNUSED(expr) \
  do {               \
    (void)(expr);    \
  } while (0)
#define TIME() std::chrono::high_resolution_clock::now()
#define TOOK_UNTIL(t1, t2) \
  (std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count())
#define TOOK(t1)                                         \
  (std::chrono::duration_cast<std::chrono::nanoseconds>( \
       std::chrono::high_resolution_clock::now() - t1)   \
       .count())
#define T_START(n) auto _tstart_##n = std::chrono::high_resolution_clock::now()
#define T_STOP(n)                                               \
  (std::chrono::duration_cast<std::chrono::nanoseconds>(        \
       std::chrono::high_resolution_clock::now() - _tstart_##n) \
       .count())

#define TODO(msg)                                                     \
  std::cerr << "\n" __FILE__ << ":" << __LINE__ << ": TODO: " << #msg \
            << std::endl;

#if defined(_WIN32)
#include <psapi.h>
#include <windows.h>
#elif defined(__unix__) || defined(__unix) || defined(unix) || \
    (defined(__APPLE__) && defined(__MACH__))
#include <sys/resource.h>
#include <unistd.h>
#if defined(__APPLE__) && defined(__MACH__)
#include <mach/mach.h>
#elif (defined(_AIX) || defined(__TOS__AIX__)) || \
    (defined(__sun__) || defined(__sun) ||        \
     defined(sun) && (defined(__SVR4) || defined(__svr4__)))
#include <fcntl.h>
#include <procfs.h>
#elif defined(__linux__) || defined(__linux) || defined(linux) || \
    defined(__gnu_linux__)
#include <stdio.h>
#endif
#else
#error "Cannot define getPeakRSS( ) or getCurrentRSS( ) for an unknown OS."
#endif

namespace util {

struct SortJob {
  SortJob() : part(0), partbuf(0) {}
  SortJob(size_t part, unsigned char* partbuf) : part(part), partbuf(partbuf) {}
  size_t part = 0;
  unsigned char* partbuf = 0;
};

inline bool operator==(const SortJob& a, const SortJob& b) {
  return a.part == b.part && a.partbuf == b.partbuf;
}

inline bool operator!=(const SortJob& a, const SortJob& b) { return !(a == b); }

struct hashPair {
  template <class T1, class T2>
  size_t operator()(const std::pair<T1, T2>& p) const {
    auto h1 = std::hash<T1>{}(p.first);
    auto h2 = std::hash<T2>{}(p.second);
    return h1 ^ h2;
  }
};

template <typename Key, typename Val, Val Def>
class SparseMatrix {
 public:
  Val get(const Key& x, const Key& y) const {
    auto a = _m.find(std::pair<Key, Key>(x, y));
    if (a == _m.end()) return Def;
    return a->second;
  }

  void set(Key x, Key y, Val v) { _m[std::pair<Key, Key>(x, y)] = v; }

  const std::map<std::pair<Key, Key>, Val>& vals() const { return _m; }

 private:
  std::map<std::pair<Key, Key>, Val> _m;
};

// _____________________________________________________________________________
inline uint64_t atoul(const char* p) {
  uint64_t ret = 0;

  while (*p) {
    ret = ret * 10 + (*p++ - '0');
  }

  return ret;
}

// _____________________________________________________________________________
inline uint64_t factorial(uint64_t n) {
  if (n < 2) return 1;
  return n * factorial(n - 1);
}

// _____________________________________________________________________________
inline bool isFloatingPoint(const std::string& str) {
  std::stringstream ss(str);
  double f;
  ss >> std::noskipws >> f;
  return ss.eof() && !ss.fail();
}

// _____________________________________________________________________________
inline double atof(const char* p, uint8_t mn, const char** end) {
  // this atof implementation works only on "normal" float strings like
  // 56.445, -345.00 or 1.5e-3, but should be faster than std::atof. Different
  // to std::atof, it returns NaN if p does not start with a number (if it
  // starts with a number any non-nuermic suffix is simply ignored!)
  static const double atoi['9' + 1] = {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  static const double pow10[23] = {
      1e0,  1e1,  1e2,  1e3,  1e4,  1e5,  1e6,  1e7,  1e8,  1e9,  1e10, 1e11,
      1e12, 1e13, 1e14, 1e15, 1e16, 1e17, 1e18, 1e19, 1e20, 1e21, 1e22};
  const char* start = p;
  while ((*p == ' ' || *p == '\n' || *p == '\t' || *p == '\r')) p++;

  double ret = 0.0;
  int exp = 0;
  bool neg = false;
  bool digits = false;
  if (*p == '-') {
    neg = true;
    p++;
  }

  if (*p == '+') {
    p++;
  }

  while (*p >= '0' && *p <= '9') {
    ret = ret * 10.0 + atoi[static_cast<unsigned char>(*p)];
    p++;
    digits = true;
  }

  // NOTE: i tried combining this loop with the one above (they only differ in
  // exp--), but checking whether we already read the decimal point was more
  // expensive than the 2 separate loops
  if (*p == '.') {
    p++;
    uint8_t n = 0;
    while (n < mn && *p >= '0' && *p <= '9') {
      ret = ret * 10.0 + atoi[static_cast<unsigned char>(*p)];
      exp--;
      n++;
      p++;
      digits = true;
    }

    // skip digits beyond the prec
    while (*p >= '0' && *p <= '9') p++;
  }

  if (!digits) {
    *end = start;
    return std::numeric_limits<double>::quiet_NaN();
  }

  // exponent notation
  if (*p == 'e' || *p == 'E') {
    const char* q = p + 1;
    // we will check negativity later by looking at p again, see below
    if (*q == '-' || *q == '+') q++;

    if (*q >= '0' && *q <= '9') {
      int e = 0;
      while (*q >= '0' && *q <= '9') {
        if (e < 10000) e = e * 10 + (*q - '0');
        // must continue until end for correct return of end point...
        q++;
      }

      exp += p[1] == '-' ? -e : e;
      p = q;
    }
  }

  // apply the decimal exponent, in steps of 22 (the largest 10 power which is
  // exact)
  while (exp > 22) {
    ret *= 1e22;
    exp -= 22;
  }
  while (exp < -22) {
    ret /= 1e22;
    exp += 22;
  }
  if (exp < 0) ret /= pow10[-exp];
  if (exp > 0) ret *= pow10[exp];

  *end = p;
  if (neg) return -ret;
  return ret;
}

// _____________________________________________________________________________
inline double atof(const char* p, uint8_t mn) {
  const char* end;
  return atof(p, mn, &end);
}

// _____________________________________________________________________________
inline double atof(const char* p) { return atof(p, 38); }

ssize_t preadAll(int file, unsigned char* buf, size_t count, size_t offset);

#ifdef PBUTIL_ZLIB_FOUND
ssize_t zreadAll(gzFile file, unsigned char* buf, size_t count);
#endif

#ifdef PBUTIL_BZLIB_FOUND
ssize_t bz2readAll(BZFILE* file, unsigned char* buf, size_t count);
#endif

ssize_t readAll(int file, unsigned char* buf, size_t count);

ssize_t pwriteAll(int file, const unsigned char* buf, size_t count,
                  size_t offset);

ssize_t writeAll(int file, const unsigned char* buf, size_t count);

// Taken from
// https://stackoverflow.com/questions/41049143/
template <typename T, typename A = std::allocator<T>>
class default_init_allocator : public A {
  typedef std::allocator_traits<A> a_t;

 public:
  using A::A;  // Inherit constructors from A

  template <typename U>
  struct rebind {
    using other =
        default_init_allocator<U, typename a_t::template rebind_alloc<U>>;
  };

  template <typename U>
  void construct(U* ptr) noexcept(
      std::is_nothrow_default_constructible<U>::value) {
    ::new (static_cast<void*>(ptr)) U;
  }

  template <typename U, typename... Args>
  void construct(U* ptr, Args&&... args) {
    a_t::construct(static_cast<A&>(*this), ptr, std::forward<Args>(args)...);
  }
};

template <typename T, typename A = std::allocator<T>>
class no_init_allocator : public A {
  typedef std::allocator_traits<A> a_t;

 public:
  using A::A;  // Inherit constructors from A

  template <typename U>
  struct rebind {
    using other = no_init_allocator<U, typename a_t::template rebind_alloc<U>>;
  };

  template <typename U>
  void construct(U*) noexcept(std::is_nothrow_default_constructible<U>::value) {
    // empty
  }

  template <typename U, typename... Args>
  void construct(U*, Args&&...) {
    // empty
  }
};

// _____________________________________________________________________________
inline float boundedSub(const float a, const float b) { return a - b; }

// _____________________________________________________________________________
inline float boundedAdd(const float a, const float b) { return a + b; }

// _____________________________________________________________________________
inline double boundedSub(const double a, const double b) { return a - b; }

// _____________________________________________________________________________
inline double boundedAdd(const double a, const double b) { return a + b; }

// _____________________________________________________________________________
template <typename T>
inline T boundedSub(const T a, const double b) {
  const double diff = static_cast<double>(a) - b;
  return diff <= static_cast<double>(std::numeric_limits<T>::lowest())
             ? std::numeric_limits<T>::lowest()
             : static_cast<T>(diff);
}

// _____________________________________________________________________________
template <typename T>
inline T boundedAdd(const T a, const double b) {
  const double sum = static_cast<double>(a) + b;
  return sum >= static_cast<double>(std::numeric_limits<T>::max())
             ? std::numeric_limits<T>::max()
             : static_cast<T>(sum);
}

// _____________________________________________________________________________
template <typename T>
inline T boundedSub(const T a, const T b) {
  return a < std::numeric_limits<T>::lowest() + b
             ? std::numeric_limits<T>::lowest()
             : a - b;
}

// _____________________________________________________________________________
template <typename T>
inline T boundedAdd(const T a, const T b) {
  return a > std::numeric_limits<T>::max() - b ? std::numeric_limits<T>::max()
                                               : a + b;
}

// _____________________________________________________________________________
inline float boundedAbsDiff(const float a, const float b) {
  return fabs(a - b);
}

// _____________________________________________________________________________
inline double boundedAbsDiff(const double a, const double b) {
  return fabs(a - b);
}

// _____________________________________________________________________________
template <typename T>
inline T boundedAbsDiff(const T a, const T b) {
  // for integer types (float and double are caught above), we convert to
  // an unsigned version of the type to have enough room left to the diff, then
  // convert back, this avoids floating point artihmetic
  typedef typename std::make_unsigned<T>::type U;
  const U d = a > b ? U(a) - U(b) : U(b) - U(a);
  return d > static_cast<U>(std::numeric_limits<T>::max())
             ? std::numeric_limits<T>::max()
             : static_cast<T>(d);
}

// _____________________________________________________________________________
template <typename V>
int merge(V* lst, V* tmpLst, size_t l, size_t m, size_t r) {
  size_t ret = 0;

  size_t lp = l;
  size_t rp = m;
  size_t outp = l;

  while (lp < m && rp < r + 1) {
    if (lst[lp] <= lst[rp]) {
      // if left element is smaller or equal, add it to return list,
      // increase left pointer
      tmpLst[outp] = lst[lp];
      lp++;
    } else {
      // if left element is bigger, add the right element, add it to ret,
      // increase right pointer
      tmpLst[outp] = lst[rp];
      rp++;

      // if the left element was bigger, everything to the right in the
      // left list is also bigger, and all these m - i elements were
      // initially in the wrong order! Count these inversions.
      ret += m - lp;
    }

    outp++;
  }

  // fill in remaining values
  if (lp < m) std::memcpy(tmpLst + outp, lst + lp, (m - lp) * sizeof(V));
  if (rp <= r) std::memcpy(tmpLst + outp, lst + rp, ((r + 1) - rp) * sizeof(V));

  // copy to output
  std::memcpy(lst + l, tmpLst + l, ((r + 1) - l) * sizeof(V));

  return ret;
}

// _____________________________________________________________________________
template <typename V>
size_t mergeInvCount(V* lst, V* tmpLst, size_t l, size_t r) {
  size_t ret = 0;
  if (l < r) {
    size_t m = (r + l) / 2;

    ret += mergeInvCount(lst, tmpLst, l, m);
    ret += mergeInvCount(lst, tmpLst, m + 1, r);

    ret += merge(lst, tmpLst, l, m + 1, r);
  }
  return ret;
}

// _____________________________________________________________________________
template <typename V>
size_t inversions(const std::vector<V>& v) {
  if (v.size() < 2) return 0;  // no inversions possible

  // unroll some simple cases
  if (v.size() == 2) return v[1] < v[0];
  if (v.size() == 3) return (v[0] > v[1]) + (v[0] > v[2]) + (v[1] > v[2]);

  auto tmpLst = new V[v.size()];
  auto lst = new V[v.size()];

  for (size_t i = 0; i < v.size(); i++) lst[i] = v[i];

  size_t ret = mergeInvCount<V>(lst, tmpLst, 0, v.size() - 1);
  delete[] tmpLst;
  delete[] lst;
  return ret;
}

std::string getHomeDir();

std::string getTmpDir();

std::string getTmpFName(std::string dir, std::string name, std::string postf);

std::string rgbToHex(int r, int g, int b);

void hsvToRgb(float* r, float* g, float* b, float h, float s, float v);

std::string normHtmlColor(const std::string& col);

std::string randomHtmlColor();

char* readableSize(double size, size_t n, char* buf);

// _____________________________________________________________________________
inline std::string readableSize(double size) {
  char buffer[30];
  return readableSize(size, 30, buffer);
}

void sortPart(int file, size_t objSize, size_t part, unsigned char* buf,
              unsigned char* partbuf, size_t bufferSize, size_t partsBufSize,
              size_t* partsize, int (*cmpf)(const void*, const void*));

void processSortQueue(util::JobQueue<SortJob>* jobs, int file, size_t objSize,
                      unsigned char* buf, size_t bufferSize,
                      size_t partsBufSize, size_t* partsize,
                      int (*cmpf)(const void*, const void*));

ssize_t externalSort(int file, int newFile, size_t size, size_t numobjs,
                     size_t numThreads, int (*cmpf)(const void*, const void*));

// _____________________________________________________________________________
class approx {
 public:
  explicit approx(double magnitude)
      : _epsilon{std::numeric_limits<float>::epsilon() * 100},
        _magnitude{magnitude} {}

  friend bool operator==(double lhs, approx const& rhs) {
    return std::abs(lhs - rhs._magnitude) < rhs._epsilon;
  }

  friend bool operator==(approx const& lhs, double rhs) {
    return operator==(rhs, lhs);
  }

  friend bool operator!=(double lhs, approx const& rhs) {
    return !operator==(lhs, rhs);
  }

  friend bool operator!=(approx const& lhs, double rhs) {
    return !operator==(rhs, lhs);
  }

  friend bool operator<=(double lhs, approx const& rhs) {
    return lhs < rhs._magnitude || lhs == rhs;
  }

  friend bool operator<=(approx const& lhs, double rhs) {
    return lhs._magnitude < rhs || lhs == rhs;
  }

  friend bool operator>=(double lhs, approx const& rhs) {
    return lhs > rhs._magnitude || lhs == rhs;
  }

  friend bool operator>=(approx const& lhs, double rhs) {
    return lhs._magnitude > rhs || lhs == rhs;
  }

  friend std::ostream& operator<<(std::ostream& out, const approx& a) {
    out << "~" << std::fixed << std::setprecision(5) << a._magnitude;
    return out;
  }

 private:
  double _epsilon;
  double _magnitude;
};

size_t getPeakRSS();
size_t getCurrentRSS();

}  // namespace util

#endif  // UTIL_MISC_H_
