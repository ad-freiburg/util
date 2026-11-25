// Copyright 2026, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#ifndef UTIL_BLOCKREADER_H_
#define UTIL_BLOCKREADER_H_

#include <algorithm>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace util {

class BlockReaderBase {
 public:
  BlockReaderBase();
  ~BlockReaderBase();

  BlockReaderBase(const BlockReaderBase&) = delete;
  BlockReaderBase& operator=(const BlockReaderBase&) = delete;

  BlockReaderBase(BlockReaderBase&& o);
  BlockReaderBase& operator=(BlockReaderBase&& o);

  bool isOpen() const { return _fd >= 0; }

 protected:
  void openFile(const std::string& fname, size_t off);
  void closeFile();

  // wrapper around preadAll with error messages
  void readBytes(void* buf, size_t count, size_t pos) const;

 private:
  int _fd;
  size_t _off;
};

template <typename T>
class BlockReader;

template <typename T>
class BlockReaderCursor {
 public:
  typedef T value_type;

  explicit BlockReaderCursor(const BlockReader<T>* reader);

  size_t size() const;
  T at(size_t i);

  template <typename K>
  size_t lowerBound(size_t i, const K& key);

 private:
  void load(size_t block);

  const BlockReader<T>* _reader;
  std::vector<T> _buf;
  size_t _bufStart;
  size_t _bufEnd;
};

// simple file reader which offers a cursor providing at() and lowerbound(),
// builds a block index of size blockSize on load used in lowerbound()
template <typename T>
class BlockReader : public BlockReaderBase {
 public:
  static const size_t DEFAULT_BLOCK_SIZE = 4096;

  BlockReader() : BlockReader(DEFAULT_BLOCK_SIZE) {}
  explicit BlockReader(size_t blockSize)
      : _blockSize(std::max<size_t>(1, blockSize)), _size(0) {}

  BlockReader(BlockReader&& o) = default;
  BlockReader& operator=(BlockReader&& o) = default;

  // this opens and build a lookup index with blocks of size blockSize
  void open(const std::string& fname, size_t off, size_t num);
  void close();

  size_t size() const { return _size; }
  size_t blockSize() const { return _blockSize; }

  const std::vector<T>& index() const { return _idx; }

  BlockReaderCursor<T> cursor() const { return Cursor(this); }

  // for convenience
  typedef BlockReaderCursor<T> Cursor;

 private:
  friend class BlockReaderCursor<T>;

  size_t _blockSize;
  size_t _size;

  std::vector<T> _idx;
};

#include "util/BlockReader.tpp"

}  // namespace util

#endif  // UTIL_BLOCKREADER_H_
