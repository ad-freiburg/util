// Copyright 2026, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

// _____________________________________________________________________________
template <typename T>
const size_t BlockReader<T>::DEFAULT_BLOCK_SIZE;

// _____________________________________________________________________________
template <typename T>
void BlockReader<T>::open(const std::string& fname, size_t off, size_t num) {
  close();
  openFile(fname, off);

  std::vector<T> buf(std::min(num, 64 * _blockSize));
  _idx.reserve(num / _blockSize + 1);

  try {
    for (size_t i = 0; i < num; i += buf.size()) {
      size_t n = std::min(buf.size(), num - i);
      readBytes(buf.data(), n * sizeof(T), i * sizeof(T));
      for (size_t k = 0; k < n; k += _blockSize) _idx.push_back(buf[k]);
    }
  } catch (...) {
    close();
    throw;
  }

  _size = num;
}

// _____________________________________________________________________________
template <typename T>
void BlockReader<T>::close() {
  closeFile();
  _size = 0;
  std::vector<T>().swap(_idx);
}

// _____________________________________________________________________________
template <typename T>
BlockReaderCursor<T>::BlockReaderCursor(const BlockReader<T>* reader)
    : _reader(reader), _bufStart(0), _bufEnd(0) {}

// _____________________________________________________________________________
template <typename T>
size_t BlockReaderCursor<T>::size() const {
  return _reader->size();
}

// _____________________________________________________________________________
template <typename T>
T BlockReaderCursor<T>::at(size_t i) {
  if (i >= _reader->size())
    throw std::out_of_range("BlockReader: out of range");
  if (i < _bufStart || i >= _bufEnd) load(i / _reader->blockSize());
  return _buf[i - _bufStart];
}

// _____________________________________________________________________________
template <typename T>
template <typename K>
size_t BlockReaderCursor<T>::lowerBound(size_t i, const K& key) {
  // this lowerBound implementation
  const auto& idx = _reader->index();
  const size_t bs = _reader->blockSize();

  // access outside range
  if (i >= _reader->size()) return _reader->size();

  // we already have the right block
  if (i >= _bufStart && i < _bufEnd && !(_buf.back() < key)) {
    return std::lower_bound(_buf.begin() + (i - _bufStart), _buf.end(), key) -
           _buf.begin() + _bufStart;
  }

  // gallopping search for the right block ...
  size_t lo = i / bs;
  size_t step = 1;
  while (lo + step < idx.size() && idx[lo + step] < key) {
    lo += step;
    step *= 2;
  }

  // ... then binary search
  size_t hi = std::min(idx.size(), lo + step);
  size_t b = std::lower_bound(idx.begin() + lo + 1, idx.begin() + hi, key) -
             idx.begin();

  // now search the block
  size_t start = std::max(i, (b - 1) * bs);
  at(start);  // load the block into buffer

  return std::lower_bound(_buf.begin() + (start - _bufStart),
                          _buf.begin() + (_bufEnd - _bufStart), key) -
         _buf.begin() + _bufStart;
}

// _____________________________________________________________________________
template <typename T>
void BlockReaderCursor<T>::load(size_t block) {
  const size_t bs = _reader->blockSize();
  size_t start = block * bs;
  size_t end = std::min(_reader->size(), start + bs);

  _buf.resize(end - start);
  _bufStart = 0;
  _bufEnd = 0;
  _reader->readBytes(_buf.data(), (end - start) * sizeof(T), start * sizeof(T));
  _bufStart = start;
  _bufEnd = end;
}
