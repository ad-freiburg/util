// Copyright 2026, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#include "util/BlockReader.h"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

#include "util/Misc.h"

using util::BlockReaderBase;

// _____________________________________________________________________________
BlockReaderBase::BlockReaderBase() : _fd(-1), _off(0) {}

// _____________________________________________________________________________
BlockReaderBase::~BlockReaderBase() { closeFile(); }

// _____________________________________________________________________________
BlockReaderBase::BlockReaderBase(BlockReaderBase&& o)
    : _fd(o._fd), _off(o._off) {
  o._fd = -1;
  o._off = 0;
}

// _____________________________________________________________________________
BlockReaderBase& BlockReaderBase::operator=(BlockReaderBase&& o) {
  if (this != &o) {
    closeFile();
    _fd = o._fd;
    _off = o._off;
    o._fd = -1;
    o._off = 0;
  }
  return *this;
}

// _____________________________________________________________________________
void BlockReaderBase::openFile(const std::string& file, size_t off) {
  closeFile();

  _fd = ::open(file.c_str(), O_RDONLY);
  if (_fd < 0) {
    throw std::runtime_error("Could not open " + file + ": " + strerror(errno));
  }
  _off = off;
}

// _____________________________________________________________________________
void BlockReaderBase::closeFile() {
  if (_fd >= 0) ::close(_fd);
  _fd = -1;
  _off = 0;
}

// _____________________________________________________________________________
void BlockReaderBase::readBytes(void* buf, size_t count, size_t pos) const {
  if (count == 0) return;
  if (_fd < 0) throw std::runtime_error("BlockReader: file not open");

  ssize_t r =
      util::preadAll(_fd, static_cast<unsigned char*>(buf), count, _off + pos);
  if (r < 0) {
    throw std::runtime_error(std::string("BlockReader: read error ") +
                             strerror(errno));
  }
  if (static_cast<size_t>(r) != count) {
    throw std::runtime_error("BlockReader: unexpected eof");
  }
}
