#ifndef NONSTD_ENDIAN_H_
#define NONSTD_ENDIAN_H_

#include <cstddef>


namespace nonstd {

  ///@name Endian Detection
  ///@{

  /// True if using little endian, false if big endian.
  extern const bool little_endian;

  ///@}
  ///@name Endian Conversion
  ///@{

  /// Reverse the order of some bytes.
  /// @param _start The starting address.
  /// @param _size The number of bytes to reverse.
  void reverse_byte_order(void* const _start, const size_t _size);

  ///@}

}

#endif
