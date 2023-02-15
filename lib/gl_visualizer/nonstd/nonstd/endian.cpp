#include "nonstd/endian.h"

#include <algorithm>
#include <cstdint>


namespace nonstd {

  /*--------------------------- Endian Detection -----------------------------*/

  /// Check if this machine uses little endian representation. This is to be
  /// called only once to set the little_endian flag.
  /// @return True if using little endian, false if big endian.
  bool
  is_little_endian()
  {
    constexpr const uint16_t one16 = 0x0001;
    constexpr const uint8_t  one8  = 0x01;

    const uint8_t converted = *reinterpret_cast<const uint8_t*>(&one16);
    return converted == one8;
  }

  const bool little_endian = is_little_endian();

  /*--------------------------- Endian Conversion ----------------------------*/

  void
  reverse_byte_order(
      void* const _start,
      const size_t _size
  ) {
    char* first = static_cast<char*>(_start),
        * last  = first + _size - 1;
    for(; first < last; ++first, ++last)
      std::swap(*first, *last);
  }

  /*--------------------------------------------------------------------------*/

}
