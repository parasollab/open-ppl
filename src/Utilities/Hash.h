#ifndef PMPL_HASH_H_
#define PMPL_HASH_H_

#include <functional>


namespace std {

  //////////////////////////////////////////////////////////////////////////////
  /// Define a hasher for a std::pair of elements with the same type.
  /// @note This implementation was borrowed from older versions of boost. I
  ///       will not use the newer version because it isn't guaranteed to give
  ///       the same behavior across multiple runs of a program, which is
  ///       detrimental randomness that we don't need (it obfuscates our control
  ///       of randomness in our sampling processes).
  /// @note Tested with 100M random pairs and no collisions.
  //////////////////////////////////////////////////////////////////////////////
  template <typename T>
  struct hash<std::pair<T, T>> {

    typedef std::pair<T, T> KeyPair;

    static constexpr size_t magicOffset = 0x9e3779b99e3779b9;

    size_t operator()(const KeyPair& _key) const noexcept {
      static constexpr std::hash<T> hasher;
      auto h1 = hasher(_key.first),
           h2 = hasher(_key.second);
      return h2 + magicOffset + (h1 << 6) + (h1 >> 2);
    }

  };


}

#endif
