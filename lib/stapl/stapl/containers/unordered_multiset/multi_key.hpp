/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MULTI_KEY_HPP
#define STAPL_CONTAINERS_MULTI_KEY_HPP

#include <stapl/views/proxy_macros.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>

namespace stapl {

template<typename Key>
class multi_key;

////////////////////////////////////////////////////////////////////
/// @brief A wrapper for the given hash function that strips away the
///   the multiplicity value and calculates the hash value for the key
////////////////////////////////////////////////////////////////////
template<typename Key, typename Hash>
struct pair_hash
  : public Hash {
  size_t operator()(multi_key<Key> const& m) const
  {
    return Hash::operator()(m.first);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Pairs a key with a multiplicity value to provide a unique GID
///   for the elements of the unordered multi-associative pContainers.
/// @todo Add functionality for operations such as adding two dereferenced
//    iterators; this operation should return the sum of the two keys.
//////////////////////////////////////////////////////////////////////
template<typename Key>
class multi_key
  : public std::pair<Key, size_t>
{
  typedef std::pair<Key, size_t> base_type;
  typedef Key                    key_type;

public:

  ////////////////////////////////////////////////////////////////////
  /// @brief Construct a default pair of invalid values
  ////////////////////////////////////////////////////////////////////
  multi_key()
    : std::pair<key_type, size_t>(index_bounds<key_type>::invalid(),
                                  index_bounds<size_t>::invalid())
  { }

  template<typename Accessor>
  multi_key(proxy<multi_key, Accessor> p)
    : std::pair<key_type, size_t>(p.first, p.second)
  { }

  ////////////////////////////////////////////////////////////////////
  /// @brief Copy constructor
  ////////////////////////////////////////////////////////////////////
  multi_key(multi_key const& m)
    : std::pair<key_type, size_t>(m.first, m.second)
  { }

  ////////////////////////////////////////////////////////////////////
  /// @brief Construct the initial pair for a key @p k
  /// @param k The key to build a pair for
  ////////////////////////////////////////////////////////////////////
  multi_key(key_type const& k)
    : std::pair<key_type, size_t>(k, 0)
  { }

  ////////////////////////////////////////////////////////////////////
  /// @brief Construct a pair from a given key @p k and multiplicity @p m
  /// @param k The key
  /// @param m The multiplicity of the given key @p k
  ////////////////////////////////////////////////////////////////////
  multi_key(key_type const& k, size_t const& m)
    : std::pair<key_type, size_t>(k, m)
  { }

  /////////////////////////////////////////////////////////////////
  /// @brief Allows the class to disregard the multiplicity value and
  ///   return only the key.
  /////////////////////////////////////////////////////////////////
  operator key_type() const&
  {
    return this->first;
  }

  /////////////////////////////////////////////////////////////////
  /// @brief Allows the class to disregard the multiplicity value and
  ///   return only the key for standard output.
  /////////////////////////////////////////////////////////////////
  key_type operator<<(multi_key const& m)
  {
    return m.first;
  }

  void define_type(stapl::typer& t)
  {
    t.base<base_type>(*this);
  }

}; // multi_key

STAPL_PROXY_HEADER_TEMPLATE(multi_key, T)
{
  STAPL_PROXY_TYPES(STAPL_PROXY_CONCAT(multi_key<T>), Accessor)
  STAPL_PROXY_IMPORT_TYPES(first_type, second_type)
  STAPL_PROXY_METHODS(STAPL_PROXY_CONCAT(multi_key<T>), Accessor)

  STAPL_PROXY_MEMBER(first, first_type)
  STAPL_PROXY_MEMBER(second, second_type)

  explicit proxy(Accessor const& acc)
   : Accessor(acc),
     first(member_referencer<first_accessor>()(acc)),
     second(member_referencer<second_accessor>()(acc))
  { }

  operator T() const
  {
    return first;
  }

};

} // namespace stapl

#endif // STAPL_CONTAINERS_MULTIKEY_HPP
