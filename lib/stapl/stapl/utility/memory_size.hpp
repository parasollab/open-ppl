/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_UTILITY_MEMORY_SIZE_HPP
#define STAPL_UTILITY_MEMORY_SIZE_HPP

#include <stapl/runtime/serialization/packed_object_storage.hpp>
#include <vector>
#include <list>
#include <map>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Returns an estimation of size of the passed object.
/// @tparam T type of the object to get its size
///
/// The default estimation returns the size required to pack an object.
///
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template<typename T>
struct memory_used
{
  typedef T value_type;

  static std::size_t size(value_type const& v) noexcept
  {
    typedef runtime::packed_object_storage<value_type> storage_type;
    return (sizeof(value_type) + storage_type::packed_size(v));
  }
};


namespace memory_size_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Estimate the size of the given std::map<> object.
/// @param map the std::map<> object whose size needs to be estimated.
/// @return Estimation of the size of the passed object.
/// @ingroup utility
///
/// @see memory_used
/// @bug This only gives a correct size for some versions of libstdc++.
/// @todo A better way for finding the size of an object is required.
//////////////////////////////////////////////////////////////////////
template <typename Map>
std::size_t helper_map_size(Map const& map)
{
  typedef typename Map::key_type key;
  typedef typename Map::value_type::second_type value;
  std::size_t sz = sizeof(map);
  typename Map::const_iterator cit = map.begin();
  typename Map::const_iterator cit_end = map.end();
  while (cit != cit_end) {
    sz += memory_used<key>::size(cit->first) +
          memory_used<value>::size(cit->second);
    ++cit;
  }
#ifdef __GLIBCXX__
  sz += map.size() * (sizeof(std::_Rb_tree_color) +
                      3 * sizeof(std::_Rb_tree_node_base*));
#endif
  return sz;
}

//////////////////////////////////////////////////////////////////////
/// @brief Estimate the size of the given std::set<> object.
/// @param set the std::set<> object whose size needs to be estimated.
/// @return Estimation of the size of the passed object.
/// @ingroup utility
///
/// @see memory_used
/// @bug This only gives a correct size for some versions of libstdc++.
/// @todo A better way for finding the size of an object is required.
//////////////////////////////////////////////////////////////////////
template <typename Set>
std::size_t helper_set_size(Set const& set)
{
  typedef typename Set::key_type key;
  std::size_t sz = sizeof(set);
  typename Set::const_iterator cit = set.begin();
  typename Set::const_iterator cit_end = set.end();
  while (cit != cit_end){
    sz += memory_used<key>::size(*cit);
    ++cit;
  }
#ifdef __GLIBCXX__
  sz += set.size() * (sizeof(std::_Rb_tree_color) +
                      3 * sizeof(std::_Rb_tree_node_base*));
#endif
  return sz;
}

} // namespace memory_size_impl

//////////////////////////////////////////////////////////////////////
/// @brief Specialization used when the type is an std::map<>.
/// @ingroup utility
///
/// @bug This function assumes knowledge of the internal representation of
///      std::map<> which is currently based on libstdc++'s implementation.
/// @bug Template parameters for comparator object and allocator are missing.
//////////////////////////////////////////////////////////////////////
template <typename Key, typename T>
struct memory_used<std::map<Key, T> >
{
  typedef std::map<Key, T> value_type;

  static size_t size(value_type const& v)
  { return memory_size_impl::helper_map_size(v); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization used when the type is an std::list<>.
/// @ingroup utility
///
/// @bug This function assumes that each node in a linked list has only two
///      pointers to its previous and next node.
/// @bug Template parameter for allocator is missing.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct memory_used<std::list<T> >
{
  typedef std::list<T> value_type;

  static size_t size(value_type const& v)
  {
    std::size_t sz = 0;
    typename value_type::const_iterator cit = v.begin();
    typename value_type::const_iterator cit_end = v.end();
    while (cit != cit_end){
      sz += memory_used<T>::size(*cit);
      sz += 2 * sizeof(void*);
      ++cit;
    }
    return sz;
  }
};

} // namespace stapl

#endif
