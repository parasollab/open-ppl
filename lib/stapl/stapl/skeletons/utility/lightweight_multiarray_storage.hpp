/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_UTILITY_LIGHTWEIGHT_MULTIARRAY_STORAGE_HPP
#define STAPL_SKELETONS_UTILITY_LIGHTWEIGHT_MULTIARRAY_STORAGE_HPP

#include <iterator>
#include <type_traits>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Basic linear storage class used by @ref lightweight_multiarray_base
/// to avoid initialization scan if desired.
///
/// @todo Will only elide initialization for fundamental types at the moment.
/// To extend further, need to use array of aligned_storage, carefully.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct lightweight_multiarray_storage
{
  static_assert(
    std::is_fundamental<T>::value,
    "non-initialized lightweight_multiarray only supports fundamental types.");

public:
  using value_type      = T;
  using size_type       = std::size_t;
  using iterator        = value_type*;
  using const_iterator  = value_type const*;
  using reference       = typename std::add_lvalue_reference<value_type>::type;
  using const_reference = typename std::add_const<reference>::type;

private:
  std::unique_ptr<value_type[]> m_data;
  const size_type               m_size;

public:
  explicit lightweight_multiarray_storage(size_t size)
    : m_data(new value_type[size]),
      m_size(size)
  { }

  reference operator[](size_type n)
  {
    return *(m_data.get() + n);
  }

  iterator begin(void) const
  {
    return iterator{m_data.get()};
  }

  iterator end(void) const
  {
    return iterator{m_data.get() + m_size};
  }

  const_iterator cbegin(void) const
  {
    return begin();
  }

  const_iterator cend(void) const
  {
    return end();
  }

  size_type size(void) const
  {
    return m_size;
  }

  bool empty(void) const
  {
    return m_size == 0;
  }

  void define_type(typer &t)
  {
    t.member(m_size);
    t.member(m_data);
  }
}; // struct lightweight_multiarray_storage

} // namespace stapl

#endif // STAPL_SKELETONS_UTILITY_LIGHTWEIGHT_MULTIARRAY_STORAGE_HPP
