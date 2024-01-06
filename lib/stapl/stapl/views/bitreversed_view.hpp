/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#ifndef STAPL_VIEWS_BITREVERSED_VIEW_HPP
#define STAPL_VIEWS_BITREVERSED_VIEW_HPP

#include <stapl/domains/indexed.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/skeletons/utility/bitreverse.hpp>
#include <cmath>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Mapping function used for views over bitreversed
///        containers.
///
/// @see bitreverse(T)
/// @see test_fft the result of FFT is produced in a bitreversed
///      order and viewing the results in normal order is helpful in
///      debugging.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct bitreverse_map_function
{
  typedef T                          index_type;
  typedef T                          gid_type;
  typedef bitreverse_map_function<T> inverse;

  const bitreverse<T> m_bitreverse;

  bitreverse_map_function()
    : m_bitreverse(T())
  { }

  bitreverse_map_function(T max_number)
    : m_bitreverse(max_number)
  { }

  bitreverse_map_function(inverse const& mf)
   : m_bitreverse(mf.m_bitreverse)
  { }

  gid_type operator()(index_type x) const
  {
    return index_type(m_bitreverse(x));
  }

  void define_type(typer& t)
  {
    t.member(m_bitreverse);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A view which looks at the elements of a 1D container in a
///        bitreversed way, meaning that given an index, it gets the
///        element in the bitreversed index of the container.
///
/// For example if the indices of a 1D container are from [0-7]:
/// <tt>
/// ---------------------------------
/// |000|001|010|011|100|101|110|111|
/// ---------------------------------
/// </tt>
/// Then the bitreversed view will map the indices as follows:
/// <tt>
///                  ---------------------------------
/// bitreversed view |000|001|010|011|100|101|110|111|
///                  ---------------------------------
///                    |   |   |   |   |   |   |   |
///                  ---------------------------------
/// container        |000|100|010|110|001|101|011|111|
///                  ---------------------------------
/// </tt>
//////////////////////////////////////////////////////////////////////
template <typename C>
array_view<C, indexed_domain<size_t>, bitreverse_map_function<size_t> >
bitreversed_view(C& container, size_t n)
{
  typedef indexed_domain<size_t>                 dom_t;
  typedef bitreverse_map_function<size_t>        map_function_t;
  typedef array_view<C, dom_t, map_function_t>   view_t;

  return view_t(container, dom_t(n), map_function_t(n-1));
}

} // namespace stapl

#endif // STAPL_VIEWS_BITREVERSED_VIEW_HPP
