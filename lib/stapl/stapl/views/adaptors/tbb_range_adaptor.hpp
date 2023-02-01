/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_VIEWS_ADAPTORS_TBB_RANGE_ADAPTOR_HPP
#define STAPL_VIEWS_ADAPTORS_TBB_RANGE_ADAPTOR_HPP

#include <tbb/tbb_stddef.h>
#include <stapl/containers/partitions/balanced.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Adaptor that allows creation of a TBB compatible range from a view.
//////////////////////////////////////////////////////////////////////
template<typename View>
class tbb_range_adaptor
{
public:
  typedef typename View::const_iterator const_iterator;
  typedef typename View::iterator       iterator;

private:
  typedef indexed_domain<size_t> offset_domain_type;

  /// View being adapted
  View const*        m_vw;

  /// Offsets of view elements an instance of the adaptor represents
  offset_domain_type m_dom;

public:
  explicit tbb_range_adaptor(View const& view)
    : m_vw(&view), m_dom(0, m_vw->domain().size()-1)
  { }

  tbb_range_adaptor(tbb_range_adaptor const& other)
    : m_vw(other.m_vw), m_dom(other.m_dom)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that splits the elements referenced by @p other
  ///        into two views.  The first half is assigned to @p other and the
  ///        second half is assigned to the newly constructed view.
  //////////////////////////////////////////////////////////////////////
  tbb_range_adaptor(tbb_range_adaptor& other, tbb::split)
    : m_vw(other.m_vw)
  {
    balanced_partition<offset_domain_type> part(other.m_dom, 2);
    other.m_dom = part[0];
    m_dom = part[1];
  }

  bool is_divisible(void) const
  {
    return (m_dom.size()>1);
  }

  bool empty(void) const
  {
    return m_dom.empty();
  }

  const_iterator begin(void) const
  {
    return const_iterator(m_vw->begin()+m_dom.first());
  }

  const_iterator end(void) const
  {
    return const_iterator(m_vw->begin()+m_dom.last()+1);
  }

  iterator begin(void)
  {
    return iterator(m_vw->begin()+m_dom.first());
  }

  iterator end(void)
  {
    return iterator(m_vw->begin()+m_dom.last()+1);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Creates a new @ref tbb_range_adaptor from the given view.
//////////////////////////////////////////////////////////////////////
template<typename View>
tbb_range_adaptor<View> make_tbb_range(View const& vw)
{
  return tbb_range_adaptor<View>(vw);
}

} // namespace stapl

#endif
