/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_REQUIREMENTS_HPP
#define STAPL_PARAGRAPH_REQUIREMENTS_HPP

namespace stapl {

namespace view_operations {

//////////////////////////////////////////////////////////////////////
/// @brief Defines the operations expected by the PARAGRAPH during
///        task creation.
//////////////////////////////////////////////////////////////////////
template<typename Derived>
class paragraph_required_operation
{
private:
  typedef typename view_traits<Derived>::value_type  subview_type;
  typedef subview_type                               value_t;

  const Derived& derived() const
  {
    return static_cast<const Derived&>(*this);
  }

public:
  typedef typename view_traits<Derived>::index_type cid_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements referenced by the view.
  /// @todo This method provides the same information as the @c size
  ///       method and could be replaced with it.
  //////////////////////////////////////////////////////////////////////
  size_t get_num_subviews() const
  {
    return derived().size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements stored locally, referenced
  ///        for the view.
  //////////////////////////////////////////////////////////////////////
  size_t get_num_local_subviews() const
  {
    return derived().container().local_size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the element associated with the position @c i.
  /// @todo To be removed, use operator[] instead of this method.
  //////////////////////////////////////////////////////////////////////
  value_t get_subview(cid_type const& i) const
  {
    return derived().get_element(i);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the global index of the given local @c index.
  //////////////////////////////////////////////////////////////////////
  cid_type get_local_vid(cid_type const& index) const
  {
    return derived().container().get_local_vid(index);
  }
}; // class paragraph_required_operation

} // namespace view_operations

} // namespace stapl

#endif // STAPL_PARAGRAPH_REQUIREMENTS_HPP
