/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_UTILITY_VIEW_INDEX_PARTITION_HPP
#define STAPL_SKELETONS_UTILITY_VIEW_INDEX_PARTITION_HPP

#include <stapl/views/type_traits/is_view.hpp>
#include <stapl/runtime/new.hpp>
#include <stapl/skeletons/utility/wf_iter_compare.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Base class used to allow @ref view_index_iterator instances over
///   different view types to be stored in an std::vector in a factory that
///   performs incremental task generation.
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
struct view_index_iterator_base
{
public:
  virtual ~view_index_iterator_base() = default;

  virtual std::size_t size() const = 0;

  virtual void reset() = 0;
};


//////////////////////////////////////////////////////////////////////
/// @brief Iterator that traverses a disjoint partition of the indices of a
///   view's elements.
/// @tparam View The type of the view whose partitioned domain is being
///   abstracted by the iterator.
///
/// The class is used in factories to easily partition the work of specifying
/// the tasks of a PARAGRAPH across all locations on which the PARAGRAPH is
/// being executed.
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
template <typename View>
struct view_index_iterator
  : public view_index_iterator_base
{
private:
  /// View for which tasks will be specified.
  View const&                        m_view;

  /// The current index of the iterator.
  std::size_t                        m_current;

  //////////////////////////////////////////////////////////////////////
  /// @brief The length of this viewset index sequence, namely v0.size().
  /// @note  Caching size isn't necessary, but may reduce collective operation
  ///        costs.
  //////////////////////////////////////////////////////////////////////
  const std::size_t                  m_vs_length;

  /// Number of local components for this view, same as m_vs_length.
  const std::size_t                  m_size;

  typedef typename View::cid_type    id_t;

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the view GID for the element at the given index.
  //////////////////////////////////////////////////////////////////////
  id_t access(std::size_t idx, std::true_type) const
  {
    return m_view.get_local_vid(idx);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the view GID for the element at the given index.
  /// @todo Determine if any "old views" that have num_local_subview > 1 but
  ///   with one real component for replicated/repeated views remain, and remove
  ///   this interface if it's no longer needed.
  //////////////////////////////////////////////////////////////////////
  id_t access(std::size_t idx, std::false_type) const
  {
    if (m_size ==  1)
      return m_view.get_local_component(0).get_id();
    else
      return m_view.get_local_component(idx).get_id();
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an iterator that will cover the elements of the view
  ///   that are local.
  /// @param view The view whose elements will be partitioned by the iterator.
  /// @param vs_len The number of local subviews.
  /// @param b_replicated Indicated if the view is a @ref repeat_view.
  ///
  /// m_vs_length can differ from m_size in cases where the factory is
  /// processing multiple views.  In that case @p vs_len is the number of local
  /// subviews of the first view, which we use to drive the behavior of the
  /// factory.
  //////////////////////////////////////////////////////////////////////
  view_index_iterator(View const& view, std::size_t vs_len, bool b_replicated)
    : m_view(view),
      m_current(0),
      m_vs_length(vs_len),
      m_size(view.get_num_local_subviews())
  { }

  STAPL_USE_MANAGED_ALLOC(view_index_iterator)

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether all local subviews of the view have been
  ///   processed in the current iteration.
  //////////////////////////////////////////////////////////////////////
  bool at_end(void) const
  {
    return m_current == m_vs_length;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of local subviews of the view, or in the case
  ///   of a multi-view algorithm the number of local subviews of the first
  ///   view.
  //////////////////////////////////////////////////////////////////////
  std::size_t size() const
  {
    return m_vs_length;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether the specified index is part of the partition
  ///   handled by this iterator.
  /// @note There is a potential for performance issues if the view method
  ///   called in @ref access generates ARMI traffic.
  /// @todo The parameter type should really be id_t instead of size_t.
  //////////////////////////////////////////////////////////////////////
  bool contains(size_t const& idx) const
  {
    for (std::size_t i=0; i < m_size; ++ i)
      if (idx == this->access(i, typename is_view<View>::type()))
        return true;

    return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Reset the iterator to allow another traversal of the view elements
  ///   by the factory.
  ///
  /// This is used in factories such as @ref map_reduce_factory where multiple
  /// traversals of the view indices are required to generate the reduce trees.
  //////////////////////////////////////////////////////////////////////
  void reset(void)
  {
    m_current = 0;
  }

  id_t operator*()
  {
    return access(m_current, typename is_view<View>::type());
  }

  view_index_iterator& operator++()
  {
    ++m_current;
    return *this;
  }

  view_index_iterator operator++(int)
  {
    view_index_iterator tmp = *this;
    ++m_current;
    return tmp;
  }
};


// Specialization for multiviews
template <typename... V>
std::size_t compute_num_subviews(V&&... v)
{
  return part_id_size<typename std::remove_reference<V>::type...>::invoke(v...);
}

//////////////////////////////////////////////////////////////////////
/// @brief Constructs a ref stapl::view_index_iterator instances for
/// the given view.
///
/// @param v The view whose elements will be processed by tasks
///          generated in the factory that invoked this method.
/// @return ref view_index_iterator instance of the given view that
///             have been partitioned based on the number of local
///             elements of the first view on each location.
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
template <typename V>
view_index_iterator<typename std::remove_reference<V>::type>
make_index_view_iterator(V&& v, std::size_t num_subviews,
                         bool b_replicated_aware = false)
{
  return view_index_iterator<
           typename std::remove_reference<V>::type>(
             v, num_subviews, b_replicated_aware);
}

//////////////////////////////////////////////////////////////////////
/// @brief Constructs a set of @ref stapl::view_index_iterator
/// instances, one for each of the input views.
///
/// @param v  Views whose elements will be processed by tasks generated
///           in the factory that invoked this method.
/// @return A tuple of @ref view_index_iterator instances that have
///         been partitioned based on the number of local elements of
///         the first view on each location.
///
/// @todo Check if the calls to get_num_copies are still needed.
/// @todo Unequal Copies aren't wrong per se, just not a case we've thought
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
template <bool b_replicated_aware, typename... V>
tuple<view_index_iterator<typename std::remove_reference<V>::type>...>
partition_id_set(V&&... v)
{
  const size_t num_subviews = compute_num_subviews(v...);

  return make_tuple(
           make_index_view_iterator(v, num_subviews, b_replicated_aware)...
         );
}

//////////////////////////////////////////////////////////////////////
/// @brief The default case of partition_id_set which assumes that
/// replication awareness is set to false.
///
/// @param v  Views whose elements will be processed by tasks generated
///           in the factory that invoked this method.
/// @return A tuple of @ref view_index_iterator instances that have
///         been partitioned based on the number of local elements of
///         the first view on each location.
///
/// @todo Check if the calls to get_num_copies are still needed.
/// @todo Unequal Copies aren't wrong per se, just not a case we've thought
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
template <typename... V>
tuple<view_index_iterator<typename std::remove_reference<V>::type>...>
partition_id_set(V&&... v)
{
  return partition_id_set<false>(v...);
}

} // namespace stapl

#endif // STAPL_SKELETONS_UTILITY_VIEW_INDEX_PARTITION_HPP

