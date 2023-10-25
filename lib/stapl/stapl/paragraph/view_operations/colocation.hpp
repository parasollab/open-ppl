/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_COLOCATION_HPP
#define STAPL_PARAGRAPH_COLOCATION_HPP

#include <stapl/views/metadata/coarseners/default.hpp>
#include <stapl/runtime/stapl_assert.hpp>
#include <stapl/utility/tuple.hpp>
#include <utility>
#include <vector>
#include <boost/utility/result_of.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Function object that performs a deep copy of input view into a
///   std::vector and returns this container.
/// @ingroup pgViewOps
///
/// Class is used as workfunction in colocation factory below to migrate a copy
/// of a read only array_view via the @p edge_container to the location where
/// task using the data will be executed to avoid remote element fetches.
//////////////////////////////////////////////////////////////////////
struct fill_edge
{
  template<typename View>
  std::vector<typename View::value_type>
  operator()(View const& v) const
  {
    std::vector<typename View::value_type> tmp(v.size());

    std::copy(v.begin(), v.end(), tmp.begin());

    return tmp;
  }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Initial implementation of view colocation strategy that copies
///   readonly data to the task's execution location to avoid excessive, fine
///   grain communication.  Assume a dense read pattern of these views.
/// @ingroup pgViewOps
///
/// This implementation exists to fix performance problem with stapl::copy,
/// currently the only user of the interface. (Hence only two view version
/// of function operator).
///
/// @sa copy
///
/// @todo The current implementation a stapl factory, limiting it's application
/// to the map_func type behavior it implements.  We need decide what the
/// desired interface is and change the implementation accordingly.  Options:
/// - Behavior becomes part of task placement policy and uses access specifiers
///   from workfunction
/// - Behavior enabled by factory by flag to add_task
/// - View directives (i.e., make_paragraph(pattern, view1, allow_copy(view2)
/// - Other options...
//////////////////////////////////////////////////////////////////////
template <typename WF>
class colocation
  : public task_factory_base
{
private:
  WF m_wf;

public:
  using coarsener_type = default_coarsener;
  using result_type = result_type;

  colocation(WF const& wf)
    : m_wf(wf)
  { }

  default_coarsener get_coarsener() const { return default_coarsener(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation of task_factory_base interface.
  ///
  /// Definition exists solely as sanity check to make sure it isn't called.
  //////////////////////////////////////////////////////////////////////
  void reset()
  {
    stapl_assert(0, "don't expect this to be called");
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function operator invoked by PARAGRAPH to populate it with tasks.
  ///
  /// @param pg_view   View of PARAGRAPH where tasks will be added.
  /// @param src_view  Source view, read only data access.
  /// @param dest_view Destination view, write only.
  ///
  /// Source/destination coarsened subview pairs are analyzed and the following
  /// algorithm is used:
  ///
  /// - If both coarsened subviews are already colocated
  ///   (i.e., preferred_location return same location), call add_task as usual.
  ///
  /// - Else, insert a task to place the source view's elements in the
  ///   container.  Then, add another task to apply the user's workfunction,
  ///   which consumes the source from edge_container, relying on that
  ///   component's protocol to move the wherever the destination view's
  ///   locality dictates the task be placed.
  //////////////////////////////////////////////////////////////////////
  template<typename PGV, typename SrcView, typename DestView>
  void operator()(PGV const& pg_view,
                  SrcView& src_view,
                  DestView& dest_view) const
  {
    stapl_assert(src_view.size() == dest_view.size(), "sizes are not equal");

    const size_t n_elements = src_view.size();

    typedef view_index_iterator<SrcView>  src_id_iter_t;
    typedef view_index_iterator<DestView> dest_id_iter_t;

    tuple<src_id_iter_t, dest_id_iter_t> id_it =
      partition_id_set(src_view, dest_view);

    src_id_iter_t  src_id_iter  = get<0>(id_it);
    dest_id_iter_t dest_id_iter = get<1>(id_it);

    for (; !src_id_iter.at_end(); ++src_id_iter, ++dest_id_iter)
    {
      const size_t tid = *src_id_iter;

      if (src_view.get_preferred_location(*src_id_iter).first
          == dest_view.get_preferred_location(*dest_id_iter).first)
       {
        // Two Coarsened Chunks are colocated, directly add_task
        pg_view.add_task(
          tid, coarse_map_wf<WF>(m_wf), (size_t) 0,
          std::make_pair(&src_view,  *src_id_iter),
          std::make_pair(&dest_view, *dest_id_iter)
        );
      }
      else
      {
        pg_view.add_task(
           tid + n_elements, detail::fill_edge(), (size_t) 1,
           std::make_pair(&src_view, *src_id_iter)
        );

        typedef typename boost::result_of<
          detail::fill_edge(typename SrcView::reference)
        >::type edge_t;

        pg_view.add_task(
          tid, coarse_map_wf<WF>(m_wf), (size_t) 0,
          consume<edge_t>(pg_view, tid + n_elements),
          std::make_pair(&dest_view, *dest_id_iter)
        );
      }
    }
  }
};

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_COLOCATION_HPP
