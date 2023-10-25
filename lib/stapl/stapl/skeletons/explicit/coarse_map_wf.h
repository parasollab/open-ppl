/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_EXPLICIT_COARSE_MAP_WF_H
#define STAPL_SKELETONS_EXPLICIT_COARSE_MAP_WF_H

#include <stapl/paragraph/paragraph_view.hpp>
#include <stapl/skeletons/explicit/task_graph_factories.h>
#include <stapl/runtime/executor/scheduler/sched.hpp>
#include <stapl/skeletons/utility/wf_iter_compare.hpp>
#include <stapl/skeletons/transformations/optimizers/utils.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Work function that wraps a fine-grain map operation in a loop in
/// order to process coarse-grain view elements provided to PARAGRAPHs that
/// are instances of map computations.
///
/// @tparam MapWF Fine-grain map operation.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename MapWF>
class coarse_map_wf
  : private MapWF
{
public:
  typedef void result_type;

  coarse_map_wf(MapWF const& wf)
    : MapWF(wf)
  { }

  void define_type(typer& t)
  {
    t.base<MapWF>(*this);
  }

  template <typename Scheduler,
            typename IterComp, typename ...Iter>
  void apply(paragraph_impl::paragraph_view<Scheduler>& tgv,
             IterComp& iter_compare, Iter... iter)
  {
    using namespace skeletons::optimizers;
    auto map_op = static_cast<MapWF&>(*this);
    for (; iter_compare(iter...); helpers::no_op(++iter...)) {
      map_op(tgv, (*iter)...);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke the fine-grain operation on each of the sets of
  /// elements in the views provided, where elements in a set have the
  /// same offset from the beginning of their respective views.
  //////////////////////////////////////////////////////////////////////
  template <typename IterComp, typename ...Iter>
  void apply(IterComp& iter_compare, Iter... iter)
  {
    using namespace skeletons::optimizers;
    auto map_op = static_cast<MapWF&>(*this);
    for (; iter_compare(iter...); helpers::no_op(++iter...)) {
      map_op((*iter)...);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke the fine-grain operation on each of the sets of
  /// elements in the views provided, where elements in a set have the
  /// same offset from the beginning of their respective views. In
  /// addition to an element from each of the input views, the
  /// @ref paragraph_view is provided to allow the task to specify
  /// additional tasks of the PARAGRAPH.
  //////////////////////////////////////////////////////////////////////
  template <typename Scheduler, typename... V>
  void operator()(paragraph_impl::paragraph_view<Scheduler>& tgv, V&&... view)
  {
    wf_iter_compare<V...> iter_compare(view...);
    return apply(tgv, iter_compare, view.begin()...);
  }

  template <typename... V>
  void operator()(V&&... view)
  {
    wf_iter_compare<V...> iter_compare(view...);
    return apply(iter_compare, view.begin()...);
  }
}; // class coarse_map_wf

} // namespace stapl

#endif // STAPL_SKELETONS_EXPLICIT_COARSE_MAP_WF_H
