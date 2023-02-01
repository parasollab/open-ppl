#ifndef STAPL_SKELETONS_UTILITY_ADD_TASK_HELPER_HPP
#define STAPL_SKELETONS_UTILITY_ADD_TASK_HELPER_HPP

#include <type_traits>
#include <stapl/skeletons/utility/dynamic_wf.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor used to call @ref paragraph_impl::paragraph_view::add_task()
/// from PARAGRAPH factories.
///
/// This is necessary to provide an interface that is independent of whether the
/// work function used as the task operation is dynamic or not.
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
template<typename WF, bool dynamic = std::is_base_of<dynamic_wf, WF>::value>
struct add_task_helper
{
  // function operator used by map_func
  template<typename TGV, typename SchedInfo, typename MapWF, typename ...V>
  void operator()(TGV const& tgv, SchedInfo const& sched_info,
                  MapWF const& wf, V const&... v)
  {
    tgv.add_task(sched_info, wf, v...);
  }

  // function operator used by map tasks of map_reduce
  template<typename TGV, typename MapWF, typename ...V>
  void operator()(TGV const& tgv, std::size_t const& tid, MapWF const& wf,
                  std::size_t const& num_succs,
                  V const&... v)
  {
    tgv.add_task(tid, wf, num_succs, v...);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for dynamic work functions that adds the
/// @ref paragraph_impl::paragraph_view as the first view in the task
/// specification.
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
template<typename WF>
struct add_task_helper<WF, true>
{
  // function operator used by map_func
  template<typename TGV, typename SchedInfo, typename MapWF, typename ...V>
  void operator()(TGV const& tgv, SchedInfo const& sched_info,
                  MapWF const& wf, V const&... v)
  {
    tgv.add_task(sched_info, wf, localize_ref(tgv), v...);
  }

  // function operator used by map tasks of map_reduce
  template<typename TGV, typename MapWF, typename ...V>
  void operator()(TGV const& tgv, std::size_t const& tid, MapWF const& wf,
                  std::size_t const& num_succs, V const&... v)
  {
    tgv.add_task(tid, wf, num_succs, localize_ref(tgv), v...);
  }
};

} // namespace stapl
#endif
