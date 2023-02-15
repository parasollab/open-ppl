/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef BOOST_PP_IS_ITERATING
#  ifndef STAPL_SKELETONS_EXPLICIT_MAP_PROTOTYPE_H
#    define STAPL_SKELETONS_EXPLICIT_MAP_PROTOTYPE_H
#    ifndef PARAGRAPH_MAX_VIEWS
#      define PARAGRAPH_MAX_VIEWS 5
#    endif

#include <stapl/skeletons/utility/view_index_partition.hpp>
#include <stapl/views/metadata/coarseners/default.hpp>
#include <stapl/views/localize_element.hpp>
#include "coarse_map_wf.h"
#include <stapl/skeletons/utility/factory_add_task_helper.hpp>

namespace stapl {

template <typename WF, bool b_coarse = false,
          typename SchedInfo = none_t>
struct map_factory;

//////////////////////////////////////////////////////////////////////
/// @brief Factory that generates the tasks needed for a map operation.
/// A map operation applies the work function provided on each set of
/// elements of the input views that have the same offset from the
/// beginning of their respective view.
///
/// @tparam WF        Fine-grain operation to be applied to each set of
///                   elements.
/// @tparam b_coarse  Indicates whether the work function provided is
///                   coarse-grain.
/// @tparam SchedInfo Scheduling information required from each task by
///                   the scheduler that will determine the relative
///                   priorities of the tasks when they are ready to
///                   execute.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
template <typename WF, bool b_coarse, typename SchedInfo>
struct map_factory
  : public task_factory_base,
    private WF
{
private:
  /// Fraction of the total number of tasks to generate on each invocation.
  const char m_percent_per_call;

  //////////////////////////////////////////////////////////////////////
  /// @brief Number of tasks to generate on each invocation.  Minimum is 1,
  /// regardless of the value of m_percent_per_call in order to guarantee that
  /// the factory will generate its tasks.
  //////////////////////////////////////////////////////////////////////
  size_t     m_tasks_per_call;

public:
 using result_type = void;
 using coarsener_type = typename std::conditional<b_coarse,
                                                  default_coarsener,
                                                  null_coarsener>::type;

  map_factory(WF const& wf)
    : task_factory_base(true),
      WF(wf),
      m_percent_per_call(20),
      m_tasks_per_call(0)
  { }

  coarsener_type get_coarsener() const
  { return coarsener_type(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Reset all data members related to the number of tasks that have
  ///   been generated in order to prepare the factory for PARAGRAPH
  ///   reinvocation.
  //////////////////////////////////////////////////////////////////////
  void reset()
  {
    this->reset_view_indices();
    this->m_finished = false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Iterate over the indices of the elements of the views provided and
  ///   specify a task that will receive a set of view elements whose offset
  ///   from the start of their respective view is the same.
  /// @param tgv @ref paragraph_impl::paragraph_view that allows tasks to be
  ///   added to the PARAGRAPH.
  /// @param v* One or more views whose elements will be processed by the tasks.
  //////////////////////////////////////////////////////////////////////
# define STAPL_MAP_FUNC_OP_DECL(z, n, const_param) \
  template<typename TGV, BOOST_PP_ENUM_PARAMS(n, typename V)> \
  void operator()(TGV const& tgv, BOOST_PP_ENUM_BINARY_PARAMS(n, V, & v));
# define BOOST_PP_LOCAL_MACRO(n) STAPL_MAP_FUNC_OP_DECL(~, n, BOOST_PP_EMPTY())
# define BOOST_PP_LOCAL_LIMITS (1, PARAGRAPH_MAX_VIEWS)
# include BOOST_PP_LOCAL_ITERATE()
# undef STAPL_MAP_FUNC_OP_DECL

}; // class map_factory

#    define BOOST_PP_ITERATION_LIMITS (1, PARAGRAPH_MAX_VIEWS)
#    define BOOST_PP_FILENAME_1 "stapl/skeletons/explicit/map_prototype.hpp"
#    include BOOST_PP_ITERATE()
#  endif // STAPL_SKELETONS_EXPLICIT_MAKE_SKELETONS_H

#else // BOOST_PP_IS_ITERATING

#define i BOOST_PP_ITERATION()


// map_factory::operator()(...)
template <typename WF, bool b_coarse, typename SchedInfo>
template <typename TGV, BOOST_PP_ENUM_PARAMS(i, typename V)>
void
map_factory<WF, b_coarse, SchedInfo>::operator()(TGV const& tgv,
                                      BOOST_PP_ENUM_BINARY_PARAMS(i, V, & view))
{
  // typedefs for view_index_iterator types
# define STAPL_SKELETONS_MAP_ID_TYPEDEF(z, n, nothing) \
   typedef view_index_iterator<V ## n> id_iter_type ## n;
  BOOST_PP_REPEAT(i, STAPL_SKELETONS_MAP_ID_TYPEDEF, none)
# undef STAPL_SKELETONS_MAP_ID_TYPEDEF

  // if this is the first invocation of operator(), do initialization
  if (!this->initialized())
  {
    tuple<BOOST_PP_ENUM_PARAMS(i, id_iter_type)> id_it =
      partition_id_set(BOOST_PP_ENUM_PARAMS(i, view));

    //store type erased id_iterators in container of base class
#   define STAPL_SKELETONS_MAP_ID_PUSHBACK(z, n, nothing) \
      this->set_view_index_iterator(n, new id_iter_type ## n (get<n>(id_it)));
    BOOST_PP_REPEAT(i, STAPL_SKELETONS_MAP_ID_PUSHBACK, none)
#   undef STAPL_SKELETONS_MAP_ID_PUSHBACK

    // assume partition is uniform to avoid global comm to find view size.
    const float ratio_per_call = static_cast<float>(m_percent_per_call) / 100;
    m_tasks_per_call =
      std::max((int)(this->view_indices_size() * ratio_per_call), 1);
  }

# define STAPL_SKELETONS_MAP_ID_INIT(z, n, nothing) \
    id_iter_type ## n * iter ## n = \
      static_cast<id_iter_type ## n *>(this->get_view_index_iterator(n));
  BOOST_PP_REPEAT(i, STAPL_SKELETONS_MAP_ID_INIT, none)
# undef STAPL_SKELETONS_MAP_ID_INIT

  size_t task_cnt = 0;

  // create the set of map tasks
# define STAPL_SKELETONS_MAP_ID_INCREMENT(z, n, nothing) \
    ++(*iter ## n)
  for (; /* task_cnt < m_tasks_per_call && */ !iter0->at_end();
       BOOST_PP_ENUM(i, STAPL_SKELETONS_MAP_ID_INCREMENT, none), ++task_cnt)
# undef STAPL_SKELETONS_MAP_ID_INCREMENT
  {
#   define STAPL_SKELETONS_MAP_ID_MAKE_PAIR(z, n, nothing) \
      std::make_pair(&view ## n, **iter ## n)
    add_task_helper<WF>()(tgv, SchedInfo(),
      choose_wf<V0, b_coarse>()(
        static_cast<WF const&>(*this), coarse_map_wf<WF>(*this)),
      BOOST_PP_ENUM(i, STAPL_SKELETONS_MAP_ID_MAKE_PAIR, none));
#   undef STAPL_SKELETONS_MAP_ID_MAKE_PAIR
  }

  if (iter0->at_end())
    this->m_finished = true;
}

# ifdef STAPL_ENABLE_NESTED_PARALLELISM
#  define MAP_HELPER_member(z, n, data)    T ## n const& m_t ## n;
#  define MAP_HELPER_args_init(z, n, data) BOOST_PP_COMMA_IF(n) m_t ## n(t ## n)
//////////////////////////////////////////////////////////////////////
/// @brief Work function used to wrap the construction and execution of a
/// PARAGRAPH when it is in a nested parallel section.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
template<typename Map, BOOST_PP_ENUM_PARAMS(i, typename T)>
struct BOOST_PP_CAT(map_helper_wf, i)
{
  typedef void result_type;

  Map const& m_fmap;
  BOOST_PP_REPEAT(i, MAP_HELPER_member, ~)

  BOOST_PP_CAT(map_helper_wf, i)(Map const& fmap,
                                 BOOST_PP_ENUM_BINARY_PARAMS(i, T, const& t))
    : m_fmap(fmap) BOOST_PP_COMMA_IF(i)
      BOOST_PP_REPEAT(i, MAP_HELPER_args_init, ~)
  {}

  void operator()(void)
  {
    paragraph<default_scheduler, map_factory<Map>, BOOST_PP_ENUM_PARAMS(i,T)>
      (map_factory<Map>(m_fmap), BOOST_PP_ENUM_PARAMS(i, m_t))();
  }
};
#  undef MAP_HELPER_args_init
#  undef MAP_HELPER_member


//////////////////////////////////////////////////////////////////////
/// @brief Construct and execute a PARAGRAPH that will perform a map operation,
/// applying the fine-grain work function to the element of the views
/// provided.
///
/// @param fmap Fine-grain map work function.
/// @param t    One or more views to process with the map work function.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
template<typename Map, BOOST_PP_ENUM_PARAMS(i, typename T)>
paragraph<
  default_scheduler, map_factory<Map>, BOOST_PP_ENUM_PARAMS(i,T)>
void
map_func(Map const& fmap, BOOST_PP_ENUM_BINARY_PARAMS(i, T, const& t))
{
  return paragraph<
    default_scheduler, map_factory<Map>, BOOST_PP_ENUM_PARAMS(i,T)
  >(map_factory<Map>(fmap), BOOST_PP_ENUM_PARAMS(i, t))();
}
# endif // STAPL_ENABLE_NESTED_PARALLELISM

namespace prototype {

//////////////////////////////////////////////////////////////////////
/// @brief Construct a PARAGRAPH that will perform a map operation,
/// applying the fine-grain work function to the element of the views
/// provided, and invoke its execution as a non-blocking operation.
///
/// @param fmap Fine-grain map work function.
/// @param t    One or more views to process with the map work function.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
template<bool b_coarse = true,
         typename Map, BOOST_PP_ENUM_PARAMS(i, typename T)>
void
map_func(Map const& fmap, BOOST_PP_ENUM_BINARY_PARAMS(i, T, const& t))
{
  typedef paragraph<
    default_scheduler, map_factory<Map, b_coarse>, BOOST_PP_ENUM_PARAMS(i,T)
  > tg_t;

  (*new tg_t(map_factory<Map, b_coarse>(fmap),
             BOOST_PP_ENUM_PARAMS(i, t)))((int) 0);
}

} // namespace prototype

# undef i

#endif // ifndef BOOST_PP_IS_ITERATING

#ifndef BOOST_PP_IS_ITERATING
#  ifndef STAPL_SKELETONS_EXPLICIT_MAP_PROTOTYPE_FOOTER_H
#    define STAPL_SKELETONS_EXPLICIT_MAP_PROTOTYPE_FOOTER_H

} // namespace stapl

#  endif
#endif // ifndef BOOST_PP_IS_ITERATING
