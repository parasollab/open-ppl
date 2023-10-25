/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef BOOST_PP_IS_ITERATING
#  ifndef STAPL_SKELETONS_EXPLICIT_MAP_REDUCE_PROTOTYPE_H
#    define STAPL_SKELETONS_EXPLICIT_MAP_REDUCE_PROTOTYPE_H
#    ifndef PARAGRAPH_MAX_VIEWS
#      define PARAGRAPH_MAX_VIEWS 5
#    endif

#include <boost/preprocessor/repetition/enum_params_with_a_default.hpp>
#include <boost/preprocessor/arithmetic/inc.hpp>
#include <boost/utility/result_of.hpp>

#include <stapl/skeletons/utility/view_index_partition.hpp>
#include <stapl/views/metadata/coarseners/default.hpp>
#include <stapl/skeletons/explicit/coarse_map_reduce_wf.h>
#include <stapl/skeletons/explicit/tree_info.h>
#include <stapl/skeletons/utility/factory_add_task_helper.hpp>

#include <stapl/runtime/type_traits/lazy_storage.hpp>

namespace stapl {

template <typename MapWF, typename ReduceWF, bool b_coarse = false>
struct map_reduce_factory;

namespace paragraph_impl {

template<typename T>
struct result_wf
{
  typedef T result_type;

  template<typename Ref>
  T operator()(Ref x) const
  {
    return x;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Determines the set of balanced binary trees that are needed
/// to represent a reduction of a specified number of view elements.
///
/// @param n     The number of view elements to be processed. A real
///              number  is provided to avoid casting within the function
///              before calling std::log2.
/// @param trees Empty vector that will be populated with the statistics
///              of the trees to be used when generating the task graph.
///
/// @todo Eliminate awkward use of float.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
int compute_combine_trees(float const& n, std::vector<tree_info>& trees);

} // namespace paragraph_impl


//////////////////////////////////////////////////////////////////////
/// @brief Factory that generates the tasks needed for a map reduce operation.
/// A map reduce operation applies the map work function provided on each set
/// of elements of the input views that have the same offset from the
/// beginning of their respective view.  The map operations return values
/// that are combined using the reduce operation provided to form a single
/// result value.  The result value is returned to each location in the gang
/// executing the map_reduce call.
///
/// @tparam MapWF    Fine-grain operation to be applied to each set of elements.
/// @tparam ReduceWF Fine-grain reduce operation used to combine the results of
///                  the applications of the map operation.
/// @tparam b_coarse Indicates whether the map operation provided is
///                  coarse-grain.
///
/// The factory uses @ref coarse_map_reduce_wf as the work function for the map
/// tasks when the MapWF is a fine-grain operation.  The results of the
/// coarse-grain map operations are combined using a binary reduction tree.  If
/// the number of tasks used to perform the coarse-grain map operations isn't a
/// power of two a set of binary trees whose widths are the powers of two that
/// sum to equal the number of coarse-grain map operations is generated, and the
/// results of the trees are combined to form the final result.  The final
/// result of the computation is pushed to all locations using a set of tasks
/// that form a broadcast tree.
///
/// @todo Remove the Result param once we don't need to know it outside of the
///       function operator.
/// @todo Determine if incremental generation related variables can be
///       compressed into less memory (e.g., a bitfield).
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
template <typename MapWF, typename ReduceWF, bool b_coarse>
struct map_reduce_factory
  : public task_factory_base
{
public:
  using coarsener_type = typename std::conditional<b_coarse,
                                                   default_coarsener,
                                                   null_coarsener>::type;

private:
 using coarse_mr_t = typename std::conditional<
   b_coarse, coarse_map_reduce_wf<MapWF, ReduceWF>, MapWF>::type;

  MapWF                                                m_map_wf;
  ReduceWF                                             m_red_wf;

  /// Information about the set of trees needed to perform the reduce.
  std::vector<paragraph_impl::tree_info>                  m_trees;

  /// Iterator to the tree in which the task to process an element resides.
  std::vector<paragraph_impl::tree_info>::iterator        m_curr_tree;

  // Incremental Generation Related Variables.

  /// Height of the tree to reduce the result.  It is ceil(log2(# map ops)).
  int                                                  m_n_steps;

  /// Current level of reduce tasks being generated.
  int                                                  m_curr_step;

  /// Percentage of the total number of tasks to generate in each invocation.
  float                                                m_percent_per_call;

  /// @brief Number of tasks to generate in each invocation.  The minimum is one
  ///   to guarantee that the factory will generate its tasks.
  size_t                                               m_tasks_per_call;

  /// Indicates whether all of the map tasks have been specified.
  bool                                                 m_map_specified;

  /// Indicates whether all of the reduce tasks have been specified.
  bool                                                 m_reduce_specified;

  /// @brief Indicates whether the previous invocation exited before all tasks
  ///   of a level were generated.
  bool                                                 m_midstep_exit;

public:
  map_reduce_factory(MapWF const& map_wf,
                     ReduceWF const& reduce_wf)
    : task_factory_base(true),
      m_map_wf(map_wf),
      m_red_wf(reduce_wf),
      m_curr_step(1), m_percent_per_call(1.0),
      m_map_specified(false), m_reduce_specified(false), m_midstep_exit(false)
  { }

  coarsener_type get_coarsener() const
  { return coarsener_type(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Computes the type of the result of the map_reduce PARAGRAPH for
  /// the set of views, map operation, and reduce operation provided.
  //////////////////////////////////////////////////////////////////////
  template<typename Signature>
  struct result;

  template <typename... V>
  struct result<map_reduce_factory<MapWF, ReduceWF, b_coarse>(V...)>
    : public boost::result_of<coarse_mr_t(typename V::reference...)>
  { };

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Method to add the specifications of the tasks to perform the
  /// reduce operations to the PARAGRAPH.
  ///
  /// @param i        Iterator representing the indices from [0, # map ops)
  ///                 used to generate the reduce tasks at each level of
  ///                 the trees.
  /// @param task_cnt The number of tasks that have been created in the
  ///                 current invocation.
  /// @param tgv      The @ref paragraph_view which receives the
  ///                 specifications of the reduce tasks to be created.
  //////////////////////////////////////////////////////////////////////
  template<typename Edge, typename IdIterator, typename TGV>
  void insert_combine_tasks(IdIterator& i, std::size_t task_cnt,
                            TGV const& tgv);

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Reset all data members related to the number of tasks that have
  ///   been generated in order to prepare the factory for PARAGRAPH
  ///   reinvocation.
  //////////////////////////////////////////////////////////////////////
  void reset()
  {
    this->reset_view_indices();
    this->m_finished = false;

    m_curr_step = 1;
    m_map_specified = false;
    m_reduce_specified = false;
    m_midstep_exit = false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Iterate over the indices of the elements of the views
  /// provided and specify the tasks that will form the binary tree of
  /// map and reduce operations needed to perform the map_reduce computation.
  ///
  /// @param tgv @ref paragraph_view that allows tasks to be added to the
  ///            PARAGRAPH.
  /// @param v   One or more views whose elements will be processed by
  ///            the tasks performing the map operations.
  //////////////////////////////////////////////////////////////////////
# define STAPL_MR_FUNC_OP_DECL(z, n, const_param) \
  template<typename TGV, BOOST_PP_ENUM_PARAMS(n, typename V)> \
  void operator()(TGV const& tgv, BOOST_PP_ENUM_BINARY_PARAMS(n, V, &v));

# define BOOST_PP_LOCAL_MACRO(n) STAPL_MR_FUNC_OP_DECL(~, n, BOOST_PP_EMPTY())
# define BOOST_PP_LOCAL_LIMITS (1, PARAGRAPH_MAX_VIEWS)
# include BOOST_PP_LOCAL_ITERATE()
# undef STAPL_MR_FUNC_OP_DECL

}; // class map_reduce_factory


template<typename MapWF, typename ReduceWF, bool b_coarse>
template<typename Edge, typename IdIterator, typename TGV>
void map_reduce_factory<MapWF, ReduceWF, b_coarse>::
insert_combine_tasks(IdIterator& i, std::size_t task_cnt, TGV const& tgv)
{
  typename std::vector<paragraph_impl::tree_info>::iterator t;

  for (; m_curr_step <= m_n_steps; ++m_curr_step)
  {
    int two_sm1 = static_cast<int>(std::pow(2.0, m_curr_step-1));
    int two_s   = 2*two_sm1;
#if 0
    int two_sp1 = 2*two_s;
#endif

    // If the last call didn't finish the step don't reset the iterators.
    if (!m_midstep_exit)
    {
      i->reset();
      m_curr_tree = m_trees.begin();
    }
    else
      m_midstep_exit = false;

    for (; /*task_cnt < m_tasks_per_call && */ !i->at_end(); ++(*i))
    {
      t = std::find_if(m_trees.begin(), m_trees.end(),
                       paragraph_impl::contains_leaf(**i));
      stapl_assert(t != m_trees.end(),
         "subview id not part of any tree in map_reduce_factory");
      if (m_curr_step <= t->height)
      {
        if (**i % two_s == 0)
        {
          const size_t tid       = t->curr_id + (**i - t->leaf_offset)/two_s;
          const size_t tid_left  =
            t->curr_id - t->width/two_sm1 + 2*(**i - t->leaf_offset)/two_s;
          const size_t tid_right = tid_left + 1;
#if 0
          size_t tid_succ;

          if (m_curr_step < t->height)
           tid_succ
             = t->curr_id + t->width/two_s + (**i - t->leaf_offset)/two_sp1;
          else
           tid_succ =
             m_trees.back().root_task_id + std::distance(t, m_trees.end());
          add_task(..., produce(tid_succ), ...);
#endif
          const size_t num_succs = 1;
          tgv.add_task(tid, m_red_wf, num_succs,
                       consume<Edge>(tgv, tid_left),
                       consume<Edge>(tgv, tid_right));
          ++task_cnt;
        }
      }
    }

#if 0
    if (task_cnt == m_tasks_per_call)
    {
      m_midstep_exit = true;
      return;
    }
#endif

    for (; /*task_cnt < m_tasks_per_call && */ m_curr_tree != m_trees.end();
        ++m_curr_tree)
    {
      if (m_curr_step <= m_curr_tree->height)
        m_curr_tree->curr_id += m_curr_tree->width/two_s;
      else
      {
        if ((m_curr_step == m_curr_tree->height + 1)
            && (std::distance(m_curr_tree, m_trees.end()) > 1))
        {
          const size_t tid =
            m_trees.back().root_task_id
            + std::distance(m_curr_tree, m_trees.end()) - 1;

          if (i->contains(m_curr_tree->leaf_offset))
          {
            const size_t tid_left  = m_curr_tree->root_task_id;
            const size_t tid_right = (m_curr_tree+1)->root_task_id;

            //const size_t tid_succ = tid + 1;
            //add_task(..., produce(tid_succ), ...);
            const size_t num_succs = 1;
            tgv.add_task(tid, m_red_wf, num_succs,
                         consume<Edge>(tgv, tid_left),
                         consume<Edge>(tgv, tid_right));
          }

          m_curr_tree->root_task_id = tid;
        }
      }
    }

#if 0
    if (task_cnt == m_tasks_per_call)
    {
      m_midstep_exit = true;
      return;
    }
#endif
  }

  const size_t loc        = this->get_location_id();
  const size_t retval_tid = m_trees.back().root_task_id + m_trees.size() + loc;

  size_t parent_tid;

  if (loc == 0)
    parent_tid = m_trees.back().root_task_id + m_trees.size() - 1;
  else
    parent_tid = m_trees.back().root_task_id + m_trees.size() + loc / 2;

  // compute the set of tasks consuming this result task.
  size_t nlocs = this->get_num_locations();

  // The local return value will at consume result_wf.
  size_t num_succs = 1;

  // Add additional consumers if I'm a non-leaf on the broadcast tree.
  if (loc == 0)
  {
    // location 0 is only consumed by location 1 if it exists.
    if (nlocs != 1)
      ++num_succs;
      //successors.push_back(1);
  }
  else if (nlocs == 2*loc + 1)
  {
    // if the tree isn't full
#if 0
    successors.push_back(2*loc);
#endif

    ++num_succs;
  }
  else if (loc < nlocs/2)
  {
    num_succs += 2;
#if 0
    successors.push_back(2*loc);
    successors.push_back(2*loc+1);
#endif
  }

#if 0
  std::cout << get_location_id() << ": adding result_wf with tid "
            << retval_tid << " consuming " << parent_tid << " with succs = "
            << num_succs << "\n";
#endif

  tgv.add_task(retval_tid, paragraph_impl::result_wf<Edge>(), num_succs,
               consume<Edge>(tgv, parent_tid));

  tgv.set_result(retval_tid); // consume<Edge>(tgv, retval_tid) ???

  m_reduce_specified = true;
} // insert_combine_tasks()


namespace prototype {

namespace result_of {

//////////////////////////////////////////////////////////////////////
/// @brief Computes the result type of a map_reduce PARAGRAPH.
///
/// Used in the implementation of the non-blocking composable PARAGRAPHs.
///
/// @ingroup skeletonsExplicitFactories
//////////////////////////////////////////////////////////////////////
template<typename Map, typename Reduce, bool b_coarse,
BOOST_PP_ENUM_PARAMS_WITH_A_DEFAULT(
  BOOST_PP_INC(PARAGRAPH_MAX_VIEWS), typename V, void
)
>
struct map_reduce;

} // namespace result_of

} // namesapce prototype

} // namespace stapl

#    define BOOST_PP_ITERATION_LIMITS (1, PARAGRAPH_MAX_VIEWS)
#    define BOOST_PP_FILENAME_1 \
       "stapl/skeletons/explicit/map_reduce_prototype.hpp"
#    include BOOST_PP_ITERATE()
#  endif // STAPL_SKELETONS_EXPLICIT_MAP_REDUCE_PROTOTYPE_H

#else // BOOST_PP_IS_ITERATING

#define i BOOST_PP_ITERATION()

namespace stapl {

# define STAPL_SKELETONS_MR_RESULT(z, n, nothing) \
  typename V ## n::reference

// map_reduce_factory::operator()(...)
template<typename MapWF, typename ReduceWF, bool b_coarse>
template <typename TGV, BOOST_PP_ENUM_PARAMS(i, typename V)>
void map_reduce_factory<MapWF, ReduceWF, b_coarse>::
operator()(TGV const& tgv, BOOST_PP_ENUM_BINARY_PARAMS(i, V, & view))
{
  // typedefs for view_index_iterator types
# define STAPL_SKELETONS_MR_ID_TYPEDEF(z, n, nothing) \
   typedef view_index_iterator<V ## n> id_iter_type ## n;
  BOOST_PP_REPEAT(i, STAPL_SKELETONS_MR_ID_TYPEDEF, none)
# undef STAPL_SKELETONS_MR_ID_TYPEDEF

  // if this is the first invocation of operator(), do initialization
  if (!this->initialized())
  {
    tuple<BOOST_PP_ENUM_PARAMS(i, id_iter_type)> id_it =
       partition_id_set(BOOST_PP_ENUM_PARAMS(i, view));

    // store type erase id_iterators on vector in base class
#   define STAPL_SKELETONS_MR_ID_PUSHBACK(z, n, nothing) \
      this->set_view_index_iterator(n, new id_iter_type ## n (get<n>(id_it)));
    BOOST_PP_REPEAT(i, STAPL_SKELETONS_MR_ID_PUSHBACK, none)
#   undef STAPL_SKELETONS_MR_ID_PUSHBACK


    // total num elements needed to determine trees needed.  Requires comm.
    //
    /// @todo get_num_suviews() is really a size() call...
    //
    const size_t n      = view0.get_num_subviews();
    const size_t n_locs = this->get_num_locations();

    m_tasks_per_call    = std::max((int)(2*(n/n_locs) * m_percent_per_call), 1);
    m_n_steps           = paragraph_impl::compute_combine_trees(n, m_trees);
    m_curr_tree         = m_trees.begin();
  }

# define STAPL_SKELETONS_MR_ID_INIT(z, n, nothing) \
    id_iter_type ## n * iter ## n = \
      static_cast<id_iter_type ## n *>(this->get_view_index_iterator(n));
  BOOST_PP_REPEAT(i, STAPL_SKELETONS_MR_ID_INIT, none)
# undef STAPL_SKELETONS_MR_ID_INIT

  size_t task_cnt = 0;

  // create the set of map tasks
  if (!m_map_specified)
  {
    typename std::vector<paragraph_impl::tree_info>::iterator t;

#   define STAPL_SKELETONS_MR_ID_INCREMENT(z, n, nothing) \
      ++(*iter ## n)
    for (; /*task_cnt < m_tasks_per_call && */ !iter0->at_end();
         BOOST_PP_ENUM(i, STAPL_SKELETONS_MR_ID_INCREMENT, none), ++task_cnt)
#   undef STAPL_SKELETONS_MR_ID_INCREMENT
    {
      t = std::find_if(m_trees.begin(), m_trees.end(),
                       paragraph_impl::contains_leaf(**iter0));
      stapl_assert(t != m_trees.end(),
        "subview id not part of any tree in map_reduce_factory");

      size_t tid = **iter0 - t->leaf_offset + t->id_offset;
#if 0
      std::size_t consumer_tid =
        t->width + (**i - t->leaf_offset)/2 + t->id_offset;
#endif
      const std::size_t num_succs = 1;

#     define STAPL_SKELETONS_MR_ID_MAKE_PAIR(z, n, nothing) \
        std::make_pair(&view ## n, **iter ## n)
      add_task_helper<MapWF>()
        (tgv, tid,
         choose_wf<V0, b_coarse>()
           (m_map_wf, coarse_map_reduce_wf<MapWF,ReduceWF>(m_map_wf, m_red_wf)),
         num_succs, BOOST_PP_ENUM(i, STAPL_SKELETONS_MR_ID_MAKE_PAIR, none));
#     undef STAPL_SKELETONS_MR_ID_MAKE_PAIR
    }

    if (iter0->at_end())
      m_map_specified = true;
  }

  // build the binary tree of combine tasks.
  if (m_map_specified && !m_reduce_specified)
  {
    /// @todo remove choose_wf crap or adjust to properly pass mapwf
    // if not new view.
    typedef typename boost::result_of<
      coarse_mr_t(BOOST_PP_ENUM(i, STAPL_SKELETONS_MR_RESULT, ~))
    >::type edge_t;

    insert_combine_tasks<edge_t>(iter0, task_cnt, tgv);

    if (m_reduce_specified)
      this->m_finished = true;
  }
} // operator()

# ifdef STAPL_ENABLE_NESTED_PARALLELISM
#  define MAP_REDUCE_HELPER_member(z, n, data)    T ## n const& m_t ## n;
#  define MAP_REDUCE_HELPER_args_init(z, n, data) \
  BOOST_PP_COMMA_IF(n) m_t ## n(t ## n)
template<typename T, typename Map, typename Reduce, typename Factory,
         BOOST_PP_ENUM_PARAMS(i, typename T)>
struct BOOST_PP_CAT(map_reduce_helper_wf, i)
{
  typedef void result_type;

  lazy_storage<T>& m_s;
  Map const&       m_fmap;
  Reduce const&    m_freduce;
  BOOST_PP_REPEAT(i, MAP_REDUCE_HELPER_member, ~)

  BOOST_PP_CAT(map_reduce_helper_wf, i)(
    lazy_storage<T>& s,
    Map const& fmap, Reduce const& freduce,
    BOOST_PP_ENUM_BINARY_PARAMS(i, T, const& t)) :
    m_s(s), m_fmap(fmap), m_freduce(freduce) BOOST_PP_COMMA_IF(i)
    BOOST_PP_REPEAT(i, MAP_REDUCE_HELPER_args_init, ~) { }

  void operator()(void)
  {
    paragraph<default_scheduler, Factory, BOOST_PP_ENUM_PARAMS(i,T)>
      pr(Factory(m_fmap, m_freduce), BOOST_PP_ENUM_PARAMS(i, m_t));
    if (get_location_id()==0)
      m_s.construct(pr());
    else
      pr();
  }
};
#  undef MAP_REDUCE_HELPER_args_init
#  undef MAP_REDUCE_HELPER_member

#  if i==1

template<typename T>
struct find_value_type
{
  typedef T type;
};

template<typename T, typename A>
struct find_value_type<proxy<T,A> >
{
  typedef typename proxy_core_access::value_type<
   proxy<T,A>
  >::type                                                    proxy_value_type;
  typedef typename find_value_type<proxy_value_type>::type   type;
};

#  endif

// map_reduce(...) with user provided fine grain map wf
template<typename Map, typename Reduce, BOOST_PP_ENUM_PARAMS(i, typename V)>
inline
typename find_value_type<
 typename boost::result_of<
  paragraph<default_scheduler,
         map_reduce_factory<Map, Reduce, false>,
         BOOST_PP_ENUM_PARAMS(i,V)>()
 >::type
>::type // @todo cannot handle proxies :(
map_reduce(Map const& fmap, Reduce const& freduce,
           BOOST_PP_ENUM_BINARY_PARAMS(i, V, const& v))
{
  typedef map_reduce_factory<Map, Reduce, false> factory_t;

  return paragraph<default_scheduler, factory_t, BOOST_PP_ENUM_PARAMS(i,V)>
    (factory_t(fmap, freduce), BOOST_PP_ENUM_PARAMS(i, v))();
}

# endif  // STAPL_ENABLE_NESTED_PARALLELISM

namespace prototype {

namespace result_of {

template<typename Map, typename Reduce, bool b_coarse,
         BOOST_PP_ENUM_PARAMS(i, typename V)>
struct map_reduce<Map, Reduce, b_coarse, BOOST_PP_ENUM_PARAMS(i, V)>
  : public boost::result_of<
      paragraph<default_scheduler,
             map_reduce_factory<Map, Reduce, b_coarse>,
             BOOST_PP_ENUM_PARAMS(i,V)
            >()
    >
{ };

} // namespace result_of


// map_reduce(...)
template <bool b_coarse = true, typename Map, typename Reduce,
          BOOST_PP_ENUM_PARAMS(i, typename V)>
inline typename result_of::map_reduce<Map, Reduce, b_coarse,
                                      BOOST_PP_ENUM_PARAMS(i, V)>::type
map_reduce(Map const& fmap, Reduce const& freduce,
           BOOST_PP_ENUM_BINARY_PARAMS(i, V, const& v)) {
  typedef map_reduce_factory<Map, Reduce, b_coarse>       factory_t;
  typedef paragraph<default_scheduler,
                 factory_t, BOOST_PP_ENUM_PARAMS(i,V)>    tg_t;

  return (*new tg_t(factory_t(fmap, freduce),
                    BOOST_PP_ENUM_PARAMS(i, v)))((int) 0);
}

} // namespace prototype

} // namespace stapl

# undef STAPL_SKELETONS_MR_RESULT
# undef i

#endif // ifndef BOOST_PP_IS_ITERATING
