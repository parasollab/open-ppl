/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/paragraph/paragraph.hpp>
#include <stapl/skeletons/map.hpp>
#include <stapl/containers/array/static_array_fwd.hpp>
#include <stapl/containers/distribution/container_manager/ordering/aux_info_ranking.hpp>
#include <stapl/containers/distribution/container_manager/ordering/base_container_ordering.hpp>

#include <stapl/views/array_view.hpp>
#include <stapl/views/repeated_view.hpp>

#include <cmath>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines a wrapper over the auxiliary information to provide
///        the interface required for the execution of tasks.
//////////////////////////////////////////////////////////////////////
template <typename View, typename CID>
struct aux_info_wrapper
{
  typedef typename View::reference::iterator::reference subview_type;
  typedef aux_info_wrapper                              cid_type;

  View    m_view;
  CID     m_cid;

  aux_info_wrapper(View const& vw, CID const& cid)
    : m_view(vw), m_cid(cid)
  { }

  bool is_local(void) const
  {
    return (get_location_id() == m_cid.location);
  }

  void pre_execute(void)
  { }

  void post_execute(void)
  { }

  void define_type(typer& t)
  {
    t.member(m_view);
    t.member(m_cid);
  }

  subview_type get_subview(cid_type const&) const
  {
    typename View::reference::iterator it = m_view[m_cid.location].begin();
    return *(it+m_cid.id);
  }

  locality_info locality(cid_type const&) const
  {
    return locality_info(
      LQ_CERTAIN, m_cid.affinity, m_cid.handle, m_cid.location
    );
  }
}; // struct aux_info_wrapper


//////////////////////////////////////////////////////////////////////
/// @brief Work function used to compute the base container ranking.
///
/// The work function spawns new tasks based on the stage it is
/// executing.
/// @tparam Node Entry node type in the ordering base container list.
/// @tparam View View type over a container of vector of auxiliary
///              information.
//////////////////////////////////////////////////////////////////////
template<typename Node, typename View>
class compute_rank_wf
{
public:
  typedef Node                           node_type;
  typedef typename node_type::ptr_type   ptr_type;
  typedef void                           result_type;

  enum stages {INIT, RANK_NEXT, PREVIOUS};

private:
  const size_t    m_rounds;
  const size_t    m_round;
  const size_t    m_rank;
  const ptr_type  m_prev_next_id;
  const stages    m_stage;
  const View      m_view;

  template <typename Ptr>
  bool is_valid(Ptr const& ptr) const
  {
    return ptr.location != index_bounds<location_type>::invalid();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Adds new tasks to the previous and next base container to
  ///        continue computing the ranking.
  ///
  /// The previous and next base containers are determined based on
  /// the @p round and the given auxiliary information.
  //////////////////////////////////////////////////////////////////////
  template <typename TGV, typename AuxInfo>
  void add_task_prev_next(TGV const& tgv, AuxInfo& curr_node, size_t round)
  {
    if (is_valid(curr_node.get_next(round))) {
      compute_rank_wf utwf(m_rounds, round+1,
                           curr_node.get_accum_rank(round),
                           curr_node.get_prev(round),
                           RANK_NEXT,
                           m_view);
      typedef aux_info_wrapper<View,ptr_type> aux_wrapper_t;
      std::pair<aux_wrapper_t*, aux_wrapper_t>
            cv(NULL, aux_wrapper_t(m_view, curr_node.get_next(round)));

      cv.first = &cv.second;

      tgv.add_task(utwf, localize_ref(tgv), cv);
    }
    else {
      compute_rank_wf utwf(m_rounds, round+1,
                           index_bounds<size_t>::invalid(),
                           ptr_type(),
                           PREVIOUS,
                           m_view);
      utwf(tgv, curr_node);
    }

    if (is_valid(curr_node.get_prev(round))) {
      compute_rank_wf utwf(m_rounds, round+1,
                           index_bounds<size_t>::invalid(),
                           curr_node.get_next(round),
                           PREVIOUS,
                           m_view);
      typedef aux_info_wrapper<View,ptr_type> aux_wrapper_t;
      std::pair<aux_wrapper_t*, aux_wrapper_t>
            cv(NULL, aux_wrapper_t(m_view, curr_node.get_prev(round)));

      cv.first = &cv.second;

      tgv.add_task(utwf, localize_ref(tgv), cv);
    }
    else {
      compute_rank_wf utwf(m_rounds, round+1,
                           0,
                           ptr_type(),
                           RANK_NEXT,
                           m_view);
      utwf(tgv, curr_node);
    }
  }

public:
  compute_rank_wf(size_t rounds, View const& view)
    : m_rounds(rounds),
      m_round(0),
      m_rank(index_bounds<size_t>::invalid()),
      m_stage(INIT),
      m_view(view)
  { }

  compute_rank_wf(size_t rounds, size_t round, size_t rank,
                  ptr_type const& prev_next_id, stages stage, View const& view)
    : m_rounds(rounds),
      m_round(round),
      m_rank(rank),
      m_prev_next_id(prev_next_id),
      m_stage(stage),
      m_view(view)
  { }

  void define_type(typer& t)
  {
    t.member(m_rounds);
    t.member(m_round);
    t.member(m_rank);
    t.member(m_prev_next_id);
    t.member(m_stage);
    t.member(m_view);
  }

  template <typename TGV, typename AuxInfo>
  void operator()(TGV const& tgv, AuxInfo& bc_aux_info)
  {
    switch (m_stage) {
     case INIT:
        if (m_round != m_rounds) {
          add_task_prev_next(tgv, bc_aux_info, m_round);
        }
        return;

     case RANK_NEXT:
       bc_aux_info.set_rank(m_round,m_rank);
       bc_aux_info.set_prev(m_round,m_prev_next_id);
       break;

     case PREVIOUS:
       bc_aux_info.set_next(m_round,m_prev_next_id);
       break;
   }

    //if both set, update rank
    if (bc_aux_info.can_update_rank(m_round)) {
      if (bc_aux_info.get_accum_rank(m_round-1) !=
          index_bounds<size_t>::invalid()) {
        size_t tmp_rank = bc_aux_info.get_rank(m_round);
        bc_aux_info.set_accum_rank(
                  m_round, bc_aux_info.get_accum_rank(m_round-1)+tmp_rank);

        if (m_round != m_rounds) {
          add_task_prev_next(tgv, bc_aux_info, m_round);

          for (size_t i=m_round+1; i<m_rounds; ++i) {
            if (bc_aux_info.can_update_rank(i)) {
              //set the rank at round i
              size_t tmp_rank = bc_aux_info.get_rank(i);
              bc_aux_info.set_accum_rank(
                        i, bc_aux_info.get_accum_rank(i-1)+tmp_rank);
              if (i != m_rounds)
                add_task_prev_next(tgv, bc_aux_info, i);
              else
                bc_aux_info.update_rank(bc_aux_info.get_accum_rank(i)-1);
            }
            else
              break;
          }
        }
        else {
          bc_aux_info.update_rank(bc_aux_info.get_accum_rank(m_round)-1);
        }
      }
    }
  }
}; // class compute_rank_wf


//////////////////////////////////////////////////////////////////////
/// @brief Work function to start the base container ranking computation.
//////////////////////////////////////////////////////////////////////
template<typename Node>
class start_computation_wf
  : public dynamic_wf
{
private:
  size_t m_steps;

public:
  typedef void result_type;

  start_computation_wf(size_t steps)
    : m_steps(steps)
  { }

  template <typename TGV, typename Reference, typename View>
  void operator()(TGV const& tgv, Reference ref, View vw)
  {
    typedef typename Reference::iterator iter_t;

    iter_t it           = ref.begin();
    const iter_t it_end = ref.end();

    compute_rank_wf<Node, View> wf(m_steps, vw);

    for (; it != it_end; ++it)
    {
      if ((*it).current_rank() != index_bounds<size_t>::invalid())
      {
        typename Reference::iterator::reference val = (*it);

        wf(tgv, val);
      }
    }
  }

  void define_type(typer& t)
  {
    t.member(m_steps);
  }
}; // class start_computation_wf


//////////////////////////////////////////////////////////////////////
/// @brief Work function used to update the base container ranks after
///        compute them.
///
/// Updates the rank of the local base containers.
///
/// @todo This is commented out since its use in @ref base_container_ranking()
///       is commented out.
//////////////////////////////////////////////////////////////////////
#if 0
template <typename Order>
struct flush_rank_wf
{
  typename void result_type;

  Order* m_order;

  flush_rank_wf(Order* order)
    : m_order(order)
    { }

  template <typename Node>
  void operator()(Node node)
  {
    size_t i = 0;
    typename Node::iterator it = node.begin();
    for (;it != node.end(); ++it,++i) {
      if ((*it).current_rank() !=  index_bounds<size_t>::invalid() )
        m_order->set_rank(i,(*it).current_rank());
    }
  }
  void define_type(typer& t)
  {
    t.member(m_order);
  }
};
#endif

//////////////////////////////////////////////////////////////////////
/// @brief Computes and updates the base container rank of the base
///        containers referenced by the given base container ordering
///        object.
///
/// @return Returns the total number of base containers.
///
/// @todo Is the mapfunc() using the flush_rank_wf required or not? It is not
///       clear why it is commented out.
//////////////////////////////////////////////////////////////////////
size_t base_container_ranking(base_container_ordering& order)
{
  // Skip compute the ranking if the ordering is correct
  if (order.m_is_ordered && order.m_total_num_bc != 0)
    return order.m_total_num_bc;

  // Computes the number of base containers
  const size_t total_num_bc   = order.total_number_base_containers();
  size_t num_proc_units = (total_num_bc < order.get_num_locations() ?
                           order.get_num_locations() : total_num_bc);

  size_t steps = size_t(std::ceil(std::log2(double(num_proc_units))));

  typedef aux_info                         bc_aux_info_t;
  typedef std::vector<bc_aux_info_t>       local_aux_info_t;
  typedef static_array<local_aux_info_t>   aux_data_type;

  aux_data_type aux_data(order.get_num_locations());

  futures<local_aux_info_t> h1 = opaque_rmi(
    all_locations, order.get_rmi_handle(),
    &base_container_ordering::get_init_rank, steps
  );

  for (size_t loc=0; loc<h1.size(); ++loc)
    aux_data[loc] = h1.get(loc);

  rmi_fence();

  // Compute ranking
  typedef array_view<aux_data_type> aux_view_t;

  aux_view_t aux_view(aux_data);

  typedef start_computation_wf<base_container_ordering::node_type> wf_t;

  wf_t wf(steps);
  map_func(wf, aux_view, make_repeat_view(aux_view));

  rmi_fence();

  const size_t loc         = order.get_location_id();
  local_aux_info_t tmp_aux = aux_data[loc];
  const size_t num_elem    = tmp_aux.size();

  // Updates the ranks
  for (size_t i = 0; i < num_elem; ++i)
  {
    if (tmp_aux[i].current_rank() != index_bounds<size_t>::invalid())
    {
      order.set_rank(i, tmp_aux[i].current_rank());
    }
  }

  order.m_is_ordered   = true;
  order.m_total_num_bc = total_num_bc;

  // map_func(flush_rank_wf<base_container_ordering>(&order), aux_view);
  // rmi_fence();

  return total_num_bc;
} // base_container_ranking()

} // namespace stapl
