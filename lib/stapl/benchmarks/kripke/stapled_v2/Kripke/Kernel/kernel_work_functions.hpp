/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_KERNEL_WORK_FUNCTIONS_HPP
#define STAPL_BENCHMARKS_KERNEL_WORK_FUNCTIONS_HPP

#include <stapl/views/slices_view.hpp>
#include <stapl/views/extended_view.hpp>
#include <stapl/views/cross_view.hpp>
#include <stapl/views/functor_view.hpp>
#include <stapl/views/index_view.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/utility/integer_sequence.hpp>

#include <stapl/skeletons/functional/wavefront.hpp>
#include <stapl/skeletons/transformations/wrapped_skeleton.hpp>
#include <stapl/skeletons/transformations/coarse.hpp>
#include <stapl/skeletons/transformations/coarse/compose.hpp>

#include "../../sweep_filters.hpp"


#include <stapl/runtime/counter/probe.hpp>

// #define KRIPKE_ENABLE_CALLGRIND

#ifdef KRIPKE_ENABLE_CALLGRIND
#  include <valgrind/callgrind.h>
#endif

struct scoped_callgrind
{
  scoped_callgrind()
  {
#   ifdef KRIPKE_ENABLE_CALLGRIND
      CALLGRIND_START_INSTRUMENTATION;
      CALLGRIND_TOGGLE_COLLECT;
#   endif
  }

  ~scoped_callgrind()
  {
#   ifdef KRIPKE_ENABLE_CALLGRIND
      CALLGRIND_TOGGLE_COLLECT;
      CALLGRIND_STOP_INSTRUMENTATION;
      CALLGRIND_DUMP_STATS;
#   endif
  }
};

//
// General
//
using stapl::use_default;


//
// Container
//
using stapl::multiarray;


//
// Algorithm Specification
//
using stapl::skeletons::direction;
using stapl::default_coarsener;
using stapl::skeletons::execute;
using stapl::skeletons::sink;
using stapl::skeletons::spans::blocked;
using stapl::map_func;
using stapl::skeletons::wavefront;
using stapl::skeletons::wrap;
using stapl::skeletons::zip;
using stapl::skeletons::skeleton_traits;
using stapl::skeletons::spans::blocked;
using stapl::skeletons::execution_params;
using stapl::skeletons::coarse;
using stapl::pg_aware_multiview_coarsener;
//
// View Creation, Element Access
//
using stapl::make_array_view;
using stapl::make_repeat_view;
using stapl::make_repeat_view_nd;
using stapl::make_slices_view;
using stapl::make_extended_view;
using stapl::index_of;
using stapl::multiarray_view;
using stapl::functor_view;
using stapl::functor_view_type;
using stapl::functor_container;
using stapl::index_sequence;
using stapl::promise;
using stapl::index_view;
using stapl::indexed_domain;
using stapl::lightweight_vector;
using stapl::lightweight_multiarray;

// Object Serialization
using stapl::typer;

class functional_diamond_difference_wf
{
private:

  int                       m_num_directions;
  std::vector<Directions>   m_directions;
  std::vector<double> const  m_dx;
  std::vector<double> const  m_dy;
  std::vector<double> const  m_dz;

public:
  functional_diamond_difference_wf(int num_directions, Directions* directions,
                                   std::vector<double> const* dx,
                                   std::vector<double> const* dy,
                                   std::vector<double> const* dz)
    : m_num_directions(num_directions), m_directions(num_directions),
      m_dx(*dx), m_dy(*dy), m_dz(*dz)
  {
    for (int i = 0; i < num_directions; ++i)
    {
      m_directions[i] = directions[i];
    }
  }

  typedef std::array<double, 3> result_type;

  template<typename PSI, typename RHS, typename SIGT, typename Index,
           typename IBndry, typename JBndry, typename KBndry>
  result_type
  __attribute__((always_inline))
  operator()(PSI&& psi, RHS&& rhs, SIGT&& sigt, Index&& psi_idx,
             IBndry&& lf_bndry, JBndry&& fr_bndry, KBndry&& bo_bndry) const
  {
     int d = std::get<3>(psi_idx);

     // Adding one to the index is needed to account for boundary cell width.
     double dzk  = m_dz[std::get<2>(psi_idx) + 1];
     double dyj  = m_dy[std::get<1>(psi_idx) + 1];
     double dxi  = m_dx[std::get<0>(psi_idx) + 1];

     double zcos_dzk = 2.0 * m_directions[d].zcos / dzk;
     double ycos_dyj = 2.0 * m_directions[d].ycos / dyj;
     double xcos_dxi = 2.0 * m_directions[d].xcos / dxi;

    // Calculate new zonal flux
    double new_psi = (rhs + lf_bndry * xcos_dxi
                          + fr_bndry * ycos_dyj
                          + bo_bndry * zcos_dzk) /
                     ((xcos_dxi + ycos_dyj + zcos_dzk + sigt));
    psi = new_psi;

    // Apply diamond-difference relationships
    new_psi *= 2.0;
    return {
      {new_psi - lf_bndry, new_psi - fr_bndry, new_psi - bo_bndry}};
  }

  void define_type(typer& t)
  {
    t.member(m_num_directions);
    t.member(m_directions);
    t.member(m_dx);
    t.member(m_dy);
    t.member(m_dz);
  }
}; // class functional_diamond_difference_wf


class diamond_difference_wf
{
private:
  int                         m_num_directions;
  Directions*                 m_directions;
  std::vector<double> const*  m_dx;
  std::vector<double> const*  m_dy;
  std::vector<double> const*  m_dz;

public:
  diamond_difference_wf(int num_directions, Directions* directions,
                        std::vector<double> const* dx,
                        std::vector<double> const* dy,
                        std::vector<double> const* dz)
    : m_num_directions(num_directions), m_directions(directions),
      m_dx(dx), m_dy(dy), m_dz(dz)
  { }


  typedef void result_type;

  template<typename PSI, typename RHS, typename SIGT,
           typename IBndry, typename JBndry, typename KBndry>
  void
  operator()(PSI&& psi, RHS&& rhs, SIGT&& sigt,
             IBndry&& lf_bndry, JBndry&& fr_bndry, KBndry&& bo_bndry) const
  {
     auto psi_idx = stapl::get_proxy_index<PSI>()(psi);
     int d        = std::get<3>(psi_idx);

     // Adding one to the index is needed to account for boundary cell width.
     double dzk  = (*m_dz)[std::get<2>(psi_idx) + 1];
     double dyj  = (*m_dy)[std::get<1>(psi_idx) + 1];
     double dxi  = (*m_dx)[std::get<0>(psi_idx) + 1];

     double zcos_dzk = 2.0 * m_directions[d].zcos / dzk;
     double ycos_dyj = 2.0 * m_directions[d].ycos / dyj;
     double xcos_dxi = 2.0 * m_directions[d].xcos / dxi;

     // Calculate new zonal flux
     double new_psi = (rhs + lf_bndry * xcos_dxi
                           + fr_bndry * ycos_dyj
                           + bo_bndry * zcos_dzk) /
                      ((xcos_dxi + ycos_dyj + zcos_dzk + sigt));
     psi = new_psi;

     // Apply diamond-difference relationships
     new_psi *= 2.0;
     lf_bndry = new_psi - lf_bndry;
     fr_bndry = new_psi - fr_bndry;
     bo_bndry = new_psi - bo_bndry;
  }

  void define_type(typer& t)
  {
    t.member(m_num_directions);
    t.member(m_directions, m_num_directions);
    t.member(m_dx);
    t.member(m_dy);
    t.member(m_dz);
  }
}; // class diamond_difference_wf


template<typename GridData, typename PSI>
diamond_difference_wf
make_diamond_difference_wf(GridData* grid_data, PSI&&,
                           int group_set, int direction_set,
                           std::tuple<size_t, size_t, size_t> cellset_id)
{
  Group_Dir_Set& gd_set = grid_data->gd_sets()[group_set][direction_set];
  Directions *direction = gd_set.directions;

#ifndef STAPL_NDEBUG
  int octant = direction[0].octant;
  std::pair<Grid_Sweep_Block, Grid_Sweep_Block> extents =
    adjust_extent(grid_data->octant_extent()[octant],
                  grid_data, cellset_id);
#endif

  std::vector<double> const* dx = &grid_data->deltas(0);
  std::vector<double> const* dy = &grid_data->deltas(1);
  std::vector<double> const* dz = &grid_data->deltas(2);

  stapl_assert((int)grid_data->deltas(0).size() ==
                 std::max(extents.second.start_i+1,extents.second.end_i)+2,
               "size and index of dx results in out-of-bounds access");
  stapl_assert((int)grid_data->deltas(1).size() ==
                 std::max(extents.second.start_j+1,extents.second.end_j)+2,
               "size and index of dy results in out-of-bounds access");
  stapl_assert((int)grid_data->deltas(2).size() ==
                 std::max(extents.second.start_k+1,extents.second.end_k)+2,
               "size and index of dz results in out-of-bounds access");

  return diamond_difference_wf(
    gd_set.num_directions, direction, dx, dy, dz);
}


template<typename GridData, typename PSI>
functional_diamond_difference_wf
make_functional_diamond_difference_wf(GridData* grid_data, PSI&&,
                                      int group_set, int direction_set,
                                      std::tuple<size_t, size_t, size_t>
                                        cellset_id)
{
  Group_Dir_Set& gd_set = grid_data->gd_sets()[group_set][direction_set];
  Directions *direction = gd_set.directions;

#ifndef STAPL_NDEBUG
  int octant = direction[0].octant;
  std::pair<Grid_Sweep_Block, Grid_Sweep_Block> extents =
    adjust_extent(grid_data->octant_extent()[octant],
                  grid_data, cellset_id);
#endif

  std::vector<double> const* dx = &grid_data->deltas(0);
  std::vector<double> const* dy = &grid_data->deltas(1);
  std::vector<double> const* dz = &grid_data->deltas(2);

  stapl_assert((int)grid_data->deltas(0).size() ==
                 std::max(extents.second.start_i+1,extents.second.end_i)+2,
               "size and index of dx results in out-of-bounds access");
  stapl_assert((int)grid_data->deltas(1).size() ==
                 std::max(extents.second.start_j+1,extents.second.end_j)+2,
               "size and index of dy results in out-of-bounds access");
  stapl_assert((int)grid_data->deltas(2).size() ==
                 std::max(extents.second.start_k+1,extents.second.end_k)+2,
               "size and index of dz results in out-of-bounds access");
  return functional_diamond_difference_wf(
    gd_set.num_directions, direction, dx, dy, dz);
}

struct get_ell
{
private:
  int m_num_dirs;
  int m_num_moments;

public:
  typedef std::vector<double> result_type;

  get_ell(int num_dirs, int num_moments)
    : m_num_dirs(num_dirs), m_num_moments(num_moments)
  { }

  template<typename Element>
  result_type operator()(Element&& e)
  {
    result_type res(m_num_dirs*m_num_moments, 0.);
    auto idx = stapl::index_of(e);
    res[idx] = e;
    return res;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_num_dirs);
    t.member(m_num_moments);
  }
};

struct vec_sum
{
  typedef std::vector<double> result_type;

  template<typename Vec0, typename Vec1>
  result_type operator()(Vec0&& v0, Vec1&& v1)
  {
    result_type res(v0);
    int sz = res.size();
    for (int i = 0; i != sz; ++i)
      res[i] += v1[i];
    return res;
  }
};


template<int RepeatDim, typename Slice>
struct ell_op_spawner;

template<int RepeatDim, std::size_t ...Indices>
struct ell_op_spawner<RepeatDim, index_sequence<Indices...>>
  : public stapl::p_object
{
  template<typename SlicedPkg0, typename RepeatedPkg, typename SlicedPkg1,
           typename WF>
  ell_op_spawner(SlicedPkg0&& slice_pkg0, RepeatedPkg&& repeat_pkg,
                 SlicedPkg1&& slice_pkg1, WF wf, stapl::promise<void> p)
  {
    stapl::gang g(*this);

    auto to_slice0 = slice_pkg0();
    auto to_repeat = repeat_pkg();
    auto to_slice1 = slice_pkg1();

    // The to_repeat matrix distribution doesn't match that of the
    // sliced views. It is relatively small, so we explicitly replicate it.
    // Otherwise, we need a way to push it into shared, read-only access
    // so all locations can read it without an RMI request.
    std::vector<double> serial_repeat =
      stapl::map_reduce(
        get_ell(wf.num_directions(), wf.num_moments()), vec_sum(),
        stapl::linear_view(to_repeat));

    auto sliced0  = make_slices_view<Indices...>(to_slice0);
    auto repeated = make_repeat_view_nd<RepeatDim>(serial_repeat);
    auto sliced1  = make_slices_view<Indices...>(to_slice1);

    map_func(wf, sliced0, repeated, sliced1);

    stapl::do_once([&] { p.set_value(); });
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Spawn the LTimes or LPlusTimes computation over the set of
/// locations on which the containers for a zoneset are stored.
///
/// This function is used to create the nested parallel section in which
/// the inner map_func over the elements associated with a zoneset is executed.
/// The function differs from invoke_kernel in that it doesn't have to handle
/// boundary views in the input and there is no return from the nested
/// computation.
//////////////////////////////////////////////////////////////////////
template<int RepeatDim, typename Slice,
         typename SlicedView0, typename RepeatedView, typename SlicedView1,
         typename WF>
void
invoke_ell_op(SlicedView0& sliced0, RepeatedView& repeated,
              SlicedView1& sliced1, WF wf)
{
  stapl::promise<void> p;
  auto f = p.get_future();

  using spawner_t = ell_op_spawner<RepeatDim, Slice>;

  stapl::async_construct<spawner_t>(
    [](spawner_t* s) { delete s; },
    sliced0.container().get_rmi_handle(), stapl::all_locations,
    stapl::transporter_packager()(sliced0),
    stapl::transporter_packager()(repeated),
    stapl::transporter_packager()(sliced1),
    wf, std::move(p)
  );

  // wait on signal that nested PARAGRAPH has finished execution
  f.get();
}


template <typename CID>
struct cid_matches
{
private:
  CID m_cid;

public:
  cid_matches(CID const& cid)
    : m_cid(cid)
  { }

  template <typename T>
  bool operator()(T const& r) const
  { return r.first == m_cid; }
};



template <typename PartitionedDims>
struct partitioned_dims_unequal;

template <std::size_t... PartitionedDims>
struct partitioned_dims_unequal<index_sequence<PartitionedDims...>>
{
private:
  template<int k, typename Index0, typename Index1>
  static bool not_equal_in(Index0 const& i0, Index1 const& i1)
  { return std::get<k>(i0) != std::get<k>(i1); }

public:
  template <typename Index0, typename Index1>
  static bool apply(Index0 const& i0, Index1 const& i1)
  {
    return stapl::pack_ops::functional::or_(
      not_equal_in<PartitionedDims>(i0, i1)...);
  }
};


template <typename DestView, typename PartitionedDims>
struct remapping_spawner
  : public stapl::p_object
{
private:
  // pointer to the destination view used when processing incoming elements.
  DestView* m_dest;

public:
  template<typename SourcePkg, typename DestPkg>
  remapping_spawner(SourcePkg&& source_pkg, DestPkg&& dest_pkg,
                    stapl::promise<void> p)
  {
    scoped_callgrind sc;
    stapl::gang g(*this);

    auto source   = source_pkg();
    auto dest     = dest_pkg();

    m_dest = &dest;

    // advancing the epoch to prevent a location from receiving a call to
    // receive_data before the m_dest pointer is set.
    this->advance_epoch();

    // Get the partition and mapping information from the destination container
    auto partition = dest.container().distribution().partition();
    auto mapper = dest.container().distribution().mapper();

    using bc_dom_t =
      decltype(dest.container().distribution().container_manager().
        begin()->domain());

    using container_t =
      typename std::remove_reference<decltype(dest.container())>::type;

    using cid_t = typename container_t::distribution_type::cid_type;

    using assign_cont_t =
      std::vector<
        std::pair<cid_t,
                  std::vector<std::pair<typename container_t::gid_type,
                                        typename container_t::value_type>>
        >
      >;

    assign_cont_t           assignment_info;
    assignment_info.reserve(this->get_num_locations());

    typename assign_cont_t::iterator assign_iter = assignment_info.end();

    // store the last cid we mapped to in order to avoid calls to find.
    typename container_t::distribution_type::cid_type curr_cid;

    // Iterate over the local base containers of the source
    auto curr_bc =
      source.container().distribution().container_manager().begin();

    auto bc_end = source.container().distribution().container_manager().end();

    for (; curr_bc != bc_end; ++ curr_bc)
    {
      bc_dom_t bc_dom((*curr_bc).domain().first(), (*curr_bc).domain().last());

      typename container_t::gid_type prev_idx(bc_dom.last());
      for (auto idx = bc_dom.first(); idx != bc_dom.last();
           idx = bc_dom.advance(idx,1))
      {
        typename container_t::distribution_type::cid_type cid;
        if (idx == bc_dom.first() ||
            partitioned_dims_unequal<PartitionedDims>::apply(prev_idx, idx))
          cid = stapl::get<0>(partition.find(idx));
        else
          cid = curr_cid;

        if (curr_cid != cid || idx == bc_dom.first())
        {
          curr_cid = cid;
          assign_iter = std::find_if(assignment_info.begin(),
                          assignment_info.end(), cid_matches<cid_t>(curr_cid));
        }

        if (assign_iter != assignment_info.end())
          assign_iter->second.push_back(std::make_pair(idx, (*curr_bc)[idx]));
        else
        {
          assignment_info.push_back(
            std::make_pair(curr_cid,
              std::vector<std::pair<typename container_t::gid_type,
                                    typename container_t::value_type>>()));
          assignment_info.back().second.push_back(
            std::make_pair(idx, (*curr_bc)[idx]));
          assign_iter = assignment_info.end()-1;
        }
        prev_idx = idx;
      }

      // handle last element of the domain
      auto idx = bc_dom.last();
      typename container_t::distribution_type::cid_type cid;
      if (idx == bc_dom.first() ||
          partitioned_dims_unequal<PartitionedDims>::apply(prev_idx, idx))
        cid = stapl::get<0>(partition.find(idx));
      else
        cid = curr_cid;

      if (curr_cid != cid)
      {
        curr_cid = cid;
        assign_iter = std::find_if(assignment_info.begin(),
                        assignment_info.end(), cid_matches<cid_t>(curr_cid));
      }

      if (assign_iter != assignment_info.end())
        assign_iter->second.push_back(std::make_pair(idx, (*curr_bc)[idx]));
      else
      {
        assignment_info.push_back(
          std::make_pair(curr_cid,
            std::vector<std::pair<typename container_t::gid_type,
                                  typename container_t::value_type>>()));
        assignment_info.back().second.reserve(1);
        assignment_info.back().second.push_back(
          std::make_pair(idx, (*curr_bc)[idx]));
        assign_iter = assignment_info.end()-1;
      }
    }

    // Iterate over blocks to assign and send them to the location where
    // the elements of dest to assign are stored.
    assign_iter = assignment_info.begin();
    for (; assign_iter != assignment_info.end(); ++assign_iter)
    {
      using mem_fun_t = void (remapping_spawner<DestView, PartitionedDims>::*)(
        typename container_t::distribution_type::cid_type const&,
        std::vector<std::pair<typename container_t::gid_type,
                              typename container_t::value_type>> const&);
      const mem_fun_t mem_fun =
        &remapping_spawner<DestView, PartitionedDims>::receive_data;

      stapl::location_type lid = mapper.map(assign_iter->first);

      if (lid != this->get_location_id())
        stapl::async_rmi(lid, this->get_rmi_handle(),
          mem_fun, assign_iter->first, assign_iter->second);
      else
        receive_data(assign_iter->first, assign_iter->second);
    }

    stapl::do_once([&] { p.set_value(); });
  }


  template <typename CID, typename ValueVector>
  void receive_data(CID const& cid, ValueVector const& values)
  {
    auto curr_bc =
      m_dest->container().distribution().container_manager().begin();
    auto bc_end = m_dest->container().distribution().container_manager().end();
    for (; curr_bc != bc_end; ++curr_bc)
    {
      if (curr_bc->cid() == cid)
      {
        typename ValueVector::const_iterator values_end = values.end();
        for (auto val_it = values.begin(); val_it != values_end; ++val_it)
          (*curr_bc)[val_it->first] = val_it->second;
        return;
      }
    }
    stapl::abort("remapping_spawner::receive_data base container not found.");
  }
};


template<typename PartitionedDims, typename SourceView, typename DestView>
void invoke_remapping(SourceView& source, DestView& dest)
{
  stapl::promise<void> p;
  auto f = p.get_future();

  using spawner_t = remapping_spawner<DestView, PartitionedDims>;
  stapl::async_construct<spawner_t>(
    [](spawner_t* s) { delete s; },
    source.container().get_rmi_handle(), stapl::all_locations,
    stapl::transporter_packager()(source),
    stapl::transporter_packager()(dest),
    std::move(p)
  );

  // wait on signal that nested PARAGRAPH has finished execution
  f.get();
}


#endif
