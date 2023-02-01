/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef BENCHMARKS_CONTAINERS_MULTIARRAY_CONTAINERS_HPP
#define BENCHMARKS_CONTAINERS_MULTIARRAY_CONTAINERS_HPP

#include <iostream>
#include <functional>

#include <Kripke/SubTVec.h>

#include <stapl/multiarray.hpp>
#include <stapl/views/slices_view.hpp>
#include <stapl/containers/multiarray/base_container.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/runtime/counter/default_counters.hpp>
#include <stapl/utility/tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>

typedef stapl::counter<stapl::default_timer>  counter_t;

struct Bench_User_Data
{
  Nesting_Order nest;
  int           nx, ny, nz;
  int           dirs, d_ds;
  int           grps, g_gs;
  int           nm;

  typedef std::tuple<size_t, size_t, size_t, size_t, size_t> index_type;

  Bench_User_Data(int argc, char** argv)
    : nest(nestingFromString(argv[1])),
      nx(atoi(argv[2])), ny(atoi(argv[3])), nz(atoi(argv[4])),
      dirs(atoi(argv[5])), d_ds(atoi(argv[8])),
      grps(atoi(argv[6])), g_gs(atoi(argv[9])), nm(atoi(argv[7]))
  { }

  index_type first(void) const
  { return index_type(0, 0, 0, 0, 0); }

  // psi and rhs have dimensions (nx, ny, nz, d_ds, g_gs)
  index_type psi_last(void) const
  { return index_type(nx-1, ny-1, nz-1, d_ds-1, g_gs-1); }

  // sigt has dimensions (nx, ny, nz, 1, g_gs)
  index_type sigt_last(void) const
  { return index_type(nx-1, ny-1, nz-1, 0, g_gs-1); }

  // phi and phi_out have dimensions (nx, ny, nz, nm, g)
  index_type phi_last(void) const
  { return index_type(nx-1, ny-1, nz-1, nm-1, grps-1); }

  // ell and ell_plus have dimensions (1, 1, 1, d, nm)
  index_type ell_last(void) const
  { return index_type(0, 0, 0, dirs-1, nm-1); }

  void print(void)
  {
    std::cerr << "Nesting: " << nestingString(nest)
              << ", nx " << nx << " ny " << ny << " nz " << nz
              << ", dirs " << dirs << " dirs/ds " << d_ds
              << ", grps " << grps << " grps/gs " << g_gs
              << ", nmts " << nm << std::endl;
  }
};

template<typename Traversal>
struct compute_bc_type;

//////////////////////////////////////////////////////////////////////
/// @brief Compute the base container type given the traversal order.
///
/// @todo Add separate types for phi and ell base containers to match what
/// Kripke SubTVec setup does to evaluate performance difference
//////////////////////////////////////////////////////////////////////
template<std::size_t... TraversalIndices>
struct compute_bc_type<stapl::index_sequence<TraversalIndices...>>
{
  typedef std::tuple<boost::mpl::int_<TraversalIndices>... > traversal_t;
  typedef stapl::indexed_domain<size_t, 5, traversal_t> domain_t;
  typedef typename domain_t::index_type                 cid_t;
  typedef stapl::multiarray_base_container_traits<double, 5, traversal_t>
            traits_t;
  typedef stapl::multiarray_base_container<double, domain_t, cid_t, traits_t>
            container_t;
};

template<Nesting_Order>
struct base_container_traits;

template<>
struct base_container_traits<NEST_ZDG>
  : public compute_bc_type<stapl::index_sequence<2, 3, 4, 1, 0>>
{ };

template<>
struct base_container_traits<NEST_ZGD>
  : public compute_bc_type<stapl::index_sequence<2, 3, 4, 0, 1>>
{ };

template<>
struct base_container_traits<NEST_GZD>
  : public compute_bc_type<stapl::index_sequence<1, 2, 3, 0, 4>>
{ };

template<>
struct base_container_traits<NEST_GDZ>
  : public compute_bc_type<stapl::index_sequence<0, 1, 2, 3, 4>>
{ };

template<>
struct base_container_traits<NEST_DGZ>
  : public compute_bc_type<stapl::index_sequence<0, 1, 2, 4, 3>>
{ };

template<>
struct base_container_traits<NEST_DZG>
  : public compute_bc_type<stapl::index_sequence<1, 2, 3, 4, 0>>
{ };

template <typename Traversal>
struct compute_multiarray_type
{
  typedef typename stapl::default_traversal<5, Traversal>::type   traversal_t;
  typedef typename stapl::multiarray<5, double, traversal_t>      container_t;
};

template<Nesting_Order>
struct multiarray_traits;

template<>
struct multiarray_traits<NEST_ZDG>
  : public compute_multiarray_type<stapl::index_sequence<2, 1, 0, 3, 4>>
{ };

template<>
struct multiarray_traits<NEST_ZGD>
  : public compute_multiarray_type<stapl::index_sequence<2, 1, 0, 4, 3>>
{ };

template<>
struct multiarray_traits<NEST_DZG>
  : public compute_multiarray_type<stapl::index_sequence<3, 2, 1, 0, 4>>
{ };

template<>
struct multiarray_traits<NEST_GZD>
  : public compute_multiarray_type<stapl::index_sequence<3, 2, 1, 4, 0>>
{ };

template<>
struct multiarray_traits<NEST_DGZ>
  : public compute_multiarray_type<stapl::index_sequence<4, 3, 2, 0, 1>>
{ };

template<>
struct multiarray_traits<NEST_GDZ>
  : public compute_multiarray_type<stapl::index_sequence<4, 3, 2, 1, 0>>
{ };

// populate function used by Boost, SubTVec, and multiarray base container
template <typename BC>
void populate(BC* bc, size_t size)
{
  double* ptr = bc->container().data();

  for (size_t i = 0; i != size; ++ptr, ++i)
    *ptr = 0;
}

// populate function used by stapl::multiarray
template <typename Multiarray>
void populate(Multiarray* cont,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t d = ds; d != dl; ++d)
    for (size_t g = gs; g != gl; ++g)
      for (size_t k = ks; k != kl; ++k)
        for (size_t j = js; j != jl; ++j)
          for (size_t i = is; i != il; ++i)
            (*cont)(i, j, k, d, g) = 0;
}

void traverse(base_container_traits<NEST_ZDG>::container_t* bc,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t k = ks; k != kl; ++k)
  {
    for (size_t j = js; j != jl; ++j)
    {
      for (size_t i = is; i != il; ++i)
      {
        for (size_t d = ds; d != dl; ++d)
        {
          for (size_t g = gs; g != gl; ++g)
          {
            (*bc)(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(base_container_traits<NEST_ZGD>::container_t* bc,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t k = ks; k != kl; ++k)
  {
    for (size_t j = js; j != jl; ++j)
    {
      for (size_t i = is; i != il; ++i)
      {
        for (size_t g = gs; g != gl; ++g)
        {
          for (size_t d = ds; d != dl; ++d)
          {
            (*bc)(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(base_container_traits<NEST_GZD>::container_t* bc,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t g = gs; g != gl; ++g)
  {
    for (size_t k = ks; k != kl; ++k)
    {
      for (size_t j = js; j != jl; ++j)
      {
        for (size_t i = is; i != il; ++i)
        {
          for (size_t d = ds; d != dl; ++d)
          {
            (*bc)(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(base_container_traits<NEST_GDZ>::container_t* bc,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t g = gs; g != gl; ++g)
  {
    for (size_t d = ds; d != dl; ++d)
    {
      for (size_t k = ks; k != kl; ++k)
      {
        for (size_t j = js; j != jl; ++j)
        {
          for (size_t i = is; i != il; ++i)
          {
            (*bc)(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(base_container_traits<NEST_DGZ>::container_t* bc,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t d = ds; d != dl; ++d)
  {
    for (size_t g = gs; g != gl; ++g)
    {
      for (size_t k = ks; k != kl; ++k)
      {
        for (size_t j = js; j != jl; ++j)
        {
          for (size_t i = is; i != il; ++i)
          {
            (*bc)(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(base_container_traits<NEST_DZG>::container_t* bc,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t d = ds; d != dl; ++d)
  {
    for (size_t k = ks; k != kl; ++k)
    {
      for (size_t j = js; j != jl; ++j)
      {
        for (size_t i = is; i != il; ++i)
        {
          for (size_t g = gs; g != gl; ++g)
          {
            (*bc)(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

template<Nesting_Order Nest>
std::tuple<std::string, double, double, double, double>
run_base_container(Bench_User_Data::index_type const& first,
                   Bench_User_Data::index_type const& last)
{
  typedef base_container_traits<Nest> traits_t;
  typedef typename traits_t::container_t container_t;
  typedef typename traits_t::domain_t    domain_t;
  typedef typename traits_t::cid_t       index_t;

  index_t cid(0,0,0,0,0);

  counter_t timer;
  timer.reset();
  std::tuple<std::string, double, double, double, double> times;
  std::get<0>(times) = nestingString(Nest);

  // Construction
  timer.start();
  container_t* bc = new container_t(domain_t(first, last, true), cid);
  std::get<1>(times) = timer.stop();
  timer.reset();

  // Initialization over raw data
  timer.start();
  populate<container_t>(bc, bc->size());
  std::get<2>(times) = timer.stop();
  timer.reset();

  // Traversal using index operators
  timer.start();
  traverse(bc, first, last);
  std::get<3>(times) = timer.stop();
  timer.reset();

  // Destruct
  timer.start();
  delete bc;
  std::get<4>(times) = timer.stop();

  return times;
}


void traverse(stapl::multiarray_view<
                base_container_traits<NEST_ZDG>::container_t> ma_vw,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t k = ks; k != kl; ++k)
  {
    for (size_t j = js; j != jl; ++j)
    {
      for (size_t i = is; i != il; ++i)
      {
        for (size_t d = ds; d != dl; ++d)
        {
          for (size_t g = gs; g != gl; ++g)
          {
            ma_vw(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(stapl::multiarray_view<
                base_container_traits<NEST_ZGD>::container_t> ma_vw,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t k = ks; k != kl; ++k)
  {
    for (size_t j = js; j != jl; ++j)
    {
      for (size_t i = is; i != il; ++i)
      {
        for (size_t g = gs; g != gl; ++g)
        {
          for (size_t d = ds; d != dl; ++d)
          {
            ma_vw(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(stapl::multiarray_view<
                base_container_traits<NEST_GZD>::container_t> ma_vw,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t g = gs; g != gl; ++g)
  {
    for (size_t k = ks; k != kl; ++k)
    {
      for (size_t j = js; j != jl; ++j)
      {
        for (size_t i = is; i != il; ++i)
        {
          for (size_t d = ds; d != dl; ++d)
          {
            ma_vw(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(stapl::multiarray_view<
                base_container_traits<NEST_GDZ>::container_t> ma_vw,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t g = gs; g != gl; ++g)
  {
    for (size_t d = ds; d != dl; ++d)
    {
      for (size_t k = ks; k != kl; ++k)
      {
        for (size_t j = js; j != jl; ++j)
        {
          for (size_t i = is; i != il; ++i)
          {
            ma_vw(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(stapl::multiarray_view<
                base_container_traits<NEST_DGZ>::container_t> ma_vw,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t d = ds; d != dl; ++d)
  {
    for (size_t g = gs; g != gl; ++g)
    {
      for (size_t k = ks; k != kl; ++k)
      {
        for (size_t j = js; j != jl; ++j)
        {
          for (size_t i = is; i != il; ++i)
          {
            ma_vw(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(stapl::multiarray_view<
                base_container_traits<NEST_DZG>::container_t> ma_vw,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t d = ds; d != dl; ++d)
  {
    for (size_t k = ks; k != kl; ++k)
    {
      for (size_t j = js; j != jl; ++j)
      {
        for (size_t i = is; i != il; ++i)
        {
          for (size_t g = gs; g != gl; ++g)
          {
            ma_vw(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

template<Nesting_Order Nest>
std::tuple<std::string, double, double, double, double>
run_bc_multiarray_view(Bench_User_Data::index_type const& first,
                       Bench_User_Data::index_type const& last)
{
  typedef base_container_traits<Nest> traits_t;
  typedef typename traits_t::container_t container_t;
  typedef typename traits_t::domain_t    domain_t;
  typedef typename traits_t::cid_t       index_t;

  index_t cid(0,0,0,0,0);

  counter_t timer;
  timer.reset();
  std::tuple<std::string, double, double, double, double> times;
  std::get<0>(times) = nestingString(Nest);

  // Construction
  timer.start();
  container_t* bc = new container_t(domain_t(first, last, true), cid);
  auto&& ma_vw = make_multiarray_view(*bc);
  std::get<1>(times) = timer.stop();
  timer.reset();

  // Initialization over raw data
  timer.start();
  populate<container_t>(bc, bc->size());
  std::get<2>(times) = timer.stop();
  timer.reset();

  // Traversal using index operators
  timer.start();
  traverse(ma_vw, first, last);
  std::get<3>(times) = timer.stop();
  timer.reset();

  // Destruct
  timer.start();
  delete bc;
  std::get<4>(times) = timer.stop();

  return times;
}

template<Nesting_Order Nest>
struct Traversal{ };

template<typename SlicesView>
void traverse(SlicesView sl_vw,
              Traversal<NEST_ZDG> traversal,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t k = ks; k != kl; ++k)
  {
    for (size_t j = js; j != jl; ++j)
    {
      for (size_t i = is; i != il; ++i)
      {
        auto&& slice_z = sl_vw(i, j, k);

        for (size_t d = ds; d != dl; ++d)
        {
          auto&& slice_z_d = slice_z[d];

          for (size_t g = gs; g != gl; ++g)
          {
            slice_z_d[g] += 1.4142135624;
          }
        }
      }
    }
  }
}

template<typename SlicesView>
void traverse(SlicesView sl_vw,
              Traversal<NEST_ZGD> traversal,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t k = ks; k != kl; ++k)
  {
    for (size_t j = js; j != jl; ++j)
    {
      for (size_t i = is; i != il; ++i)
      {
        auto&& slice_z = sl_vw(i, j, k);

        for (size_t g = gs; g != gl; ++g)
        {
          auto&& slice_z_g = slice_z[g];

          for (size_t d = ds; d != dl; ++d)
          {
            slice_z_g[d] += 1.4142135624;
          }
        }
      }
    }
  }
}

template<typename SlicesView>
void traverse(SlicesView sl_vw,
              Traversal<NEST_GZD> traversal,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t g = gs; g != gl; ++g)
  {
    auto&& slice_g = sl_vw[g];

    for (size_t k = ks; k != kl; ++k)
    {
      for (size_t j = js; j != jl; ++j)
      {
        for (size_t i = is; i != il; ++i)
        {
          auto&& slice_g_z = slice_g(i, j, k);

          for (size_t d = ds; d != dl; ++d)
          {
            slice_g_z[d] += 1.4142135624;
          }
        }
      }
    }
  }
}

template<typename SlicesView>
void traverse(SlicesView sl_vw,
              Traversal<NEST_GDZ> traversal,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t g = gs; g != gl; ++g)
  {
    auto&& slice_g = sl_vw[g];

    for (size_t d = ds; d != dl; ++d)
    {
      auto&& slice_g_d = slice_g[d];

      for (size_t k = ks; k != kl; ++k)
      {
        for (size_t j = js; j != jl; ++j)
        {
          for (size_t i = is; i != il; ++i)
          {
            slice_g_d(i, j, k) += 1.4142135624;
          }
        }
      }
    }
  }
}

template<typename SlicesView>
void traverse(SlicesView sl_vw,
              Traversal<NEST_DGZ> traversal,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t d = ds; d != dl; ++d)
  {
    auto&& slice_d = sl_vw[d];

    for (size_t g = gs; g != gl; ++g)
    {
      auto&& slice_d_g = slice_d[g];

      for (size_t k = ks; k != kl; ++k)
      {
        for (size_t j = js; j != jl; ++j)
        {
          for (size_t i = is; i != il; ++i)
          {
            slice_d_g(i, j, k) += 1.4142135624;
          }
        }
      }
    }
  }
}

template<typename SlicesView>
void traverse(SlicesView sl_vw,
              Traversal<NEST_DZG> traversal,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t d = ds; d != dl; ++d)
  {
    auto&& slice_d = sl_vw[d];

    for (size_t k = ks; k != kl; ++k)
    {
      for (size_t j = js; j != jl; ++j)
      {
        for (size_t i = is; i != il; ++i)
        {
          auto&& slice_d_z = slice_d(i, j, k);

          for (size_t g = gs; g != gl; ++g)
          {
            slice_d_z[g] += 1.4142135624;
          }
        }
      }
    }
  }
}

template<Nesting_Order Nest>
struct slices_for_nest;

template<>
struct slices_for_nest<NEST_DGZ>
{
  using outer = stapl::index_sequence<3, 4>;
  using inner = stapl::index_sequence<0>;
};

template<>
struct slices_for_nest<NEST_DZG>
{
  using outer = stapl::index_sequence<0, 1, 2, 3>;
  using inner = stapl::index_sequence<3>;
};

template<>
struct slices_for_nest<NEST_GDZ>
{
  using outer = stapl::index_sequence<3, 4>;
  using inner = stapl::index_sequence<1>;
};

template<>
struct slices_for_nest<NEST_GZD>
{
  using outer = stapl::index_sequence<0, 1, 2, 4>;
  using inner = stapl::index_sequence<3>;
};

template<>
struct slices_for_nest<NEST_ZDG>
{
  using outer = stapl::index_sequence<0, 1, 2, 3>;
  using inner = stapl::index_sequence<0, 1, 2>;
};

template<>
struct slices_for_nest<NEST_ZGD>
{
  using outer = stapl::index_sequence<0, 1, 2, 4>;
  using inner = stapl::index_sequence<0, 1, 2>;
};

template<Nesting_Order Nest>
std::tuple<std::string, double, double, double, double>
run_bc_slices_view(Bench_User_Data::index_type const& first,
                   Bench_User_Data::index_type const& last)
{
  typedef base_container_traits<Nest>    traits_t;
  typedef typename traits_t::container_t container_t;
  typedef typename traits_t::domain_t    domain_t;
  typedef typename traits_t::cid_t       index_t;

  index_t cid(0,0,0,0,0);

  counter_t timer;
  timer.reset();
  std::tuple<std::string, double, double, double, double> times;
  std::get<0>(times) = nestingString(Nest);

  // Construction
  timer.start();
  container_t* bc = new container_t(domain_t(first, last, true), cid);
  auto&& ma_vw = make_multiarray_view(*bc);
  std::get<1>(times) = timer.stop();
  timer.reset();

  // Initialization over raw data
  timer.start();
  populate<container_t>(bc, bc->size());
  std::get<2>(times) = timer.stop();
  timer.reset();

  // Traversal using index operators
  using outer_slices = typename slices_for_nest<Nest>::outer;
  using inner_slices = typename slices_for_nest<Nest>::inner;

  timer.start();
  auto tmp_slice = stapl::make_slices_view<outer_slices>(ma_vw);
  auto slice_vw = stapl::make_slices_view<inner_slices>(tmp_slice);
  std::get<1>(times) += timer.stop();
  timer.reset();

  timer.start();
  traverse(slice_vw, Traversal<Nest>(), first, last);
  std::get<3>(times) = timer.stop();
  timer.reset();

  // Destruct
  timer.start();
  delete bc;
  std::get<4>(times) = timer.stop();

  return times;
}

void traverse(multiarray_traits<NEST_ZDG>::container_t* cont,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t k = ks; k != kl; ++k)
  {
    for (size_t j = js; j != jl; ++j)
    {
      for (size_t i = is; i != il; ++i)
      {
        for (size_t d = ds; d != dl; ++d)
        {
          for (size_t g = gs; g != gl; ++g)
          {
            (*cont)(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(multiarray_traits<NEST_ZGD>::container_t* cont,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t k = ks; k != kl; ++k)
  {
    for (size_t j = js; j != jl; ++j)
    {
      for (size_t i = is; i != il; ++i)
      {
        for (size_t g = gs; g != gl; ++g)
        {
          for (size_t d = ds; d != dl; ++d)
          {
            (*cont)(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(multiarray_traits<NEST_GZD>::container_t* cont,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t g = gs; g != gl; ++g)
  {
    for (size_t k = ks; k != kl; ++k)
    {
      for (size_t j = js; j != jl; ++j)
      {
        for (size_t i = is; i != il; ++i)
        {
          for (size_t d = ds; d != dl; ++d)
          {
            (*cont)(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(multiarray_traits<NEST_GDZ>::container_t* cont,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t g = gs; g != gl; ++g)
  {
    for (size_t d = ds; d != dl; ++d)
    {
      for (size_t k = ks; k != kl; ++k)
      {
        for (size_t j = js; j != jl; ++j)
        {
          for (size_t i = is; i != il; ++i)
          {
            (*cont)(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(multiarray_traits<NEST_DGZ>::container_t* cont,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t d = ds; d != dl; ++d)
  {
    for (size_t g = gs; g != gl; ++g)
    {
      for (size_t k = ks; k != kl; ++k)
      {
        for (size_t j = js; j != jl; ++j)
        {
          for (size_t i = is; i != il; ++i)
          {
            (*cont)(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(multiarray_traits<NEST_DZG>::container_t* cont,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t d = ds; d != dl; ++d)
  {
    for (size_t k = ks; k != kl; ++k)
    {
      for (size_t j = js; j != jl; ++j)
      {
        for (size_t i = is; i != il; ++i)
        {
          for (size_t g = gs; g != gl; ++g)
          {
            (*cont)(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

template<Nesting_Order Nest>
std::tuple<std::string, double, double, double, double>
run_multiarray(Bench_User_Data::index_type const& first,
               Bench_User_Data::index_type const& last)
{
  typedef multiarray_traits<Nest>                   traits_t;
  typedef typename traits_t::container_t            container_t;
  typedef typename container_t::dimensions_type     dim_t;

  size_t i(std::get<0>(last) + 1 - std::get<0>(first));
  size_t j(std::get<1>(last) + 1 - std::get<1>(first));
  size_t k(std::get<2>(last) + 1 - std::get<2>(first));
  size_t d(std::get<3>(last) + 1 - std::get<3>(first));
  size_t g(std::get<4>(last) + 1 - std::get<4>(first));

  dim_t dims = dim_t(i, j, k, d, g);

  counter_t timer;
  timer.reset();
  std::tuple<std::string, double, double, double, double> times;
  std::get<0>(times) = nestingString(Nest);

  // Construction
  timer.start();
  container_t* cont = new container_t(dims);
  std::get<1>(times) = timer.stop();
  timer.reset();

  // Initialization over raw data
  timer.start();
  populate(cont, first, last);
  std::get<2>(times) = timer.stop();
  timer.reset();

  // Traversal using index operators
  timer.start();
  traverse(cont, first, last);
  std::get<3>(times) = timer.stop();
  timer.reset();

  // Destruct
  timer.start();
  delete cont;
  std::get<4>(times) = timer.stop();

  return times;
}

void traverse(stapl::multiarray_view<
                multiarray_traits<NEST_ZDG>::container_t> ma_vw,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t k = ks; k != kl; ++k)
  {
    for (size_t j = js; j != jl; ++j)
    {
      for (size_t i = is; i != il; ++i)
      {
        for (size_t d = ds; d != dl; ++d)
        {
          for (size_t g = gs; g != gl; ++g)
          {
            ma_vw(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(stapl::multiarray_view<
                multiarray_traits<NEST_ZGD>::container_t> ma_vw,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t k = ks; k != kl; ++k)
  {
    for (size_t j = js; j != jl; ++j)
    {
      for (size_t i = is; i != il; ++i)
      {
        for (size_t g = gs; g != gl; ++g)
        {
          for (size_t d = ds; d != dl; ++d)
          {
            ma_vw(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(stapl::multiarray_view<
                multiarray_traits<NEST_GZD>::container_t> ma_vw,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t g = gs; g != gl; ++g)
  {
    for (size_t k = ks; k != kl; ++k)
    {
      for (size_t j = js; j != jl; ++j)
      {
        for (size_t i = is; i != il; ++i)
        {
          for (size_t d = ds; d != dl; ++d)
          {
            ma_vw(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(stapl::multiarray_view<
                multiarray_traits<NEST_GDZ>::container_t> ma_vw,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t g = gs; g != gl; ++g)
  {
    for (size_t d = ds; d != dl; ++d)
    {
      for (size_t k = ks; k != kl; ++k)
      {
        for (size_t j = js; j != jl; ++j)
        {
          for (size_t i = is; i != il; ++i)
          {
            ma_vw(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(stapl::multiarray_view<
                multiarray_traits<NEST_DGZ>::container_t> ma_vw,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t d = ds; d != dl; ++d)
  {
    for (size_t g = gs; g != gl; ++g)
    {
      for (size_t k = ks; k != kl; ++k)
      {
        for (size_t j = js; j != jl; ++j)
        {
          for (size_t i = is; i != il; ++i)
          {
            ma_vw(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

void traverse(stapl::multiarray_view<
                multiarray_traits<NEST_DZG>::container_t> ma_vw,
              Bench_User_Data::index_type const& first,
              Bench_User_Data::index_type const& last)
{
  size_t is(std::get<0>(first)), il(std::get<0>(last)+1);
  size_t js(std::get<1>(first)), jl(std::get<1>(last)+1);
  size_t ks(std::get<2>(first)), kl(std::get<2>(last)+1);
  size_t ds(std::get<3>(first)), dl(std::get<3>(last)+1);
  size_t gs(std::get<4>(first)), gl(std::get<4>(last)+1);

  for (size_t d = ds; d != dl; ++d)
  {
    for (size_t k = ks; k != kl; ++k)
    {
      for (size_t j = js; j != jl; ++j)
      {
        for (size_t i = is; i != il; ++i)
        {
          for (size_t g = gs; g != gl; ++g)
          {
            ma_vw(i, j, k, d, g) += 1.4142135624;
          }
        }
      }
    }
  }
}

template<Nesting_Order Nest>
std::tuple<std::string, double, double, double, double>
run_multiarray_view(Bench_User_Data::index_type const& first,
                    Bench_User_Data::index_type const& last)
{
  typedef multiarray_traits<Nest>                   traits_t;
  typedef typename traits_t::container_t            container_t;
  typedef typename container_t::dimensions_type     dim_t;

  size_t i(std::get<0>(last) + 1 - std::get<0>(first));
  size_t j(std::get<1>(last) + 1 - std::get<1>(first));
  size_t k(std::get<2>(last) + 1 - std::get<2>(first));
  size_t d(std::get<3>(last) + 1 - std::get<3>(first));
  size_t g(std::get<4>(last) + 1 - std::get<4>(first));

  dim_t dims = dim_t(i, j, k, d, g);

  counter_t timer;
  timer.reset();
  std::tuple<std::string, double, double, double, double> times;
  std::get<0>(times) = nestingString(Nest);

  // Construction
  timer.start();
  container_t* cont = new container_t(dims);
  auto&& ma_vw = make_multiarray_view(*cont);
  std::get<1>(times) = timer.stop();
  timer.reset();

  // Initialization over raw data
  timer.start();
  populate(cont, first, last);
  std::get<2>(times) = timer.stop();
  timer.reset();

  // Traversal using index operators
  timer.start();
  traverse(ma_vw, first, last);
  std::get<3>(times) = timer.stop();
  timer.reset();

  // Destruct
  timer.start();
  delete cont;
  std::get<4>(times) = timer.stop();

  return times;
}


template<Nesting_Order Nest>
std::tuple<std::string, double, double, double, double>
run_slices_view(Bench_User_Data::index_type const& first,
                Bench_User_Data::index_type const& last)
{
  typedef multiarray_traits<Nest>                   traits_t;
  typedef typename traits_t::container_t            container_t;
  typedef typename container_t::dimensions_type     dim_t;

  size_t i(std::get<0>(last) + 1 - std::get<0>(first));
  size_t j(std::get<1>(last) + 1 - std::get<1>(first));
  size_t k(std::get<2>(last) + 1 - std::get<2>(first));
  size_t d(std::get<3>(last) + 1 - std::get<3>(first));
  size_t g(std::get<4>(last) + 1 - std::get<4>(first));

  dim_t dims = dim_t(i, j, k, d, g);

  counter_t timer;
  timer.reset();
  std::tuple<std::string, double, double, double, double> times;
  std::get<0>(times) = nestingString(Nest);

  // Construction
  timer.start();
  container_t* cont = new container_t(dims);
  auto&& ma_vw = make_multiarray_view(*cont);
  std::get<1>(times) = timer.stop();
  timer.reset();

  // Initialization over raw data
  timer.start();
  populate(cont, first, last);
  std::get<2>(times) = timer.stop();
  timer.reset();

  // Traversal using index operators
  using outer_slices = typename slices_for_nest<Nest>::outer;
  using inner_slices = typename slices_for_nest<Nest>::inner;

  timer.start();
  auto tmp_slice = stapl::make_slices_view<outer_slices>(ma_vw);
  auto slice_vw = stapl::make_slices_view<inner_slices>(tmp_slice);
  std::get<1>(times) += timer.stop();
  timer.reset();

  timer.start();
  traverse(slice_vw, Traversal<Nest>(), first, last);
  std::get<3>(times) = timer.stop();
  timer.reset();

  // Destruct
  timer.start();
  delete cont;
  std::get<4>(times) = timer.stop();

  return times;
}

struct traverse_element
{
  using result_type = void;

  template <typename Element>
  result_type
  operator()(Element e)
  {
    e += 1.4142135624;
  }
};

template<Nesting_Order Nest>
std::tuple<std::string, double, double, double, double>
run_multiarray_view_pg(Bench_User_Data::index_type const& first,
                       Bench_User_Data::index_type const& last)
{
  typedef multiarray_traits<Nest>                   traits_t;
  typedef typename traits_t::container_t            container_t;
  typedef typename container_t::dimensions_type     dim_t;

  size_t i(std::get<0>(last) + 1 - std::get<0>(first));
  size_t j(std::get<1>(last) + 1 - std::get<1>(first));
  size_t k(std::get<2>(last) + 1 - std::get<2>(first));
  size_t d(std::get<3>(last) + 1 - std::get<3>(first));
  size_t g(std::get<4>(last) + 1 - std::get<4>(first));

  dim_t dims = dim_t(i, j, k, d, g);

  counter_t timer;
  timer.reset();
  std::tuple<std::string, double, double, double, double> times;
  std::get<0>(times) = nestingString(Nest);

  // Construction
  timer.start();
  container_t* cont = new container_t(dims);
  auto&& ma_vw = make_multiarray_view(*cont);
  std::get<1>(times) = timer.stop();
  timer.reset();

  // Initialization over raw data
  timer.start();
  populate(cont, first, last);
  std::get<2>(times) = timer.stop();
  timer.reset();

  // Traversal using index operators
  timer.start();
  stapl::map_func(traverse_element(), ma_vw);
  std::get<3>(times) = timer.stop();
  timer.reset();

  // Destruct
  timer.start();
  delete cont;
  std::get<4>(times) = timer.stop();

  return times;
}

struct traverse_inner_slice
{
  using result_type = void;

  template <typename Slice>
  result_type
  operator()(Slice s)
  {
    map_func(traverse_element(), s);
  }
};

struct traverse_outer_slice
{
  using result_type = void;

  template <typename Slice>
  result_type
  operator()(Slice s)
  {
    map_func(traverse_inner_slice(), s);
  }
};

template<Nesting_Order Nest>
std::tuple<std::string, double, double, double, double>
run_slices_view_pg(Bench_User_Data::index_type const& first,
                   Bench_User_Data::index_type const& last)
{
  typedef multiarray_traits<Nest>                   traits_t;
  typedef typename traits_t::container_t            container_t;
  typedef typename container_t::dimensions_type     dim_t;

  size_t i(std::get<0>(last) + 1 - std::get<0>(first));
  size_t j(std::get<1>(last) + 1 - std::get<1>(first));
  size_t k(std::get<2>(last) + 1 - std::get<2>(first));
  size_t d(std::get<3>(last) + 1 - std::get<3>(first));
  size_t g(std::get<4>(last) + 1 - std::get<4>(first));

  dim_t dims = dim_t(i, j, k, d, g);

  counter_t timer;
  timer.reset();
  std::tuple<std::string, double, double, double, double> times;
  std::get<0>(times) = nestingString(Nest);

  // Construction
  timer.start();
  container_t* cont = new container_t(dims);
  auto&& ma_vw = make_multiarray_view(*cont);
  std::get<1>(times) = timer.stop();
  timer.reset();

  // Initialization over raw data
  timer.start();
  populate(cont, first, last);
  std::get<2>(times) = timer.stop();
  timer.reset();

  // Traversal using index operators
  using outer_slices = typename slices_for_nest<Nest>::outer;
  using inner_slices = typename slices_for_nest<Nest>::inner;

  timer.start();
  auto tmp_slice = stapl::make_slices_view<outer_slices>(ma_vw);
  auto slice_vw = stapl::make_slices_view<inner_slices>(tmp_slice);
  std::get<1>(times) += timer.stop();
  timer.reset();

  timer.start();
  stapl::map_func(traverse_outer_slice(), slice_vw);
  std::get<3>(times) = timer.stop();
  timer.reset();

  // Destruct
  timer.start();
  delete cont;
  std::get<4>(times) = timer.stop();

  return times;
}

struct traverse_coarse
{
  using result_type = void;

  template <typename CoarsenedView>
  result_type
  operator()(CoarsenedView cv)
  {
    auto domain = cv.domain();

    size_t is(std::get<0>(domain.first())), il(std::get<0>(domain.last())+1);
    size_t js(std::get<1>(domain.first())), jl(std::get<1>(domain.last())+1);
    size_t ks(std::get<2>(domain.first())), kl(std::get<2>(domain.last())+1);
    size_t ds(std::get<3>(domain.first())), dl(std::get<3>(domain.last())+1);
    size_t gs(std::get<4>(domain.first())), gl(std::get<4>(domain.last())+1);

    for (size_t d = ds; d != dl; ++d)
      for (size_t g = gs; g != gl; ++g)
        for (size_t k = ks; k != kl; ++k)
          for (size_t j = js; j != jl; ++j)
            for (size_t i = is; i != il; ++i)
              cv(i, j, k, d, g) += 1.4142135624;
  }
};

struct check_values
{
  using result_type = void;

  template <typename Value>
  result_type
  operator()(Value v)
  {
    stapl_assert(fabs(v - 1.4142135624) < 0.001,
                 "Multiarray value incorrect");
  }
};

template<Nesting_Order Nest>
std::tuple<std::string, double, double, double, double>
run_multiarray_view_pg_coarse(Bench_User_Data::index_type const& first,
                              Bench_User_Data::index_type const& last)
{
  typedef multiarray_traits<Nest>                   traits_t;
  typedef typename traits_t::container_t            container_t;
  typedef typename container_t::dimensions_type     dim_t;

  size_t i(std::get<0>(last) + 1 - std::get<0>(first));
  size_t j(std::get<1>(last) + 1 - std::get<1>(first));
  size_t k(std::get<2>(last) + 1 - std::get<2>(first));
  size_t d(std::get<3>(last) + 1 - std::get<3>(first));
  size_t g(std::get<4>(last) + 1 - std::get<4>(first));

  dim_t dims = dim_t(i, j, k, d, g);

  counter_t timer;
  timer.reset();
  std::tuple<std::string, double, double, double, double> times;
  std::get<0>(times) = nestingString(Nest);

  // Construction
  timer.start();
  container_t* cont = new container_t(dims);
  auto&& ma_vw = make_multiarray_view(*cont);
  std::get<1>(times) = timer.stop();
  timer.reset();

  // Initialization over raw data
  timer.start();
  populate(cont, first, last);
  std::get<2>(times) = timer.stop();
  timer.reset();

  // Traversal using index operators
  timer.start();
  stapl::map_func<stapl::skeletons::tags::with_coarsened_wf>
    (traverse_coarse(), ma_vw);
  std::get<3>(times) = timer.stop();
  timer.reset();

  // Untimed map_func to check that all values were touched in coarsened wf
  stapl::map_func(check_values(), ma_vw);

  // Destruct
  timer.start();
  delete cont;
  std::get<4>(times) = timer.stop();

  return times;
}

#endif
