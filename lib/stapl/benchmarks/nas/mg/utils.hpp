/*
// Copyright (c) 2000-2010, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_NAS_MG_UTILS
#define STAPL_BENCHMARKS_NAS_MG_UTILS

#include <stapl/utility/tuple.hpp>

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/algorithms/algorithm.hpp>


//////////////////////////////////////////////////////////////////////
/// @brief View that returns a known value based on how far a 3D point
///        is from the origin.
///
///        Used in 27-point stencils to a return a different weight
///        based on how far a point is in the stencil from the original
///        point.
//////////////////////////////////////////////////////////////////////
struct coefficient_view
{
  const double m_c0, m_c1, m_c2, m_c3;

  coefficient_view(double c0, double c1, double c2, double c3)
    : m_c0(c0), m_c1(c1), m_c2(c2), m_c3(c3)
  { }

  double const& operator()(int i, int j, int k) const
  {
    const int distance =
      (i != 0 ? 1 : 0) + (j != 0 ? 1 : 0) + (k != 0 ? 1 : 0);

    switch (distance)
    {
      case 0: return m_c0;
      case 1: return m_c1;
      case 2: return m_c2;
      default: return m_c3;
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_c0);
    t.member(m_c1);
    t.member(m_c2);
    t.member(m_c3);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Create a multiarray view over a multiarray that has the
///        size dim in each dimension
///
/// @param dim Grid size in one dimension
/// @return The multiarray view over a new container
//////////////////////////////////////////////////////////////////////
template<typename Container>
stapl::multiarray_view<Container>
make_multiarray_view(int dim)
{
  return stapl::multiarray_view<Container>(
    new Container(stapl::make_tuple(dim, dim, dim))
  );
}


//////////////////////////////////////////////////////////////////////
/// @brief Create hierarchical grids for the solution and residual.
///
/// @param dim Largest grid size in one dimension
//////////////////////////////////////////////////////////////////////
template<typename UGrids, typename RGrids>
void setup(UGrids& u_grids, RGrids& r_grids, int dim)
{
  typedef typename stapl::view_traits<
    typename UGrids::value_type
  >::container                                    u_container;

  typedef typename stapl::view_traits<
    typename RGrids::value_type
  >::container                                    r_container;

  const int num_levels = std::log2(dim);

  for (int k = 0; k < num_levels; ++k)
  {
    u_grids.push_back(make_multiarray_view<u_container>(dim));
    r_grids.push_back(make_multiarray_view<r_container>(dim));

    dim /= 2;
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Compute the L2 norm of a grid.
///
/// @param view Grid to calculate.
//////////////////////////////////////////////////////////////////////
template<typename View>
double norm2u3(View const& view)
{
  const double sum =
    stapl::map_reduce(
      boost::bind(stapl::multiplies<double>(), _1, _1),
      stapl::plus<double>(),
      linear_view(view)
    );

  return std::sqrt(sum / view.size());
}

//////////////////////////////////////////////////////////////////////
/// @brief Fill a grid with all zeroes
///
/// @param grid Grid to fill
//////////////////////////////////////////////////////////////////////
template<typename View>
void zero3(View const& grid)
{
  stapl::fill(grid, 0.);
}


#endif
