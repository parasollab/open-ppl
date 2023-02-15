/*
// Copyright (c) 2000-2010, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/multiarray.hpp>
#include <stapl/views/stencil_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/overlap_view.hpp>
#include <stapl/views/slices_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/utility/do_once.hpp>

#include "../../test_report.hpp"

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief Work function to check that each subview of a stencil view
///        has been assigned correct values.
//////////////////////////////////////////////////////////////////////
struct check_elem_wf
{
  check_elem_wf(std::size_t row_sz)
    : m_row_sz(row_sz)
  { }

  template<typename SubView, typename Res, typename Idx>
  void operator()(SubView&& subview, Res&& res, Idx&& idx)
  {
    auto first = subview.domain().first();
    using index_type = decltype(first);

    std::size_t i0, j0;
    std::tie(i0, j0) = first;
    std::size_t J = get<1>(index_type(idx));

    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        res = subview(i0+i, j0+j) == (J==0) ? m_row_sz : (J + j-1) % m_row_sz;
  }

  void define_type(typer& t)
  { t.member(m_row_sz); }

private:
  std::size_t m_row_sz;
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function to fill a row of a multiarray with values
///        ranging from 0 to row.size()-1.
//////////////////////////////////////////////////////////////////////
struct row_iota_wf
{
  template<typename Row>
  void operator()(Row&& row)
  { iota(row, 0); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper struct that returns different coefficients based
///        on how far a certain point is from the initial point.
//////////////////////////////////////////////////////////////////////
struct coefficient_view
{
  const double c0, c1, c2;

  coefficient_view(double c0_, double c1_, double c2_)
    : c0(c0_), c1(c1_), c2(c2_)
  { }

  double const& operator()(int i, int j) const
  {
    const int distance = (i != 0 ? 1 : 0) + (j != 0 ? 1 : 0);

    switch (distance)
    {
      case 0: return c0;
      case 1: return c1;
      default: return c2;
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(c0);
    t.member(c1);
    t.member(c2);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Stencil work function that receives a single point and
///        the 9-point stencil view of its neighbors.
///
/// This computes a weighted average, where the weights for the points
/// with Manhattan distance of 0, 1 and 2 are specified in the constructor.
//////////////////////////////////////////////////////////////////////
struct stencil9
{
  typedef void result_type;

  coefficient_view m_coeff;

  stencil9(double const c0, double const c1, double const c2)
    : m_coeff(c0, c1, c2)
  { }

  template<typename T, typename View>
  void operator()(T x, View vw)
  {
    auto const& first = vw.domain().first();
    auto const& last = vw.domain().last();

    for (std::size_t i = get<0>(first); i <= get<0>(last); ++i)
      for (std::size_t j = get<1>(first); j <= get<1>(last); ++j)
      {
        // get coefficient based on distance
        const int x_off = static_cast<int>(i)-get<0>(first)-1;
        const int y_off = static_cast<int>(j)-get<1>(first)-1;
        const double c = m_coeff(x_off, y_off);

        // add contribution of this point
        x += c * vw(i,j);
      }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_coeff);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Compute the L2 norm of a multiarray view.
//////////////////////////////////////////////////////////////////////
template<typename View>
double norm(View const& view)
{
  const double sum =
    map_reduce(
      boost::bind(stapl::multiplies<double>(), _1, _1),
      stapl::plus<double>(),
      linear_view(view)
    );

  auto dims = view.dimensions();

  return std::sqrt(sum / get<1>(dims) / get<0>(dims));
}

exit_code stapl_main(int argc, char** argv)
{
  // FIXME (coarsening): these tests needs to be executed coarse-grained when
  // coarsening for the stencil view is available.
  using skeletons::tags::no_coarsening;

  if (argc < 2) {
    return EXIT_FAILURE;
  }

  const std::size_t n = atol(argv[1]);

  using grid_type = multiarray<2, double>;
  using view_type = multiarray_view<grid_type>;

  // create the initial grid function and a view for accessing its values
  grid_type rhs_grid(make_tuple(n, n));
  view_type rhs(rhs_grid);

  // for the first test, initialize the rhs grid function by values from
  // 0 to n-1 in each row
  map_func(row_iota_wf(), make_slices_view<0>(rhs));

  // create the stencil view
  auto rhs_stencil = make_stencil_view<2>(rhs);

  // create the multiarray and corresponding view for storing the results
  // of the first test
  multiarray<2, bool> res(rhs.dimensions(), false);
  auto res_view = make_multiarray_view(res);

  // the first test passes if the values have been properly assigned to lhs
  map_func<no_coarsening>(check_elem_wf(n), rhs_stencil, res_view,
    counting_view_nd<2>(rhs.dimensions(), make_tuple(0ul,0ul)));

  bool passed = count(linear_view(res_view), true) == res_view.size();

  STAPL_TEST_REPORT(passed, "Testing 9-point stencil initialization");

  // initialize the grid function for the results of the computation test
  grid_type lhs_grid(make_tuple(n, n));
  view_type lhs(lhs_grid);

  // for the computation test, fill the grid functions with all zeros except
  // for the first value, which is one
  fill(lhs, 0);
  fill(rhs, 0);

  do_once([&](){
    rhs[make_tuple(0, 0)] = 1;
  });

  // compute the initial norm
  double n0 = norm(rhs);

  // create a view for the stencil computation
  auto lhs_stencil = make_stencil_view<2>(rhs);

  // apply the 9-point stencil operator
  map_func<no_coarsening>(stencil9(0.5, 0.25, 0.125), lhs, lhs_stencil);

  // the computation test passes if the L2 norm of the initial grid function
  // decreases after applying the stencil operator
  passed = norm(lhs) < n0;

  STAPL_TEST_REPORT(passed, "Testing 9-point stencil computation");

  return EXIT_SUCCESS;
}
