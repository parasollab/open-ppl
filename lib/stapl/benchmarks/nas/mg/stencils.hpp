/*
// Copyright (c) 2000-2010, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_NAS_MG_STENCILS
#define STAPL_BENCHMARKS_NAS_MG_STENCILS

#include <stapl/utility/tuple.hpp>

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/skeletons/spans/blocked.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/views/strided_view.hpp>
#include <stapl/views/periodic_boundary_view.hpp>
#include <stapl/views/stencil_view.hpp>

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief 27-point stencil that computes the weighted average of the
///        values in the view. The weights are dependent on how far
///        the point is from the original point.
//////////////////////////////////////////////////////////////////////
class stencil27
{
private:
  const coefficient_view m_coefs;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Define weights for stencil.
  ///
  /// @param c0 Weight for the center point in the stencil
  /// @param c1 Weight for points 1 hop away from the center
  /// @param c2 Weight for points 2 hops away from the center
  /// @param c3 Weight for points 3 hops away from the center
  //////////////////////////////////////////////////////////////////////
  stencil27(double c0, double c1, double c2, double c3)
    : m_coefs(c0, c1, c2, c3)
  { }

  template<typename Elements>
  double operator()(Elements&& s) const
  {
    using stapl::get;

    double seq_val = 0.;
    auto&& first = s.domain().first();
    auto&& last = s.domain().last();


    for (std::size_t i = get<0>(first); i <= get<0>(last); ++i)
    {
      for (std::size_t j = get<1>(first); j <= get<1>(last); ++j)
      {
        for (std::size_t k = get<2>(first); k <= get<2>(last); ++k)
        {
          int x_off = static_cast<int>(i)-get<0>(first)-1;
          int y_off = static_cast<int>(j)-get<1>(first)-1;
          int z_off = static_cast<int>(k)-get<2>(first)-1;

          double d = s(i, j, k);
          seq_val += d * m_coefs(x_off, y_off, z_off);
        }
      }
    }

    return seq_val;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_coefs);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Stencil for trilinear interpolation. Summation of all points
///        in the subview weighted by a known coefficient.
//////////////////////////////////////////////////////////////////////
struct interp_stencil
{
  const int    x_min;
  const int    y_min;
  const int    z_min;
  const double coefficient;

  //////////////////////////////////////////////////////////////////////
  /// @brief Initialize stencil values based on the phase of the trilinear
  ///        interpolation. There are 8 phases, specified by 3 binary
  ///        digits.
  ///
  /// @param x Phase in the x dimension
  /// @param y Phase in the y dimension
  /// @param z Phase in the z dimension
  //////////////////////////////////////////////////////////////////////
  interp_stencil(int x, int y, int z)
    : x_min(x-1), y_min(y-1), z_min(z-1),
      coefficient(1.0 / (x ? 2.0 : 1.0) / (y ? 2.0 : 1.0) / (z ? 2.0 : 1.0))
  { }

  template<typename Elements>
  double operator()(Elements&& s) const
  {
    using stapl::get;

    double tmp = 0.0;

    auto&& first = s.domain().first();
    auto&& last = s.domain().last();

    for (std::size_t i = get<0>(first)-x_min; i <= get<0>(last)-1; ++i)
      for (std::size_t j = get<1>(first)-y_min; j <= get<1>(last)-1; ++j)
        for (std::size_t k = get<2>(first)-z_min; k <= get<2>(last)-1; ++k)
          tmp += s(i, j, k);

    return coefficient * tmp;
  }

  void define_type(stapl::typer& t)
  {
    t.member(x_min);
    t.member(y_min);
    t.member(z_min);
    t.member(coefficient);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Apply a stencil operator to the RHS argument and increment
///        the LHS argument with the return.
//////////////////////////////////////////////////////////////////////
template<typename StencilOperator>
struct apply_increment
{
  const StencilOperator m_stencil_operator;

  apply_increment(StencilOperator const& stencil_operator)
    : m_stencil_operator(stencil_operator)
  { }

  typedef void result_type;

  template<typename LHS, typename RHS>
  void operator()(LHS&& lhs, RHS&& rhs) const
  {
    lhs += m_stencil_operator(rhs);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_stencil_operator);
  }
};



//////////////////////////////////////////////////////////////////////
/// @brief Apply a stencil operator to the RHS argument and assign
///        the LHS argument the return, overwriting it.
//////////////////////////////////////////////////////////////////////
template<typename StencilOperator>
struct apply_assign
{
  typedef void result_type;

  const StencilOperator m_stencil_operator;

  apply_assign(StencilOperator const& stencil_operator)
    : m_stencil_operator(stencil_operator)
  { }

  template<typename LHS, typename RHS>
  void operator()(LHS&& lhs, RHS&& rhs) const
  {
    lhs = m_stencil_operator(rhs);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_stencil_operator);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Apply a stencil operator to the last argument, subtract it
///        from the middle argument and assign it to the first argument.
//////////////////////////////////////////////////////////////////////
template<typename StencilOperator>
struct resid_wf
{
  const StencilOperator m_stencil_operator;

  typedef void result_type;

  resid_wf(StencilOperator const& stencil_operator)
    : m_stencil_operator(stencil_operator)
  { }

  template<typename LHS, typename RHS1, typename RHS2>
  void operator()(LHS&& lhs, RHS1&& rhs1, RHS2&& rhs2) const
  {
    lhs = rhs1 - m_stencil_operator(rhs2);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_stencil_operator);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Project a grid onto the next coarser grid using a trilinear
///        finite element projection:  s = r' = P r
///
/// @param fine The fine grid
/// @param coarse The coarse grid that has half as many elements in each
///             dimension as the fine grid
//////////////////////////////////////////////////////////////////////
template<typename Fine, typename Coarse>
void rprj3(Fine&& fine, Coarse&& coarse)
{
  // FIXME (coarsening): this needs to be executed coarse-grained when
  // coarsening for the stencil view is available.
  auto stencil_view = stapl::make_stencil_view<3>(fine);

  // Note that the domain of a stencil view starts at index 1, so in order
  // to start striding from the second element in each dimension, the start
  // parameter has to be set to 2.
  auto strided_stencil_view = stapl::make_strided_view(
    stencil_view,
    stapl::make_tuple(2, 2, 2),
    stapl::make_tuple(2, 2, 2)
  );

  auto stencil_skeleton = skeletons::zip<2>(
    apply_assign<stencil27>(stencil27(0.5, 0.25, 0.125, 0.0625)),
    skeletons::skeleton_traits<skeletons::spans::blocked<3>>()
  );

  skeletons::execute(
    skeletons::default_execution_params(),
    stencil_skeleton, coarse, strided_stencil_view
  );
}



//////////////////////////////////////////////////////////////////////
/// @brief Add the trilinear interpolation of the correction from the
///        coarser grid to the current approximation:  u = u + Qu'.
///
/// @param coarse The coarse grid
/// @param fine The fine grid that has twice as many elements in each
///             dimension as the coarse grid
//////////////////////////////////////////////////////////////////////
template<typename Coarse, typename Fine>
void interp(Coarse&& coarse, Fine&& fine)
{
  // FIXME (coarsening): this needs to be executed coarse-grained when
  // coarsening for the stencil view is available.
  auto stencil_view = stapl::make_stencil_view<3>(coarse);

  for (int a = 0; a <= 1; ++a)
  {
    for (int b = 0; b <= 1; ++b)
    {
      for (int c = 0; c <= 1; ++c)
      {
        auto strided = stapl::make_strided_view(
          fine,
          stapl::make_tuple(2, 2, 2),
          stapl::make_tuple(1-a, 1-b, 1-c)
        );

        auto stencil_skeleton = skeletons::zip<2>(
          apply_increment<interp_stencil>(interp_stencil(a,b,c)),
          skeletons::skeleton_traits<skeletons::spans::blocked<3>>()
        );

        skeletons::execute(
          skeletons::default_execution_params(),
          stencil_skeleton, strided, stencil_view
        );
      }
    }
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Apply an approximate inverse as smoother:  u = u + Cr
///
/// @param r The residual grid
/// @param u The solution grid
//////////////////////////////////////////////////////////////////////
template<typename Grid1, typename Grid2>
void psinv(Grid1&& r, Grid2&& u)
{
  // FIXME (coarsening): this needs to be executed coarse-grained when
  // coarsening for the stencil view is available.
  auto stencil_view = stapl::make_stencil_view<3>(r);

  auto stencil_skeleton = skeletons::zip<2>(
    apply_increment<stencil27>(stencil27(-3.0/8.0, 1.0/32.0, -1.0/64.0, 0.0)),
    skeletons::skeleton_traits<skeletons::spans::blocked<3>>()
  );

  skeletons::execute(
    skeletons::default_execution_params(),
    stencil_skeleton, u, stencil_view
  );
}



//////////////////////////////////////////////////////////////////////
/// @brief Compute the residual r = v - Au.
//////////////////////////////////////////////////////////////////////
template<typename U, typename V, typename R>
void resid(U const& u, V const& v, R const& r)
{
  // FIXME (coarsening): this needs to be executed coarse-grained when
  // coarsening for the stencil view is available.
  auto stencil_view = stapl::make_stencil_view<3>(u);

  auto stencil_skeleton = skeletons::zip<3>(
    resid_wf<stencil27>(stencil27(-8.0/3.0, 0.0, 1.0/6.0, 1.0/12.0)),
    skeletons::skeleton_traits<skeletons::spans::blocked<3>>()
  );

  skeletons::execute(
    skeletons::default_execution_params(),
    stencil_skeleton, r, v, stencil_view
  );
}

#endif
