/*
// Copyright (c) 2000-2010, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <stapl/array.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/multiarray.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/skeletons/functional/stencil.hpp>
#include <stapl/skeletons/functional/copy.hpp>
#include <stapl/skeletons/functional/sink.hpp>
#include <stapl/skeletons/functional/skeleton_traits.hpp>
#include <stapl/skeletons/functional/zip.hpp>
#include <stapl/skeletons/operators/compose.hpp>
#include <stapl/skeletons/operators/elem.hpp>
#include <stapl/skeletons/param_deps/stencil_2d_pd.hpp>
#include <stapl/skeletons/spans/blocked.hpp>
#include <stapl/skeletons/transformations/coarse/stencil.hpp>
#include <stapl/skeletons/transformations/coarse/compose.hpp>
#include <stapl/skeletons/utility/coarse_identity.hpp>
#include <stapl/views/metadata/coarseners/null.hpp>
#include <stapl/utility/do_once.hpp>
#include "../test_report.hpp"

// #define TEST_FINE_GRAIN

using namespace stapl;

namespace {

template <typename V>
bool validate(V&& v, int init_val, int num_points)
{
  bool passed = true;
  const int expected = init_val * num_points;
  const auto dom = v.domain();
  auto cur = dom.first();
  for (size_t i = 0; passed and i < dom.size(); ++i)
  {
    passed &= expected == v[cur];
    cur = dom.advance(cur, 1);
  }
  return passed;
}

//////////////////////////////////////////////////////////////////////
/// @brief Helper struct that returns different coefficients based
///        on how far a certain point is from the initial point.
//////////////////////////////////////////////////////////////////////
struct coefficient_view
{
  const double m_c0, m_c1, m_c2;

  coefficient_view(double c0, double c1, double c2)
    : m_c0(c0), m_c1(c1), m_c2(c2)
  { }

  double const& operator()(int i, int j) const
  {
    const int distance = (i != 0 ? 1 : 0) + (j != 0 ? 1 : 0);

    switch (distance)
    {
      case 0: return m_c0;
      case 1: return m_c1;
      default: return m_c2;
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_c0);
    t.member(m_c1);
    t.member(m_c2);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Stencil work function that receives a single point and
///        the 9-point stencil view of its neighbors.
///
///        This computes a weighted average, where the weights for
///        the points with Manhattan distance of 0, 1 and 2 can be
///        specified in the constructor.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct stencil9
{
private:
  using coord_t = std::tuple<std::size_t, std::size_t>;
  using values_order_t = std::vector<std::tuple<coord_t>>;
  coefficient_view m_coeff;
  coord_t          m_center;
  values_order_t   m_values_order;
public:
  using result_type = T;

  stencil9(double const c0, double const c1, double const c2)
    : m_coeff(c0, c1, c2)
  { }

  void set_values_order(values_order_t const& values_order)
  {
    m_values_order = values_order;
  }

  void set_center(coord_t const& center)
  {
    m_center = center;
  }

  template<typename P>
  T operator()(P&& points) const
  {
    stapl_assert(points.size() == m_values_order.size(),
                 "The dependencies order size does not match points size.");
    T result = 0;

    std::size_t i = std::get<0>(m_center);
    std::size_t j = std::get<1>(m_center);

    for (std::size_t index = 0; index < points.size(); ++index)
    {
      auto cur = std::get<0>(m_values_order[index]);
      std::size_t cur_i = std::get<0>(cur);
      std::size_t cur_j = std::get<1>(cur);
      const int x_off = static_cast<int>(cur_i) - i - 1;
      const int y_off = static_cast<int>(cur_j) - j - 1;
      const double c = m_coeff(x_off, y_off);
      result += c * points[index];
    }

    return result;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_coeff);
    t.member(m_center);
    t.member(m_values_order);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Stencil work function that receives arbitrary points and
///        return the sum of all points.
//////////////////////////////////////////////////////////////////////
template<typename P>
struct stencil_op
{
  using result_type = P;

  template <typename Point>
  result_type operator()(Point&& point) const
  {
    return point;
  }

  template <typename Point, typename... Points>
  result_type operator()(Point&& point, Points&&... points) const
  {
    return point + (*this)(points...);
  }
};

template<size_t N, class Op>
struct strict_op {
  Op op;

  strict_op() = default;

  using result_type = typename Op::result_type;

  template<class... Ts>
  constexpr result_type
  operator()(Ts&&... ts) const {
    static_assert(sizeof...(Ts) == N, "Must have N arguments exactly");
    return op(std::forward<Ts>(ts)...);
  }

  void define_type(stapl::typer& t)
  {
    t.member(op);
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
      bind(multiplies<double>(), _1, _1),
      plus<double>(),
      linear_view(view)
    );

  auto dims = view.dimensions();

  return std::sqrt(sum / get<1>(dims) / get<0>(dims));
}

template<typename ValueType, size_t N, size_t P, size_t... Bounds>
void test_stencil(size_t n, index_sequence<Bounds...>) {
  using namespace skeletons;

  constexpr size_t NP = 2*N*P+1;

  using multiarray_type = multiarray<N, ValueType>;
  using dimensions_type = typename multiarray_type::dimensions_type;
  using op_t = strict_op<NP, stencil_op<ValueType>>;

  dimensions_type dims = homogeneous_tuple<N>(n);
  multiarray_type input(dims);
  multiarray_type output(dims);


  auto input_view  = make_multiarray_view(input);
  auto output_view = make_multiarray_view(output);

  fill(input_view, n);


#ifdef TEST_FINE_GRAIN
  skeletons::execute(
    skeletons::default_execution_params(),
    sink<ValueType, spans::blocked<N>>(
      stencil<tags::stencil<N, NP, true>> (op_t{})),
    input_view,
    output_view);
#else

  skeletons::execute(
    skeletons::execution_params(default_coarsener()),
    sink<ValueType>(
      compose(map(coarse_identity<ValueType, N>(),
                  skeleton_traits<spans::blocked<N>>()),
              coarse(stencil<tags::stencil<N, NP>>
                       (op_t{}))),
      coarse(copy<ValueType>(skeleton_traits<spans::blocked<N>>()))),
    input_view, output_view);
#endif

  std::ostringstream os;
  os << "Testing " << NP << "-point " << N << "D stencil";
  std::string test_name = os.str();

  STAPL_TEST_REPORT(validate(output_view, n, NP),
                    test_name + "(periodic)"
#ifndef TEST_FINE_GRAIN
                    " COARSE "
#endif
                    );

  multiarray_type boundary_array(dims);

  auto boundary_view = make_multiarray_view(boundary_array);


#ifdef TEST_FINE_GRAIN
  skeletons::execute(
    skeletons::default_execution_params(),
    sink<ValueType, spans::blocked<N>>(
      stencil<tags::stencil<N, NP, true>> (op_t{})),
    input_view,
    ignore_index<Bounds>(boundary_view)...,
    output_view);

#else
  skeletons::execute(
    skeletons::execution_params(default_coarsener()),
    sink<ValueType>(
      compose(map(coarse_identity<ValueType, N>(),
                  skeleton_traits<spans::blocked<N>>()),
              coarse(stencil<tags::stencil<N, NP>>
                       (op_t{}))),
      coarse(copy<ValueType>(skeleton_traits<spans::blocked<N>>()))),
    input_view,
    ignore_index<Bounds>(boundary_view)...,
    output_view);

#endif
  STAPL_TEST_REPORT(validate(output_view, n, NP),
                    test_name + "(non-periodic)"
#ifndef TEST_FINE_GRAIN
                    " COARSE "
#endif
                    );
}

// 1D specialization
template<typename ValueType, size_t N, size_t P>
void test_stencil(size_t n, index_sequence<0, 1>) {
  using namespace skeletons;

  /// Testing coarse-grain 1D stencil
  array<ValueType> in(n);
  array<ValueType> out(n);
  auto&& in_view  = make_array_view(in);
  auto&& out_view = make_array_view(out);

  static_assert(N == 1, "If this fails, something is wrong with the index seq");
  constexpr size_t NP = 2*N*P+1;
  using tag = tags::stencil<N, NP>;

  using op_t = strict_op<NP, stencil_op<ValueType>>;

  fill(in_view, n);

  /// @todo the identity phase can be removed whenever reading partial values
  /// (filtered) is made possible (consume_from(view[i], filter))
#ifdef TEST_FINE_GRAIN
  skeletons::execute(
    skeletons::default_execution_params(),
    sink<ValueType>(compose(map(identity<ValueType>()),
                            stencil<tag>(op_t{}))),
    in_view, out_view);
#else
  skeletons::execute(
    skeletons::execution_params(default_coarsener()),
    coarse(sink<ValueType>(compose(map(identity<ValueType>()),
                               stencil<tag>(op_t{})))),
    in_view, out_view);
#endif

  /// verification of results
  do_once([&out_view, &n](){
    std::ostringstream os;
    os << "Testing " << NP << "-point 1D stencil";
    std::string test_name = os.str();
    bool passed = true;
    for (std::size_t i = 0 ; passed and i < n; ++i)
    {
      passed &= (out_view[i] == n*NP) ;
    }
    STAPL_TEST_REPORT(passed, test_name);
  });
}

template<typename ValueType, size_t N, size_t P>
void test_stencil(size_t n) {
  test_stencil<ValueType,N,P>(n, make_index_sequence<2*N>{});
}

template <typename ValueType>
void test_custom_2d_stencil(size_t n) {
  using namespace skeletons;

  // Testing fine-grain 2D stencil
  // create grids and views
  multiarray<2, ValueType> input_grid(make_tuple(n, n));
  auto&& input_grid_view = make_multiarray_view(input_grid);

  // fill grids with all zeros except for the first value, which is one
  fill(input_grid_view, 0);

  do_once([&](){
    input_grid_view[make_tuple(0, 0)] = 1;
  });

  // compute norm
  double n0 = norm(input_grid_view);

  using namespace skeletons;
  skeletons::execute(
    skeletons::default_execution_params(),
    sink<ValueType, spans::blocked<2>>(
      compose(
        map(identity<ValueType>(), skeleton_traits<spans::blocked<2>>()),
        elem<spans::blocked<2>>(
          stencil_2d_pd(stencil9<ValueType>(0.5, 0.25, 0.125))))
      ),
    input_grid_view, input_grid_view);

  /// compute post-stencil norm and make sure that it went down
  double n1 = norm(input_grid_view);

  STAPL_TEST_REPORT(n1 < n0, "Testing 9-point 2D Square stencil");

}

template<typename ValueType>
void test_custom_1d_stencil(size_t n) {
  using namespace skeletons;
  /// Testing coarse-grain 1D stencil
  array<ValueType> in(n);
  array<ValueType> out(n);
  auto&& in_view  = make_array_view(in);
  auto&& out_view = make_array_view(out);

  fill(in_view, n);

  /// @todo the identity phase can be removed whenever reading partial values
  /// (filtered) is made possible (consume_from(view[i], filter))
  skeletons::execute(
    skeletons::execution_params(default_coarsener()),
    coarse(sink<ValueType>(compose(map(identity<ValueType>()),
                               stencil(stencil_op<ValueType>())))),
    in_view, out_view);

  /// verification of results
  do_once([&out_view, &n](){
  bool passed = true;
    for (std::size_t i = 0 ; passed and i < n ; ++i)
    {
      passed &= (out_view[i] == 3*n) ;
    }
    STAPL_TEST_REPORT(passed, "Testing 3-point 1D stencil");
  });
}

} //namespace

exit_code stapl_main(int argc, char** argv)
{

  if (argc < 2)
  {
    std::cout << "usage: test_stencil <n>" << std::endl;
    exit(1);
  }

  const std::size_t n = atol(argv[1]);
  using value_type = int;

  test_custom_1d_stencil<value_type>(n);
  test_custom_2d_stencil<value_type>(n);

  test_stencil<value_type, 1, 2>(n);
  test_stencil<value_type, 2, 2>(n);
  return EXIT_SUCCESS;
}
