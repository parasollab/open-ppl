/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/
#include <iomanip>
#include <functional>
#include <stapl/utility/do_once.hpp>
#include <stapl/multiarray.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/skeletons/functional/transpose_2d.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/skeletons/spans/blocked.hpp>
#include <stapl/skeletons/functional/zip.hpp>
#include "../test_report.hpp"

using namespace stapl;

namespace {

#ifdef SHOW_RESULTS
template <typename V>
void print_2d_array(V&& v, std::string title)
{
  std::size_t nx, ny;
  std::tie(nx, ny) = v.domain().dimensions();
  std::size_t startx, starty;
  std::tie(startx, starty) = v.domain().first();

  std::cout << title << std::endl;
  for (std::size_t i = 0; i < nx; ++i)
  {
    for (std::size_t j = 0; j < ny; ++j) {
      std::cout << std::setw(4)  << v[make_tuple(startx+i, starty+j)];
    }
    std::cout << std::endl;
  }
}
#endif

template <int i, typename... D>
void test_multiarray(D... dimensions)
{
  typedef size_t                                    value_type;
  typedef multiarray<i, value_type>                 multiarray_type;
  typedef multiarray_view<multiarray_type>          view_type;
  typedef typename multiarray_type::traversal_type  traversal_type;
  typedef typename multiarray_type::dimensions_type dimensions_type;

  dimensions_type dims = dimensions_type(dimensions...);
  multiarray_type input(dims);
  multiarray_type output(dims);

  view_type input_view(input);
  view_type output_view(output);

  using namespace skeletons;

  stapl::iota(linear_view(input_view), value_type(0));

  auto p = skeletons::zip<2>(stapl::plus<value_type>(),
                             skeleton_traits<spans::blocked<i>>());

  skeletons::execute(
    skeletons::default_execution_params(),
    sink<value_type, spans::blocked<i>>(p),
    input_view, input_view, output_view);

#ifdef VALIDATE_RESULTS
  do_once([&input_view, &output_view](void) {
    auto linear_input_view = linear_view(input_view);
    auto linear_output_view = linear_view(output_view);
    bool is_valid = std::equal(linear_output_view.begin(),
                               linear_output_view.end(),
                               boost::make_transform_iterator(
                                 linear_input_view.begin(),
                                 boost::bind(std::plus<value_type>(), _1, _1)));

    std::stringstream s;
    s << "multiarray<" << i << "> test";
    STAPL_TEST_REPORT(is_valid, s.str());
  });
#endif
}

//////////////////////////////////////////////////////////////////////
/// @brief Computes a balanced partition across each dimension for
/// a given 2D domain.
///
/// @param dims  a 2D dimension
/// @param parts a tuple consisting of number of partitions per dimension
///
/// @return a @c partition2 with balanced partition on each dimension.
//////////////////////////////////////////////////////////////////////
template <typename Dims, typename Parts>
nd_partition<
  stapl::tuple<
    balanced_partition<indexed_domain<std::size_t>>,
    balanced_partition<indexed_domain<std::size_t>>>>
compute_partition(Dims&& dims, Parts&& parts)
{
  typedef indexed_domain<std::size_t>              dom_t;
  typedef balanced_partition<dom_t>                bal_t;
  typedef nd_partition<stapl::tuple<bal_t, bal_t>> partition_t;
  bal_t p0(dom_t(0, stapl::get<0>(dims)-1, true),
           stapl::get<0>(parts));
  bal_t p1(dom_t(0, stapl::get<1>(dims)-1, true),
           stapl::get<1>(parts));
  return partition_t(p0, p1);
}

void test_transpose_2d(std::array<std::size_t, 2> dims,
                       std::array<std::size_t, 2> parts)
{
  using boost::mpl::int_;

  typedef double value_t;
  typedef tuple<int_<0>, int_<1>> reverse_traversal;

  auto p0 = compute_partition(make_tuple(dims[0], dims[1]),
                              make_tuple(parts[0], parts[1]));
  auto p1 = compute_partition(make_tuple(dims[1], dims[0]),
                              make_tuple(parts[0], parts[1]));

  multiarray<2, value_t, reverse_traversal, decltype(p0)> u0_container(p0);
  multiarray<2, value_t, reverse_traversal, decltype(p1)> u1_container(p1);

  auto u0 = make_multiarray_view(u0_container);
  auto u1 = make_multiarray_view(u1_container);

  auto lu0 = linear_view(u0);

  stapl::iota(lu0, value_t(0));

  /// The views need to be coarsened separately from each other
  /// @todo this custom data coarsener should be generalized and refactored
  stapl::default_coarsener coarsener;
  auto c1 = coarsener(std::make_tuple(u0));
  auto c2 = coarsener(std::make_tuple(u1));

  skeletons::execute(
    skeletons::default_execution_params(),
    skeletons::transpose_2d<value_t>(p1),
    linear_view(stapl::get<0>(c1)), linear_view(stapl::get<0>(c2)));

#ifdef VALIDATE_RESULTS
  do_once([&](void) {
    typedef decltype(u0.domain().first()) index_t;
    bool is_valid = true;
    for (std::size_t i = 0; i < dims[0]; ++i)
    {
      for (std::size_t j = 0; j < dims[1]; ++j)
      {
        is_valid &= u0[index_t(i,j)] == u1[index_t(j, i)];
      }
    }

    std::stringstream s;
    s << "transpose_2d test";
    STAPL_TEST_REPORT(is_valid, s.str());
  });
#endif
#ifdef SHOW_RESULTS
  stapl::do_once([&](){
    print_2d_array(u0, "u0 ---");
    print_2d_array(u1, "u1 ---");
  });
#endif
}

} // namespace

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2)
  {
    std::cout<< "usage: test_multiarray <nElemPerDim>" <<std::endl;
    exit(1);
  }

  size_t n = atol(argv[1]);

  test_multiarray<2>(n, n);
  test_multiarray<3>(n, n, n);
  test_multiarray<4>(n, n, n, n);

  test_transpose_2d({{n, n}}, {{1, get_num_locations()}});
  test_transpose_2d({{n, n}}, {{get_num_locations(), 1}});

  return EXIT_SUCCESS;
}
