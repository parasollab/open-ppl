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
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/views/functor_view.hpp>
#include <stapl/skeletons/functional/wavefront.hpp>
#include <stapl/skeletons/functional/sink.hpp>
#include <stapl/skeletons/spans/blocked.hpp>
#include <stapl/skeletons/transformations/coarse/compose.hpp>
#include <stapl/skeletons/transformations/coarse/wavefront.hpp>
#include "../test_report.hpp"

using namespace stapl;

namespace {

template <int i, typename T>
struct wf_sum;


//////////////////////////////////////////////////////////////////////
/// @brief A generator for complex numbers which produces [0+0j, 1+0j,
/// ..., n-1+0j]
///
/// @tparam T the type of real part of the generated complex number
//////////////////////////////////////////////////////////////////////
template <typename T, int i>
struct boundary_generator
{
private:
  T m_value;
public:
  typedef typename indexed_domain<
                     std::size_t, i, typename default_traversal<i>::type
                   >::index_type index_type;
  typedef T                      result_type;
  boundary_generator(T value)
    : m_value(value)
  { }

  result_type operator()(index_type const& idx) const
  {
    return m_value;
  }

  void define_type(typer& t)
  {
    t.member(m_value);
  }
};


template <typename T>
struct wf_sum<3, T>
{
  typedef T result_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Checks if all the values received from non-view arguments
  /// are equal. If so it increments both view elements by 1.
  ///
  /// @param v1 first input
  /// @param v2 second input
  /// @param v3 the input from the wavefront first direction
  /// @param v4 the input from the wavefront second direction
  /// @param v5 the input from the wavefront third direction
  ///
  /// @return v1 + 1 if all received values are equal, otherwise 0.
  //////////////////////////////////////////////////////////////////////
  template <typename V1, typename V2,
            typename V3, typename V4, typename V5>
  result_type operator()(V1&& v1, V2&& v2,
                         V3&& v3, V4&& v4, V5&& v5) const
  {
    T max_v = std::max({T(v3), T(v4), T(v5)});
    /// The values in v3, v4, and v5 are either all zero, coming from the
    /// boundary cases, or they are the same.
    bool correct = ((v3 == 0) or (v3 == max_v)) and
                   ((v4 == 0) or (v4 == max_v)) and
                   ((v5 == 0) or (v5 == max_v));

    v1 = correct ? max_v + 1 : 0;
    v2 = v1;
    return v1;
  }
};

template <typename T>
struct wf_sum<2, T>
{
  typedef T result_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Checks if all the values received from non-view arguments
  /// are equal. If so it increments both view elements by 1.
  ///
  /// @param v1 first input
  /// @param v2 the input from the wavefront first direction
  /// @param v3 the input from the wavefront second direction
  ///
  /// @return v1 + 1 if all received values are equal, otherwise 0.
  //////////////////////////////////////////////////////////////////////
  template <typename V1, typename V2, typename V3>
  result_type operator()(V1&& v1, V2&& v2, V3&& v3) const
  {
    T max_v = std::max({T(v2), T(v3)});
    /// The values in v2 and v3 are either all zero, coming from the
    /// boundary cases, or they are the same.
    bool correct =
      ((v2 == 0) or (v2 == max_v)) and ((v3 == 0) or (v3 == max_v));

    v1 = correct ? max_v + 1 : 0;
    return v1;
  }
};



template <typename T>
struct sum_op
{
  typedef T result_type;

  void set_directions(
         std::array<skeletons::direction, 2>)
  { }

  template <typename V1, typename V2, typename V3>
  result_type operator()(V1&& v1, V2&& v2, V3&& v3) const
  {
    v1 = v1 + v2 + v3;
    return v1;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Verifies a 2D wavefront skeleton initialized with predefined
/// values. The results should start from the corner specified with
/// all 1s and each subsequent row should be the partial sum of the values
/// of its previous row. The direction of partial sum is also defined
/// based on the corner selected.
///
/// @param title       title of this test.
/// @param input_view  the input view which contains the values to be verified.
/// @param output_view the output view which should have the same values as the
///                    the input view.
//////////////////////////////////////////////////////////////////////
template <typename View>
void verify(std::string title, View& input_view, View& output_view,
            std::array<skeletons::position, 2> corners)
{
  bool eq = stapl::equal(linear_view(input_view), linear_view(output_view));
  stapl::do_once([&](){
    std::size_t n1, n2;
    std::tie(n1, n2) = input_view.domain().dimensions();

    typedef typename View::value_type value_type;
    typedef typename View::index_type index_type;
    std::vector<value_type> res(n1, 1);
    bool is_valid = true;
    using namespace skeletons;

    using wp = position;
    auto wavefront_idx =
      [&](std::size_t i, std::size_t j)
      {
        i = corners[0] == wp::first ? i : n1-i-1;
        j = corners[1] == wp::first ? j : n2-j-1;
        return index_type(i, j);
      };

    for (std::size_t i = 0; i < n1; ++i)
    {
      for (std::size_t j = 0; j < n2; ++j) {
        is_valid &= input_view[wavefront_idx(i, j)] == res[j];
      }
      std::partial_sum(res.begin(), res.end(), res.begin());
    }
    // print_2d_array(output_view, title);
    STAPL_TEST_REPORT(is_valid and eq, title);
  });
}

//////////////////////////////////////////////////////////////////////
/// @brief Tests the 3D wavefront skeleton from a given corner.
///
/// @param view1   input
/// @param view2   input
/// @param corners determines the starting corner.
/// @param n1      first dimension size
/// @param n2      second dimension size
/// @param n3      third dimension size
//////////////////////////////////////////////////////////////////////
template <typename View>
void test_wavefront(View& view1, View& view2,
                    std::array<skeletons::position, 3> corners)
{
  using namespace skeletons;
  using namespace skeletons::wavefront_utils;
  using value_type = typename View::value_type;

  auto wavefront_skeleton = wavefront<2>(wf_sum<3, value_type>(), corners);

  auto const size = view1.domain().dimensions();
  auto const f = first_index(corners, size);
  auto const l =  last_index(corners, size);

  std::string title = "3D wavefront from ";
  title += (corners[2] == position::first ? "front " : "back ");
  title += (corners[0] == position::first ? "top " : "bottom ");
  title += (corners[1] == position::first ? "left " : "right ");
  title += "corner";

  auto boundary_v1 = functor_view(size, boundary_generator<value_type, 3>(0));
  auto boundary_v2 = functor_view(size, boundary_generator<value_type, 3>(0));
  auto boundary_v3 = functor_view(size, boundary_generator<value_type, 3>(0));

  // Testing 3D wavefront
  stapl::fill(view1, value_type(0));
  stapl::fill(view2, value_type(0));

  view1[f] = 1;
  view2[f] = 1;

  skeletons::execute(
    skeletons::default_execution_params(),
    wavefront_skeleton,
    view1, view2,
    boundary_v1, boundary_v2, boundary_v3);

  std::size_t const expected_sum = std::get<0>(size) +
                                   std::get<1>(size) +
                                   std::get<2>(size) - 2;

  bool passed = (view1[l] == expected_sum) && (view2[l] == expected_sum);
  STAPL_TEST_REPORT(passed, title);

  // Testing coarsened 3D wavefront
  stapl::fill(view1, value_type(0));
  stapl::fill(view2, value_type(0));

  view1[f] = 1;
  view2[f] = 1;

  skeletons::execute(
    skeletons::execution_params(default_coarsener()),
    skeletons::coarse(wavefront_skeleton),
    view1, view2,
    boundary_v1, boundary_v2, boundary_v3);

  title += " (coarsened version 1)";
  passed = (view1[l] == expected_sum) && (view2[l] == expected_sum);
  STAPL_TEST_REPORT(passed, title);

  // Testing coarsened version 2 3D wavefront
  stapl::fill(view1, value_type(0));
  stapl::fill(view2, value_type(0));

  view1[f] = 1;
  view2[f] = 1;

  skeletons::execute(
    skeletons::execution_params(default_coarsener()),
    skeletons::coarse<tags::only_boundary, tags::sequential_execution_partial>(
      wavefront_skeleton),
    view1, view2, boundary_v1, boundary_v2, boundary_v3);

  title[title.size() - 2] = '2';
  passed = (view1[l] == expected_sum) && (view2[l] == expected_sum);
  STAPL_TEST_REPORT(passed, title);
}

//////////////////////////////////////////////////////////////////////
/// @brief Tests the 3D wavefront skeleton from all corners.
///
/// @param n1 first dimension size
/// @param n2 second dimension size
/// @param n3 third dimension size
//////////////////////////////////////////////////////////////////////
void test_wavefront(std::size_t n1, std::size_t n2, std::size_t n3)
{
  using value_type      = size_t;
  using multiarray_type = multiarray<3, value_type>;;
  using dimensions_type = typename multiarray_type::dimensions_type;
  using corner_t        = skeletons::position;
  using corner_type     = std::array<corner_t, 3>;

  dimensions_type dims = dimensions_type(n1, n2, n3);
  multiarray_type input1(dims);
  multiarray_type input2(dims);
  auto view1 = make_multiarray_view(input1);
  auto view2 = make_multiarray_view(input2);

  using namespace skeletons;
  using namespace wavefront_utils;

  auto f = position::first;
  auto l = position::last;

  test_wavefront(view1, view2, corner_type{{f, f, f}});
  test_wavefront(view1, view2, corner_type{{f, l, f}});
  test_wavefront(view1, view2, corner_type{{l, f, f}});
  test_wavefront(view1, view2, corner_type{{l, l, f}});
  test_wavefront(view1, view2, corner_type{{f, f, l}});
  test_wavefront(view1, view2, corner_type{{f, l, l}});
  test_wavefront(view1, view2, corner_type{{l, f, l}});
  test_wavefront(view1, view2, corner_type{{l, l, l}});
}

//////////////////////////////////////////////////////////////////////
/// @brief Tests the 2D wavefront skeleton from a given corner.
///
/// @param view input
/// @param n1   first dimension size
/// @param n2   second dimension size
/// @param n3   third dimension size
//////////////////////////////////////////////////////////////////////
template <typename View>
void test_wavefront(View& input_view, View& output_view,
                    std::array<skeletons::position, 2> corners)
{
  using namespace skeletons;
  using namespace wavefront_utils;
  using value_type = typename View::value_type;

  auto wavefront_skeleton = wavefront(sum_op<value_type>(), corners);

  std::string title = "2D wavefront from ";
  title += (corners[0] == position::first ? "top " : "bottom ");
  title += (corners[1] == position::first ? "left " : "right ");
  title += "corner";

  auto const size = input_view.domain().dimensions();
  auto const f = first_index(corners, size);
  auto const l = last_index (corners, size);

  auto boundary_v1 = functor_view(size, boundary_generator<value_type, 2>(0));
  auto boundary_v2 = functor_view(size, boundary_generator<value_type, 2>(0));

  // testing 2D wavefront
  stapl::fill( input_view, value_type(0));
  stapl::fill(output_view, value_type(0));

  input_view[f] = 1;

  skeletons::execute(
    skeletons::default_execution_params(),
    sink<value_type, spans::blocked<2>>(wavefront_skeleton),
    input_view,
    boundary_v1, boundary_v2,
    output_view);

  verify(title, input_view, output_view, corners);

  // testing coarse 2D wavefront version 1
  stapl::fill( input_view, value_type(0));
  stapl::fill(output_view, value_type(0));

  input_view[f] = 1;

  skeletons::execute(
    skeletons::execution_params(default_coarsener()),
    skeletons::coarse(sink<value_type, spans::blocked<2>>(wavefront_skeleton)),
    input_view,
    boundary_v1, boundary_v2,
    output_view);
  title += " (coarsened version 1)";

  verify(title, input_view, output_view, corners);

  // testing coarse 2D wavefront version 2
  stapl::fill( input_view, value_type(0));

  input_view[f] = 1;

  auto wavefront_sk2 = wavefront(wf_sum<2, value_type>(), corners);

  skeletons::execute(
    skeletons::execution_params(default_coarsener()),
    skeletons::coarse<tags::only_boundary, tags::sequential_execution_partial>(
      wavefront_sk2),
    input_view,
    boundary_v1, boundary_v2);

  title[title.size() - 2] = '2';

  std::size_t const expected_sum = std::get<0>(size) +
                                   std::get<1>(size) - 1;

  bool passed = input_view[l] == expected_sum;

  STAPL_TEST_REPORT(passed, title);
}

//////////////////////////////////////////////////////////////////////
/// @brief Tests the 2D wavefront skeleton from all corners.
///
/// @param n1 first dimension size
/// @param n2 second dimension size
//////////////////////////////////////////////////////////////////////
void test_wavefront(std::size_t n1, std::size_t n2)
{
  using value_type      = size_t;
  using multiarray_type = multiarray<2, value_type>;
  using dimensions_type = typename multiarray_type::dimensions_type;
  using corner_t        = skeletons::position;
  using corner_type     = std::array<corner_t, 2>;

  dimensions_type dims = dimensions_type(n1, n2);
  multiarray_type input(dims);
  multiarray_type output(dims);

  auto input_view = make_multiarray_view(input);
  auto output_view = make_multiarray_view(output);

  using namespace skeletons;
  using namespace wavefront_utils;

  auto f = position::first;
  auto l = position::last;

  test_wavefront(input_view, output_view, corner_type{{f, f}});
  test_wavefront(input_view, output_view, corner_type{{f, l}});
  test_wavefront(input_view, output_view, corner_type{{l, f}});
  test_wavefront(input_view, output_view, corner_type{{l, l}});
}

} // namespace


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2)
  {
    std::cout<< "usage: test_wavefront <nElemPerDim>" <<std::endl;
    exit(1);
  }

  size_t n = atol(argv[1]);

  test_wavefront(n, n);
  test_wavefront(n, n, n);

  return EXIT_SUCCESS;
}
