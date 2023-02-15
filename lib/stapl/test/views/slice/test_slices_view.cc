/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/
#include <stapl/multiarray.hpp>
#include <stapl/views/slices_view.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include <boost/format.hpp>
#include <cmath>

#include "../../test_report.hpp"

#include "../../confint.hpp"
#include <stapl/runtime/counter/default_counters.hpp>

using namespace stapl;
using no_coarsening = skeletons::tags::no_coarsening;

//////////////////////////////////////////////////////////////////////
/// @brief Workfunction to increment a scalar by one.
//////////////////////////////////////////////////////////////////////
struct increment
{
  typedef void result_type;

  template<typename T>
  void operator()(T x)
  {
    x += 1;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Workfunction that receives a slice of a view and divides
///        each element of that slice by a fixed integer.
//////////////////////////////////////////////////////////////////////
struct normalize_slice
{
private:
  const int m_n;

public:
  normalize_slice(int n)
    : m_n(n)
  {}

  typedef void result_type;

  template<typename View>
  void operator()(View const& slice) const
  {
    auto const& dom = slice.domain();

    domain_map(dom, [&](decltype(dom.first()) const& i) {
      slice[i] /= m_n;
    });
  }

  void define_type(typer& t)
  {
    t.member(m_n);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Workfunction that receives a slice of a view and spawns
///        a nested PARAGRAPH to increment all elements of that slice.
/// @tparam Coarsened Flag indicating whether to coarsen the nested PARAGRAPH
/// @tparam Linearized Flag indicating whether to linearize the slice before
///         passing it to the nested PARAGRAPH
/// @todo A proper nested parallel section should be created here instead
///       of just invoking the inner map_func in the gang of size 1
///       (@seealso Kernel_3d_DGZ::LPlusTimes(void) in
///       @ref benchmarks/kripke/stapled_v2/Kripke/Kernel/Kernel_3d_DGZ.cpp).
//////////////////////////////////////////////////////////////////////
template<bool Coarsened, bool Linearized>
struct increment_slice;

template <>
struct increment_slice<true, false>
{
  typedef void result_type;

  template<typename View>
  void operator()(View&& slice)
  {
    map_func(increment(), slice);
  }
};

template <>
struct increment_slice<true, true>
{
  typedef void result_type;

  template<typename View>
  void operator()(View&& slice)
  {
    map_func(increment(), stapl::linear_view(slice));
  }
};

template <>
struct increment_slice<false, false>
{
  typedef void result_type;

  template<typename View>
  void operator()(View&& slice)
  {
    map_func<no_coarsening>(increment(), slice);
  }
};

template <>
struct increment_slice<false, true>
{
  typedef void result_type;

  template<typename View>
  void operator()(View&& slice)
  {
    map_func<no_coarsening>(increment(), stapl::linear_view(slice));
  }
};


template<int D, int NumSlices,
         typename Indices = make_index_sequence<D>>
class test_slicer;


//////////////////////////////////////////////////////////////////////
/// @brief Test the slices view by creating a D-dimensional multiarray
///        and a NumSlices-dimensional @ref slices_segmented_view over it.
///        An outer-level @p map_func is invoked to loop over the elements
///        of the slices view (instances of @ref SLICED_view), while an
///        inner-level @p map_func is used to increment all entries of each
///        @p SLICED_view. The test passes if all elements in the original
///        multiarray are equal to 1.
//
/// @tparam D The number of dimensions of the multiarray
/// @tparam NumSlices The number of dimensions of the slices view
//////////////////////////////////////////////////////////////////////
template<int D, int NumSlices, std::size_t... Indices>
class test_slicer<D, NumSlices, index_sequence<Indices...>>
{
  typedef int                                         value_type;
  typedef multiarray<D, value_type>                   multiarray_type;
  typedef multiarray_view<multiarray_type>            view_type;
  typedef typename multiarray_type::traversal_type    traversal_type;
  typedef typename multiarray_type::dimensions_type   dimensions_type;
  typedef make_index_sequence<NumSlices>              slices_index_sequence_t;

  /// Coarsening specification for the two levels of map_func nesting.
  enum class Coarsening
  {
    NONE,
    OUTER_ONLY,
    INNER_ONLY,
    OUTER_INNER
  };

  /// Number of repetitions to obtain timing statistics.
  int m_num_samples;

  /// Flag indicating whether the validation mode or the timing mode is active.
  ///
  /// In validation mode, all slicing tests are run once and linearization of
  /// the inner sliced view is also tested. In timing mode, all tests except
  /// the linearization ones are run @p m_num_samples times and timing
  /// statistics are reported.
  bool m_validation_mode;

  /// String stream into which to output timing results.
  std::stringstream m_perf_ss;

  /// Get the string describing the current coarsening setting.
  template<Coarsening coarsening>
  constexpr char const* make_desc(void) const
  {
    return (coarsening == Coarsening::INNER_ONLY) ?
      "inner PARAGRAPH coarsened" : (coarsening == Coarsening::OUTER_ONLY) ?
      "outer PARAGRAPH coarsened" : (coarsening == Coarsening::OUTER_INNER) ?
      "outer and inner PARAGRAPH coarsened" :
      "fine-grained";
  }

  /// Get the string describing the current setting of inner view linearization.
  template<bool linearized>
  constexpr char const* make_desc(void) const
  {
    return m_validation_mode ?
             linearized ?
               ", linearized SLICED_view" : ", n-dimensional SLICED_view"
             : "";
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a NumSlices-dimensional slices view over a D-dimensional
  ///        multiarray view, where the dimensions  0, 1, ..., NumSlices-1
  ///        are sliced off.
  //////////////////////////////////////////////////////////////////////
  template<typename View, std::size_t... SlicesIndices>
  static auto create_slices_view(View const& view,
                                 index_sequence<SlicesIndices...>)
    -> decltype(make_slices_view<SlicesIndices...>(view))
  {
    return make_slices_view<SlicesIndices...>(view);
  }

  typedef decltype(create_slices_view(
    std::declval<view_type>(), std::declval<slices_index_sequence_t>()
  )) slices_view_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Manually iterate over the domain of the slices view. Manually
  ///        extract the subview for each element, and then iterate over
  ///        the subview. For each inner-most element, increment it by 1.
  ///
  /// @param View The slices view to test
  //////////////////////////////////////////////////////////////////////
  template<typename View>
  void iterate_domain(View const& view)
  {
    confidence_interval_controller controller(m_num_samples, m_num_samples);
    counter<default_timer> timer;

    auto const& dom = view.domain();

    while (controller.iterate())
    {
      timer.reset();
      timer.start();

      stapl::do_once([&](){
        domain_map(dom, [&](decltype(dom.first()) const& idx) {
          auto const& subview = view[idx];
          auto const& subdom = subview.domain();

          domain_map(subdom, [&](decltype(subdom.first()) const& i) {
            subview[i] += 1;
          });
        });
      });

      timer.stop();
      controller.push_back(timer.value());
    }

    if (not m_validation_mode)
    {
      controller.report(m_perf_ss);

      stapl::do_once([&](){
        domain_map(dom, [&](decltype(dom.first()) const& idx) {
          auto const& subview = view[idx];
          auto const& subdom = subview.domain();

          domain_map(subdom, [&](decltype(subdom.first()) const& i) {
            subview[i] /= m_num_samples;
          });
        });
      });
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke a PARAGRAPH over the slices view, where
  ///        the work function spawns a nested map_func over the inner
  ///        view that increments all values by 1.
  ///
  /// @tparam linearized Flag indicating whether to linearize the inner sliced
  ///         view before passing it to the inner PARAGRAPH
  /// @param view The slices view to test
  /// @param coarsening Tag specifying the coarsening to be used
  //////////////////////////////////////////////////////////////////////
  template<bool linearized, typename View>
  void invoke_paragraph(View const& view, Coarsening coarsening)
  {
    confidence_interval_controller controller(m_num_samples, m_num_samples);
    counter<default_timer> timer;

    switch(coarsening)
    {
      case Coarsening::OUTER_ONLY:
        while (controller.iterate())
        {
          timer.reset();
          timer.start();
          map_func(increment_slice<false, linearized>(), view);
          timer.stop();
          controller.push_back(timer.value());
        }
        break;
      case Coarsening::INNER_ONLY:
        while (controller.iterate())
        {
          timer.reset();
          timer.start();
          map_func<no_coarsening>(increment_slice<true, linearized>(), view);
          timer.stop();
          controller.push_back(timer.value());
        }
        break;
      case Coarsening::OUTER_INNER:
        while (controller.iterate())
        {
          timer.reset();
          timer.start();
          map_func(increment_slice<true, linearized>(), view);
          timer.stop();
          controller.push_back(timer.value());
        }
        break;
      case Coarsening::NONE:
      default:
        while (controller.iterate())
        {
          timer.reset();
          timer.start();
          map_func<no_coarsening>(increment_slice<false, linearized>(), view);
          timer.stop();
          controller.push_back(timer.value());
        }
    }

    if (not m_validation_mode)
    {
      controller.report(m_perf_ss);

      // Divide each element of the view by the number of repetitions performed
      // for time measuring purposes, so that the expected value of each element
      // is one.
      map_func(normalize_slice(m_num_samples), view);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a D-dimensional multiarray and NumSlices-dimensional
  ///        slices view. With the slices view, attempt to increment
  ///        all values by 1 either through a map_func or manual iteration
  ///        and test that all of the values are correctly set.
  ///
  /// @tparam Sizes A D-dimensional tuple of sizes for the multiarray
  /// @tparam F A function that encapsulates incrementing the values
  /// @param lin_desc String representation of the linearization flag
  /// @param coarsening_desc String representation of the coarsening setting
  /// @return Boolean indicating whether the test passed or failed
  //////////////////////////////////////////////////////////////////////
  template<typename Sizes, typename F>
  bool create_and_test(char const* lin_desc,
    char const* coarsening_desc, Sizes const& sizes, F&& f)
  {
    dimensions_type s = dimensions_type(get<Indices>(sizes)...);
    multiarray_type c(s, 0);

    view_type v(c);

    auto sv = create_slices_view(v, slices_index_sequence_t());

    m_perf_ss.str("");

    f(sv);

    const bool passed =
      stapl::all_of(linear_view(v), boost::bind(stapl::equal_to<int>(), _1, 1));

    std::stringstream sz;
    print_tuple(sz, sizes);

    const std::string msg =
      str(boost::format("%dd dim %s: slicing off %d dimensions (%s%s)")
        % D % sz.str().c_str() % NumSlices % coarsening_desc % lin_desc);

    STAPL_TEST_REPORT(passed, msg);

    if (passed and not m_validation_mode)
      stapl::do_once([&] {
        std::cout << m_perf_ss.str() << std::endl;
      });

    return passed;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Test the map_func over a slices view that invokes an inner
  ///        map_func over the sliced views, as well as over the linearized
  ///        sliced views.
  ///
  /// @param Sizes A D-dimensional tuple of sizes for the multiarray that is
  ///        going to be sliced
  /// @tparam linearized Flag indicating whether to linearize the sliced
  ///         views before passing them to the inner PARAGRAPH
  /// @return Boolean indicating whether the test passed or failed
  //////////////////////////////////////////////////////////////////////
  template<bool linearized, typename Sizes>
  bool create_and_test_paragraph(Sizes const& sizes)
  {
    auto lin_desc = make_desc<linearized>();

    // create a slices view and invoke a non-coarsened PARAGRAPH on it;
    // coarsening disabled for the nested PARAGRAPH over the sliced views
    bool b1 = create_and_test(lin_desc, make_desc<Coarsening::NONE>(),
      sizes,
      [this](slices_view_type const& view) {
        invoke_paragraph<linearized>(view, Coarsening::NONE);
      });

    // create a slices view and invoke a non-coarsened PARAGRAPH on it;
    // coarsening enabled for the nested PARAGRAPH over the sliced views
    bool b2 = create_and_test(lin_desc, make_desc<Coarsening::INNER_ONLY>(),
      sizes,
      [this](slices_view_type const& view) {
        invoke_paragraph<linearized>(view, Coarsening::INNER_ONLY);
      });

    // create a slices view and invoke a coarsened PARAGRAPH on it;
    // coarsening disabled for the nested PARAGRAPH over the sliced views
    bool b3 = create_and_test(lin_desc, make_desc<Coarsening::OUTER_ONLY>(),
      sizes,
      [this](slices_view_type const& view) {
        invoke_paragraph<linearized>(view, Coarsening::OUTER_ONLY);
    });

    // create a slices view and invoke a coarsened PARAGRAPH on it;
    // coarsening enabled for the nested PARAGRAPH over the sliced views
    bool b4 = create_and_test(lin_desc, make_desc<Coarsening::OUTER_INNER>(),
      sizes,
      [this](slices_view_type const& view) {
        invoke_paragraph<linearized>(view, Coarsening::OUTER_INNER);
    });

    return b1 && b2 && b3 && b4;
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an object managing the execution of slicing tests.
  ///        If in the validation mode (<tt>num_samples == 1</tt>), testing
  ///        of linearization of the inner sliced views will also be enabled.
  ///
  /// @param num_samples Number of repetitions to obtain timing statistics.
  //////////////////////////////////////////////////////////////////////
  test_slicer(int num_samples)
    : m_num_samples(num_samples), m_validation_mode(num_samples == 1)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Test the slices view for map_func and manual iteration of
  ///        its domain, and the domains of its subviews.
  ///
  /// @param Sizes A D-dimensional tuple of sizes for the multiarray
  /// @return Boolean indicating whether the test passed or failed
  //////////////////////////////////////////////////////////////////////
  template<typename Sizes>
  bool call(Sizes const& sizes)
  {
    // create a slices view and manually iterate over its domain
    bool b0 = create_and_test("manual iteration", "", sizes,
      [this](slices_view_type const& view) {
        iterate_domain(view);
      });

    // create a slices view and invoke a PARAGRAPH on it
    bool b1 = create_and_test_paragraph<false>(sizes);

    // create a slices view and invoke a PARAGRAPH on it; linearize the sliced
    // view before passing it to the inner PARAGRAPH if in the validation mode
    bool b2 = m_validation_mode ? create_and_test_paragraph<true>(sizes) : true;

    return b0 && b1 && b2;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief For a given dimension D, call the main test routine for each
///        possible value of the number of slices.
///
///        For example, for the 4D case, call the tester with slicing
///        1 dimension, 2 dimensions and 3 dimensions.
//////////////////////////////////////////////////////////////////////
template<int D, int NumSlices = 1>
struct test_all_slices_for_dimension
{
  template<typename Sizes>
  static bool call(Sizes const& sizes, int num_samples)
  {
    return test_slicer<D, NumSlices>(num_samples).call(sizes) &&
      test_all_slices_for_dimension<D, NumSlices+1>::call(sizes, num_samples);
  }
};

template<int D>
struct test_all_slices_for_dimension<D, D>
{
  template<typename Sizes>
  static bool call(Sizes const&, int)
  {
    return true;
  }
};

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 4)
  {
    stapl::do_once([]{
      std::cout << "usage: <exec_cmd> "
                   "test_slices_view n2d n3d n4d [nrep=1] [weak=0]\n";
    });

    exit(1);
  }

  // number of repetitions (for performance measuring purposes)
  const int rep = (argc < 5) ? 1 : atoi(argv[4]);

  // number of processors (for weak scaling purposes)
  const auto weak_nproc = (argc < 6) ?
                            1ul : (argv[5] == 0) ?
                                    1ul : get_num_locations();

  // determine the multiplier of the size along a single dimension so that
  // for the weak scaling test, the total volume of the multidimensional
  // input is (approximately) proportional to the number of processors
  const double mult2 = weak_nproc >= 8 ? std::pow(weak_nproc, 1./2.) : 1.;
  const double mult3 = weak_nproc >= 8 ? std::pow(weak_nproc, 1./3.) : 1.;
  const double mult4 = weak_nproc >= 8 ? std::pow(weak_nproc, 1./4.) : 1.;

  // number of elements in the input multiarray along one dimension
  const std::size_t n2d = std::round( mult2 * atol(argv[1]) );
  const std::size_t n3d = std::round( mult3 * atol(argv[2]) );
  const std::size_t n4d = std::round( mult4 * atol(argv[3]) );

  // size of the input multiarray: if collecting performance results,
  // equal number of elements along all dimensions is being used; if
  // validating, non-equal sizes are used along each dimension
  const auto size2 = (rep > 1) ?
    make_tuple(n2d, n2d) : make_tuple(n2d, n2d+1);
  const auto size3 = (rep > 1) ?
    make_tuple(n3d, n3d, n3d) : make_tuple(n3d, n3d+1, n3d+2);
  const auto size4 = (rep > 1) ?
    make_tuple(n4d, n4d, n4d, n4d) : make_tuple(n4d, n4d+1, n4d+2, n4d+3);

  // run the tests for 2,3,4 dimensional input (5 dimensions already blow
  // up compiler memory usage)
  test_all_slices_for_dimension<2>::call(size2, rep);
  test_all_slices_for_dimension<3>::call(size3, rep);
  test_all_slices_for_dimension<4>::call(size4, rep);

  return EXIT_SUCCESS;
}
