/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#include <cstdlib>
#include <boost/program_options.hpp>
#include <stapl/runtime.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/array.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/skeletons/operators/compose.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/skeletons/functional/scan.hpp>
#include <stapl/skeletons/functional/sink.hpp>
#include <stapl/skeletons/functional/zip.hpp>
#include <stapl/skeletons/map_reduce.hpp>
#include <stapl/skeletons/transformations/coarse/butterfly.hpp>
#include <stapl/skeletons/transformations/coarse/compose.hpp>
#include <stapl/skeletons/transformations/coarse/reduce.hpp>
#include <stapl/skeletons/transformations/coarse/scan.hpp>
#include <stapl/skeletons/transformations/coarse/zip.hpp>
#include <stapl/skeletons/utility/coarse_identity_op.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/views/metadata/coarseners/null.hpp>
#include <stapl/views/metadata/coarseners/default.hpp>
#include <stapl/views/metadata/coarseners/multiview.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include "../confint.hpp"

template <typename ExecParams, typename S, typename... V>
void
time_me(ExecParams&& exec_params, std::string const& title, S&& s, V&&... v)
{
  using std::accumulate;
  using std::cout;
  using std::endl;
  using std::inner_product;
  using std::sqrt;
  using std::transform;
  using std::vector;
  using stapl::counter;
  using stapl::default_timer;
  using stapl::do_once;
  using stapl::skeletons::execution_params;

  vector<double> elapseds;
  counter<default_timer> t;
  for (std::size_t i = 0; i < 32; ++i) {
    t.reset();
    t.start();
    stapl::skeletons::execute(exec_params, std::forward<S>(s),
      std::forward<V>(v)...);

    elapseds.push_back(t.stop());
  }

  do_once([&title, &elapseds]() {
    auto&& stats = compute_stats(elapseds);
    cout << "dbx.obs {"
         << R"("title" : ")" << title << "\","
         << R"("mean" : )" << stats.avg << ","
         << R"("stdev" : )" << stats.stddev << ","
         << R"("conf_interval" : )" << stats.conf_interval << ","
         << R"("num_samples" : )" << elapseds.size()
         << "}\n";
  });
}

template <typename T>
auto unary_equal_to(T value)
  -> decltype(std::bind(stapl::equal_to<T>(), value, std::placeholders::_1))
{
  return std::bind(stapl::equal_to<T>(), value, std::placeholders::_1);
}

template <typename S>
auto coarsen_skeleton(S&& s, std::true_type)
  -> decltype(stapl::skeletons::coarse(s))
{
  return stapl::skeletons::coarse(s);
}

template <typename T, typename S>
auto add_identity(S&& s, std::true_type)
  -> decltype(
       stapl::skeletons::compose(
         stapl::skeletons::map(stapl::identity<T>()), s))
{
  using stapl::skeletons::compose;
  using stapl::skeletons::map;
  using stapl::identity;
  return compose(map(identity<T>()), s);
}

template <typename T, typename S>
S add_identity(S&& s, std::false_type)
{
  return s;
}

template <typename S>
S coarsen_skeleton(S&& s, std::false_type)
{
  return s;
}

template <typename T>
struct some_op
{
  template <typename... V>
  T operator()(V&&... v) const
  {
    std::initializer_list<T> vs = {v...};
    return std::accumulate(vs.begin(), vs.end(), T{0});
  }
};

void verify(bool is_correct, std::string const& text)
{

  stapl::do_once([&] {
    if (!is_correct){
        std::cout << text << " failed to produce the correct result"
                  << std::endl;
    }
  });
}

template <typename ValueType, bool B>
void test_zip(std::size_t n, std::integral_constant<bool, B> b)
{
  using stapl::all_of;
  using stapl::array;
  using stapl::default_coarsener;
  using stapl::make_array_view;
  using stapl::plus;
  using stapl::skeletons::sink;
  using stapl::skeletons::execution_params;
  using stapl::null_coarsener;

  using coarsener =
    typename std::conditional<B, default_coarsener, null_coarsener>::type;
  auto ep = execution_params(coarsener());
  using stapl::skeletons::zip;

  stapl::array<ValueType> in(n, ValueType{1});
  stapl::array<ValueType> out(n);
  auto in_view = make_array_view(in);
  auto out_view = make_array_view(out);

  std::string prefix = B ? "coarse-" : "";
  std::string title = prefix + "zip<1>";
  time_me(ep, title,
          coarsen_skeleton(sink<ValueType>(zip<1>(some_op<ValueType>())), b),
          in_view, out_view);
  verify(all_of(out_view, unary_equal_to<ValueType>(1)),title);

  title = prefix + "zip<2>";
  time_me(ep, title,
          coarsen_skeleton(sink<ValueType>(zip<2>(some_op<ValueType>())), b),
          in_view, in_view, out_view);
  verify(all_of(out_view, unary_equal_to<ValueType>(2)),title);


  title = prefix + "zip<3>";
  time_me(ep, title,
          coarsen_skeleton(sink<ValueType>(zip<3>(some_op<ValueType>())), b),
          in_view, in_view, in_view, out_view);
  verify(all_of(out_view, unary_equal_to<ValueType>(3)),title);

  title = prefix + "zip<4>";
  time_me(ep, title,
          coarsen_skeleton(sink<ValueType>(zip<4>(some_op<ValueType>())), b),
          in_view, in_view, in_view, in_view, out_view);
  verify(all_of(out_view, unary_equal_to<ValueType>(4)),title);

  title = prefix + "zip<5>";
  time_me(ep, title,
          coarsen_skeleton(sink<ValueType>(zip<5>(some_op<ValueType>())), b),
          in_view, in_view, in_view, in_view, in_view, out_view);
  verify(all_of(out_view, unary_equal_to<ValueType>(5)),title);
}

/// Composes a given skeleton with itself for N times
template <typename Skeleton, std::size_t... Index>
auto compose_n_times(Skeleton&& s, stapl::index_sequence<Index...>&&)
  -> decltype(
      stapl::skeletons::compose(stapl::get<Index-Index>(
        stapl::make_tuple(s))...)
    )
{
  return stapl::skeletons::compose(stapl::get<Index-Index>(
           stapl::make_tuple(s))...);
}

/// Composes a given skeleton with itself for N times
template <std::size_t N, typename Skeleton>
auto compose_n_times(Skeleton&& s)
  -> decltype(compose_n_times(std::forward<Skeleton>(s),
                              stapl::make_index_sequence<N>()))
{
  return compose_n_times(std::forward<Skeleton>(s),
                         stapl::make_index_sequence<N>());
}

template <typename ValueType, bool B>
void test_compose(std::size_t n, std::integral_constant<bool, B> b)
{
  using stapl::all_of;
  using stapl::array;
  using stapl::default_coarsener;
  using stapl::make_array_view;
  using stapl::plus;
  using stapl::skeletons::compose;
  using stapl::skeletons::map;
  using stapl::skeletons::sink;
  using stapl::skeletons::execution_params;
  using stapl::null_coarsener;

  using coarsener =
    typename std::conditional<B, default_coarsener, null_coarsener>::type;
  auto ep = execution_params(coarsener());
  using stapl::skeletons::zip;

  stapl::array<ValueType> in(n, ValueType{1});
  stapl::array<ValueType> out(n);
  auto in_view = make_array_view(in);
  auto out_view = make_array_view(out);

  std::string prefix = B ? "coarse-" : "";

  auto skeleton = map(some_op<ValueType>());
  auto compose2  = compose_n_times<2>(skeleton);
  auto compose3  = compose_n_times<3>(skeleton);
  auto compose4  = compose_n_times<4>(skeleton);
  auto compose5  = compose_n_times<5>(skeleton);
  auto compose6  = compose_n_times<6>(skeleton);
  auto compose7  = compose_n_times<7>(skeleton);
  auto compose8  = compose_n_times<8>(skeleton);
  auto compose9  = compose_n_times<9>(skeleton);
  auto compose10 = compose_n_times<10>(skeleton);

  std::string title = prefix + "map";
  time_me(ep, title, coarsen_skeleton(sink<ValueType>(skeleton), b),
          in_view, out_view);
  verify(all_of(out_view, unary_equal_to<ValueType>(1)),title);

  title = prefix + "compose(map...) [2 times]";
  time_me(ep, title,
          coarsen_skeleton(sink<ValueType>(compose2), b),
          in_view, out_view);
  verify(all_of(out_view, unary_equal_to<ValueType>(1)),title);

  title = prefix + "compose(map...) [3 times]";
  time_me(ep, title,
          coarsen_skeleton(sink<ValueType>(compose3), b),
          in_view, out_view);
  verify(all_of(out_view, unary_equal_to<ValueType>(1)),title);

  title = prefix + "compose(map...) [4 times]";
  time_me(ep, title,
          coarsen_skeleton(sink<ValueType>(compose4), b),
          in_view, out_view);
  verify(all_of(out_view, unary_equal_to<ValueType>(1)),title);

  title = prefix + "compose(map...) [5 times]";
  time_me(ep, title,
          coarsen_skeleton(sink<ValueType>(compose5), b),
          in_view, out_view);
  verify(all_of(out_view, unary_equal_to<ValueType>(1)),title);

  title = prefix + "compose(map...) [6 times]";
  time_me(ep, title,
          coarsen_skeleton(sink<ValueType>(compose6), b),
          in_view, out_view);
  verify(all_of(out_view, unary_equal_to<ValueType>(1)),title);

  title = prefix + "compose(map...) [7 times]";
  time_me(ep, title,
          coarsen_skeleton(sink<ValueType>(compose7), b),
          in_view, out_view);
  verify(all_of(out_view, unary_equal_to<ValueType>(1)),title);

  title = prefix + "compose(map...) [8 times]";
  time_me(ep, title,
          coarsen_skeleton(sink<ValueType>(compose8), b),
          in_view, out_view);
  verify(all_of(out_view, unary_equal_to<ValueType>(1)),title);

  title = prefix + "compose(map...) [9 times]";
  time_me(ep, title,
          coarsen_skeleton(sink<ValueType>(compose9), b),
          in_view, out_view);
  verify(all_of(out_view, unary_equal_to<ValueType>(1)),title);

  title = prefix + "compose(map...) [10 times]";
  time_me(ep, title,
          coarsen_skeleton(sink<ValueType>(compose10), b),
          in_view, out_view);
  verify(all_of(out_view, unary_equal_to<ValueType>(1)),title);
}

template <typename ValueType, bool B>
void test_butterfly(std::size_t n, std::integral_constant<bool, B> b)
{
  using stapl::all_of;
  using stapl::array;
  using stapl::default_coarsener;
  using stapl::make_array_view;
  using stapl::plus;
  using stapl::skeletons::butterfly;
  using stapl::skeletons::reverse_butterfly;
  using stapl::skeletons::sink;
  using stapl::skeletons::execution_params;
  using stapl::null_coarsener;

  using coarsener =
    typename std::conditional<B, default_coarsener, null_coarsener>::type;
  auto ep = execution_params(coarsener());
  using stapl::skeletons::zip;


  stapl::array<ValueType> in(n, ValueType{1});
  stapl::array<ValueType> out(n);
  auto in_view = make_array_view(in);
  auto out_view = make_array_view(out);

  ValueType sum = stapl::accumulate(in_view, ValueType{0});

  std::string prefix = B ? "coarse-" : "";

  std::string title = prefix + "butterfly";
  time_me(ep, title,
          coarsen_skeleton(
            sink<ValueType>(
              add_identity<ValueType>(butterfly(plus<ValueType>()), b)),
            b),
          in_view, out_view);
  verify(all_of(out_view, unary_equal_to<ValueType>(sum)), title);

  title = prefix + "reverse-butterfly";
  time_me(ep, title,
          coarsen_skeleton(
            sink<ValueType>(reverse_butterfly(plus<ValueType>())), b),
          in_view, out_view);
  verify(all_of(out_view, unary_equal_to<ValueType>(sum)), title);
}

template <typename ValueType>
void test_scan(std::size_t n, std::false_type)
{
  using stapl::skeletons::default_execution_params;
  using stapl::skeletons::zip;
  using stapl::skeletons::sink;
  using stapl::skeletons::scan;
  using stapl::array;
  using stapl::plus;
  using stapl::make_array_view;

  auto ep = default_execution_params();

  array<ValueType> in(n, ValueType{1});
  array<ValueType> out(n);
  auto in_view  = make_array_view(in);
  auto out_view = make_array_view(out);
  auto plus_i   = plus<ValueType>();

  ValueType sum = stapl::accumulate(in_view, ValueType{0});

  std::string title = "scan<binomial>";
  time_me(ep, title,
          sink<ValueType>(scan(plus_i)),
          in_view, out_view);
  verify(out_view[n - 1] == sum, title);

  title = "scan<hillis-steele>";
  time_me(ep, title,
          sink<ValueType>(
            scan<stapl::skeletons::tags::hillis_steele>(plus_i)),
          in_view, out_view);
  verify(out_view[n - 1] == sum, title);

  title = "scan<blelloch>";
  time_me(ep, title,
          sink<ValueType>(scan<stapl::skeletons::tags::blelloch>(plus_i, 0)),
          in_view, out_view);
  verify(out_view[n - 1] == (sum - in_view[n - 1]), title);
}

template <typename ValueType>
void test_scan(std::size_t n, std::true_type)
{
  using stapl::skeletons::execution_params;
  using stapl::skeletons::scan;
  using stapl::skeletons::sink;
  using stapl::skeletons::zip;
  using stapl::skeletons::coarse;
  using stapl::array;
  using stapl::default_coarsener;
  using stapl::plus;
  using stapl::make_array_view;

  auto ep = execution_params(default_coarsener());

  array<ValueType> in(n, ValueType{1});
  array<ValueType> out(n);
  auto in_view  = make_array_view(in);
  auto out_view = make_array_view(out);
  auto plus_i   = plus<ValueType>();

  ValueType sum = stapl::accumulate(in_view, ValueType{0});

  std::string title = "coarse-scan<binomial>";
  time_me(ep, title,
          coarse(scan(plus_i)),
          in_view, out_view);
  verify(out_view[n - 1] == sum, title);

  title = "coarse-scan<hillis-steele>";
  time_me(ep, title,
          coarse(
            scan<stapl::skeletons::tags::hillis_steele>(plus_i)),
          in_view, out_view);
  verify(out_view[n - 1] == sum, title);

  title = "coarse-scan<blelloch>";
  time_me(ep, title,
          coarse(
            scan<stapl::skeletons::tags::blelloch>(plus_i, 0)),
          in_view, out_view);
  verify(out_view[n - 1] == (sum - in_view[n - 1]), title);
}

template <typename ValueType, typename Sizes, bool B>
void run_tests(Sizes&& sizes, std::integral_constant<bool, B> b)
{
  std::string coarse_prefix_str = "coarse-";
  auto size =
    sizes[(B ? coarse_prefix_str : "") + "size"]
      .template as<std::size_t>();
  auto butterfly_size =
    sizes[(B ? coarse_prefix_str : "") + "butterfly-size"]
      .template as<std::size_t>();

  test_zip<ValueType>(size, b);
  test_compose<ValueType>(size, b);
  test_scan<ValueType>(size, b);
  test_butterfly<ValueType>(butterfly_size, b);
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  // The supported options.
  namespace options = boost::program_options;
  options::options_description desc("Allowed options");
  desc.add_options()("help", "produce help message")
     ("size",
      options::value<std::size_t>()->required(),
      "Regular input size for coarse-grain tests")
     ("coarse-size",
      options::value<std::size_t>()->required(),
      "Regular input size for fine-grain-tests")
     ("butterfly-size",
      options::value<std::size_t>()->required(),
      "Input size for fine-grain butterfly tests")
     ("coarse-butterfly-size",
      options::value<std::size_t>()->required(),
      "Input size for coarse-grain butterfly tests");

  options::variables_map vm;
  options::store(options::parse_command_line(argc, argv, desc), vm);

  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return EXIT_SUCCESS;
  }

  options::notify(vm);

  using value_t = int;

  run_tests<value_t>(vm, std::true_type());
  run_tests<value_t>(vm, std::false_type());

  return EXIT_SUCCESS;
}
