/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#include <iostream>
#include <vector>
#include <cstdlib>
#include <string>
#include <stapl/runtime.hpp>
#include <stapl/array.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/functor_view.hpp>
#include <stapl/utility/do_once.hpp>
//includes from skeletons
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/views/bitreversed_view.hpp>
#include <stapl/skeletons/functional/allreduce.hpp>
#include <stapl/skeletons/functional/broadcast.hpp>
#include <stapl/skeletons/functional/broadcast_to_locs.hpp>
#include <stapl/skeletons/functional/butterfly.hpp>
#include <stapl/skeletons/functional/copy.hpp>
#include <stapl/skeletons/functional/fft.hpp>
#include <stapl/skeletons/functional/inner_product.hpp>
#include <stapl/skeletons/functional/map.hpp>
#include <stapl/skeletons/functional/map_reduce.hpp>
#include <stapl/skeletons/functional/reduce.hpp>
#include <stapl/skeletons/functional/reduce_to_locs.hpp>
#include <stapl/skeletons/functional/scan.hpp>
#include <stapl/skeletons/functional/scan_reduce.hpp>
#include <stapl/skeletons/functional/sink.hpp>
#include <stapl/skeletons/functional/sink_value.hpp>
#include <stapl/skeletons/functional/zip.hpp>
#include <stapl/skeletons/operators/do_while.hpp>
#include <stapl/skeletons/operators/repeat.hpp>
#include <stapl/skeletons/operators/compose.hpp>
#include <stapl/skeletons/spans/per_location.hpp>

#ifdef GRAPHVIZ_OUTPUT
#include <stapl/skeletons/environments/graphviz_env.hpp>
#endif
#include "../expect.hpp"

STAPL_IS_BASIC_TYPE(std::complex<double>)

using namespace stapl;

namespace {

enum FFT_Direction{FORWARD_FFT, BACKWARD_FFT};

template <typename View>
void print_values(View& view, std::string title, std::ostream& o = std::cout)
{
#ifdef SHOW_RESULTS
  do_once([&view, &title, &o](void) {
    o << "\n" << title << "(size = " << view.size() << ")\n";
    std::copy(view.begin(), view.end(),
              std::ostream_iterator<typename View::value_type> (o, ", "));
    o << "\n";
  });
#endif
}

template <typename T>
void print_value(T& t, std::string title, std::ostream& o = std::cout)
{
#ifdef SHOW_RESULTS
  do_once([&t, &title, &o](void) {
    o << "\n" << title << ")\n";
    o << t << "\n";
  });
#endif
}

//////////////////////////////////////////////////////////////////////
/// @brief A generator for complex numbers which produces [0+0j, 1+0j,
/// ..., n-1+0j]
///
/// @tparam T the type of real part of the generated complex number
//////////////////////////////////////////////////////////////////////
template <typename T>
struct complex_seq
{
private:
  typedef std::complex<T> value_t;
  value_t m_init_v;
public:
  typedef std::size_t     index_type;
  typedef value_t         result_type;
  complex_seq(std::complex<T> init_v)
    : m_init_v(init_v)
  { }

  value_t operator()(index_type const& idx) const
  {
    return value_t(idx) + m_init_v;
  }

  void define_type(typer& t)
  {
    t.member(m_init_v);
  }
};


void test_fft(std::size_t n)
{
  using namespace skeletons;
  typedef double                           component_t;
  typedef std::complex<component_t>        value_t;
  typedef array<value_t>                   array_t;
  typedef array_view<array_t>              array_vt;
  typedef indexed_domain<size_t>           dom_t;
  typedef balanced_partition<dom_t>        partition_t;
  partition_t p_bal(dom_t(0, n-1), n);

  array_t  input_array(p_bal);
  array_t  output_array(p_bal);
  array_vt input_view(input_array);
  array_vt output_view(output_array);

  // create an incremental complex input (0 + 0j), (1 + 0j), ..., (n-1 + 0j)
  complex_seq<component_t> seq_gen(value_t(0, 0));

  skeletons::execute(skeletons::default_execution_params(),
                     skeletons::zip(stapl::assign<value_t>()),
                     functor_view(n, seq_gen), input_view);

  print_values(input_view, "Input");

  // FFT DIF (Decimation In Frequency)
  skeletons::execute(
    skeletons::default_execution_params(),
    sink<value_t>(
      butterfly<true>(skeletons_impl::fft_dif_wf<component_t>())),
    input_view, bitreversed_view(output_view, n));

  print_values(output_view, "FFT DIF Result using bitreversed_view");

  // FFT DIT (Decimation In Time)
  skeletons::execute(
    skeletons::default_execution_params(),
    sink<value_t>(
      reverse_butterfly<true>(
        skeletons_impl::fft_dit_wf<component_t>())),
    bitreversed_view(input_view, n), output_view);

  print_values(output_view, "FFT DIT Result using bitreversed_view");

  skeletons::execute(
    skeletons::default_execution_params(),
    sink<value_t>(skeletons::fft<component_t, tags::dit>()),
    input_view, output_view);

  print_values(output_view, "FFT skeleton DIT Result");

  skeletons::execute(
    skeletons::default_execution_params(),
    sink<value_t>(skeletons::fft<component_t,tags::dif>()),
    input_view, output_view);

  print_values(output_view, "FFT skeleton DIF Result");
}

template <typename T>
struct continuation_cond
{
  T m_threshold;

  template <typename Element>
  bool operator()(Element & element) const
  {
#ifdef SHOW_RESULTS
    if (get_location_id() == 0)
      std::cout << element << " < " << m_threshold << " " <<
        (element <  m_threshold) << std::endl;
#endif
    return element < m_threshold;
  }

  void define_type(typer& t)
  {
    t.member(m_threshold);
  }
};

template <typename F, typename... Args>
void run_test(F const& f, std::string const& title, Args&&... args) {
#ifdef GRAPHVIZ_OUTPUT
  skeletons::execute(skeletons::execution_params(
                       null_coarsener(), skeletons::graphviz_env(title)),
                     std::forward<Args>(args)...);
#else
  skeletons::execute(skeletons::default_execution_params(),
                     std::forward<Args>(args)...);
#endif

  stapl::do_once([&title, &f](void) {
    stapl::tests::expect(f()) << title;
  });
}

template <typename ResultType, typename F, typename... Args>
void run_single_output_test(F const& f, std::string const& title,
                            Args&&... args) {
#ifdef GRAPHVIZ_OUTPUT
  ResultType res = skeletons::execute(skeletons::execution_params<ResultType>(
                                        null_coarsener(),
                                        skeletons::graphviz_env(title)),
                                      std::forward<Args>(args)...);
#else
  ResultType res = skeletons::execute(skeletons::execution_params<ResultType>(),
                                      std::forward<Args>(args)...);
#endif

  stapl::do_once([&title, &f, &res]() {
    stapl::tests::expect(f(res)) << title;
  });
}

void test_all(std::size_t n) {
  using namespace skeletons;
  using value_t     = int;
  using container_t = array<value_t>;
  using view_t      = array_view<container_t>;
  using ref_t       = view_t::reference;

  container_t input_array(n);
  container_t output_array(n);
  view_t      in_view(input_array);
  view_t      out_view(output_array);

  container_t n_to_p_results(get_num_locations());
  view_t n_to_p_results_view(n_to_p_results);

  value_t do_while_thresh = 2*n + 1;

  // NOTICE:
  // Known fact. You might notice that when you call an in place algorithm
  // your algorithms works fine and correctly once and then the other time it
  // doesn't on more than one processor. This is due to the fact that each
  // processor will handle the tasks separately in STAPL. In fact, it might
  // happen that one processor starts reading input values later than another
  // processor writes its value back in the input.
  auto cv = counting_view<value_t> (n, value_t(0));

  stapl::plus<value_t>     plus_i;

  using skeletons::map;
  using skeletons::map_reduce;
  using skeletons::scan;
  using skeletons::reduce;


  // Testing copy
  auto test_title ="copy_" + std::to_string(n);
  auto copy_verif = [&in_view, &cv](void) {
    return std::equal(in_view.begin(), in_view.end(), cv.begin());
  };

  run_test(copy_verif, test_title,
           skeletons::copy<value_t>(),
           cv, in_view);

  // Testing do-while
  test_title = "do_while(map(+2),reduce_to_p(+),continueif(<" +
               std::to_string(do_while_thresh) + "))";
  auto do_while_verif =
      [&in_view, &out_view, n, &do_while_thresh](void) {
    std::vector<value_t> result(n);
    std::copy(in_view.begin(), in_view.end(), result.begin());
    value_t red_value;
    do {
      std::transform(result.begin(), result.end(), result.begin(),
                     boost::bind(std::plus<value_t>(), _1, 2));
      red_value = std::accumulate(result.begin(), result.end(), 0);
    } while (red_value < do_while_thresh);
    return std::equal(out_view.begin(), out_view.end(), result.begin());
  };

  run_test(do_while_verif, test_title,
           sink<value_t>(do_while(map(stapl::bind1st(plus_i, 2)),
                                  reduce_to_locs(plus_i),
                                  continuation_cond<value_t>{do_while_thresh})),
           in_view, out_view);

  // Testing reduce
  test_title = "reduce(+)_" + std::to_string(n);
  //there is no validation in this case since the value is reduced to the result
  //of a task, it is not propagated to other processors
  auto reduce_verif = [](){ return true; };

  run_test(reduce_verif, test_title,
           reduce(plus_i),
           in_view);

  // Testing allreduce
  test_title = "allreduce(+)_" + std::to_string(n);
  auto allreduce_verif = [&in_view, &out_view](void) {
    value_t red_value = std::accumulate(in_view.begin(), in_view.end(), 0);
    return find_if(out_view.begin(), out_view.end(),
                   [red_value](ref_t element) {
             return element != red_value;
           }) == out_view.end();
  };

  run_test(allreduce_verif, test_title + "(reduce-broadcast)",
           sink<value_t>(allreduce(plus_i)),
           in_view, out_view);
  run_test(allreduce_verif, test_title + "(butterfly)",
           sink<value_t>(allreduce<tags::butterfly<>>(plus_i)),
           in_view, out_view);
  run_test(allreduce_verif, test_title + "(reverse-butterfly)",
           sink<value_t>(allreduce<tags::reverse_butterfly<>>(plus_i)),
           in_view, out_view);
  run_test(allreduce_verif, test_title + "(reduce-broadcast-right-aligned)",
           sink<value_t>(allreduce<tags::right_aligned>(plus_i)),
           in_view, out_view);
  run_test(allreduce_verif, test_title + "(reduce-broadcast-left-aligned)",
           sink<value_t>(allreduce<tags::left_aligned>(plus_i)),
           in_view, out_view);

  // Testing repeat
  test_title = "repeat(allredcue(+))_" + std::to_string(n);
  auto repeat_verif =  [&in_view, &out_view, n](void) {
    value_t red_value = std::accumulate(in_view.begin(), in_view.end(), 0);
    red_value = red_value * n * n;
    return find_if(out_view.begin(), out_view.end(),
                            [red_value](ref_t element) {
                              return element != red_value;
                            }) == out_view.end();
  };

  run_test(repeat_verif, test_title,
           sink<value_t>(repeat(allreduce(plus_i), lazysize(3))),
           in_view, out_view);

  // Testing map
  test_title = "map(+2)_" + std::to_string(n);
  auto map_verif = [&in_view, &out_view](void) {
    return std::equal(out_view.begin(), out_view.end(), in_view.begin(),
                      [](ref_t out, ref_t in) { return out == (in + 2); });
  };

  run_test(map_verif, test_title,
           sink<value_t>(map(stapl::bind1st(plus_i, 2))),
           in_view, out_view);

  // Testing compose
  test_title = "compose(map(+2),map(+3))_" + std::to_string(n);
  auto compose_verif = [&in_view, &out_view](void) {
    return std::equal(out_view.begin(), out_view.end(), in_view.begin(),
                      [](ref_t out, ref_t in) { return out == (in + 5); });
  };

  run_test(compose_verif, test_title,
           sink<value_t>(
             compose(
               map(stapl::bind1st(plus_i, 2)),
               map(stapl::bind1st(plus_i, 3)))),
           in_view, out_view);


  // Testing map-reduce
  test_title = "map_reduce(+2,+)_" + std::to_string(n);
  auto map_reduce_verif = [&in_view, &n_to_p_results_view](void) {
    std::vector<value_t> result_test(n_to_p_results_view.size());
    value_t red_value = std::accumulate(
        in_view.begin(), in_view.end(), value_t(0),
        [](value_t in1, value_t in2) { return in1 + (in2 + 2); });
    return find_if(n_to_p_results_view.begin(), n_to_p_results_view.end(),
                   [red_value](ref_t element) {
             return element != red_value;
           }) == n_to_p_results_view.end();
  };

  run_test(map_reduce_verif, test_title,
           sink<value_t, spans::per_location>(
             compose(
               map_reduce(stapl::bind1st(plus_i, 2), plus_i),
               broadcast_to_locs())),
           in_view, n_to_p_results_view);

  // Testing scan
  test_title = "scan(+)_" + std::to_string(n);
  auto inclusive_scan_verif = [&in_view, &out_view, n](void) {
    std::vector<value_t> result_test(n);
    std::partial_sum(in_view.begin(), in_view.end(), result_test.begin());
    return std::equal(out_view.begin(), out_view.end(), result_test.begin());
  };
  auto exclusive_scan_verif = [&in_view, &out_view, n](void) {
    std::vector<value_t> result_test(n);
    std::partial_sum(in_view.begin(), in_view.end(), result_test.begin());
    return std::equal(++out_view.begin(), out_view.end(),
                      result_test.begin()) &&
           (*out_view.begin() == 0);
  };

  run_test(inclusive_scan_verif, test_title + "(Jaja)",
           sink<value_t>(scan(plus_i)),
           in_view, out_view);

  run_test(inclusive_scan_verif, test_title + "(Hillis-Steele)",
           sink<value_t>(scan<tags::hillis_steele>(plus_i)),
           in_view, out_view);

  run_test(exclusive_scan_verif, test_title + "(Blelloch)",
           sink<value_t>(scan<tags::blelloch>(plus_i, 0)),
           in_view, out_view);

  // Testing reduce_n_to_p(+)
  test_title = "reduce_n_to_p(+)_" + std::to_string(n);
  auto reduce_to_p_verif = [&in_view, &n_to_p_results_view](void) {
    value_t red_value = std::accumulate(in_view.begin(), in_view.end(), 0);
    return find_if(n_to_p_results_view.begin(), n_to_p_results_view.end(),
                   [red_value](ref_t element) {
             return element != red_value;
           }) == n_to_p_results_view.end();
  };

  run_test(reduce_to_p_verif, test_title,
           sink<value_t, spans::per_location>(reduce_to_locs(plus_i)),
           in_view, n_to_p_results_view);

  // Testing inner-product
  test_title = "inner_product_" + std::to_string(n);
  auto inner_product_verif = [&in_view](value_t const& res) {
    return res ==
           std::accumulate(
               in_view.begin(), in_view.end(), (value_t)0,
               [](value_t in1, value_t in2) { return in1 + in2 * in2; });
  };

  run_single_output_test<value_t>(inner_product_verif, test_title,
                                  compose(
                                    skeletons::inner_product<value_t>(),
                                    broadcast_to_locs<true>()),
                                  in_view, in_view);

  // Testing scan-reduce
  test_title = "scan_reduce_" + std::to_string(n);
  auto scan_reduce_verif =
    [&n](value_t const& res) { return 2 * res == (value_t)(n * (n - 1));};

  run_single_output_test<value_t>(scan_reduce_verif, test_title,
                                  scan_reduce<value_t>(plus_i),
                                  cv, out_view);
}

} // namespace

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2){
    std::cout << "<exec> <nElem>" << std::endl;
    exit(1);
  }
  size_t n = atol(argv[1]);
  test_all(n);
  test_fft(n);
  return EXIT_SUCCESS;
}
