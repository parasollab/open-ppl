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
#include <algorithm>
#include <iterator>
#include <stapl/runtime.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/sorting.hpp>

//includes from skeletons
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/skeletons/functional/sink.hpp>
#include <stapl/skeletons/environments/graphviz_env.hpp>
#include <stapl/skeletons/param_deps/bitonic_sort_pd.hpp>
#include <stapl/skeletons/param_deps/odd_even_merge_sort_pd.hpp>
#include <stapl/skeletons/utility/lazy_sizes.hpp>
#include "../test_report.hpp"

using namespace stapl;

namespace {

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

//////////////////////////////////////////////////////////////////////
/// @brief Comparison workfunction used in bitonic sorting
///
/// @tparam T          the type of input elements to be sorted
//////////////////////////////////////////////////////////////////////
template <typename T>
struct bs_wf
{
  typedef T                  result_type;

private:
  std::size_t     m_row;
  std::size_t     m_col;
  std::size_t     m_sb;

public:
  bs_wf() :
    m_row(0),
    m_col(0),
    m_sb(1)
  { }

  void set_position(std::size_t row,
                    std::size_t col,
                    std::size_t sb)
  {
    m_row = row;
    m_col = col;
    m_sb = sb;
  }

  template <typename Element>
  T operator()(Element el1, Element el2) const
  {
    //is sb-th bit in col the same as row-th bit in col
    if ((((1 << m_sb) & m_col) && 1) ^ (((1<<m_row) & m_col) &&1)) {
      return std::max(el1,el2);
    }
    else {
      return std::min(el1,el2);
    }
  }

  void define_type(typer &t)
  {
    t.member(m_row);
    t.member(m_col);
    t.member(m_sb);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Comparison workfunction used in oddeven merge sorting
///
/// @tparam T                   the type of input elements to be sorted
/// @var m_is_max_point  the elementary dependency pattern is based on
///                             this property in its formation. As the
///                             workfunction also utilizes it, we will retrieve
///                             it from the pattern instead of recomputing it
///                             here.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct odd_even_merge_sort_wf
{
  typedef T                  result_type;

private:
  bool m_is_max_point;

public:
  odd_even_merge_sort_wf() :
    m_is_max_point(true)
  { }

  void set_point_property(bool point_info)
  {
    m_is_max_point = point_info;
  }

  template <typename Element>
  T operator()(Element el1, Element el2) const
  {
    if (m_is_max_point) {
      return std::max(el1,el2);
    }
    else {
      return std::min(el1,el2);
    }
  }

  void define_type(typer &t)
  {
    t.member(m_is_max_point);
  }
};

template <typename F, typename... Args>
void run_test(F const& f, std::string const& title, Args&&... args) {
#ifdef GRAPHVIZ_OUTPUT
  skeletons::execute(
      skeletons::execution_params(use_default(), graphviz_env(title)),
      std::forward<Args>(args)...);
#else
  skeletons::execute(skeletons::default_execution_params(),
                     std::forward<Args>(args)...);
#endif
  bool is_valid = f();
  stapl::do_once([&title, is_valid](void) {
    STAPL_TEST_REPORT(is_valid, title);
  });
}


//////////////////////////////////////////////////////////////////////
/// @brief Tests sorting algorithms written using skeletons
/// for a given input of size @c n
///
/// @param n problem size
//////////////////////////////////////////////////////////////////////
void test_sorting(std::size_t n)
{
  using namespace skeletons;
  typedef std::size_t                      value_t;
  typedef array<value_t>                   array_t;
  typedef array_view<array_t>              array_vt;

  array_t  input_array(n);
  array_t  output_array(n);
  array_vt input_view(input_array);
  array_vt output_view(output_array);


  //populate input view
  stapl::generate(input_view, stapl::random_sequence(10000));

  // define global bitonic sort pattern
  auto bitonic_sort_sk =
         skeletons::repeat(
           skeletons::elem(skeletons::bitonic_sort_pd(bs_wf<value_t>())),
           lazy_divide_and_conquer_size()
         );

  // verification lambda
  auto sort_verif = [&input_view, &output_view](){
    stapl::sort(input_view);
    return stapl::equal(input_view, output_view);
  };

  // Test bitonic sort
  run_test(sort_verif, "bitonic_" + std::to_string(n),
           skeletons::sink<value_t>(bitonic_sort_sk),
           input_view, output_view);

  print_values(input_view, "Input", std::cout);
  print_values(output_view, "Result", std::cout);

  //shuffling the input
  stapl::random_shuffle(input_view);

  // Test odd-even merge sort
  auto odd_even_merge_sort_sk =
         skeletons::repeat(
           skeletons::elem(skeletons::odd_even_merge_sort_pd(
             odd_even_merge_sort_wf<value_t>())),
           lazy_divide_and_conquer_size()
         );

  run_test(sort_verif, "oddevenmerge_" + std::to_string(n),
           skeletons::sink<value_t>(odd_even_merge_sort_sk),
           input_view, output_view);
}

bool isPowerOfTwo (unsigned int x)
{
  while (((x % 2) == 0) && x > 1)
    x /= 2;
  return (x == 1);
}

} // namespace

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2){
    std::cout << "<exec> <nElems>" << std::endl;
    return EXIT_FAILURE;
  }
  size_t n = atol(argv[1]);

  if (isPowerOfTwo(n)) {
    test_sorting(n);
    return EXIT_SUCCESS;
  }
  else {
    if (get_location_id() == 0) {
      std::cout << "Bitonic Sort can only accept inputs of power-of-two sizes"
                << std::endl;
    }
    return EXIT_FAILURE;
  }
}
