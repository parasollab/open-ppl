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
#include <random>
#include <stapl/runtime.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/functor_view.hpp>
#include <stapl/utility/do_once.hpp>

//includes from skeletons
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/lightweight_vector.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/skeletons/functional/alltoall.hpp>
#include <stapl/skeletons/functional/allgather.hpp>
#include <stapl/skeletons/functional/scatter.hpp>
#include <stapl/skeletons/functional/gather.hpp>
#include <stapl/skeletons/functional/map.hpp>

#include <stapl/skeletons/environments/graphviz_env.hpp>

#include "../expect.hpp"

using namespace stapl;

namespace {

//////////////////////////////////////////////////////////////////////
/// @brief Generates vector of vectors populated with random values.
///
/// The generated outer vector will be of the given size @c outer_elem_count.
/// The inner vector generated will have a max size bounded by
/// @c max_inner_elem_count. Finally, each element in the inner vector
/// will have a random value in the range [0..max_value).
///
/// @tparam T type of the elements in the inner vector
//////////////////////////////////////////////////////////////////////
template <typename T>
struct generate_vector_data
{
  std::size_t m_max_value;
  std::size_t m_outer_elem_count;
  std::size_t m_max_elem_per_loc;

  typedef std::size_t            index_type;
  typedef typename T::value_type fine_value_t;
  typedef std::vector<T>         result_type;

  generate_vector_data(std::size_t max_value,
                       std::size_t outer_elem_count,
                       std::size_t max_elem_per_loc)
    : m_max_value(max_value),
      m_outer_elem_count(outer_elem_count),
      m_max_elem_per_loc(max_elem_per_loc)
  { }

  result_type operator()(index_type) const
  {
    result_type res;
    res.reserve(m_outer_elem_count);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> val_dist(1, m_max_value);
    std::uniform_int_distribution<> size_dist(1, m_max_elem_per_loc);

    for (std::size_t i = 0; i < m_outer_elem_count; ++i)
    {
      std::size_t size = size_dist(gen);
      T inner_data;
      inner_data.reserve(size);
      std::generate_n(std::back_inserter(inner_data), size,
                      [&] { return val_dist(gen); });
      res.push_back(inner_data);
    }
    return res;
  }

  void define_type(typer& t)
  {
    t.member(m_max_value);
    t.member(m_outer_elem_count);
    t.member(m_max_elem_per_loc);
  }
};

template <typename F, typename... Args>
void run_test(F const& f, std::string const& title, Args&&... args) {
  skeletons::execute(skeletons::default_execution_params(),
                     std::forward<Args>(args)...);
  stapl::do_once([&title, &f](void) {
    stapl::tests::expect(f()) << title;
  });
}

//////////////////////////////////////////////////////////////////////
/// @brief Tests the available versions of the alltoall skeleton.
//////////////////////////////////////////////////////////////////////
void test_alltoall(std::size_t max_elem_value, std::size_t max_elem_per_loc)
{
  using namespace skeletons;

  typedef int                         val_t;
  typedef lightweight_vector<val_t>   alltoall_val_t;
  typedef std::vector<alltoall_val_t> value_t;
  typedef array<value_t>              container_t;
  typedef array_view<container_t>     view_t;

  std::size_t n = get_num_locations();
  container_t input_array(n);
  container_t output_array(n);
  view_t      input_view(input_array);
  view_t      output_view(output_array);


  generate_vector_data<alltoall_val_t> seq_gen(max_elem_value, n,
                                               max_elem_per_loc);

  stapl::copy(functor_view(n, seq_gen), input_view);

  stapl::identity<value_t> id_wf;

  auto alltoall_verif =
    [&input_view, &output_view] {
      bool is_valid = true;
      for (std::size_t i = 0; i < input_view.size(); ++i) {
        std::size_t n = input_view[i].size();
        for (std::size_t j = 0; j < n; ++j) {
          auto v1 = input_view[i][j];
          auto v2 = output_view[j][i];
          is_valid &= std::equal(v1.begin(), v1.end(), v2.begin());
        }
      }
      return is_valid;
    };

  // butterfly-based alltoall
  run_test(alltoall_verif, "alltoall(butterfly)",
    skeletons::sink<value_t>(
      skeletons::compose(
        map(id_wf),
        alltoall<alltoall_val_t, tags::butterfly<>>())),
    input_view, output_view);

  // flat alltoall
  run_test(alltoall_verif, "alltoall(flat)",
    skeletons::sink<value_t>(
      skeletons::compose(
        map(id_wf),
        alltoall<alltoall_val_t, tags::flat>())),
    input_view, output_view);

  // hybrid alltoall
  run_test(alltoall_verif, "alltoall(hybrid)",
    skeletons::sink<value_t>(
      skeletons::compose(
        map(id_wf),
        alltoall<alltoall_val_t, tags::hybrid>())),
    input_view, output_view);

  // pairwise exchange alltoall
  run_test(alltoall_verif, "alltoall(pairwise_exchange)",
    skeletons::sink<value_t>(
      skeletons::compose(
        map(id_wf),
        alltoall<alltoall_val_t, tags::pairwise_exchange>())),
    input_view, output_view);
}

//////////////////////////////////////////////////////////////////////
/// @brief Tests the available versions of the allgather skeleton.
//////////////////////////////////////////////////////////////////////
void test_allgather(std::size_t max_elem_value, std::size_t max_elem_per_loc)
{
  using namespace skeletons;

  using val_t           = int;
  using allgather_val_t = lightweight_vector<val_t>;
  using value_t         = std::vector<allgather_val_t>;
  using container_t     = array<value_t>;
  using view_t          = array_view<container_t>;

  std::size_t n = get_num_locations();
  container_t input_array(n);
  container_t output_array(n);
  view_t      input_view(input_array);
  view_t      output_view(output_array);

  generate_vector_data<allgather_val_t> seq_gen(max_elem_value, 1,
                                                max_elem_per_loc);

  stapl::copy(functor_view(n, seq_gen), input_view);

  stapl::identity<value_t> id_wf;

  auto allgather_verif =
    [&input_view, &output_view] {
      bool is_valid = true;
      for (std::size_t i = 0; i < input_view.size(); ++i) {
        auto v1 = input_view[i][0];
        auto v2 = output_view[0][i];
        is_valid &= std::equal(v1.begin(), v1.end(), v2.begin());
      }
      return is_valid;
    };

  // reverse-butterfly allgather
  run_test(allgather_verif, "allgather(reverse-butterfly)",
    skeletons::sink<value_t>(
      skeletons::compose(
        map(id_wf),
        skeletons::allgather<allgather_val_t, tags::reverse_butterfly<>>())),
    input_view, output_view);

  // left-aligned allgather
  run_test(allgather_verif, "allgather(left-aligned)",
    skeletons::sink<value_t>(
      skeletons::compose(
        map(id_wf),
        skeletons::allgather<allgather_val_t, tags::left_aligned>())),
    input_view, output_view);

  // right-aligned allgather
  run_test(allgather_verif, "allgather(right-aligned)",
    skeletons::sink<value_t>(
      skeletons::compose(
        map(id_wf),
        skeletons::allgather<allgather_val_t, tags::right_aligned>())),
    input_view, output_view);
}

//////////////////////////////////////////////////////////////////////
/// @brief Tests the available versions of the gather and scatter
/// skeletons.
//////////////////////////////////////////////////////////////////////
void test_gather_scatter(std::size_t max_elem_value,
                         std::size_t max_elem_per_loc)
{
  using namespace skeletons;

  typedef int                               val_t;
  typedef lightweight_vector<val_t>         gather_scatter_val_t;
  typedef std::vector<gather_scatter_val_t> value_t;
  typedef array<value_t>                    container_t;
  typedef array_view<container_t>           view_t;

  std::size_t n = get_num_locations();
  container_t input_array(n);
  container_t output_array(n);
  view_t      input_view(input_array);
  view_t      output_view(output_array);

  generate_vector_data<gather_scatter_val_t> seq_gen(max_elem_value, 1,
                                                     max_elem_per_loc);

  stapl::copy(functor_view(n, seq_gen), input_view);

  stapl::identity<value_t> id_wf;

  auto gather_scatter_verif =
    [&input_view, &output_view] {
      bool is_valid = true;
      for (std::size_t i = 0; i < input_view.size(); ++i) {
        for (std::size_t j = 0; j < input_view[i].size(); ++j) {
          auto v1 = input_view[i][j];
          auto v2 = output_view[i][j];
          is_valid &= std::equal(v1.begin(), v1.end(), v2.begin());
        }
      }
      return is_valid;
    };

  // left-aligned gather_scatter
  run_test(gather_scatter_verif, "gather-scatter(left-aligned)",
    skeletons::sink<value_t>(
      skeletons::compose(
        map(id_wf),
        skeletons::gather<gather_scatter_val_t, tags::left_aligned>(),
        skeletons::scatter<gather_scatter_val_t, tags::left_aligned>())),
    input_view, output_view);

  // right-aligned gather_scatter
  run_test(gather_scatter_verif, "gather-scatter(right-aligned)",
    skeletons::sink<value_t>(
      skeletons::compose(
        map(id_wf),
        skeletons::gather<gather_scatter_val_t, tags::right_aligned>(),
        skeletons::scatter<gather_scatter_val_t, tags::right_aligned>())),
    input_view, output_view);

  // left-aligned gather_scatter
  run_test(gather_scatter_verif, "gather-scatter(left_skewed)",
    skeletons::sink<value_t>(
      skeletons::compose(
        map(id_wf),
        skeletons::gather<gather_scatter_val_t, tags::left_skewed>(),
        skeletons::scatter<gather_scatter_val_t, tags::left_skewed>())),
    input_view, output_view);
}

bool isPowerOfTwo (unsigned int x)
{
  return (x != 0) and ((x & (x-1)) == 0);
}

} // namespace

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3){
    std::cout << "<exec> <maxElemPerLoc> <maxValue>" << std::endl;
    exit(1);
  }

  size_t max_elem_per_loc = atol(argv[1]);
  size_t max_elem_value = atol(argv[2]);
  if (isPowerOfTwo (get_num_locations())) {
    test_alltoall(max_elem_value, max_elem_per_loc);
  }
  test_allgather(max_elem_value, max_elem_per_loc);
  test_gather_scatter(max_elem_value, max_elem_per_loc);

  return EXIT_SUCCESS;
}
