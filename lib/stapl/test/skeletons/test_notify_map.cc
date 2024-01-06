/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#include <cstdlib>
#include <vector>
#include <numeric>
#include <stapl/runtime.hpp>
#include <stapl/array.hpp>
#include <stapl/skeletons/operators/compose.hpp>
#include <stapl/skeletons/functional/map.hpp>
#include <stapl/skeletons/functional/notify_map.hpp>
#include <stapl/skeletons/functional/reduce_to_locs.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/views/metadata/coarseners/null.hpp>
#include "sample_flows.hpp"
#include "../test_report.hpp"

using namespace stapl;

namespace  {

//////////////////////////////////////////////////////////////////////
/// @brief Fills elements of a given view with a std::vector of the
/// given size.
///
/// @tparam T type of elements to be inserted
//////////////////////////////////////////////////////////////////////
template <typename T>
struct fill_with_vector
{
private:
  std::size_t m_size;

public:
  typedef bool result_type;

  fill_with_vector(std::size_t size)
    : m_size(size)
  { }

  template <typename V>
  result_type operator()(V&& v)
  {
    std::vector<T> elements(m_size);
    std::iota(elements.begin(), elements.end(), T());
    v = elements;
    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_size);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Checks if a given view element has the correct number of
/// elements in it. In this specific test, if the check is passed
/// the notify_task has worked properly.
//////////////////////////////////////////////////////////////////////
struct check_size
{
private:
  std::size_t m_size;

public:
  typedef bool result_type;

  check_size(std::size_t size)
    : m_size(size)
  { }

  template <typename V>
  result_type operator()(V&& v)
  {
    return v.size() == m_size;
  }

  void define_type(typer& t)
  {
    t.member(m_size);
  }
};

} // namespace

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2){
    std::cout << "<exec> <n>" << std::endl;
    exit(1);
  }

  std::size_t n = atol(argv[1]);

  typedef std::size_t                inner_value_t;
  typedef std::vector<inner_value_t> value_t;
  typedef array<value_t>             container_t;
  typedef array_view<container_t>    view_t;

  using namespace stapl::skeletons;
  container_t input_array(n);
  view_t      in_view(input_array);
  std::size_t test_size = 10000;

  /// Corresponding composition specification:
  /// input  = [in_view]
  /// v1     = map(fill_with_vector<inner_value_t>(test_size)) [in_view]
  /// v2     = notify_map<1>(check_size(test_size)) [v1 in_view]
  /// v3     = reduce_to_locs(stapl::logical_and<bool>()) [v2]
  /// output = [v3]

  auto s = compose<flows::sample_flows>(
             skeletons::map(fill_with_vector<inner_value_t>(test_size)),
             skeletons::notify_map<1>(check_size(test_size)),
             skeletons::reduce_to_locs<true>(stapl::logical_and<bool>()));

  bool passed = skeletons::execute(skeletons::execution_params<bool>(),
                                   s, in_view);
  STAPL_TEST_REPORT(passed, "Testing notify_map skeleton");

  return EXIT_SUCCESS;
}
