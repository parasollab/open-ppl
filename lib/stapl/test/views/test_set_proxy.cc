/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/containers/array/static_array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <sstream>

#include "../test_report.hpp"

using set_type = std::set<std::string>;

/// Functor that creates a union of two given sets.
struct set_union_wf
{
  using result_type = set_type;

  template <typename S1, typename S2>
  result_type operator() (S1&& set1, S2&& set2) const
  {
    result_type result = set1;
    result.insert(set2.begin(), set2.end());
    return result;
  }
};

/// Functor that inserts a string to given set.
struct set_insert_wf
{
  set_insert_wf(std::string const& str)
    : m_str(str)
  {}

  template<typename Set>
  void operator() (Set&& set) const
  {
    set.insert(m_str);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_str);
  }

private:
  std::string m_str;
};

stapl::exit_code stapl_main(int argc, char* argv[])
{
  std::size_t nlocs = stapl::get_num_locations();
  std::size_t loc = stapl::get_location_id();

  // Create an array with one std::set per location.
  stapl::static_array<set_type> set_array(nlocs);

  // Insert a string common for all locations to this location's set.
  set_array.apply_set(loc, set_insert_wf("Common string"));

  // Insert two location-specific strings. One of them overlaps with a string
  // from another location's set.
  std::stringstream ss;
  ss << "Location string " << loc;
  set_array.apply_set(loc, set_insert_wf(ss.str()));
  ss.str("");
  ss << "Location string " << (loc + 1) % nlocs;
  set_array.apply_set(loc, set_insert_wf(ss.str()));

  // Get a union of the sets on all locations.
  auto union_set = stapl::accumulate(stapl::make_array_view(set_array),
                                     set_type(), set_union_wf());

  stapl::do_once([nlocs, &union_set] {
    set_type expected_union_set { "Common string" };

    for (std::size_t loc = 0; loc < nlocs; ++loc) {
      std::stringstream ss;
      ss << "Location string " << loc;
      expected_union_set.insert(ss.str());
    }

    STAPL_TEST_REPORT(union_set == expected_union_set,
                      "Testing specialization of proxy for std::set")
  });

  return EXIT_SUCCESS;
}
