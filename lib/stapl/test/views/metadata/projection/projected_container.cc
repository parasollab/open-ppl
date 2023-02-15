/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/array.hpp>
#include <stapl/views/metadata/extract.hpp>
#include <stapl/views/metadata/container/projected.hpp>
#include <stapl/utility/distributed_value.hpp>

#define REPORT_WITH_COLOR
#include "../../../test_report.hpp"

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout << "usage: exe n" << std::endl;
    exit(1);
  }

  const std::size_t n = atoi(argv[1]);

  using array_type = stapl::array<int>;
  using partition_type = array_type::partition_type;
  using base_container_type
    = array_type::distribution_type::base_container_type;

  using view_type = stapl::array_view<array_type>;

  // Create an array with 2*p partitions
  array_type a(partition_type{{0, n-1}, 2*stapl::get_num_locations()});
  view_type v(a);

  // Extract metadata from the array
  auto md_cont = stapl::metadata::extract(v);

  using metadata_container_type
    = std::remove_pointer<decltype(md_cont)::pointer>::type;

  using projected_container_type
    = stapl::metadata::projected_container<view_type, metadata_container_type>;

  using metadata_type = projected_container_type::value_type;

  // Create projected metadata container from the array's metadata
  auto projected = projected_container_type{v, md_cont.release()};

  bool passed = projected.size() == 2*stapl::get_num_locations();
  STAPL_TEST_REPORT(passed, "Size is correct");

  passed = std::distance(projected.begin(), projected.end()) == 2
    && projected.local_size() == 2;

  STAPL_TEST_REPORT(passed, "Correct number of local entries");

  // Iterate over the local metadata entries and determine if they
  // are the same as this location's base containers
  passed
    = std::equal(projected.begin(),
                 projected.end(),
                 a.distribution().container_manager().begin(),
                 [](metadata_type const& md, base_container_type const& bc) {
                   return md.id() == bc.cid()
                     && md.domain().first() == bc.domain().first()
                     && md.domain().last() == bc.domain().last();
                 });

  passed = stapl::distributed_value<bool>{ passed }
             .reduce(std::logical_and<bool>{})
             .get();

  STAPL_TEST_REPORT(passed, "Entries span entire base container");

  return EXIT_SUCCESS;
}
