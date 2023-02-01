/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#include <cmath>
#include <stapl/runtime.hpp>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/skeletons/operators/compose.hpp>
#include <stapl/skeletons/operators/elem.hpp>
#include <stapl/skeletons/operators/repeat.hpp>
#include <stapl/skeletons/param_deps/zip_pd.hpp>
#include <stapl/skeletons/spans/blocked.hpp>
#include <stapl/domains/indexed.hpp>
#include "../expect.hpp"
#include "fake.hpp"

/// Tests the 1D balanced span
template <typename Skeleton, typename Dims, typename Verify>
void test_skeleton(Skeleton& skeleton,
                   std::size_t num_locations, std::size_t location_id,
                   Dims const& dimensions, Verify&& verify)
{
  using stapl::default_traversal;
  using stapl::skeletons::tests::fake_container;
  using stapl::skeletons::tests::fake_view;
  using stapl::skeletons::tests::fake_spawner;
  using stapl::skeletons::tests::make_partition;

  using value_type = int;
  auto partition = make_partition(dimensions, num_locations);
  using container_type = fake_container<value_type, decltype(partition)>;

  // creating a fake container and a fake view over it to test the Span
  auto container = container_type{partition, location_id};
  auto view = fake_view<container_type>{container};

  // expand the span given the input view using a fake spawner
  skeleton.set_dimensions(fake_spawner{num_locations, location_id}, view);

  // start testing
  verify(skeleton);
}
stapl::exit_code stapl_main(int argc, char* argv[])
{
  using stapl::balanced_partition;
  using stapl::get;
  using stapl::indexed_domain;
  using stapl::make_tuple;
  using stapl::skeletons::compose;
  using stapl::skeletons::elem;
  using stapl::skeletons::repeat;
  using stapl::skeletons::spans::blocked;
  using stapl::skeletons::zip_pd;
  using stapl::tests::expect_eq;
  using stapl::tests::expect;
  using std::log2;
  using std::lround;

  std::size_t num_locations = 4;
  std::size_t location_id = 1;
  std::size_t dims1 = 10ul;

  // Testing elem
  auto elem_1d = elem(zip_pd<1>([](size_t i) { return i;}));
  test_skeleton(elem_1d, num_locations, location_id, dims1,
    [dims1](decltype(elem_1d) const& skeleton) {
      std::cout << "Testing 1D elem" << std::endl;
      expect_eq(skeleton.dimensions(), dims1) << "dimensions";
      expect_eq(skeleton.last_id(), dims1) << "last_id";
    });

  auto dims2 = make_tuple(dims1, dims1);
  auto elem_2d = elem<blocked<2>>(zip_pd<1>([](size_t i) { return i;}));
  test_skeleton(elem_2d, num_locations, location_id, dims2,
    [dims2](decltype(elem_2d) const& skeleton) {
      std::size_t last_id = get<0>(dims2) * get<1>(dims2);
      std::cout << "Testing 2D elem" << std::endl;
      expect_eq(skeleton.dimensions(), dims2) << "dimensions";
      expect_eq(skeleton.last_id(), last_id) << "last_id";
    });

  auto dims3 = make_tuple(dims1, dims1, dims1);
  auto elem_3d = elem<blocked<3>>(zip_pd<1>([](size_t i) { return i;}));
  test_skeleton(elem_3d, num_locations, location_id, dims3,
    [dims3](decltype(elem_3d) const& skeleton) {
      std::size_t last_id = get<0>(dims3) * get<1>(dims3) * get<2>(dims3);
      std::cout << "Testing 3D elem" << std::endl;
      expect_eq(skeleton.dimensions(), dims3) << " dimensions";
      expect_eq(skeleton.last_id(), last_id) << " dimensions";
    });

  // Testing repeat
  auto repeat_log2 = repeat(elem_1d,
                            [](std::size_t n) {
                              return lround(log2(n));
                            });
  test_skeleton(repeat_log2, num_locations, location_id, dims1,
    [dims1](decltype(repeat_log2) const& skeleton) {
      std::cout << "Testing repeat(elem_1d, log_size(_1))" << std::endl;
      std::size_t depth = lround(log2(dims1));
      std::size_t last_id = depth * dims1;
      expect_eq(skeleton.dimensions(), depth) << "dimensions";
      expect_eq(skeleton.last_id(), last_id) << "last_id";
    });

  // Testing repeat
  auto repeat_log2_2d = repeat(elem_2d,
                               [](decltype(dims2) n) {
                                 return lround(log2(get<0>(n) * get<1>(n)));
                               });
  test_skeleton(repeat_log2_2d, num_locations, location_id, dims2,
    [dims2](decltype(repeat_log2_2d) const& skeleton) {
      std::cout << "Testing repeat(elem_2d, log_size(_1*_1))" << std::endl;
      std::size_t iteration_size = get<0>(dims2) * get<1>(dims2);
      std::size_t depth = lround(log2(iteration_size));
      std::size_t last_id = depth * iteration_size;
      expect_eq(skeleton.dimensions(), depth) << "dimensions";
      expect_eq(skeleton.last_id(), last_id) << "last_id";
    });

  // Testing compose
  auto compose_case_1 = compose(repeat_log2, elem_1d);
  test_skeleton(compose_case_1, num_locations, location_id, dims1,
    [dims1](decltype(compose_case_1) const& skeleton) {
      std::cout << "Testing compose(repeat_1d, elem_1d)" << std::endl;
      std::size_t last_id = skeleton.get_skeleton<0>().last_id() +
                            skeleton.get_skeleton<1>().last_id();
      expect_eq(skeleton.dimensions(), 2) << "dimensions";
      expect_eq(skeleton.last_id(), last_id) << "last_id";
    });

  auto compose_case_2 = compose(elem_1d, repeat_log2);
  test_skeleton(compose_case_2, num_locations, location_id, dims1,
    [](decltype(compose_case_2) const& skeleton) {
      std::cout << "Testing compose(elem_1d, repeat_log2)" << std::endl;
      std::size_t last_id = skeleton.get_skeleton<0>().last_id() +
                            skeleton.get_skeleton<1>().last_id();
      expect_eq(skeleton.dimensions(), 2) << "dimensions";
      expect_eq(skeleton.last_id(), last_id) << "last_id";
    });

  auto compose_case_3 = compose(elem_2d, repeat_log2_2d);
  test_skeleton(compose_case_3, num_locations, location_id, dims2,
    [](decltype(compose_case_3) const& skeleton) {
      std::cout << "Testing compose(elem_2d, repeat_log2_2d)" << std::endl;
      std::size_t last_id = skeleton.get_skeleton<0>().last_id() +
                            skeleton.get_skeleton<1>().last_id();
      expect_eq(skeleton.dimensions(), 2) << "dimensions";
      expect_eq(skeleton.last_id(), last_id) << "last_id";
    });

  auto compose_case_4 = compose(elem_1d);
  test_skeleton(compose_case_4, num_locations, location_id, dims1,
    [](decltype(compose_case_4) const& skeleton) {
      std::cout << "Testing compose(elem_1d)" << std::endl;
      std::size_t last_id = skeleton.get_skeleton<0>().last_id();
      expect_eq(skeleton.dimensions(), 1) << "dimensions";
      expect_eq(skeleton.last_id(), last_id) << "last_id";
    });

  auto compose_case_5 = compose(elem_2d, elem_2d);
  test_skeleton(compose_case_5, num_locations, location_id, dims2,
    [](decltype(compose_case_5) const& skeleton) {
      std::cout << "Testing compose(elem_2d, elem_2d)" << std::endl;
      std::size_t last_id = skeleton.get_skeleton<0>().last_id() +
                            skeleton.get_skeleton<1>().last_id();
      expect_eq(skeleton.dimensions(), 2) << "dimensions";
      expect_eq(skeleton.last_id(), last_id) << "last_id";
    });

  auto compose_case_6 = compose(elem_3d, elem_3d, elem_3d);
  test_skeleton(compose_case_6, num_locations, location_id, dims3,
    [](decltype(compose_case_6) const& skeleton) {
      std::cout << "Testing compose(elem_3d, elem_3d, elem_3d)" << std::endl;
      std::size_t last_id = skeleton.get_skeleton<0>().last_id() +
                            skeleton.get_skeleton<1>().last_id() +
                            skeleton.get_skeleton<2>().last_id();
      expect_eq(skeleton.dimensions(), 3) << "dimensions";
      expect_eq(skeleton.last_id(), last_id) << "last_id";
    });

  // TODO(mani) unittests for do-while need to be added.

  return EXIT_SUCCESS;
}

