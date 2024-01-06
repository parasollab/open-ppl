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
#include <stapl/skeletons/operators/consumer_count.hpp>
#include <stapl/skeletons/spans/balanced.hpp>
#include <stapl/skeletons/spans/binomial_tree.hpp>
#include <stapl/skeletons/spans/blocked.hpp>
#include <stapl/skeletons/spans/nearest_pow_two.hpp>
#include <stapl/skeletons/spans/per_location.hpp>
#include <stapl/skeletons/spans/reduce_to_pow_two.hpp>
#include <stapl/skeletons/spans/tree.hpp>
#include <stapl/domains/indexed.hpp>
#include "../expect.hpp"
#include "fake.hpp"

/// Tests a span with a given verification function.
template <typename Span, typename Dims, typename Verify>
void test_span(std::size_t num_locations, std::size_t location_id,
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
  auto span = Span();

  // expand the span given the input view using a fake spawner
  span.set_size(fake_spawner{num_locations, location_id}, view);

  // start testing
  verify(span);
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  using stapl::skeletons::spans::balanced;
  using stapl::skeletons::spans::binomial_tree;
  using stapl::skeletons::spans::blocked;
  using stapl::skeletons::spans::nearest_pow_two;
  using stapl::skeletons::spans::per_location;
  using stapl::skeletons::spans::reduce_to_pow_two;
  using stapl::skeletons::spans::tree;
  using stapl::skeletons::spans::reverse_tree;
  using stapl::skeletons::tags::down_phase;
  using stapl::skeletons::tags::left_skewed;
  using stapl::skeletons::tags::left_aligned;
  using stapl::skeletons::tags::right_aligned;
  using stapl::skeletons::tags::up_phase;
  using stapl::tests::expect_eq;
  using stapl::tests::expect;
  using stapl::make_tuple;
  using stapl::get;

  std::size_t num_locations = 4;
  std::size_t location_id = 1;
  std::size_t dom1 = 10ul;

  // Testing spans::per_location with a view.size > num_locations.
  test_span<per_location>(
    num_locations, location_id, dom1,
    [num_locations](per_location& span) {
      std::cout << "Testing per_location case 1" << std::endl;
      auto&& local_domain = span.local_domain();
      expect_eq(local_domain.size(), 1) << "local_domain.size";
      expect_eq(local_domain[0].size(), 1) << "local_domain[0].size";
      expect_eq(local_domain[0].first(), 1) << "local_domain[0].first";
      expect_eq(local_domain[0].last(), 1) << "local_domain[0].last";
      // should spawn all the elements
      expect(span.should_spawn(0, 0)) << "should_spawn";
      expect_eq(span.dimensions(), num_locations) << "dimensions";
      expect_eq(span.size(), num_locations) << "size";
      expect_eq(span.linearize(5), 5) << "linearize";
      // TODO(mani) more rigorous tests are needed for nested cases
      expect_eq(span.total_dimension(), num_locations) << "total_dimensions";
      expect_eq(span.task_dimension(), 1) << "task_dimensions";
    });

  // TODO(mani) spans::per_location should behave differently when the
  // size of the input view is smaller than the number of locations. This
  // case is not correctly handled at this point and requires set_size to
  // be called with set_size<bool>(spawner, views...) for it to work properly.
  //
  // Testing spans::per_location with a view.size < num_locations
  // test_span<per_location>(
  //   num_locations, location_id, num_locations - 1,
  //   [num_locations](per_location& span) {
  //     std::cout << "Testing per_location case 2" << std::endl;
  //     auto&& local_domain = span.local_domain();
  //     expect_eq(local_domain.size(), 1) << "local_domain.size";
  //     expect_eq(local_domain[0].size(), 1) << "local_domain[0].size";
  //     expect_eq(local_domain[0].first(), 1) << "local_domain[0].first";
  //     expect_eq(local_domain[0].last(), 1) << "local_domain[0].last";
  //     // should spawn all the elements
  //     expect(span.should_spawn(0, 0)) << "should_spawn";
  //     expect_eq(span.dimensions(), 3) << "dimensions";
  //     expect_eq(span.size(), 3) << "size";
  //     expect_eq(span.linearize(5), 5) << "linearize";
  //     // TODO(mani) more rigorous tests are needed for nested cases
  //     expect_eq(span.total_dimension(), 3) << "total_dimensions";
  //     expect_eq(span.task_dimension(), 1) << "task_dimensions";
  //   });

  // Testing spans::balanced<1>
  test_span<balanced<1>>(
    num_locations, location_id, dom1,
    [dom1](balanced<1>& span) {
      std::cout << "Testing balanced<1>" << std::endl;
      auto&& local_domain = span.local_domain();
      expect_eq(local_domain.size(), 1) << "local_domain.size";
      expect_eq(local_domain[0].size(), 3) << "local_domain[0].size";
      expect_eq(local_domain[0].first(), 3) << "local_domain[0].first";
      expect_eq(local_domain[0].last(), 5) << "local_domain[0].last";
      // should spawn all the elements
      expect(span.should_spawn(0, 0)) << "should_spawn";
      expect_eq(span.dimensions(), dom1) << "dimensions";
      expect_eq(span.size(), dom1) << "size";
      expect_eq(span.linearize(5), 5) << "linearize";
      // TODO(mani) more rigorous tests are needed for nested cases
      expect_eq(span.total_dimension(), dom1) << "total_dimensions";
      expect_eq(span.task_dimension(), 1) << "task_dimensions";
    });

  // Testing spans::blocked<1>
  test_span<blocked<1>>(
    num_locations, location_id, dom1,
    [dom1](blocked<1>& span) {
      std::cout << "Testing blocked<1>" << std::endl;
      auto&& local_domain = span.local_domain();
      expect_eq(local_domain.size(), 1) << "local_domain.size";
      expect_eq(local_domain[0].size(), 3) << "local_domain[0].size";
      expect_eq(local_domain[0].first(), 3) << "local_domain[0].first";
      expect_eq(local_domain[0].last(), 5) << "local_domain[0].last";
      // should spawn all the elements
      expect(span.should_spawn(0, 0)) << "should_spawn";
      expect_eq(span.dimensions(), dom1) << "dimensions";
      expect_eq(span.size(), dom1) << "size";
      expect_eq(span.linearize(5), 5) << "linearize";
      // TODO(mani) more rigorous tests are needed for nested cases
      expect_eq(span.total_dimension(), dom1) << "total_dimensions";
      expect_eq(span.task_dimension(), 1) << "task_dimensions";
    });

  // Testing spans::blocked<2>
  auto dom2 = make_tuple(dom1, dom1);
  test_span<blocked<2>>(
    num_locations, location_id, dom2,
    [dom2](blocked<2>& span) {
      std::cout << "Testing blocked<2>" << std::endl;
      auto&& local_domain = span.local_domain();
      expect_eq(local_domain.size(), 1) << "local_domain.size";
      expect_eq(local_domain[0].size(), 25) << "local_domain[0].size";
      expect_eq(local_domain[0].first(), make_tuple(0ul, 5ul))
        << "local_domain[0].first";
      expect_eq(local_domain[0].last(), make_tuple(4ul, 9ul))
        << "local_domain[0].last";
      expect(span.should_spawn(make_tuple(0, 0), make_tuple(0, 0)))
        << "should_spawn";
      expect_eq(span.dimensions(), dom2) << "dimensions";
      expect_eq(span.size(), get<0>(dom2) * get<1>(dom2)) << "size";
      expect_eq(span.linearize(make_tuple(5, 5)), 55) << "linearize";
      // TODO(mani) more rigorous tests are needed for nested cases
      expect_eq(span.total_dimension(), dom2) << "total_dimensions";
      expect_eq(span.task_dimension(), make_tuple(1, 1)) << "task_dimensions";
    });

  // Testing spans::blocked<3>
  auto dom3 = make_tuple(dom1, dom1, dom1);
  test_span<blocked<3>>(
    num_locations, location_id, dom3,
    [dom3](blocked<3>& span) {
      std::cout << "Testing blocked<3>" << std::endl;
      auto&& local_domain = span.local_domain();
      expect_eq(local_domain.size(), 1) << "local_domain.size";
      expect_eq(local_domain[0].size(), 250) << "local_domain[0].size";
      expect_eq(local_domain[0].first(), make_tuple(0ul, 0ul, 5ul))
        << "local_domain[0].first";
      expect_eq(local_domain[0].last(), make_tuple(4ul, 9ul, 9ul))
        << "local_domain[0].last";
      expect(span.should_spawn(make_tuple(0, 0, 0), make_tuple(0, 0, 0)))
        << "should_spawn";
      expect_eq(span.dimensions(), dom3) << "dimensions";
      expect_eq(span.size(), get<0>(dom3) * get<1>(dom3) * get<2>(dom3))
        << "size";
      expect_eq(span.linearize(make_tuple(5, 5, 5)), 555) << "linearize";
      // TODO(mani) more rigorous tests are needed for nested cases
      expect_eq(span.total_dimension(), dom3) << "total_dimensions";
      expect_eq(span.task_dimension(), make_tuple(1, 1, 1))
        << "task_dimensions";
    });

  // Testing spans::reduce_to_pow_two<balanced<1>>O
  test_span<reduce_to_pow_two<balanced<1>>>(
    num_locations, location_id, dom1,
    [dom1](reduce_to_pow_two<balanced<1>>& span) {
      std::cout << "Testing reduce to pow two" << std::endl;
      auto&& local_domain = span.local_domain();
      expect_eq(local_domain.size(), 1) << "local_domain.size";
      // the size of the span is still dom1 but the last (dom1 - log(dom1))
      // elements should return false for should_spawn.
      expect_eq(local_domain[0].size(), 3) << "local_domain[0].size";
      expect_eq(local_domain[0].first(), 3) << "local_domain[0].first";
      expect_eq(local_domain[0].last(), 5) << "local_domain[0].last";
      std::size_t nearest_pow_two = 1ul << std::lround(std::log2(dom1));
      std::size_t i = 0;
      for (; i < nearest_pow_two; ++i) {
        expect(span.should_spawn(make_tuple(dom1, 0), make_tuple(i, 0)))
          << "should_spawn - case" << i;
      }
      for (; i < dom1; ++i) {
        expect(!span.should_spawn(make_tuple(dom1, 0), make_tuple(i, 0)))
          << "!should_spawn - case" << i;
      }
      expect_eq(span.dimensions(), dom1) << "dimensions";
      expect_eq(span.size(), dom1) << "size";
      expect_eq(span.linearize(5), 5) << "linearize";
      // TODO(mani) more rigorous tests are needed for nested cases
      expect_eq(span.total_dimension(), dom1) << "total_dimensions";
      expect_eq(span.task_dimension(), 1) << "task_dimensions";
    });

  // Testing spans::reduce_to_pow_two<balanced<1>>
  test_span<nearest_pow_two<balanced<1>>>(
    num_locations, location_id, dom1,
    [dom1](nearest_pow_two<balanced<1>>& span) {
      std::cout << "Testing nearest_pow_two" << std::endl;
      auto&& local_domain = span.local_domain();
      expect_eq(local_domain.size(), 1) << "local_domain.size";
      expect_eq(local_domain[0].size(), 2) << "local_domain[0].size";
      expect_eq(local_domain[0].first(), 2) << "local_domain[0].first";
      expect_eq(local_domain[0].last(), 3) << "local_domain[0].last";
      expect(span.should_spawn(0, 0)) << "should_spawn";
      //computing nearest power of two
      std::size_t nearest_pow_two = 1ul << std::lround(std::log2(dom1));
      expect_eq(span.dimensions(), nearest_pow_two) << "dimensions";
      expect_eq(span.size(), nearest_pow_two) << "size";
      expect_eq(span.linearize(5), 5) << "linearize";
      // TODO(mani) more rigorous tests are needed for nested cases
      expect_eq(span.total_dimension(), nearest_pow_two)
        << "total_dimensions";
      expect_eq(span.task_dimension(), 1) << "task_dimensions";
    });

  // Testing tree-based spans:
  // These spans require the size to be power of two. For simplicity we use
  // 8 as the input size and check various alignments. The depth of the tree
  // is assumed to be log2(8) = 3.
  std::size_t tree_width = 8;
  std::size_t tree_depth = 3;
  // TODO(mani) These spans are very simplistic and do not consider k-ary
  // trees. They should be changed to cover more cases and to include tree
  // structure within them.
  // TODO(mani) All the tree tests can be combined with polymorphic lambdas
  // into 1 test with one extra lambda for the comparisons of should_spawn.

  // Testing spans::tree<balanced<1>, left_skewed>
  test_span<tree<balanced<1>, left_skewed>>(
    num_locations, location_id, tree_width,
    [tree_width, tree_depth](tree<balanced<1>, left_skewed>& span) {
      std::cout << "Testing left-skewed tree" << std::endl;
      auto&& local_domain = span.local_domain();
      expect_eq(local_domain.size(), 1) << "local_domain.size";
      expect_eq(local_domain[0].size(), 2) << "local_domain[0].size";
      expect_eq(local_domain[0].first(), 2) << "local_domain[0].first";
      expect_eq(local_domain[0].last(), 3) << "local_domain[0].last";

      auto tree_size = make_tuple(tree_width, tree_depth);
      for (std::size_t level = 0; level < tree_depth; ++level) {
        for (std::size_t index = 0; index < tree_width; ++index) {
          auto coord = make_tuple(index, level);
          if (index < (1ul << (tree_depth - level - 1))) {
            expect(span.should_spawn(tree_size, coord))
              << "should_spawn";
          }
          else {
            expect(!span.should_spawn(tree_size, coord))
              << "!should_spawn";
          }
        }
      }
      //computing nearest power of two
      expect_eq(span.dimensions(), tree_width) << "dimensions";
      expect_eq(span.size(), tree_width) << "size";
      expect_eq(span.linearize(5), 5) << "linearize";
      // TODO(mani) more rigorous tests are needed for nested cases
      expect_eq(span.total_dimension(), tree_width)
        << "total_dimensions";
      expect_eq(span.task_dimension(), 1) << "task_dimensions";
    });

  // Testing spans::tree<balanced<1>, left_skewed>
  test_span<tree<balanced<1>, left_aligned>>(
    num_locations, location_id, tree_width,
    [tree_width, tree_depth](tree<balanced<1>, left_aligned>& span) {
      std::cout << "Testing left-aligned tree" << std::endl;
      auto&& local_domain = span.local_domain();
      expect_eq(local_domain.size(), 1) << "local_domain.size";
      expect_eq(local_domain[0].size(), 2) << "local_domain[0].size";
      expect_eq(local_domain[0].first(), 2) << "local_domain[0].first";
      expect_eq(local_domain[0].last(), 3) << "local_domain[0].last";

      auto tree_size = make_tuple(tree_width, tree_depth);
      for (std::size_t level = 0; level < tree_depth; ++level) {
        for (std::size_t index = 0; index < tree_width; ++index) {
          auto coord = make_tuple(index, level);
          if (index % (1ul << (level + 1)) == 0) {
            expect(span.should_spawn(tree_size, coord))
              << "should_spawn";
          }
          else {
            expect(!span.should_spawn(tree_size, coord))
              << "!should_spawn";
          }
        }
      }
      //computing nearest power of two
      expect_eq(span.dimensions(), tree_width) << "dimensions";
      expect_eq(span.size(), tree_width) << "size";
      expect_eq(span.linearize(5), 5) << "linearize";
      // TODO(mani) more rigorous tests are needed for nested cases
      expect_eq(span.total_dimension(), tree_width)
        << "total_dimensions";
      expect_eq(span.task_dimension(), 1) << "task_dimensions";
    });

    // Testing spans::tree<balanced<1>, right_aligned>
  test_span<tree<balanced<1>, right_aligned>>(
    num_locations, location_id, tree_width,
    [tree_width, tree_depth](tree<balanced<1>, right_aligned>& span) {
      std::cout << "Testing right-aligned tree" << std::endl;
      auto&& local_domain = span.local_domain();
      expect_eq(local_domain.size(), 1) << "local_domain.size";
      expect_eq(local_domain[0].size(), 2) << "local_domain[0].size";
      expect_eq(local_domain[0].first(), 2) << "local_domain[0].first";
      expect_eq(local_domain[0].last(), 3) << "local_domain[0].last";

      auto tree_size = make_tuple(tree_width, tree_depth);
      for (std::size_t level = 0; level < tree_depth; ++level) {
        for (std::size_t index = 0; index < tree_width; ++index) {
          auto coord = make_tuple(index, level);
          if ((index+1) % (1ul << (level + 1)) == 0) {
            expect(span.should_spawn(tree_size, coord))
              << "should_spawn";
          }
          else {
            expect(!span.should_spawn(tree_size, coord))
              << "!should_spawn";
          }
        }
      }
      //computing nearest power of two
      expect_eq(span.dimensions(), tree_width) << "dimensions";
      expect_eq(span.size(), tree_width) << "size";
      expect_eq(span.linearize(5), 5) << "linearize";
      // TODO(mani) more rigorous tests are needed for nested cases
      expect_eq(span.total_dimension(), tree_width)
        << "total_dimensions";
      expect_eq(span.task_dimension(), 1) << "task_dimensions";
    });

    // Testing spans::reverse_tree<balanced<1>, left_skewed>
    test_span<reverse_tree<balanced<1>, left_skewed>>(
    num_locations, location_id, tree_width,
    [tree_width, tree_depth](reverse_tree<balanced<1>, left_skewed>& span) {
      std::cout << "Testing left-skewed reverse-tree" << std::endl;
      auto&& local_domain = span.local_domain();
      expect_eq(local_domain.size(), 1) << "local_domain.size";
      expect_eq(local_domain[0].size(), 2) << "local_domain[0].size";
      expect_eq(local_domain[0].first(), 2) << "local_domain[0].first";
      expect_eq(local_domain[0].last(), 3) << "local_domain[0].last";

      auto tree_size = make_tuple(tree_width, tree_depth);
      for (std::size_t level = 0; level < tree_depth; ++level) {
        for (std::size_t index = 0; index < tree_width; ++index) {
          auto coord = make_tuple(index, level);
          if (index < (1ul << (level + 1))) {
            expect(span.should_spawn(tree_size, coord)) << "should_spawn";
          }
          else {
            expect(!span.should_spawn(tree_size, coord)) << "!should_spawn";
          }
        }
      }
      //computing nearest power of two
      expect_eq(span.dimensions(), tree_width) << "dimensions";
      expect_eq(span.size(), tree_width) << "size";
      expect_eq(span.linearize(5), 5) << "linearize";
      // TODO(mani) more rigorous tests are needed for nested cases
      expect_eq(span.total_dimension(), tree_width)
        << "total_dimensions";
      expect_eq(span.task_dimension(), 1) << "task_dimensions";
    });

  // Testing spans::reverse_tree<balanced<1>, left_aligned>
  test_span<reverse_tree<balanced<1>, left_aligned>>(
    num_locations, location_id, tree_width,
    [tree_width, tree_depth](reverse_tree<balanced<1>, left_aligned>& span) {
      std::cout << "Testing left-aligned reverse-tree" << std::endl;
      auto&& local_domain = span.local_domain();
      expect_eq(local_domain.size(), 1) << "local_domain.size";
      expect_eq(local_domain[0].size(), 2) << "local_domain[0].size";
      expect_eq(local_domain[0].first(), 2) << "local_domain[0].first";
      expect_eq(local_domain[0].last(), 3) << "local_domain[0].last";

      auto tree_size = make_tuple(tree_width, tree_depth);
      for (std::size_t level = 0; level < tree_depth; ++level) {
        for (std::size_t index = 0; index < tree_width; ++index) {
          auto coord = make_tuple(index, level);
          if (index % (1ul << (tree_depth - level - 1)) == 0) {
            expect(span.should_spawn(tree_size, coord))
              << "should_spawn";
          }
          else {
            expect(!span.should_spawn(tree_size, coord))
              << "!should_spawn";
          }
        }
      }
      //computing nearest power of two
      expect_eq(span.dimensions(), tree_width) << "dimensions";
      expect_eq(span.size(), tree_width) << "size";
      expect_eq(span.linearize(5), 5) << "linearize";
      // TODO(mani) more rigorous tests are needed for nested cases
      expect_eq(span.total_dimension(), tree_width)
        << "total_dimensions";
      expect_eq(span.task_dimension(), 1) << "task_dimensions";
    });

  // binomial trees can handle non-power of two sizes.
  tree_width = dom1;
  tree_depth = std::lround(log2(dom1));

  // Testing spans::binomial_tree<balanced<1>, up_phase>
  test_span<binomial_tree<balanced<1>, up_phase>>(
    num_locations, location_id, tree_width,
    [tree_width, tree_depth](binomial_tree<balanced<1>, up_phase>& span) {
      std::cout << "Testing binomial-tree up-phase" << std::endl;
      auto&& local_domain = span.local_domain();
      expect_eq(local_domain.size(), 1) << "local_domain.size";
      expect_eq(local_domain[0].size(), 3) << "local_domain[0].size";
      expect_eq(local_domain[0].first(), 3) << "local_domain[0].first";
      expect_eq(local_domain[0].last(), 5) << "local_domain[0].last";

      auto tree_size = make_tuple(tree_width, tree_depth);
      for (std::size_t level = 0; level < tree_depth; ++level) {
        for (std::size_t index = 0; index < tree_width; ++index) {
          auto coord = make_tuple(index, level);
          std::size_t mask = (1ul << (level+1)) - 1;
          if ((mask & index) == mask) {
            expect(span.should_spawn(tree_size, coord))
              << "should_spawn";
          }
          else {
            expect(!span.should_spawn(tree_size, coord))
              << "!should_spawn";
          }
        }
      }
      //computing nearest power of two
      expect_eq(span.dimensions(), tree_width) << "dimensions";
      expect_eq(span.size(), tree_width) << "size";
      expect_eq(span.linearize(5), 5) << "linearize";
      // TODO(mani) more rigorous tests are needed for nested cases
      expect_eq(span.total_dimension(), tree_width)
        << "total_dimensions";
      expect_eq(span.task_dimension(), 1) << "task_dimensions";
    });

  // Testing spans::binomial_tree<balanced<1>, down_phase>
  test_span<binomial_tree<balanced<1>, down_phase>>(
    num_locations, location_id, tree_width,
    [tree_width, tree_depth](binomial_tree<balanced<1>, down_phase>& span) {
      std::cout << "Testing binomial-tree down-phase" << std::endl;
      auto&& local_domain = span.local_domain();
      expect_eq(local_domain.size(), 1) << "local_domain.size";
      expect_eq(local_domain[0].size(), 3) << "local_domain[0].size";
      expect_eq(local_domain[0].first(), 3) << "local_domain[0].first";
      expect_eq(local_domain[0].last(), 5) << "local_domain[0].last";

      auto tree_size = make_tuple(tree_width, tree_depth);
      for (std::size_t level = 0; level < tree_depth; ++level) {
        for (std::size_t index = 0; index < tree_width; ++index) {
          auto coord = make_tuple(index, level);
          std::size_t mask = (1ul << (tree_depth - level - 1)) - 1;
          if ((index & mask) == mask) {
            expect(span.should_spawn(tree_size, coord))
              << "should_spawn";
          }
          else {
            expect(!span.should_spawn(tree_size, coord))
              << "!should_spawn";
          }
        }
      }
      //computing nearest power of two
      expect_eq(span.dimensions(), tree_width) << "dimensions";
      expect_eq(span.size(), tree_width) << "size";
      expect_eq(span.linearize(5), 5) << "linearize";
      // TODO(mani) more rigorous tests are needed for nested cases
      expect_eq(span.total_dimension(), tree_width)
        << "total_dimensions";
      expect_eq(span.task_dimension(), 1) << "task_dimensions";
    });
  return EXIT_SUCCESS;
}
