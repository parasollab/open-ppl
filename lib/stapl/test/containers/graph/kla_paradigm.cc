/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <utility>
#include <vector>

#include <stapl/containers/graph/multigraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/connected_components.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/disjointed_complete.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "test_util.h"

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief Work function that creates a map keyed with a vertex's label
//////////////////////////////////////////////////////////////////////
struct label_to_map
{
  using result_type = std::map<std::size_t, std::size_t>;

  template<typename Vertex>
  result_type operator()(Vertex&& v) const
  {
    result_type m;
    m[v.property().cc()] = 1;
    return m;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Reduction work function that merges two label maps to create
///        a count of the number of vertices that share a label
//////////////////////////////////////////////////////////////////////
struct merge_maps
{
  using result_type = std::map<std::size_t, std::size_t>;

  template<typename T, typename U>
  result_type operator()(T&& lhs, U&& rhs) const
  {
    result_type merged = lhs;
    result_type rhs_copy = rhs;

    for (auto&& kv : rhs_copy)
      merged[kv.first] += kv.second;

    return merged;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Run connected components on a disjointed complete graph and
///        verify that the answer is correct.
//////////////////////////////////////////////////////////////////////
bool test_connected_components(std::size_t component_size,
                               std::size_t num_components,
                               std::size_t k)
{
  using graph_type = stapl::digraph<stapl::properties::cc_property>;
  using view_type = stapl::graph_view<graph_type>;

  auto g = stapl::generators::make_disjointed_complete<view_type>(
    num_components, component_size);

  auto policy = stapl::sgl::make_execution_policy("kla", g, k);

  connected_components(policy, g);

  // Compute a map of label information. The map will be of the form
  //   label -> # of vertices sharing same label
  auto labels = stapl::map_reduce(label_to_map{}, merge_maps{}, g);

  // By definition of the disjointed complete graph, there should be exactly
  // `num_components` components
  const bool expected_num_components = labels.size() == num_components;

  // Every component should have exactly component_size members
  const bool expected_component_size
    = std::all_of(labels.begin(),
                  labels.end(),
                  [=](const std::pair<const std::size_t, std::size_t> label) {
                    return label.second == component_size;
                  });

  const bool passed = expected_num_components && expected_component_size;

  if (!passed)
    stapl::do_once([&](){
      std::cout << "Labels: " << std::endl;

      for (auto kv : labels)
        std::cout << kv.first << ": " << kv.second << " members\n";
    });

  return passed;
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout << "usage: " << argv[0] << " seed" << std::endl;
    exit(1);
  }

  constexpr std::size_t max_component_size = 512;
  constexpr std::size_t max_num_components = 16;
  constexpr std::size_t max_num_edges = 131072;
  constexpr std::size_t num_trials = 32;

  const std::size_t seed = atol(argv[1]);
  std::mt19937 gen{seed};

  stapl::do_once([=]() {
    std::cout << "Testing " << num_trials << " trials with seed " << seed
              << std::endl;
  });

  std::uniform_int_distribution<> component_size_dis(1, max_component_size);
  std::uniform_int_distribution<> num_components_dis(1, max_num_components);

  std::size_t trial = 0;
  while (trial < num_trials)
  {
    // Generate random parameters for graph
    const std::size_t component_size = component_size_dis(gen);
    const std::size_t num_components = num_components_dis(gen);

    // Don't use this configuration if it's too big
    if (component_size*component_size*num_components > max_num_edges)
      continue;

    // Select a random k
    std::uniform_int_distribution<> k_dis(0, component_size+1);
    const std::size_t k = k_dis(gen);

    if (!test_connected_components(component_size, num_components, k)) {
      stapl::do_once([=]() {
        std::cout << "Configuration failed: " << component_size << " x "
                  << num_components << " with k = " << k << std::endl;
      });

      return EXIT_FAILURE;
    }

    ++trial;
  }

  one_print(true);
  return EXIT_SUCCESS;
}
