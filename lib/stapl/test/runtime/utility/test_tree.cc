/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#define STAPL_RUNTIME_TEST_MODULE trees
#include "utility.h"
#include <stapl/runtime/utility/tree.hpp>
#include <algorithm>
#include <vector>

const int N = 32;

using namespace stapl::runtime;

BOOST_AUTO_TEST_CASE( test_flat_tree )
{
  for (int n=1; n<=N; ++n) {
    bool root_found = false;
    std::vector<int> parents(n, -1);

    int count = 0;

    for (int id = 0; id < n; ++id) {
      // generate topology
      const auto r        = make_flat_tree(id, n);
      const auto root     = std::get<0>(r);
      const auto parent   = std::get<1>(r);
      const auto children = std::get<2>(r);

      // check if it is root
      if (id==root) {
        // make sure we have only one root
        BOOST_REQUIRE_EQUAL(root_found, false);
        root_found = true;

        // make sure that the root's parent is the root
        BOOST_REQUIRE_EQUAL(root, parent);

        // count root
        ++count;
      }

      // check if a parent is set if it is the same as the expected
      if (parents[id]!=-1) {
        BOOST_REQUIRE_EQUAL(parents[id], parent);
      }
      else {
        parents[id] = parent;
      }

      // check children
      for (auto const& v : children) {
        // child should not be above total number of children
        BOOST_REQUIRE(v<n);

        // child cannot be root
        BOOST_REQUIRE(v!=root);

        // child cannot be parent
        BOOST_REQUIRE(v!=parent);

        // check if a parent is set if it is the same as the expected
        if (parents[v]!=-1) {
          BOOST_REQUIRE_EQUAL(parents[v], id);
        }
        else {
          parents[v] = id;
        }

        // count child
        ++count;
      }
    }

    // make sure we have one root
    BOOST_REQUIRE_EQUAL(root_found, true);

    // make sure we have all the nodes
    BOOST_REQUIRE_EQUAL(count, n);

    // make sure every node has a parent
    BOOST_REQUIRE(std::find(parents.begin(), parents.end(), -1)==parents.end());
  }
}


BOOST_AUTO_TEST_CASE( test_binary_tree )
{
  for (int n=1; n<=N; ++n) {
    bool root_found = false;
    std::vector<int> parents(n, -1);

    int count = 0;

    for (int id = 0; id < n; ++id) {
      // generate topology
      const auto r        = make_binary_tree(id, n);
      const auto root     = std::get<0>(r);
      const auto parent   = std::get<1>(r);
      const auto children = std::get<2>(r);

      // check if it is root
      if (id==root) {
        // make sure we have only one root
        BOOST_REQUIRE_EQUAL(root_found, false);
        root_found = true;

        // make sure that the root's parent is the root
        BOOST_REQUIRE_EQUAL(root, parent);

        // count root
        ++count;
      }

      // check if a parent is set if it is the same as the expected
      if (parents[id]!=-1) {
        BOOST_REQUIRE_EQUAL(parents[id], parent);
      }
      else {
        parents[id] = parent;
      }

      // check children
      for (auto const& v : children) {
        // child should not be above total number of children
        BOOST_REQUIRE(v<n);

        // child cannot be root
        BOOST_REQUIRE(v!=root);

        // child cannot be parent
        BOOST_REQUIRE(v!=parent);

        // check if a parent is set if it is the same as the expected
        if (parents[v]!=-1) {
          BOOST_REQUIRE_EQUAL(parents[v], id);
        }
        else {
          parents[v] = id;
        }

        // count child
        ++count;
      }
    }

    // make sure we have one root
    BOOST_REQUIRE_EQUAL(root_found, true);

    // make sure we have all the nodes
    BOOST_REQUIRE_EQUAL(count, n);

    // make sure every node has a parent
    BOOST_REQUIRE(std::find(parents.begin(), parents.end(), -1)==parents.end());
  }
}


BOOST_AUTO_TEST_CASE( test_binomial_tree )
{
  for (int n=1; n<=N; ++n) {
    bool root_found = false;
    std::vector<int> parents(n, -1);

    std::cout << "-----> " << N << std::endl;

    int count = 0;

    for (int id = 0; id < n; ++id) {
      // generate topology
      const auto r        = make_binomial_tree(id, n);
      const auto root     = std::get<0>(r);
      const auto parent   = std::get<1>(r);
      const auto children = std::get<2>(r);

      std::cout << id << ": (" << root << "," << parent << ") ( ";
      for (auto&& v : children)
        std::cout << v << ' ';
      std::cout << ')' << std::endl;

      // check if it is root
      if (id==root) {
        // make sure we have only one root
        BOOST_REQUIRE_EQUAL(root_found, false);
        root_found = true;

        // make sure that the root's parent is the root
        BOOST_REQUIRE_EQUAL(root, parent);

        // count root
        ++count;
      }

      // check if a parent is set if it is the same as the expected
      if (parents[id]!=-1) {
        BOOST_REQUIRE_EQUAL(parents[id], parent);
      }
      else {
        parents[id] = parent;
      }

      // check children
      for (auto const& v : children) {
        // child should not be above total number of children
        BOOST_REQUIRE(v<n);

        // child cannot be root
        BOOST_REQUIRE(v!=root);

        // child cannot be parent
        BOOST_REQUIRE(v!=parent);

        // check if a parent is set if it is the same as the expected
        if (parents[v]!=-1) {
          BOOST_REQUIRE_EQUAL(parents[v], id);
        }
        else {
          parents[v] = id;
        }

        // count child
        ++count;
      }
    }

    // make sure we have one root
    BOOST_REQUIRE_EQUAL(root_found, true);

    // make sure we have all the nodes
    BOOST_REQUIRE_EQUAL(count, n);

    // make sure every node has a parent
    BOOST_REQUIRE(std::find(parents.begin(), parents.end(), -1)==parents.end());
  }
}
