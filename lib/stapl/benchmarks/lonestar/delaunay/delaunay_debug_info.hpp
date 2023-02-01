/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_LONESTAR_DELAUNAY_DEBUG_INFO_HPP
#define STAPL_BENCHMARK_LONESTAR_DELAUNAY_DEBUG_INFO_HPP

#include <benchmarks/lonestar/delaunay/delaunay_utilities.hpp>
#include <benchmarks/lonestar/delaunay/delaunay_printer.hpp>


namespace stapl
{


namespace delaunay
{


///////////////////////////////////////////////////////////////////////////////
/// @brief Debugging class used to store to check validity of triangulation.
///////////////////////////////////////////////////////////////////////////////
struct debug_info
{
  size_t total_vertices;
  size_t total_edges;
  size_t min_degree;
  size_t max_degree;
  double average_degree;
  size_t failed_vertices;
  size_t passed_edges;
  size_t failed_edges;

  debug_info()
    : total_vertices(0), total_edges(0),
      min_degree(10000000), max_degree(0), average_degree(0),
      failed_vertices(0), passed_edges(0), failed_edges(0)
  { }

  template <typename String, typename Value>
  int print(String const& str, Value const& value, bool assertion = true)
  {
    if (assertion) {
      std::cout << str << value  << std::endl;
      return 0;
    } else {
      std::cout << "\e[1;40;31m" << str << value << "\e[0m" << std::endl;
      return 1;
    }
  }

  void print()
  {
    average_degree = double(total_edges) / double(total_vertices);
    size_t max_edges = 2*(3*total_vertices-6);
    int test = 0;
    test += print("   Total Vertices: ", total_vertices);
    test += print(" Invalid Vertices: ", failed_vertices, failed_vertices == 0);
    test += print("      Total Edges: ", total_edges, total_edges <= max_edges);
    test += print("      Valid Edges: ", passed_edges);
    test += print("    Invalid Edges: ", failed_edges, failed_edges == 0);
    test += print("       Min Degree: ", min_degree, min_degree > 0);
    test += print("       Max Degree: ", max_degree);
    test += print("   Average Degree: ", average_degree, average_degree >= 3);

    std::cout << std::endl;

    if (test == 0) {
      print("PASSED ALL TESTS", "", test == 0);
    } else {
      print("FAILED TEST COUNT: ", test, test == 0);
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(total_vertices);
    t.member(total_edges);
    t.member(min_degree);
    t.member(max_degree);
    t.member(average_degree);
    t.member(failed_vertices);
    t.member(passed_edges);
    t.member(failed_edges);
  }
};


///////////////////////////////////////////////////////////////////////////////
/// @brief Workfunction to generate debugging data.
///////////////////////////////////////////////////////////////////////////////
struct generate_debug_info_wf
{
  typedef debug_info result_type;

  template <typename Vertex, typename EdgeIter>
  void circle_test(debug_info& info, Vertex v,
    EdgeIter edgeA, EdgeIter edgeB,
                                     EdgeIter edgeC, EdgeIter edgeD)
  {
    auto B = v.property().make_edge(v, (*edgeA)).get_Dest();
    auto Y = v.property().make_edge(v, (*edgeB)).get_Org();
    auto X = v.property().make_edge(v, (*edgeC)).get_Dest();
    auto A = v.property().make_edge(v, (*edgeD)).get_Dest();
    if (in_circle(A, X, Y, B)) {
      ++info.failed_edges;
    } else {
      ++info.passed_edges;
    }
  }

  template <typename NView>
  debug_info operator()(NView nview)
  {
    auto& domain = nview.domain();
    auto dom_low = domain.first();
    auto dom_high = domain.last();
    debug_info info;
    info.total_vertices += dom_high - dom_low + 1;

    for (auto i = dom_low; i <= dom_high; ++i) {
      auto v = nview[i];
      nview.sort_vertex_edges(i);
      info.total_edges += v.size();
      if (v.size() > info.max_degree) {
        info.max_degree = v.size();
      }
      if (v.size() < info.min_degree) {
        info.min_degree = v.size();
      }
      if (v.size() >= 3) {
        auto eit   = v.begin();
        auto eit_e = v.end() - 2;

        circle_test(info, v, eit_e, eit_e+1, eit_e+1, eit);
        circle_test(info, v, eit_e+1, eit, eit, eit+1);
        for (; eit != eit_e; ++eit) {
          circle_test(info, v, eit, eit+1, eit+1, eit+2);
        }
      } else if (v.size() == 2) {
        info.passed_edges += 2;
      } else if (v.size() == 1) {
        ++info.failed_vertices;
        ++info.failed_edges;
      } else if (v.size() == 0) {
        ++info.failed_vertices;
      }
    }
    return info;
  }
};


///////////////////////////////////////////////////////////////////////////////
/// @brief Workfunction to combine debugging data across processes.
///////////////////////////////////////////////////////////////////////////////
struct collect_debug_info_wf
{
  typedef debug_info result_type;

  debug_info operator()(debug_info info1, debug_info info2)
  {
    if (info2.max_degree > info1.max_degree) {
      info1.max_degree = info2.max_degree;
    }
    if (info2.min_degree < info1.min_degree) {
      info1.min_degree = info2.min_degree;
    }

    info1.total_edges     += info2.total_edges;
    info1.total_vertices  += info2.total_vertices;
    info1.failed_vertices += info2.failed_vertices;
    info1.passed_edges    += info2.passed_edges;
    info1.failed_edges    += info2.failed_edges;

    return info1;
  }

};


} // namespace delaunay


} // namespace stapl


#endif /* STAPL_BENCHMARK_LONESTAR_DELAUNAY_DEBUG_INFO_HPP */
