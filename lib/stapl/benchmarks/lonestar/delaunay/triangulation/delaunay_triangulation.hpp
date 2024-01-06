/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_LONESTAR_DELAUNAY_TRIANGULATION_HPP
#define STAPL_BENCHMARK_LONESTAR_DELAUNAY_TRIANGULATION_HPP

#include <benchmarks/lonestar/delaunay/delaunay_mesh_view.hpp>

#include <benchmarks/lonestar/delaunay/triangulation/delaunay_set_points.hpp>
#include <benchmarks/lonestar/delaunay/triangulation/delaunay_triangulation_sequential.hpp>
#include <benchmarks/lonestar/delaunay/triangulation/delaunay_triangulation_merge_sequential.hpp>
#include <benchmarks/lonestar/delaunay/triangulation/delaunay_triangulation_merge_parallel.hpp>


namespace stapl
{


namespace delaunay
{


#ifndef graph_template
#define graph_template \
    template<graph_attributes,graph_attributes,typename ...OptionalParams> class
#endif


template <graph_template Graph>
struct delaunay_triangulation
{
  typedef delaunay_vertex                      vertex_property;
  typedef delaunay_reduced_edge<delaunay_edge> edge_property;

  typedef Graph<DIRECTED, MULTIEDGES, vertex_property, edge_property> mesh_type;
  typedef delaunay_mesh_view<mesh_type>                               view_type;

  set_points* m_points;
  view_type   m_mesh_view;
  print_stat  m_main_stat;
  print_stat* m_algo_stat;
  print_stat* m_tri_stat;
  counter<default_timer> m_algo_timer;
  double m_total_time;

  delaunay_triangulation()
    : m_main_stat("Total Time", true)
  { }

  ~delaunay_triangulation()
  {
    delete m_points;
  }

  void generate_random_points(size_t num_points, size_t seed)
  {
    print_formatted_label("Filling With Random Points");
    auto generation_stat = m_main_stat.add_stat("Generation Time");
    generation_stat->start();

    m_points = new set_points(num_points);
    m_points->fill_with_random_points(seed);

    auto time = generation_stat->stop();
    print_formatted_time(time, 2);
  }

  void sort_set_points()
  {
    print_formatted_label("Sorting");
    auto sorting_stat = m_algo_stat->add_stat("Sort Time");
    sorting_stat->start();

    m_points->sort_points();

    auto time = sorting_stat->stop();
    print_formatted_time(time);
  }

  void remove_duplicates()
  {
    print_formatted_label("Removing Duplicate Points");
    auto duplicate_stat = m_algo_stat->add_stat("Remove Duplicates Time");
    duplicate_stat->start();
    size_t num_points = m_points->size();
    m_points->remove_duplicates();

    auto time = duplicate_stat->stop();
    do_once([&]() {
      std::cout << num_points - m_points->size() << " points removed... ";
    });
    print_formatted_time(time);
  }

  void copy_point_set()
  {
    print_formatted_label("Copying Points");
    auto copy_stat = m_algo_stat->add_stat("Copy Time");
    copy_stat->start();

    m_points->copy_points(m_mesh_view);

    auto time = copy_stat->stop();
    print_formatted_time(time);
  }


  void triangulate(size_t num_points, size_t seed)
  {
    m_main_stat.start();

    generate_random_points(num_points, seed);

    print_formatted_label("Starting Algorithm", true);
    m_algo_stat = m_main_stat.add_marked_stat("Algorithm Time");
    m_algo_stat->start();

    sort_set_points();
    remove_duplicates();
    copy_point_set();
    locally_triangulate();

    auto time = m_algo_stat->stop();
    print_formatted_label("Algorithm Finished");
    print_formatted_time(time, 2);

    m_main_stat.stop();
    print_timing();

    generate_debug_info_wf wf1;
    collect_debug_info_wf  wf2;
    debug_info info = map_reduce(wf1, wf2, native_view(m_mesh_view));

    do_once([&]() { info.print(); });
  }

  void locally_triangulate()
  {
    print_formatted_label("Starting Triangulation");
    m_tri_stat = m_algo_stat->add_marked_stat("Triangulation Time");
    auto local_stat = m_tri_stat->add_stat("Local Triangulation Time");
    auto merge_stat = m_tri_stat->add_stat("Merge Triangulation Time");
    m_tri_stat->start();

    local_tri_wf<view_type> wf1(&m_mesh_view, local_stat->m_timer,
                                get_location_id(), 0, m_mesh_view.size()-1);
    merge_wf<view_type>     wf2(&m_mesh_view, merge_stat->m_timer);

    map_reduce(wf1, wf2, native_view(m_mesh_view));

    auto time = m_tri_stat->stop();
    print_formatted_time(time);
  }


  template <typename Label>
  void print_formatted_label(Label const& label, int newlines = false)
  {
    do_once([&]() {
      std::cout << label << "... ";
      while (newlines-- > 0) {
        std::cout << std::endl;
      }
      std::cout.flush();
    });
  }

  template <typename Time>
  void print_formatted_time(Time const& time, int newlines = 1)
  {
    do_once([&]() {
      boost::io::ios_flags_saver ifs(std::cout);
      std::cout.precision(8);
      std::cout << "[" << time << "s]";
      std::cout.precision(6);
      while (newlines-- > 0) {
        std::cout << std::endl;
      }
      std::cout.flush();
    });
  }

  void print_timing()
  {
    // Requires [this] instead [&] to fix lambda capture bug in gcc-4.7.2.
    do_once([this]() {
      print_stat::m_header_label= "Delaunay Triangulation";
      this->m_main_stat.print_main(this->m_total_time);
    });
  }
};

#undef graph_template


} // namespace delaunay


} // namespace stapl


#endif /* STAPL_BENCHMARK_LONESTAR_DELAUNAY_TRIANGULATION_HPP */
