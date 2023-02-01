/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/domains/indexed.hpp>

#ifndef TEST_UTIL_H
#define TEST_UTIL_H

inline void one_print(const char* s)
{
  stapl::rmi_fence();
  if (stapl::get_location_id() == 0)
    std::cout << s << std::flush;
  stapl::rmi_fence();
}

inline void one_print(bool b)
{
  stapl::rmi_fence();
  if (stapl::get_location_id() == 0) {
    if (b) std::cout << "\t\x1b[;32m[PASSED]\x1b[;0m" << std::endl;
    else   std::cout << "\t\x1b[;31m[FAILED]\x1b[;0m" << std::endl;
  }
  stapl::rmi_fence();
}

inline void stapl_print(const char* s)
{
  stapl::rmi_fence();
  if (stapl::get_location_id() == 0) {
    std::cout << s << std::flush;
  }
  stapl::rmi_fence();
}

inline void check_reset_error(bool& err)
{
  if (err)
    stapl_print("Failed\n");
  else
    stapl_print("Passed\n");
  err = false;
}

inline double compute_stats(std::string name, std::vector<double>& data)
{
  //The predicate can be different that m_pred here if we want
  //statistics on other fields of the counter; default will be on
  //time
  double avg, max,min,variance,stddev,confidenceinterval;
  size_t iterations = data.size();
  avg=0.0;
  min=max=data[0];
  for ( size_t i = 0; i < iterations; i++) {
    double d = data[i];
    avg += d;
    if ( d > max )
      max = d;
    if ( d < min )
      min = d;
  }
  avg /= iterations;
  variance=0.0;
  for ( size_t i = 0; i < iterations; i++) {
    double d = data[i] - avg;
    variance += d*d;
  }
  variance /= iterations;
  stddev = sqrt(variance);
  // 1.96 is z-table entry for 95%
  //The formula for min number of iteration is
  // n=[(1.96*stddev)/(useconfrange*avg)]^2
  //double nit = (1.96*stddev*1.96*stddev)/(0.05 * avg * 0.05 * avg);
  //std::cout<<"Min Number of iterations"<<nit<<"\n";
  confidenceinterval = (1.96*stddev) / sqrt((double) iterations);
  if (stapl::get_location_id()==0) {
    std::cout<<name<<"= "<<avg<<"\n";
    std::cout<<name<<"_min= "<<min<<"\n";
    std::cout<<name<<"_max= "<<max<<"\n";
    std::cout<<name<<"_conf= "<<confidenceinterval<<"\n\n";
  }
  return avg;
}

inline std::vector<std::vector<std::pair<size_t, size_t> > >
mesh_independent_sets(size_t const& x, size_t const& y)
{
  std::vector<std::vector<std::pair<size_t, size_t> > > is;

  typedef stapl::indexed_domain<size_t> domain_type;
  typedef stapl::balanced_partition<domain_type> partition_type;

  // group rows starting at 0
  std::vector<std::pair<size_t, size_t> > edges;
  domain_type row_dom(0, x-1);
  partition_type row_part(row_dom, x/2);
  for (size_t i = 0; i < row_part.size(); ++i) {
    domain_type row = row_part[i];
    for (size_t j = 0; j < y; ++j) {
      if (row.size() == 2)
        edges.push_back(std::make_pair(x*j + row.first(), x*j + row.last()));
    }
  }
  is.push_back(edges);
  //
  // group rows starting at 1
  edges.clear();
  row_dom = domain_type(1, x-1);
  row_part = partition_type(row_dom, x/2);
  for (size_t i = 0; i < row_part.size(); ++i) {
    domain_type row = row_part[i];
    for (size_t j = 0; j < y; ++j) {
      if (row.size() == 2)
        edges.push_back(std::make_pair(x*j + row.first(), x*j + row.last()));
    }
  }
  is.push_back(edges);
  //
  // group cols starting at 0
  edges.clear();
  domain_type col_dom(0, y-1);
  partition_type col_part(col_dom, y/2);
  for (size_t i = 0; i < col_part.size(); ++i) {
    domain_type col = col_part[i];
    for (size_t j = 0; j < x; ++j) {
      if (col.size() == 2)
        edges.push_back(std::make_pair(x*col.first() + j, x*col.last() + j));
    }
  }
  is.push_back(edges);
  // group cols starting at 1
  edges.clear();
  col_dom = domain_type(1, y-1);
  col_part = partition_type(col_dom, y/2);
  for (size_t i = 0; i < col_part.size(); ++i) {
    domain_type col = col_part[i];
    for (size_t j = 0; j < x; ++j) {
      if (col.size() == 2)
        edges.push_back(std::make_pair(x*col.first() + j, x*col.last() + j));
    }
  }
  is.push_back(edges);

#ifdef DEBUG_INFO
  size_t g = 0;
  for (typename std::vector<std::vector<std::pair<size_t, size_t> > >::iterator
          is_it = is.begin(); is_it != is.end(); ++is_it) {
    std::cout << "grouping " << g++ << std::endl;
    for (typename std::vector<std::pair<size_t, size_t> >::iterator
           it = (*is_it).begin(); it != (*is_it).end(); ++it) {
      std::cout << "(" << (*it).first << ", " << (*it).second << ") ";
    }
    std::cout << std::endl;
  }
#endif
  return is;
}


//////////////////////////////////////////////////////////////////////
/// @brief Work function to compare two vertices in terms of their edges.
//////////////////////////////////////////////////////////////////////
struct vertex_similarity_wf
{
  using result_type = bool;

  template<typename V1, typename V2>
  result_type operator() (V1 v1, V2 v2)
  {
    using edge_type = typename V1::adj_edge_iterator::value_type;
    using v2_edge_type = typename V2::adj_edge_iterator::value_type;

    const auto aei2 = v2.begin();
    const auto aei2_end = v2.end();

    const bool b = std::all_of(v1.begin(), v1.end(), [&](edge_type const& e) {
      return graph_find(aei2, aei2_end,
        stapl::eq_target<size_t>(e.target())) != aei2_end;
    });

    const bool b2 = std::all_of(v2.begin(), v2.end(),
      [&](v2_edge_type const& e) {
        return graph_find(v1.begin(), v1.end(),
          stapl::eq_target<size_t>(e.target())) != v1.end();
      }
    );

    return b && b2;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Compare two graphs. Two graphs are considered equal if they
/// have the same number of vertices / edges and for each vertex with
/// the same ID, they have the same adjacency list.
///
/// @return Whether the input graphs are the same
//////////////////////////////////////////////////////////////////////
template<typename ViewA, typename ViewB>
bool compare_graphs(ViewA v1, ViewB v2)
{
  bool passed = (v1.num_vertices() == v2.num_vertices()) &&
      (v1.num_edges() == v2.num_edges());

  passed &= stapl::map_reduce(
    vertex_similarity_wf(), stapl::logical_and<bool>(), v1, v2
  );

  return passed;
}

// generic property with color for use in tests.
class my_vertex_property
{
 public:
  typedef size_t color_value_type;
 private:
  color_value_type   m_color;

 public:
  my_vertex_property(size_t i = 999)
    : m_color(i)
  { }

  my_vertex_property(my_vertex_property const& other) = default;

  color_value_type get_color() const
  { return m_color; }

  void set_color(color_value_type const& c)
  { m_color = c; }

  void define_type(stapl::typer& t)
  { t.member(m_color); }

  friend std::ostream& operator<< (std::ostream& stream,
                                   my_vertex_property const& v)
  {
    stream << v.get_color();
    return stream;
  }
};

namespace stapl {
template <typename Accessor>
class proxy<my_vertex_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef my_vertex_property target_t;

public:
  typedef size_t value_type;

  explicit proxy(Accessor const& acc)
  : Accessor(acc)
  { }

  operator target_t() const
  { return Accessor::read(); }

  proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}

  size_t get_color() const
  { return Accessor::const_invoke(&target_t::get_color); }

  void set_color(size_t const& c)
  { Accessor::invoke(&target_t::set_color, c); }

  friend std::ostream& operator<< (std::ostream& stream,
                                   proxy<my_vertex_property, Accessor> const& v)
  {
    stream << v.get_color();
    return stream;
  }
}; //struct proxy
}

#endif
