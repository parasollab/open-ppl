/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_BENCHMARK_LONESTAR_DELAUNAY_TRIANGULATION_MERGE_SEQUENTIAL_HPP
#define STAPL_BENCHMARK_LONESTAR_DELAUNAY_TRIANGULATION_MERGE_SEQUENTIAL_HPP

#include <benchmarks/lonestar/delaunay/delaunay_mesh_view.hpp>


namespace stapl
{


namespace delaunay
{


///////////////////////////////////////////////////////////////////////////////
/// @brief Main Merge workfunction that merges two partitions together.
///////////////////////////////////////////////////////////////////////////////
template <typename View>
struct seq_merge_wf
{
  typedef seq_merge_wf<View> this_type;
  typedef std::pair<delaunay_edge, delaunay_edge> edge_pair;
  typedef edge_pair result_type;

  typedef typename View::vertex_iterator   VIter;
  typedef typename View::adj_edge_iterator EIter;

  View*  m_view;
  size_t m_dom_low;
  size_t m_dom_mid;
  size_t m_dom_high;

  seq_merge_wf(View* view)
    : m_view(view)
  { }

  seq_merge_wf(View* view, size_t dom_low, size_t dom_high)
    : m_view(view), m_dom_low(dom_low), m_dom_mid((dom_high + dom_low) / 2),
      m_dom_high(dom_high)
  { }

  seq_merge_wf(View* view, size_t dom_low, size_t dom_mid, size_t dom_high)
    : m_view(view), m_dom_low(dom_low), m_dom_mid(dom_mid), m_dom_high(dom_high)
  { }

  template <typename WF>
  seq_merge_wf(View* view, WF&& wf)
    : m_view(view), m_dom_low(wf.m_dom_low),
      m_dom_mid(wf.m_dom_mid), m_dom_high(wf.m_dom_high)
  { }

  bool check_crosses_boundary(delaunay_edge const& basel)
  {
    return (basel.source() <= m_dom_mid || basel.target() <= m_dom_mid) &&
           (basel.source() >  m_dom_mid || basel.target() >  m_dom_mid);
  }

  bool check_right_to_left(delaunay_edge const& basel)
  {
    return basel.source() > basel.target();
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Finds the lower common tangent of L and R.
  /////////////////////////////////////////////////////////////////////////////
  template <typename Pair>
  void update_inner_edges(Pair& left, Pair& right)
  {
    while (true) {
      if (left.second.left_of(right.first.get_Org())) {
        left.second = left.second.get_Lnext(*m_view);
      } else if (right.first.right_of(left.second.get_Org())) {
        right.first = right.first.get_Rprev(*m_view);
      } else {
        break;
      }
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Updates the outer edges that correspond to the left-most and
  /// right-most edges of the partitions.
  /////////////////////////////////////////////////////////////////////////////
  template <typename Pair>
  void update_outer_edges(Pair& left, Pair& right,
                          delaunay_edge const& basel)
  {
    if (left.second.source() == left.first.source()) {
      left.first = basel.get_Sym();
    }
    if (right.first.source() == right.second.source()) {
      right.second = basel;
    }
    stapl_assert(left.first.source()   == m_dom_low,  "m_ldo not left most");
    stapl_assert(right.second.source() == m_dom_high, "m_rdo not right most");
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Connects the basel edge and provides error checking.
  /// The basel edge much cross from the right partition to the left partition.
  /////////////////////////////////////////////////////////////////////////////
  delaunay_edge connect_basel_edge(delaunay_edge const& edge1,
                                   delaunay_edge const& edge2)
  {
    auto basel = m_view->connect(edge1, edge2);
    stapl_assert(check_crosses_boundary(basel), "Basel doesn't cross boundary");
    stapl_assert(check_right_to_left(basel),    "Basel points wrong direction");
    return basel;
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Connects a new basel edge between the candidates if at least one
  /// is valid. If both are invalid, returns false indicating the merge process
  /// has completed. If both candidates are valid the circle test is applied
  /// to determine which candidate to use.
  /////////////////////////////////////////////////////////////////////////////
  bool connect_basel_edge(delaunay_edge& basel,
                          delaunay_edge const& lcand,
                          delaunay_edge const& rcand)
  {
    if (!basel.valid(lcand) && !basel.valid(rcand)) {
      return false;
    } else if (!basel.valid(lcand) ||
        (basel.valid(rcand) && in_circle(lcand.get_Dest(), lcand.get_Org(),
                                         rcand.get_Org(), rcand.get_Dest()))) {
      basel = connect_basel_edge(rcand, basel.get_Sym());
    } else {
      basel = connect_basel_edge(basel.get_Sym(), lcand.get_Sym());
    }
    return true;
  }

  result_type merge(result_type left, result_type right) {
    update_inner_edges(left, right);
    auto basel = connect_basel_edge(right.first.get_Sym(), left.second);
    update_outer_edges(left, right, basel);

    while (true) {
      auto lcand = find_lcand_wf(*this, basel)();
      auto rcand = find_rcand_wf(*this, basel)();

      if (!connect_basel_edge(basel, lcand, rcand)) {
        break;
      }
    }

    return result_type(left.first, right.second);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_view);
    t.member(m_dom_low);
    t.member(m_dom_mid);
    t.member(m_dom_high);
  }


  /////////////////////////////////////////////////////////////////////////////
  /// @brief Base class for finding either left or right candidate.
  /////////////////////////////////////////////////////////////////////////////
  template <int DIR>
  struct find_cand_wf
  {
    typedef std::pair<delaunay_edge, delaunay_edge> edge_pair;
    typedef delaunay_edge result_type;

    View* m_view;
    delaunay_edge m_basel;
    size_t m_dom_low;
    size_t m_dom_mid;
    size_t m_dom_high;

    find_cand_wf(View* view, delaunay_edge basel, size_t low, size_t high)
      : m_view(view), m_basel(basel), m_dom_low(low), m_dom_mid(m_dom_mid),
        m_dom_high(high)
    { }

    find_cand_wf(seq_merge_wf<View>& wf, delaunay_edge basel)
      : m_view(wf.m_view), m_basel(basel), m_dom_low(wf.m_dom_low),
        m_dom_mid(wf.m_dom_mid), m_dom_high(wf.m_dom_high)
    { }

    template <typename WF>
    find_cand_wf(View* view, WF&& wf)
      : m_view(view), m_basel(wf.m_basel), m_dom_low(wf.m_dom_low),
        m_dom_mid(wf.m_dom_mid), m_dom_high(wf.m_dom_high)
    { }

    size_t loc() const
    {
      if (DIR < 0) {
        return this->m_view->vertex_location(this->m_basel.target());
      } else {
        return this->m_view->vertex_location(this->m_basel.source());
      }
    }

    bool is_local() const
    { return (loc() == m_view->get_location_id()); }

    stapl::future<delaunay_edge> async()
    { return m_view->async_command(loc(), *this); }

    stapl::future<delaunay_edge> async(size_t loc)
    { return m_view->async_command(loc, *this); }

    delaunay_edge sync()
    { return m_view->sync_command(loc(), *this); }

    delaunay_edge sync(size_t loc)
    { return m_view->sync_command(loc, *this); }

    bool cand_in_circle(delaunay_edge const& cand,
                        delaunay_edge const& onp_edge) const
    {
      return in_circle(m_basel.m_target_point,
                       m_basel.m_source_point,
                       cand.m_target_point,
                       onp_edge.m_target_point);
    }

    template <typename VertexIter, typename EdgeIter>
    bool find_edge(delaunay_edge& cand, VertexIter& vit, EdgeIter& e_it) const
    {
      if (m_basel.valid(cand)) {
        vit = m_view->find_vertex(cand.source());
        auto v = *(vit);
        if (v.size() > 1) {
          auto const edge_target = cand.target();
          auto const e_ite = v.end();

          for (e_it = v.begin(); e_it != e_ite; ++e_it) {
            if ((*e_it).target() == edge_target) {
              return true;
            }
          }
        }
      }
      return false;
    }

    bool valid_cand_test(delaunay_edge& cand)
    {
      bool test;
      if (DIR < 0) {
        test = (cand.source() <= m_dom_high && cand.target() <= m_dom_high);
      } else {
        test = (cand.source() >= m_dom_low && cand.target() >= m_dom_low);
      }

      // Might be able to recover by simply reversing edge
      stapl_assert(test, "cand on wrong side");
      if (!test) {
        cand.reverse();
      }

      return test;
    }

    ///////////////////////////////////////////////////////////////////////////
    /// @brief Loops ccw/cw over the edges on the left/right vertex of the basel
    /// edge, deleting invalid edges in the process. Breaks when either a valid
    /// candidate is found or deleting an edge would cause a discontinuity in
    /// the graph.
    ///////////////////////////////////////////////////////////////////////////
    delaunay_edge operator()()
    {
      auto cand = (DIR < 0) ? this->m_basel.get_Sym().get_Onext(*m_view)
                            : this->m_basel.get_Oprev(*m_view);


      valid_cand_test(cand);

      VIter vit;
      EIter e_it_cand;

      if (!this->find_edge(cand, vit, e_it_cand)) {
        return cand;
      }

      auto v = (*vit);
      size_t size = v.size();

      while ((size--) > 1) {
        EIter e_it;
        bool test;

        // Get next edge (wrapping around edgelist if necessary)
        if (DIR < 0) {
          test = (e_it_cand != v.begin());
          e_it = test ? (e_it_cand + DIR) : (v.end()-1);
        } else {
          test = (e_it_cand != v.end()-1);
          e_it = test ? (e_it_cand + DIR) : (v.begin());
        }

        delaunay_edge temp_edge(v, (*e_it));
        if (!this->cand_in_circle(cand, temp_edge)) {
          return cand;
        }

        m_view->delete_delaunay_edge(cand);

        // Set e_it_cand to next edge (handles deletion of vertex)
        if (DIR < 0) {
          e_it_cand = test ? e_it : v.end() - 1;
        } else if (!test) {
          e_it_cand = v.begin();
        }

        cand = temp_edge;
        valid_cand_test(cand);

      }
      return cand;
    }

    void define_type(stapl::typer& t)
    {
      t.member(m_view);
      t.member(m_basel);
      t.member(m_dom_low);
      t.member(m_dom_mid);
      t.member(m_dom_high);
    }
  };

  typedef find_cand_wf<-1> find_lcand_wf;
  typedef find_cand_wf<+1> find_rcand_wf;
};


} // namespace delaunay


} // namespace stapl


#endif /* STAPL_BENCHMARK_LONESTAR_DELAUNAY_TRIANGULATION_MERGE_SEQUENTIAL_HPP*/
