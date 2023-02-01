/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PROFILING_GRAPH_HPP
#define STAPL_PROFILING_GRAPH_HPP

namespace stapl {

namespace profiling {

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the add_vertex() function of the graph container.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer> >
class add_vertex_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;

public:
  add_vertex_profiler(std::string name, ADT* adt, size_t nt,
                      int argc=0, char **argv=nullptr)
    : base_type(adt, name+"::add_vertex", nt, argc, argv)
  { }

  void run()
  {
    for (size_t i=0; i < this->m_test_size; ++i)
      this->m_adt->add_vertex();
    rmi_fence();
  }

  void finalize_iteration()
  {
    this->m_adt->clear();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the add_edge() function of the graph container.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer> >
class add_edge_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using vd_type = typename ADT::vertex_descriptor;
  using vedges_type = std::vector<std::pair<vd_type,vd_type> >;

  /// Iterators for the input container
  // typename vedges_type::iterator it_begin, it_end;
  typename std::vector<std::pair<vd_type,vd_type> >::iterator it_begin, it_end;

public:
  add_edge_profiler(std::string name, ADT* adt, size_t nt,
                    vedges_type /*const*/& edges,
                    int argc=0, char **argv=nullptr)
    : base_type(adt, name+"::add_edge", nt, argc, argv),
      it_begin(edges.begin()), it_end(edges.end())
  { }

  void initialize_iteration()
  {
    this->m_adt->clear();
    for (size_t i=0; i < this->m_test_size; ++i)
      this->m_adt->add_vertex();
    rmi_fence();
  }

  void run()
  {
    for (auto it = it_begin; it != it_end; ++it)
      this->m_adt->add_edge(it->first, it->second);
    rmi_fence();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the add_edge_async() function of the graph container.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer> >
class add_edge_async_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using vd_type = typename ADT::vertex_descriptor;
  using ed_type = typename ADT::edge_descriptor  ;
  using vedges_type = std::vector<std::pair<vd_type,vd_type> >;

  /// Iterators for the input container
  typename vedges_type::iterator it_begin, it_end;

public:
  add_edge_async_profiler(std::string name, ADT* adt, size_t nt,
                          vedges_type /*const*/& edges,
                          int argc=0, char **argv=nullptr)
    : base_type(adt, name+"::add_edge_async", nt, argc, argv),
      it_begin(edges.begin()), it_end(edges.end())
  { }

  void initialize_iteration()
  {
    this->m_adt->clear();
    for (size_t i=0; i < this->m_test_size; ++i)
      this->m_adt->add_vertex();
    rmi_fence();
  }

  void run()
  {
    for (auto it = it_begin; it != it_end; ++it)
      this->m_adt->add_edge_async(ed_type(it->first, it->second));
    rmi_fence();
  }

  void finalize_iteration()
  {
    this->m_adt->clear();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the delete_edge() function of the graph container.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer> >
class delete_edge_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using vd_type = typename ADT::vertex_descriptor;
  using ed_type = typename ADT::edge_descriptor;
  using vedges_type = std::vector<std::pair<vd_type,vd_type> >;

  /// Iterators for the input container
  typename vedges_type::iterator it_begin, it_end;

public:
  delete_edge_profiler(std::string name, ADT* adt, size_t nt,
                       vedges_type /*const*/& edges,
                       int argc=0, char **argv=nullptr)
    : base_type(adt, name+"::delete_edge", nt, argc, argv),
      it_begin(edges.begin()), it_end(edges.end())
  {  }

  void initialize_iteration()
  {
    for (auto it = it_begin; it != it_end; ++it)
      this->m_adt->add_edge(ed_type(it->first, it->second));
    rmi_fence();
  }

  void run()
  {
    for (auto it = it_begin; it != it_end; ++it)
      this->m_adt->delete_edge(it->first, it->second);
    rmi_fence();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the find_vertex() function of the graph container.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer> >
class find_vertex_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using vd_type = typename ADT::vertex_descriptor;
  using verts_type = std::vector<vd_type>;

  /// Iterators for the input container
  typename verts_type::iterator  it_begin, it_end;

  /// Iterators for the target container
  typename ADT::vertex_iterator   vit, vit_end;

public:
  find_vertex_profiler(std::string name, ADT* adt, size_t nt, verts_type& v,
                      int argc=0, char **argv=nullptr)
    : base_type(adt, name+"::find_vertex", nt, argc, argv),
      it_begin(v.begin()), it_end(v.end()), vit_end(adt->end())
  { }

  void run()
  {
    for (auto it = it_begin; it != it_end; ++it) {
      vit = this->m_adt->find_vertex(*it);
      stapl_assert(vit != vit_end, "Failed find vertex");
    }
    rmi_fence();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the find_edge() function of the graph container.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer> >
class find_edge_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using ed_type = typename ADT::edge_descriptor;
  using vd_type = typename ADT::vertex_descriptor;
  using vedges_type = std::vector<std::pair<vd_type,vd_type> >;

  /// Iterators for the input container
  typename vedges_type::iterator it_begin, it_end;

  /// Iterator to the vertices of the container
  typename ADT::vertex_iterator   vit;

  /// Iterator to the edges of the container
  typename ADT::adj_edge_iterator eit;

public:
  find_edge_profiler(std::string name, ADT* adt, size_t nt, vedges_type& v,
                     int argc=0, char **argv=nullptr)
    : base_type(adt, name+"::find_edge", nt, argc, argv),
      it_begin(v.begin()), it_end(v.end())
  { }

  void run()
  {
    for (auto it = it_begin; it != it_end; ++it) {
#ifndef STAPL_NDEBUG
      bool b = this->m_adt->find_edge(ed_type(it->first, it->second),vit,eit);
      stapl_assert(b, "Failed find edge");
#else
      this->m_adt->find_edge(ed_type(it->first, it->second),vit,eit);
#endif
    }
    rmi_fence();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the delete_vertex() function of the graph container.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer> >
class delete_vertex_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using vd_type = typename ADT::vertex_descriptor;
  using verts_type = std::vector<vd_type>;

  /// Iterators for the input container
  typename verts_type::iterator  it_begin, it_end;

public:
  delete_vertex_profiler(std::string name, ADT* adt, size_t nt, verts_type& v,
                         int argc=0, char **argv=nullptr)
    : base_type(adt, name+"::delete_vertex", nt, argc, argv),
      it_begin(v.begin()), it_end(v.end())
  { }

  void run()
  {
    for (auto it = it_begin; it != it_end; ++it)
    {
      this->m_adt->delete_vertex(*it);
    }
    rmi_fence();
  }
};

} // namespace profiling

} // namespace stapl

#endif // STAPL_PROFILING_GRAPH_HPP
