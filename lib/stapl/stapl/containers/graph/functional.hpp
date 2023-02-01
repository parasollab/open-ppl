/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_FUNCTIONAL_HPP
#define STAPL_CONTAINERS_GRAPH_FUNCTIONAL_HPP

#include <stapl/views/proxy/as_reference.hpp>
#include <boost/mpl/bool.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Class for assigning value to a reference.
///
/// @tparam T Value type.
///
/// @ingroup pgraphDistObj
///
/// @p T must be default assignable, copyable and assignable.
///
/// @see property_accessor
//////////////////////////////////////////////////////////////////////
template<typename T>
struct assign_val
{
  typedef T                  first_argument_type;
  typedef void               result_type;

private:
  first_argument_type m_val;

public:
  explicit assign_val(T const& val)
    : m_val(val)
  { }

  explicit assign_val(T&& val)
    : m_val(std::move(val))
  { }

  explicit assign_val() = default;

  template<typename Reference>
  void operator()(Reference& y) const
  {
    y = m_val;
  }

  void define_type(typer& t)
  {
    t.member(m_val);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Class for extracting a property from a vertex reference and applying
///        the functor to it.
///
/// @tparam Function Function object type.
///
/// @ingroup pgraphDistObj
///
/// Used by @c vp_apply() and @c vp_apply_async() for vertices.
//////////////////////////////////////////////////////////////////////
template<typename Function>
class extract_property_apply
: private Function
{
public:
  typedef typename Function::result_type result_type;

  explicit extract_property_apply() = default;

  template<typename F>
  explicit extract_property_apply(F const& f)
    : Function(f)
  { }

  template<typename V>
  result_type operator()(V&& v) const
  {
    return static_cast<Function const&>(*this)(std::forward<V>(v).property());
  }

  template<typename V>
  result_type operator()(V&& v)
  {
    return static_cast<Function&>(*this)(std::forward<V>(v).property());
  }

  void define_type(typer& t)
  {
    t.base<Function>(*this);
  }
};

template<typename Function>
class extract_property_apply_set
: private Function
{
public:
  typedef typename Function::result_type result_type;

  explicit extract_property_apply_set() = default;

  template<typename F>
  explicit extract_property_apply_set(F const& f)
    : Function(f)
  { }

  template<typename V>
  result_type operator()(V&& v) const
  {
    auto& ref = as_reference(std::forward<V>(v));

    return static_cast<Function const&>(*this)(ref.property());
  }

  template<typename V>
  result_type operator()(V&& v)
  {
    auto& ref = as_reference(std::forward<V>(v));

    return static_cast<Function&>(*this)(std::forward<V>(v).property());
  }

  void define_type(typer& t)
  {
    t.base<Function>(*this);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Comparator for sorting edges based on target's home location.
/// @tparam Graph Type of the graph whose edges are to be sorted.
//////////////////////////////////////////////////////////////////////
template<typename Graph>
struct edge_target_location_comp
{
  Graph* m_g;

  explicit edge_target_location_comp(Graph* g)
    : m_g(g)
  { }

  template<typename Edge1, typename Edge2>
  bool operator()(Edge1 const& e1, Edge2 const& e2) const
  {
    return m_g->locality(e1.target()).location()
           < m_g->locality(e2.target()).location();
  }

  void define_type(typer& t)
  {
    t.member(m_g);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Comparator for testing < of two edge targets
//////////////////////////////////////////////////////////////////////
struct edge_target_comp
{
  template <class Edge1, class Edge2>
  bool operator()(Edge1 const& e1, Edge2 const& e2)
  {
    return e1.target() < e2.target();
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Comparator for testing if two edges point to the same target
//////////////////////////////////////////////////////////////////////
struct edge_target_equal
{
  template <class Edge1, class Edge2>
  bool operator()(Edge1 const& e1, Edge2 const& e2)
  {
    return e1.target() == e2.target();
  }
};

template<typename Property>
struct extract_property
{
  using result_type = std::size_t;

  template<typename Vertex>
  result_type operator()(Vertex&& v) const
  {
    return v.property();
  }
};

} // namespace detail

} // stapl namespace

#endif /* STAPL_CONTAINERS_GRAPH_FUNCTIONAL_HPP */
