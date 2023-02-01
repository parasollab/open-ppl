/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_UNDIRECTED_VIEW_HPP
#define STAPL_CONTAINERS_GRAPH_UNDIRECTED_VIEW_HPP

#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/paragraph/paragraph.hpp>
#include <stapl/views/core_view.hpp>
#include <stapl/views/mapping_functions/mapping_functions.hpp>
#include <stapl/utility/use_default.hpp>

namespace stapl {

namespace graph_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Work-function that adds bidirectional edges to the output graph
/// for each directed edge in the input vertex.
//////////////////////////////////////////////////////////////////////
struct global_edge_copy
{
  typedef void result_type;

  template<typename Vert, typename View>
  void operator()(Vert v, View vw)
  {
    typedef typename Vert::adj_edge_iterator edge_iterator;

    for (edge_iterator it = v.begin(); it != v.end(); ++it)
    {
      /// @fixme Get the edge property.
      vw.add_edge_async(typename View::edge_descriptor((*it).target(),
                                                       (*it).source()));
      vw.add_edge_async(typename View::edge_descriptor((*it).source(),
                                                       (*it).target()));
    }
  }
};

} // namespace graph_impl


#if 0
//////////////////////////////////////////////////////////////////////
/// @brief Undirected view is a @ref graph_view that presents undirected
/// edges for a directed graph.
///
/// @tparam C Container type.
/// @tparam Dom Domain type. By default uses the same domain type
///             provided for the container.
/// @tparam MapFunc Mapping function type. (default: identity mapping function)
/// @tparam Derived Type of the most derived class (default: itself)
//////////////////////////////////////////////////////////////////////
template <typename PG,
          typename Dom     = typename PG::domain_type,
          typename MapFunc = f_ident<typename Dom::index_type>,
          typename Derived = use_default>
class undirected_view
  : public graph_view<PG,Dom,MapFunc>,
    public readwrite_operation<undirected_view<PG,Dom,MapFunc,Derived> >,
    public sequence_operation<typename select_derived<Derived,
      undirected_view<PG, Dom, MapFunc, Derived> >::type>
{
  typedef graph_view<PG,Dom,MapFunc>                      base_type;

  typedef typename select_derived<
    Derived,
    graph_view<PG, Dom, MapFunc, Derived>
  >::type                                                derived_type;

  typedef sequence_operation<derived_type>               sequence_op_type;
public:
  typedef typename sequence_op_type::iterator iterator;
  typedef typename sequence_op_type::iterator vertex_iterator;
  typedef typename PG::adj_edge_iterator       adj_edge_iterator;
  typedef typename PG::vertex_property         vertex_property;
  typedef typename PG::edge_property           edge_property;
  typedef typename PG::vertex_descriptor       vertex_descriptor;
  typedef typename PG::edge_descriptor         edge_descriptor;
  typedef typename PG::vertex_reference        vertex_reference;
  STAPL_VIEW_REFLECT_TRAITS(undirected_view)

protected:
   view_container_type* m_undirected_container;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used to pass ownership of the container to the view.
  ///
  /// @param vcont pointer to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mfunc mapping function to transform view indices to container
  ///        gids.
  //////////////////////////////////////////////////////////////////////
  undirected_view(view_container_type* vcont, domain_type const& dom,
                  map_func_type mfunc=MapFunc())
    : base_type(vcont, dom, mfunc)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that does not takes ownership over the passed
  ///        container.
  ///
  /// @param vcont reference to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mfunc mapping function to transform view indices to container
  ///        gids.
  ///
  /// Creates the output graph and populates it with the bidirected
  /// edges from the input graph.
  //////////////////////////////////////////////////////////////////////
  undirected_view(view_container_type& vcont, domain_type const& dom,
                  map_func_type mfunc=MapFunc())
  {
    this->m_undirected_container = new view_container_type(vcont.size());
    this->m_domain = dom;

    typedef graph_view<view_container_type> this_view_type;
    this_view_type vw(*m_undirected_container);
    this_view_type other_view(vcont);

    stapl::map_func(graph_impl::global_edge_copy(), other_view,
                    make_repeat_view(vw));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view that can reference all the elements of
  ///        the passed container.
  /// @param vcont pointer to the container used to forward the operations.
  ///
  /// Creates the output graph and populates it with the bidirected
  /// edges from the input graph.
  //////////////////////////////////////////////////////////////////////
  undirected_view(view_container_type& vcont)
  {
    this->m_undirected_container = new view_container_type(vcont.size());
    this->m_domain = domain_type(this->m_undirected_container->domain());


    typedef graph_view<view_container_type> this_view_type;
    this_view_type vw(*m_undirected_container);
    this_view_type other_view(vcont);

    stapl::map_func(graph_impl::global_edge_copy(), other_view,
                    make_repeat_view(vw));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view that can reference all the elements of
  ///        the passed container. The view takes ownership of the container.
  ///
  /// @param vcont pointer to the container used to forward the operations.
  //////////////////////////////////////////////////////////////////////
  undirected_view(view_container_type* vcont)
    : base_type( vcont, domain_type(vcont) )
  { }

  void define_type(typer& t)
  {
    t.member(m_undirected_container);
  }
};

#endif


//////////////////////////////////////////////////////////////////////
/// @brief Helper struct to get the type for @ref undirected_view.
/// @tparam View The type of the input view.
//////////////////////////////////////////////////////////////////////
template<typename View>
struct undirected_view_type
{
  typedef View type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Function to return an undirected view of a directed graph.
/// @param view The type of the input @ref graph_view over a directed graph.
/// @return A @ref graph_view over the undirected graph.
/// @note This function copies the internal graph and adds the bidirectional
/// edges. The view returned is over a copy of the input graph, so changes
/// made to the output view will not be reflected in the original container.
/// @todo This function does the same thing as the @ref undirected_view,
/// we need to figure out which one to remove.
//////////////////////////////////////////////////////////////////////
template<typename View>
typename undirected_view_type<View>::type undirected_view(View& view)
{
  typename view_traits<View>::container* undirected_container
    = new typename view_traits<View>::container(view.size());
  typename undirected_view_type<View>::type ud_view(undirected_container);

  stapl::map_func(graph_impl::global_edge_copy(), view, make_repeat_view(ud_view));

  return ud_view;
}


} // namespace stapl

#endif

