/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_DIRECTED_PREDS_GRAPH_VIEW_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_DIRECTED_PREDS_GRAPH_VIEW_HPP

#include <stapl/utility/use_default.hpp>

#include "graph_view_property_adaptor.h"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief View providing predecessor info for graphs.
/// Specialization when the underlying view/graph is DPG.
/// @ingroup graph
///
/// In general there are two cases; one where this view is defined over
/// a DPG graph, in which case the view is just a wrapper; and second
/// where the underlying graph/view is not DPG and then we have to keep
/// extra storage to keep track of predecessors in the view separate
/// from the graph.
/// These two cases are implemented using different partial specializations.
//////////////////////////////////////////////////////////////////////
template <class Graph, class VAdaptor = use_default,
    class EAdaptor = use_default>
class dpg_view
  : public dynamic_graph_view<Graph, VAdaptor, EAdaptor>
{
  typedef dynamic_graph_view<Graph, VAdaptor, EAdaptor> base_type;
  typedef dpg_view<Graph, VAdaptor, EAdaptor> this_type;
public:
  typedef typename base_type::vertex_descriptor vertex_descriptor;
  typedef typename base_type::edge_descriptor edge_descriptor;

  typedef typename base_type::vertex_iterator vertex_iterator;
  typedef typename base_type::const_vertex_iterator const_vertex_iterator;
  typedef typename base_type::adj_edge_iterator adj_edge_iterator;
  typedef typename base_type::const_adj_edge_iterator const_adj_edge_iterator;
  typedef typename base_type::edge_iterator edge_iterator;
  typedef typename base_type::const_edge_iterator const_edge_iterator;

  typedef typename vertex_iterator::property_type vertex_property;
  typedef typename adj_edge_iterator::property_type edge_property;

  //new relative to dynamic view
  typedef typename Graph::preds_iterator preds_iterator;
  typedef typename Graph::const_preds_iterator const_preds_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a dpg view over a dpg graph.
  //////////////////////////////////////////////////////////////////////
  dpg_view(Graph& g)
    : base_type(g)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the graph for the lazy-update mode.
  //////////////////////////////////////////////////////////////////////
  void set_async_update(void)
  {
    this->m_g.set_async_update();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the graph for the always update mode.
  //////////////////////////////////////////////////////////////////////
  void set_sync_update(void)
  {
    this->m_g.set_sync_update();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief When in the lazy update mode this method should be called to
  /// update all predecessors.
  //////////////////////////////////////////////////////////////////////
  void set_predecessors(void)
  {
    this->m_g.set_predecessors();
  }
};


}//end namespace stapl

#endif
