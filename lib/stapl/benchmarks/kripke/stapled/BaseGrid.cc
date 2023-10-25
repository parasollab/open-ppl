/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include "BaseGrid.h"

// BaseGrid constructor
BaseGrid::BaseGrid(Input_Variables *input_vars, Directions *directions)
  : pGraphType()
{}
#if 0
void BaseGrid::prepare_for_sweep(EnergyGroupSet& gs)
{
  lcl_edge_iterator aei;
  lcl_cell_iterator lci = this->distribution().container_manager().begin()->begin();
  lcl_cell_iterator end = this->distribution().container_manager().begin()->end();
  for (; lci != end; ++lci)
  {
    for (aei = (*lci).begin(); aei != (*lci).end(); ++aei)
    {
      (*aei).property().prepare_for_sweep(gs);
    }
  }
}

void BaseGrid::zero_partial_currents(EnergyGroupSet& gs)
{
  lcl_edge_iterator aei;
  lcl_cell_iterator lci = this->distribution().container_manager().begin()->begin();
  lcl_cell_iterator end = this->distribution().container_manager().begin()->end();
  for (; lci != end; ++lci)
    for (aei = (*lci).begin(); aei != (*lci).end(); ++aei)
      (*aei).property().zero_partial_currents(gs);
}



// we don't need this for regular grids
void BaseGrid::build_cell_ddgs(grid_detail::DDG_List_Type& ddgs,
  EnergyAggregation& egs)
{
  // Resize the list of ddgs
  size_t total_num_anglesets = egs.get_total_num_anglesets();
  ddgs.resize( total_num_anglesets );

  // Loop over anglesets
  for( size_t as=0; as!=total_num_anglesets; ++as )
  {
    // Loop over the cellset graph
    grid_detail::csg_vertex_iterator
      v_it = m_cs_graph.distribution().container_manager().begin()->begin(),
      v_end = m_cs_graph.distribution().container_manager().begin()->end();
    for( ; v_it!=v_end; ++v_it )
    {
      // Retrieve the cell set data
      grid_detail::CellSet& cell_set( (*v_it).property() );
      size_t csid = cell_set.cell_set_id;

      // Allocate the ddg
      ddgs[as].insert( std::make_pair( csid, new grid_detail::Cell_DDG_Type ) );

      // Populate the vertices
      std::vector<size_t>::iterator cell_id = cell_set.cell_ids.begin(),
        cell_end = cell_set.cell_ids.end();
      for( ; cell_id!=cell_end; ++cell_id )
      {
        // Add the vertex to the ddg
        ddgs[ as ][ csid ]->add_vertex( *cell_id, grid_detail::DDG_Vertex(*cell_id) );
      }
    } // cs verts
  } // as

  // We can't add the edges until the vertices are in place
  stapl::rmi_fence();

  // Loop over all groupsets
  int outer_asid = 0;
  std::vector<EnergyGroupSet>::iterator gs_it = egs.begin(), gs_end = egs.end();
  for( ; gs_it!=gs_end; ++gs_it )
  {
    int num_as = gs_it->num_anglesets();

    // In this routine, all of the graph edges test every angle for incidence.
    prepare_for_sweep( *gs_it );

    // Traverse the graph.
    lcl_cell_iterator
      v_it = this->distribution().container_manager().begin()->begin(),
      v_end = this->distribution().container_manager().begin()->end();
    for( ; v_it!=v_end; ++v_it )
    {
      size_t csid = (*v_it).property().get_csid();
      lcl_edge_iterator e_it = (*v_it).begin(), e_end = (*v_it).end();
      for( ; e_it!=e_end; ++e_it )
      {
        // Skip global boundary edges.
        Edge& edge( (*e_it).property() );

        size_t target_csid = edge.get_target_cs();
        if( target_csid != csid )  continue;

        // Retrieve direction information
        const std::vector<EdgeDirection>& incidence = edge.get_incidence();

        // Loop over anglesets
        for( int as=0; as!=num_as; ++as )
        {
          // If the edge is incident, add it to the ddg
          if( incidence[as] == ED_INCIDENT )
          {
            // The two graphs have reversed direction, so swap source and target
            ddgs[ outer_asid+as ][ csid ]->add_edge(
              // (*e_it).source(), (*e_it).target() );
              (*e_it).target(), (*e_it).source() );
              // (*e_it).target(), (*e_it).source(), edge.get_normal() );
          }

          else if( incidence[as] == ED_MIXED )
          {
            std::stringstream str; WHERE(str);
            str << "GridDirecting found an edge-angleset pair with "
              << "ED_MIXED.  This should have been caught upstream.  PDT is "
              << "not yet supporting grids/quadratures with cycles.\n";
            throw CommonException( str, CET_INTERNAL_ERROR );
          }
        } // as
      } // edges
    } // verts
    outer_asid += num_as;
  } // gs
}
#endif
