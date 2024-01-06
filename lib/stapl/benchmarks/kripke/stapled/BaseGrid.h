/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

/*!
  \file BaseGrid.h

   This file defines a high level parallel grid of arbitrary geometry.  It is
   used as a base for hexahedral grids etc.
*/

#ifndef STAPL_BENCHMARKS_KRIPKE_BASE_GRID_H
#define STAPL_BENCHMARKS_KRIPKE_BASE_GRID_H

// System includes
#include <iostream>
#include <vector>

// STAPL Includes
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/mesh/geom_vector.hpp>
#include <stapl/runtime.hpp>

#include "Cell.hpp"
#include "Kripke/Input_Variables.h"
#include "Kripke/Directions.h"
#include "GroupDirSet.hpp"

typedef stapl::geom_vector<3, double> Vector3D;

class BaseGrid
  : public stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
                                Cell3D, Vector3D>
{
public:

  typedef stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
                               Cell3D,Vector3D> pGraphType;
  typedef typename pGraphType::distribution_type::container_manager_type bc_type;
  typedef typename bc_type::base_container_type::vertex_iterator lcl_cell_iterator;
  typedef typename bc_type::base_container_type::adj_edge_iterator lcl_edge_iterator;
  typedef typename pGraphType::vertex_descriptor       vertex_descriptor;
  typedef typename pGraphType::edge_descriptor         edge_descriptor;
  typedef typename pGraphType::vertex_iterator         cell_iterator;
  typedef typename pGraphType::adj_edge_iterator       edge_iterator;

public:

  BaseGrid(Input_Variables *input_vars, Directions *directions);

  size_t cell_count()
  {
    return this->distribution().container_manager().begin()->size();
  }

  /*!
    \brief Construct the cell ddgs that will determine sweep order within cellsets.
  */
//  void build_cell_ddgs(grid_detail::DDG_List_Type& ddgs, EnergyAggregation& egs);

//  void prepare_for_sweep(EnergyGroupSet& gs);
};

#endif
