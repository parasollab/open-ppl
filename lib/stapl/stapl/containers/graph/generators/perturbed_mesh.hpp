/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_PERTURBED_MESH_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_PERTURBED_MESH_HPP

namespace stapl {

namespace generators {


///////////////////////////////////////////////////////////////////////////////
/// @brief Populates a graph so it represents a perturbed mesh using a
/// simulated number of processors.
///////////////////////////////////////////////////////////////////////////////
template<typename G>
struct perturbed_mesh
{
protected:
  G&     m_graph;
  size_t m_vertices_dim;
  size_t m_vertices;

public:
  perturbed_mesh(G& g)
    : m_graph(g), m_vertices_dim(round(pow(g.size(), 1/3.0))),
      m_vertices(pow(m_vertices_dim, 3))
  { }

  void add_vertices(void)
  {
    size_t block_size = m_vertices/get_num_locations();
    size_t subview_start = block_size*get_location_id();
    size_t subview_end = subview_start + block_size;

    for (size_t i = subview_start; i < subview_end; ++i) {
      m_graph.add_vertex(i,typename G::vertex_property());
    }
  }

  void add_edges(double rate, size_t simulated_procs = get_num_locations())
  {
    //find the simulated divisions
    double simu_procsleft = simulated_procs;
    int simuz = (simu_procsleft > 1) ? 2 : 1;
    simu_procsleft /= simuz;
    int simuy = pow(2, floor(log(simu_procsleft)/log(4)));
    simu_procsleft /= simuy;
    int simux = (int)simu_procsleft;

    //find the (simulated) dimensions of each block
    size_t zdim = m_vertices_dim/simuz;
    size_t ydim = m_vertices_dim/simuy;
    size_t xdim = m_vertices_dim/simux;

    //ensure consistency
    size_t simu_per_real = simulated_procs/get_num_locations();
    size_t block_size = m_vertices/simulated_procs;

    //simulate each processor
    //(in the loop, p := get_simulated_location_id() and
    //  simulated_procs := get_num_simulated_locations())
    size_t simu_start = simu_per_real*get_location_id();
    size_t simu_end = simu_start+simu_per_real;

    for (size_t p = simu_start; p < simu_end; ++p) {
      size_t subview_start = block_size*p;

      bool addXs = (p + 1)%simux;
      bool addYs = (p/simux + 1)%simuy;
      bool addZs = p < simulated_procs/2;

      srand48((p+65)*4321);
      for (size_t i=0; i<xdim; ++i) {
        for (size_t j=0; j<ydim; ++j) {
          for (size_t k=0; k<zdim; ++k) {
            size_t ix = subview_start + i + j*xdim + k*xdim*ydim;

            if (addXs || i+1 < xdim) {
              size_t nx = ix + (addXs ? (block_size + 1-xdim) : 1);
              add_edge(ix, nx, rate);
            }
            if (addYs || j+1 < ydim) {
              size_t nx = ix +
                (addYs ? (block_size*simux + xdim*(1-ydim)) : xdim);
              add_edge(ix, nx, rate);
            }
            if (addZs || k+1 < zdim) {
              size_t nx = ix +
                (addZs ? (block_size*simux*simuy + xdim*ydim*(1-zdim))
                 : xdim*ydim);
              add_edge(ix, nx, rate);
            }
          }
        }
      }
    }
  }

  void add_edge(size_t& ix, size_t& nx, double& rate)
  {
    if (drand48() >= rate) {
      m_graph.add_edge_async(ix, nx);
    } else {
      m_graph.add_edge_async(nx, ix);
    }
  }
};

} // namespace generators

} // namespace stapl

#endif
