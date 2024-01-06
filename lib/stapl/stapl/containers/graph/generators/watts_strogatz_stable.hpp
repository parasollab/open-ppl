/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_WS_STABLE_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_WS_STABLE_HPP

namespace stapl {

namespace generators {

//////////////////////////////////////////////////////////////////////
/// @brief A stable implementation of watts_strogatz that generates the
/// the same random Watts-Strogatz graph for a specified number of
/// simulated processors (irrespective to the actual number of processes
/// being run on).
///
/// @todo Need to fix the stability of @ref ws_rewire and use
/// @ref make_watts_strogatz instead.
//////////////////////////////////////////////////////////////////////
template<typename G>
struct watts_strogatz_stable
{
  protected:
    G&     m_graph;
    size_t m_n;
    size_t m_k;
    double m_p;

  public:
    watts_strogatz_stable(G& g, size_t k, double p)
      : m_graph(g), m_n(g.size()), m_k(k), m_p(p)
    { }

    void add_edges(size_t simulated_procs)
    {
      size_t p_per = simulated_procs/get_num_locations();
      size_t p_start = get_location_id()*p_per;
      size_t p_end = p_start + p_per;

      size_t block_size = m_n/simulated_procs;

      //simulate each processor
      //(in the loop, p := get_simulated_location_id() and
      //  simulated_procs := get_num_simulated_locations())
      for (size_t p = p_start; p < p_end; ++p) {
        srand48((p+65)*4321);

        size_t subview_start = block_size*p;
        size_t subview_end = subview_start+block_size;

        for (size_t i=subview_start; i<subview_end; ++i) {
          std::set<size_t> dests;
          for (size_t k=1; k<m_k/2+1; ++k) {
            size_t j = (i+k)%m_n;
            // if we draw a good prob, just add the edge
            // (there is a chance we already randomly added the edge;
            //   in this case, we should redo the random rewiring)
            // otherwise, randomly "rewire" the edge
            if (drand48() >= m_p) {
              if (!dests.insert(j).second) {
                while (!dests.insert(generate_random(i)).second);
              }
            } else {
              while (!dests.insert(generate_random(i)).second);
            }
          }
          for (auto&& d : dests) {
            m_graph.add_edge_async(i, d);
          }
        }
      }
    }

    size_t generate_random(size_t source)
    {
      size_t dest = drand48()*m_n;
      while (dest == source) {
        dest = drand48()*m_n;
      }
      return dest;
    }
};

} // namespace generators

} // namespace stapl

#endif
