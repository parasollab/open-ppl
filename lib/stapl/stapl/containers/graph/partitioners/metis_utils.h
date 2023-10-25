#ifndef STAPL_CONTAINERS_GRAPH_PARTITIONERS_METIS_UTILS_HPP
#define STAPL_CONTAINERS_GRAPH_PARTITIONERS_METIS_UTILS_HPP

#include <stapl/runtime.hpp>
#include <metis.h>
#include <vector>

namespace stapl {

namespace partitioner_details {

struct metis_graph
{
  int nvtxs;
  int nedges;
  int ncon;
  std::vector<int> xadj;
  std::vector<int> adjncy;
  std::vector<int> vwgt;
  std::vector<int> adjwgt;
  std::vector<int> label;

  metis_graph()
    : nvtxs(0), nedges(0), ncon(1), xadj(),
      adjncy(), vwgt(),
      adjwgt(), label()
  { }


  metis_graph(int const& nvertices, int const& nedges)
    : nvtxs(nvertices), nedges(nedges), ncon(1), xadj(nvertices+1),
      adjncy(nedges), vwgt(nvertices),
      adjwgt(nedges), label(nvertices)
  {
    for (int i=0; i < nvtxs; ++i)
      label[i] = i;
  }

  void print() const
  {
    if (stapl::get_location_id() == 0) {
      std::cout << "\nv=" << nvtxs << " E=" << nedges;

      std::cout << std::endl << "V-LABELS:  \n";
      for (size_t i=0; i<label.size(); ++i)
        std::cout << label[i] << "  ";

      std::cout << std::endl << "VERTICES:  \n";
      for (size_t i=0; i<xadj.size(); ++i)
        std::cout << xadj[i] << "  ";
      std::cout << std::endl << "EDGES:  \n";
      for (size_t i=0; i<adjncy.size(); ++i)
        std::cout << adjncy[i] << "  ";


      std::cout << std::endl << "V-WGHTS:  \n";
      for (size_t i=0; i<vwgt.size(); ++i)
        std::cout << vwgt[i] << "  ";
      std::cout << std::endl << "E-WGHTS:  \n";
      for (size_t i=0; i<adjwgt.size(); ++i)
        std::cout << adjwgt[i] << "  ";
      std::cout << std::endl << std::endl;
    }
  }

  void define_type(typer& t)
  {
    t.member(nvtxs);
    t.member(nedges);
    t.member(ncon);
    t.member(xadj);
    t.member(adjncy);
    t.member(vwgt);
    t.member(adjwgt);
    t.member(label);
  }
};

//Function picked from ParMETIS
void keep_part(metis_graph *graph, int *part, int mypart)
{
  int h, i, j, k;
  int mynvtxs, mynedges;

  std::vector<int> rename(graph->nvtxs);

  for (mynvtxs=0, i=0; i<graph->nvtxs; i++) {
    if (part[i] == mypart)
      rename[i] = mynvtxs++;
  }

  for (mynvtxs=0, mynedges=0, j=graph->xadj[0], i=0; i<graph->nvtxs; i++) {
    if (part[i] == mypart) {
      for (; j<graph->xadj[i+1]; j++) {
        k = graph->adjncy[j];
        if (part[k] == mypart) {
          graph->adjncy[mynedges] = rename[k];
          graph->adjwgt[mynedges++] = graph->adjwgt[j];
        }
      }
      j = graph->xadj[i+1];  /* Save graph->xadj[i+1] for later use */

      for (h=0; h<graph->ncon; h++)
        graph->vwgt[mynvtxs*graph->ncon+h] = graph->vwgt[i*graph->ncon+h];

      graph->label[mynvtxs] = graph->label[i];
      graph->xadj[++mynvtxs] = mynedges;
    }
    else {
      j = graph->xadj[i+1];  /* Save graph->xadj[i+1] for later use */
    }
  }

  graph->nvtxs  = mynvtxs;
}

} //namespace partitioner_details

} //namespace stapl

#endif
