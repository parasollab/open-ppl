// gralg/topological_sort.cc

#include <utility>

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/topological_sort.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/generators/binary_tree.hpp>

#include <stapl/algorithms/sorting.hpp>
#include "gralghelp.hpp"

using namespace std;
using stapl::properties::topological_sort_property;

typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
                     topological_sort_property>               graf_tp;
typedef stapl::graph_view<graf_tp>                            graf_vw_tp;

typedef topological_sort_property::vertex_value_type          topol_rank_tp;
typedef std::pair<graf_tp::vertex_descriptor, topol_rank_tp>  vtx_data_tp;
typedef stapl::array<vtx_data_tp>                             vtx_array_tp;

template <typename T>
struct topol_rank_comparator
{
  typedef bool result_type;
  result_type operator() (T const& a, T const& b) const
  { return a.second < b.second; }
};

struct fill_vertex_array_wf
{
  typedef void result_type;
  template <typename Vtx, typename Elem>
  result_type operator() (Vtx v, Elem e)
  { e = std::make_pair(v.descriptor(), v.property().rank()); }
};

struct get_vtx_desc_wf
{
  typedef graf_tp::vertex_descriptor result_type;
  template <typename Pair>
  result_type operator() (Pair vtx_data)
  { return vtx_data.first; }
};

class put_vtx_desc_wf
{
private:
  stapl::stream<ofstream> m_zout;

public:
  put_vtx_desc_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Pair>
  result_type operator()(Pair vtx_data)
  { m_zout << vtx_data.first << " "; }

  void define_type(stapl::typer& t)
  { t.member(m_zout); }
};

stapl::exit_code stapl_main(int argc, char **argv)
{
  // construct graph
  size_t n = 40;
  graf_vw_tp gr_vw = stapl::generators::make_binary_tree<graf_vw_tp>(n, false);

  // apply algorithm

  size_t k = 0;
  stapl::topological_sort<graf_vw_tp>(gr_vw, k);

  vtx_array_tp verts(gr_vw.size());
  auto verts_vw = stapl::make_array_view(verts);

  stapl::map_func(fill_vertex_array_wf(), gr_vw, verts_vw);

  stapl::sort(verts_vw, topol_rank_comparator<vtx_data_tp>());

  // display results
  stapl::stream<ofstream> zout;
  zout.open("refman_topsort.txt");
  stapl::do_once( msg_val<int>( zout, "VERT ", gr_vw.num_vertices() ));
  stapl::do_once( msg_val<int>( zout, "EDGE ", gr_vw.num_edges() ));
  stapl::do_once( msg( zout, "TOPOLOGICAL ORDER:" ) );
  stapl::serial_io( put_vtx_desc_wf(zout), verts_vw );

  stapl::write_dot(gr_vw, "refman_topsort.dot");

  return EXIT_SUCCESS;
}
