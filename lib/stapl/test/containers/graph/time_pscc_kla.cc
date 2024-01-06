
#include "test_util.h"
#include <stack>
#include <stapl/runtime/runtime.hpp>
#include <stapl/containers/graph/algorithms/pscc_kla.hpp>
#include <stapl/containers/graph/algorithms/pscc_kla_priority_aware.hpp>
#include <stapl/containers/graph/algorithms/pscc_kla_single.hpp>
#include <stapl/containers/graph/algorithms/pscc_kla_hong.hpp>
#include <stapl/containers/graph/algorithms/pscc_kla_schudy.hpp>
#include <test/containers/graph/reach_queries_kla.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/generators/cycles.hpp>
#include <stapl/containers/graph/generators/perturbed_mesh.hpp>
#include <stapl/containers/graph/generators/watts_strogatz.hpp>
#include <stapl/containers/graph/generators/watts_strogatz_stable.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/array/array.hpp>
#include <benchmarks/graph/g500/g500.h>

template <typename VertexGIDType>
class test_pscc_vertex_property
{
public:
  typedef VertexGIDType                         color_type;

private:
  color_type      m_cc;

public:
  test_pscc_vertex_property(void)
    : m_cc(stapl::index_bounds<color_type>::invalid())
  { }

  color_type get_cc(void) const
  { return m_cc; }

  void set_cc(color_type c)
  { m_cc = c; }

  void define_type(stapl::typer& t)
  { t.member(m_cc); }
};


namespace stapl {

template <typename VertGID, typename Accessor>
class proxy<test_pscc_vertex_property<VertGID>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef test_pscc_vertex_property<VertGID>      target_t;
  typedef typename target_t::color_type           color_type;

public:
  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t(void) const
  { return Accessor::read(); }

  proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this; }

  void set_cc(color_type c)
  { Accessor::invoke(&target_t::set_cc, c); }

  color_type get_cc(void) const
  { return Accessor::const_invoke(&target_t::get_cc); }
}; //struct proxy

} //namespace stapl


using namespace std;
using namespace stapl;

struct analysis_map_insert
{
  typedef void result_type;

  template<typename MapPair, typename T>
  void operator()(MapPair& where, const T&) const
  {
    ++where.second;
  }
};


struct analysis_wf
{
  typedef void result_type;

  template <typename Vertex, typename Map>
  void operator()(Vertex v, Map m) const
  {
    typedef analysis_map_insert insert_func;

    size_t cc = v.property().get_cc();
    m[cc] += 1;
  }
};


struct get_map_second
{
  typedef double result_type;

  template <typename Map>
  result_type operator()(Map m) const
  {
    double max = -1.;
    size_t m_size = m.size();
    if (m_size == 0) {
      return 0.;
    }

    for (auto&& i : m) {
      if (i.second > max) {
        max = i.second;
      }
    }
    return max;
  }
};


struct elem_var
{
  typedef double result_type;

  double m_mean;

  elem_var(double mean)
    : m_mean(mean)
  { }

  template <typename Ref>
  result_type operator()(Ref r) const
  {
    if (r != size_t(0)) {
      return (r-m_mean)*(r-m_mean);
    } else {
      return 0.0;
    }
  }

  void define_type(typer &t)
  {
    t.member(m_mean);
  }
};


struct non_zero
{
  typedef bool result_type;

  template <typename Ref>
  bool operator()(Ref r)
  { return r != size_t(0); }

};


template<typename V>
void analyze_sccs(V& gv)
{
  typedef stapl::static_array<size_t> counts_map;
  typedef stapl::array_view<counts_map> counts_view;

  counts_map cm(gv.size(), 0);
  counts_view cv(cm);

  stapl::map_func(analysis_wf(), gv, make_repeat_view(cv));
  size_t largest = stapl::map_reduce(stapl::identity<size_t>(),
                                     stapl::max<size_t>(), cv);
  size_t num_cycles = stapl::count_if(cv, non_zero());

  double mean = double(gv.size())/double(num_cycles);
  double var = stapl::map_reduce(elem_var(mean), stapl::plus<double>(), cv);
  double std = sqrt(var/(num_cycles-1));

  if (get_location_id() == 0) {
    cout << "SSum: " << gv.size() << endl;
    cout << "CCount: " << num_cycles << endl;
    cout << "MMax: " << largest << endl;
    cout << "MMean: " << mean << endl;
    cout << "VVar: " << var << endl;
    cout << "SstdDev: " << std << endl;
    cout << endl;
  }

  rmi_fence();
}


template<typename V>
void run(V& gv, size_t alg, size_t k, float degt, size_t max_procs)
{
  stapl::counter<stapl::default_timer> ttime;
  for (int ct=0; ct<10; ++ct) {
    ttime.reset();
    ttime.start();
    int recursion_depth = 0;
    if (get_location_id() == 0) {
      cout << "starting" << endl;
    }
    switch (alg)
    {
      case 0: pscc(gv, k, degt);        break;
      case 1: pscc_priority_aware(gv, k, degt);  break;
      case 2: pscc_single(gv, k, degt); break;
      case 3: recursion_depth = pscc_schudy(gv, k, degt); break;
      case 4: reach_queries(gv, k, degt); break;
      case 5: pscc_hong(gv, k, degt);   break;
      default: cerr << "Invalid algorithm." << endl; exit(1);
    }

    double dtime = ttime.stop();

    //only analyze the graph on the first iteration
    // and only when we use scc_multi (since we will always do
    // scc_multi if we use anything else)
    if (alg == 0 && ct == 0) {
      stapl::counter<stapl::default_timer> atimer;
      atimer.start();
      analyze_sccs(gv);
      double atime = atimer.stop();
      if (get_location_id() == 0) {
        cout << "Analysis Time: " << atime << endl;
      }
    }

    if (get_location_id() == 0) {
      if (alg == 3) {
        cout << "TTime: " << dtime << " depth: " << recursion_depth << endl;
      } else {
        cout << "TTime: " << dtime << endl;
      }
    }
  }
}


void cycle_test(size_t alg, size_t test, size_t ccs, size_t sz, size_t k,
                  float degt, size_t max_procs)
{
  typedef test_pscc_vertex_property<size_t>                 vertex_property;
  typedef stapl::graph<stapl::DIRECTED,
                       stapl::MULTIEDGES, vertex_property>  graph_type;
  typedef stapl::graph_view<graph_type>                     gview_type;
  typedef typename gview_type::domain_type                  gdomain_type;
  typedef stapl::array<size_t>                              array_type;
  typedef stapl::array_view<array_type>                     aview_type;

  gview_type vw;
  std::stack<graph_type::vertex_descriptor> stk;
  switch (test)
  {
    case 0:
      vw = generators::make_unconnected_cycles<gview_type>(ccs, sz, false);
      break;
    case 1:
      vw = generators::make_cycle_chain<gview_type>(ccs, sz, false);
      break;
    default:
      if (get_location_id() == 0) {
        cerr << "No valid test selected." << endl;
      }
  }

  run(vw, alg, k, degt, max_procs);
}


void mesh_test(size_t alg, size_t size, double rate, size_t k, float degt,
                 size_t max_procs)
{
  typedef test_pscc_vertex_property<size_t>                 vertex_property;
  typedef stapl::graph<stapl::DIRECTED,
                       stapl::MULTIEDGES, vertex_property>  graph_type;
  typedef stapl::graph_view<graph_type>                     gview_type;
  typedef typename gview_type::domain_type                  gdomain_type;
  typedef stapl::array<size_t>                              array_type;
  typedef stapl::array_view<array_type>                     aview_type;

  stapl::counter<stapl::default_timer> setup_time;
  setup_time.start();
  graph_type g(size);
  stapl::generators::perturbed_mesh<graph_type> generator(g);
  generator.add_edges(rate, max_procs);
  rmi_fence();

  gview_type vw(g);
  std::stack<graph_type::vertex_descriptor> stk;
  double stime = setup_time.stop();
  if (get_location_id() == 0) {
    cout << "Setup Time: " << stime << endl;
  }

  run(vw, alg, k, degt, max_procs);
}


void ws_test(size_t alg, size_t size, size_t neighbors, double rate,
                 size_t k, float degt, size_t max_procs)
{
  typedef test_pscc_vertex_property<size_t>                 vertex_property;
  typedef stapl::graph<stapl::DIRECTED,
                       stapl::MULTIEDGES, vertex_property>  graph_type;
  typedef stapl::graph_view<graph_type>                     gview_type;
  typedef typename gview_type::domain_type                  gdomain_type;
  typedef stapl::array<size_t>                              array_type;
  typedef stapl::array_view<array_type>                     aview_type;

  stapl::counter<stapl::default_timer> setup_time;
  setup_time.start();
  graph_type g(size);
  stapl::generators::watts_strogatz_stable<graph_type> generator(g, neighbors,
                                                                 rate);
  gview_type vw(g);
  generator.add_edges(max_procs);
  rmi_fence();

  std::stack<graph_type::vertex_descriptor> stk;
  double stime = setup_time.stop();
  if (get_location_id() == 0) {
    cout << "Setup Time: " << stime << endl;
  }

  run(vw, alg, k, degt, max_procs);
}


struct pscc_graph_type_reflector
{
  typedef test_pscc_vertex_property<size_t>  vertex_property;
  typedef stapl::graph<stapl::DIRECTED,
                       stapl::MULTIEDGES, vertex_property>  graph_t;
};


void g500_test(size_t alg, size_t scale, size_t edge_factor,
                 size_t k, float degt, size_t max_procs)
{
  typedef test_pscc_vertex_property<size_t>                 vertex_property;
  typedef stapl::graph<stapl::DIRECTED,
                       stapl::MULTIEDGES, vertex_property>  graph_type;
  typedef stapl::graph_view<graph_type>                     gview_type;
  typedef typename gview_type::domain_type                  gdomain_type;
  typedef stapl::array<size_t>                              array_type;
  typedef stapl::array_view<array_type>                     aview_type;

  size_t nverts(std::pow(2.0, double(scale)));

  stapl::counter<stapl::default_timer> setup_time;
  setup_time.start();
  gview_type vw =
    graph500_benchmark_generator<pscc_graph_type_reflector>
      (nverts, edge_factor, scale);
  double stime = setup_time.stop();
  if (get_location_id() == 0) {
    cout << "Setup Time: " << stime << endl;
  }

  run(vw, alg, k, degt, max_procs);
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc <= 5) {
    char* exec = argv[0];
    stapl::do_once([exec] {
      cout << "usage: " << exec
           << " [0/1/2/3/4/5 sccmulti/sccm2/dcsc/schudy/rq/hong]"
           << " [0/1 = separate/list] [cycle count] [cycle size] [k]"
           << " [degree threshold] [max procs]" << endl;
      cout << " or    " << exec
           << " [0/1/2/3/4/5 sccmulti/sccm2/dcsc/schudy/rq/hong] [2 = mesh]"
           << " [node count ((i*p)^3)] [flip %] [k] [degree threshold]"
           << " [max procs]" << endl;
      cout << " or    " << exec
           << " [0/1/2/3/4/5 sccmulti/sccm2/dcsc/schudy/rq/hong]"
           << " [3 = watts/strogatz] [node count] [degree] [k]"
           << " [degree threshold] [max procs]" << endl;
      cout << " or    " << exec
           << " [0/1/2/3/4/5 sccmulti/sccm2/dcsc/schudy/rq/hong] [4 = g500]"
           << " [scale] [degree] [k] [degree threshold]" << endl;
    });
    return EXIT_FAILURE;
  }
  size_t alg  = (argc > 1) ? atoi(argv[1]) : 0;
  size_t test = (argc > 2) ? atoi(argv[2]) : 0;
  size_t op1  = (argc > 3) ? atoi(argv[3]) : 256;
  size_t op2  = (argc > 4) ? atoi(argv[4]) : 3;
  size_t k    = (argc > 5) ? atoi(argv[5]) : 1000;
  float  degt = (argc > 6) ? atof(argv[6]) : 0.1;
  size_t max_procs = (argc > 7) ? atoi(argv[7]) : stapl::get_num_locations();

  if (test == 2) {
    mesh_test(alg, op1, op2/100.0, k, degt, max_procs);
  } else if (test == 3) {
    ws_test(alg, op1, op2, 0.5, k, degt, max_procs);
  } else if (test == 4) {
     g500_test(alg, op1, op2, k, degt, max_procs);
  } else {
    cycle_test(alg, test, op1, op2, k, degt, max_procs);
  }

  return EXIT_SUCCESS;
}

