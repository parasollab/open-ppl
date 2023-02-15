
#include "test_util.h"
#include <stack>
#include <stapl/runtime/runtime.hpp>
#include <stapl/containers/graph/algorithms/pscc_kla.hpp>
#include <stapl/containers/graph/algorithms/pscc_kla_priority_aware.hpp>
#include <stapl/containers/graph/algorithms/pscc_kla_single.hpp>
#include <stapl/containers/graph/algorithms/pscc_kla_schudy.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/generators/cycles.hpp>
#include <stapl/containers/graph/generators/perturbed_mesh.hpp>
#include <stapl/containers/graph/generators/watts_strogatz.hpp>
#include <stapl/containers/graph/generators/watts_strogatz_stable.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/algorithms/sorting.hpp>

template <typename VertexGIDType>
class test_pscc_vertex_property
{
public:
  typedef VertexGIDType color_type;

private:
  color_type m_cc;

public:
  test_pscc_vertex_property(void)
    : m_cc(stapl::index_bounds<color_type>::invalid())
  { }

  color_type get_cc(void) const
  { return m_cc; }

  void set_cc(color_type c)
  { m_cc = c; }

  void define_type(stapl::typer& t)
  {
    t.member(m_cc);
  }
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
    : Accessor(acc) { }

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


namespace stapl {

//returns false if the cc is not valid for this node
struct find_scc_marks
{
private:
  size_t m_size;

public:
  typedef void result_type;

  find_scc_marks(size_t s) : m_size(s) {}

  template<typename Vertex, typename ArrayView>
  result_type operator()(Vertex v, ArrayView av)
  {
    av[v.descriptor()/m_size] = v.property().get_cc();
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_size);
  }
};


struct in_same_scc
{
private:
  size_t m_size;

public:
  typedef bool    result_type;

  in_same_scc(size_t s)
    : m_size(s)
  { }

  template<typename Vertex, typename ArrayView>
  result_type operator()(Vertex v, ArrayView av)
  {
    return (av[v.descriptor()/m_size] == v.property().get_cc());
  }

  void define_type(stapl::typer& t)
  { t.member(m_size); }
};

} //namespace stapl


stapl::exit_code stapl_main(int argc, char* argv[])
{
  using namespace std;
  using namespace stapl;

  stapl_print("Testing strongly connected components...\t");

  typedef test_pscc_vertex_property<size_t>                 vertex_property;
  typedef stapl::graph<stapl::DIRECTED,
                       stapl::MULTIEDGES, vertex_property>  graph_type;
  typedef stapl::graph_view<graph_type>                     gview_type;

  if (argc == 1) {
    if (get_location_id() == 0) {
      cout << "usage: " << argv[0]
           << " [0/1/2/3 sccm/sccm2/dcsc/schudy] [cycle count] [cycle size]"
           << endl;
    }
    return EXIT_FAILURE;
  }
  size_t alg = (argc > 1) ? atoi(argv[1]) : 0;
  size_t ccs = (argc > 2) ? atoi(argv[2]) : 256;
  size_t sz  = (argc > 3) ? atoi(argv[3]) : 3;

  gview_type vw;
  vw = generators::make_cycle_chain<gview_type>(ccs, sz, false);

  switch(alg)
  {
    case 0:   pscc(vw, 5);        break;
    case 1:   pscc_priority_aware(vw, 5);  break;
    case 2:   pscc_single(vw, 5); break;
    case 3:   pscc_schudy(vw, 5); break;
    default:
      if (get_location_id() == 0) {
        cerr << "No valid algorithm selected." << endl;
      }
      return EXIT_FAILURE;
  }

  typedef stapl::array<size_t> correctness_array;
  typedef stapl::array_view<correctness_array> correctness_view;

  correctness_array ar(ccs);
  correctness_view av(ar);

  //get one color from each cc, check that all in cc have that color,
  // and that ccs are unique
  stapl::map_func(find_scc_marks(sz), vw, make_repeat_view(av));
  bool passed = stapl::map_reduce(in_same_scc(sz),
                                  stapl::logical_and<bool>(),
                                  vw, make_repeat_view(av));
  stapl::sort(av);
  auto uvp = stapl::unique(av, std::equal_to<size_t>());
  passed &= (uvp.second.size() == 0);

  stapl_print(passed ? "[PASSED]\n" : "[FAILED]\n");

  return EXIT_SUCCESS;
}

