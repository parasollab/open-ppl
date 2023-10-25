#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_BORUVKA
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_BORUVKA

#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/views/property_maps.hpp>
#include <stapl/containers/graph/algorithms/create_level_partial_info.hpp>

namespace stapl {

namespace mst_algo_detail {

///////////////////////////////////////////////////////////////////////
/// @brief Stores the user-defined comparator function and executes the
/// edge selection.
///
/// @tparam Comparator represents the user-defined comparator function
//////////////////////////////////////////////////////////////////////
template<typename Comparator>
struct select_edge_wf
{
  /// the user-defined comparator function
  Comparator wf;

  select_edge_wf(Comparator const& c)
    : wf(c)
  { }

  typedef void result_type;

  template<typename Vertex, typename Tree>
  result_type operator()(Vertex v, Tree& mst)
  {
    typename Vertex::vertex_descriptor vert = v.descriptor();
    typename Tree::edge_property       min_edge_prop;

    if (v.size() == 0) {
      v.property().property = vert;
      return;
    }

    typedef typename Tree::edge_property::property_type::weight_type weight_t;
    min_edge_prop.property.weight(std::numeric_limits<weight_t>::max());

    int weight;
    for (auto e : v) {
      weight = e.property().property.weight();
      if (wf(weight, min_edge_prop.property.weight()) &&
          e.source() != e.target()) {
        vert = e.target();
        min_edge_prop.property = e.property().property;
      } else if (min_edge_prop.property.weight() == weight &&
                 e.source() != e.target() &&
                 e.target() < vert) {
        vert = e.target();
        min_edge_prop.property = e.property().property;
      }
    }

    mst.add_edge_async(min_edge_prop.property.source(),
                       min_edge_prop.property.target(),
                       min_edge_prop);
    v.property().property = vert;
  }

  void define_type(typer& t)
  { t.member(wf); }
};


///////////////////////////////////////////////////////////////////
/// @brief Uses a user-defined comparator function to select a single edge
/// for each vertex.
///
/// @param v the graph view to select edges from
/// @param c the user-provided comparator work function
/// @param t the container in which to store the selected edges
///
/// @note The descriptor of the selected edge's target vertex is stored as
/// the property of the current vertex and the edge is added to @p t.
///////////////////////////////////////////////////////////////////
template<typename View, typename Comparator, typename Tree>
void select_edge(View& v, Comparator const& c, Tree& t)
{
  nc_map_func(select_edge_wf<Comparator>(c), v, make_repeat_view(t));
}


/////////////////////////////////////////////////////////////////
/// @brief This struct returns the smaller of two values.
///
/// @tparam C the comparator work function
///
/// @note This work function acts as the user-provided comparator
/// for Boruvka's algorithm.
////////////////////////////////////////////////////////////////
template<typename C>
struct edge_property_reducer
 : private C
{
  C& compare()
  { return static_cast<C&>(*this); }

  template<typename T>
  T operator()(T const& x, T const& y)
  { return this->compare()(x.weight(), y.weight()) ? x : y; }
};


/////////////////////////////////////////////////////////////////
/// @brief This struct provides access to the vertex properties.
///
/// @note All of the functions needed to access and update the property
/// values of the vertices are kept in this struct.
////////////////////////////////////////////////////////////////
struct vd_access_functor
{
  typedef size_t value_type;

  template<typename Property>
  value_type get(Property p)
  { return p.property; }

  template<typename Property>
  void put(Property p, value_type v)
  { p.property = v; }

  template<typename Property, typename F>
  void apply(Property p, F f)
  { f(p.property); }
};

}


///////////////////////////////////////////////////////////////////
/// @brief This method implements Boruvka's Minimum-Spanning Tree (MST)
/// algorithm.
///
/// @param g the graph view of the undirected input graph.
/// @return The graph view storing the resulting minimum
/// spanning tree of @p g.
///////////////////////////////////////////////////////////////////
template<typename View>
View boruvka(View& g)
{
  using namespace mst_algo_detail;
  // copy the input view before rewriting with hierarchy below.
  View v = g;

  // create the output MST graph.
  typedef typename View::view_container_type graph_type;
  graph_type* mst = new graph_type(v.size());
  View mst_vw(mst);

  typedef graph_internal_property_map<graph_type, vd_access_functor> prop_map_t;

  size_t old_size = 0;
  while (v.size() != old_size && v.size() > 1)
  {
    old_size = v.size();
    select_edge(v, std::less<size_t>(), mst_vw);

    prop_map_t match_map(v.container(), vd_access_functor());

    v = create_level_partial_info(v, match_map, std::plus<size_t>(),
                                  edge_property_reducer<std::less<int> >());
  }

  return mst_vw;
}

} // namespace stapl

#endif
