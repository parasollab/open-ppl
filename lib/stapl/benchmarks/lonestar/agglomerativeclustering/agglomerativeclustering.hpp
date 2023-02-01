/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_LONESTAR_AGGLOMERATIVECLUSTERING_HPP
#define STAPL_BENCHMARK_LONESTAR_AGGLOMERATIVECLUSTERING_HPP

#include <iostream>
#include <sstream>
#include <map>

#include <stapl/utility/do_once.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>

#include <stapl/containers/graph/hierarchical_graph.hpp>
#include <stapl/containers/graph/views/hgraph_view.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/hierarchical_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/views/native_view.hpp>
#include <stapl/containers/graph/algorithms/create_level.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/views/property_maps.hpp>

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <boost/random.hpp>

using namespace stapl;


//////////////////////////////////////////////////////////////////////
/// @brief Holds the actual points to be clustered.
//////////////////////////////////////////////////////////////////////
struct my_point
{
  double m_x, m_y, m_z;

  my_point(double val=0.0)
    : m_x(val), m_y(val), m_z(val)
  { }

  void set(double x, double y, double z)
  {
    m_x = x;
    m_y = y;
    m_z = z;
  }

  void set(my_point const& other)
  {
    m_x = other.m_x;
    m_y = other.m_y;
    m_z = other.m_z;
  }

  void scale(double factor)
  {
    m_x *= factor;
    m_y *= factor;
    m_z *= factor;
  }

  void sub(my_point const& other)
  {
    m_x -= other.m_x;
    m_y -= other.m_y;
    m_z -= other.m_z;
  }

  void add(my_point const& other)
  {
    m_x += other.m_x;
    m_y += other.m_y;
    m_z += other.m_z;
  }

  double get_Magnitude() const
  {
    return m_x*m_x + m_y*m_y + m_z*m_z;
  }

  double get_SquaredEuclideanDistance(my_point const& other) const
  {
    return pow(other.m_x - m_x, 2)
         + pow(other.m_y - m_y, 2)
         + pow(other.m_z - m_z, 2);
  }

  double get_EuclideanDistance(my_point const& other) const
  {
    return sqrt(get_SquaredEuclideanDistance(other));
  }

  double get_ManhattanDistance(my_point const& other) const
  {
    return std::abs(other.m_x - m_x)
         + std::abs(other.m_y - m_y)
         + std::abs(other.m_z - m_z);
  }

  double get_DotProduct(my_point const& other) const
  {
    return other.m_x * m_x
         + other.m_y * m_y
         + other.m_z * m_z;
  }

  double get_CosineSimilarity(my_point const& other) const
  {
    return get_DotProduct(other) / (get_Magnitude() * other.get_Magnitude());
  }

  double get_Distance(my_point const& other) const
  {
    return get_SquaredEuclideanDistance(other);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_x);
    t.member(m_y);
    t.member(m_z);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Stores the points in a full cluster, weight, and nearest neighbor.
//////////////////////////////////////////////////////////////////////
class my_vertex_property
{
private:
  /// @todo Currently only a single point average of all
  /// the true points is stored. This is naive. It will later be changed.
  my_point m_centroid;

  /// The number of points in the cluster.
  int m_count;

  /// The ID of the closest cluster.
  size_t m_my_nearest_neighbor;

public:
  typedef int property_type;

  my_vertex_property()
    : m_centroid(0.0), m_count(0), m_my_nearest_neighbor(0)
  { }

  my_vertex_property(my_point const& other)
    : m_centroid(other), m_count(1), m_my_nearest_neighbor(0)
  { }

  my_point get_Centroid() const
  {
    return m_centroid;
  }

  void set_Centroid(double new_x, double new_y, double new_z)
  {
    m_centroid.set(new_x, new_y, new_z);
  }

  void set_Centroid(my_vertex_property const& other)
  {
    m_centroid = other.m_centroid;
    m_count = other.m_count;
  }

  size_t get_nearest_neighbor() const
  {
    return m_my_nearest_neighbor;
  }

  void set_nearest_neighbor(size_t nearest_neighbor_id)
  {
    m_my_nearest_neighbor = nearest_neighbor_id;
  }

  double get_Distance(const my_vertex_property& other) const
  {
    return m_centroid.get_Distance(other.m_centroid);
  }

  int get_Count() const
  {
    return m_count;
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Merges this vertex with another and returns a
  /// new my_vertex_property. This is currently naive, using centroids.
  /// @param other The other cluster to be merged.
  /// @return my_vertex_property The new, merged, cluster.
  //////////////////////////////////////////////////////////////////////
  my_vertex_property mergeVertex(const my_vertex_property& other) const
  {
    //calculate centroid
    my_point p1(m_centroid);
    my_point p2(other.m_centroid);

    p1.scale(m_count);
    p2.scale(other.m_count);

    p1.add(p2);
    p1.scale(1.0 / (other.m_count + m_count));

    my_vertex_property vp_out(p1);
    vp_out.m_count = other.m_count + m_count;

    return vp_out;
  }

  my_vertex_property operator=(my_vertex_property const& vertex)
  {
    m_centroid = vertex.m_centroid;
    m_count = vertex.m_count;
    m_my_nearest_neighbor = vertex.m_my_nearest_neighbor;
    return my_vertex_property(vertex);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_centroid);
    t.member(m_count);
    t.member(m_my_nearest_neighbor);
  }
};


namespace stapl
{
  STAPL_PROXY_HEADER(my_vertex_property)
  {
    STAPL_PROXY_DEFINES(my_vertex_property)
    STAPL_PROXY_METHOD_RETURN(get_Centroid, my_point)
    STAPL_PROXY_METHOD(set_Centroid, double, double, double)
    STAPL_PROXY_METHOD(set_Centroid, my_vertex_property)
    STAPL_PROXY_METHOD_RETURN(get_nearest_neighbor, size_t)
    STAPL_PROXY_METHOD(set_nearest_neighbor, size_t)
    STAPL_PROXY_METHOD_RETURN(get_Distance, my_vertex_property&, double)
    STAPL_PROXY_METHOD_RETURN(get_Count, int)
    STAPL_PROXY_METHOD_RETURN(mergeVertex, my_vertex_property,
      my_vertex_property)
    STAPL_PROXY_METHOD_RETURN(operator=, my_vertex_property,
      my_vertex_property)
    STAPL_PROXY_METHOD(define_type, stapl::typer)
  };
}


//////////////////////////////////////////////////////////////////////
/// @brief Work function to merge vertices.
//////////////////////////////////////////////////////////////////////
struct functor_merge_vertex
{
  typedef my_vertex_property result_type;

  result_type operator()(my_vertex_property& vertex1,
                         my_vertex_property const& vertex2)
  {
    return vertex1.mergeVertex(vertex2);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Creates pairs of the vertices to be merged in the next level.
//////////////////////////////////////////////////////////////////////
struct make_node_pairs
{
  typedef std::pair<size_t, double> result_type;

  my_vertex_property m_vertex_prop;
  size_t m_vertex_descriptor;

  make_node_pairs(my_vertex_property clust_prop, size_t vertex_id)
    : m_vertex_prop(clust_prop), m_vertex_descriptor(vertex_id)
  { }

  template<typename GraphNode>
  result_type operator()(GraphNode element) const
  {
    if (element.descriptor() != m_vertex_descriptor) {
      double distance =
         m_vertex_prop.get_Distance(element.property().property);
      return std::make_pair(element.descriptor(), distance);
    }
    return std::make_pair(0, 0.0);
  }

  void define_type(typer &t)
  {
    t.member(m_vertex_prop);
    t.member(m_vertex_descriptor);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Reduces two single element pairs to the smaller value.
/// Multi-element pairs are preferred.
//////////////////////////////////////////////////////////////////////
struct pick_smaller_node
{
  typedef std::pair<size_t, double> result_type;

  result_type operator()(result_type el1, result_type el2) const
  {
    const result_type null_value = result_type(0, 0.0);

    if (el1 != null_value && el2 != null_value) {
      if (el1.second < el2.second) {
        return el1;
      } else {
        return el2;
      }
    } else if (el1 == null_value) {
      return el2;
    } else {
      return el1;
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to determine the nearest neighbors
/// for all of the clusters.
//////////////////////////////////////////////////////////////////////
struct find_nearest_node
{
  typedef void result_type;

  find_nearest_node()
  { }

  template<typename Node, typename GraphView>
  result_type operator()(Node node, const GraphView g_view)
  {
    make_node_pairs pairs_wf(
            node.property().property, node.descriptor());
    size_t mynearestnode =
        (nc_map_reduce(pairs_wf, pick_smaller_node(), g_view)).first;
    node.property().property.set_nearest_neighbor(mynearestnode);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Gives an empty property. This is needed for create_level.
//////////////////////////////////////////////////////////////////////
struct empty_functor
{
  typedef int value_type;

  template<class Graph>
  properties::no_property operator()(Graph& g, size_t lvl) const
  {
    return properties::no_property();
  }

  template<class EdgePropertyType>
  properties::no_property operator()(EdgePropertyType& p,
                                     properties::no_property const& ref) const
  {
    return properties::no_property();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Creates a property map of the graph, indicating which vertices are
/// to be clustered with which other vertices.
//////////////////////////////////////////////////////////////////////
struct create_property_map
{
  typedef void result_type;

  create_property_map()
  { }

  template<typename Vertex, typename PropMap, typename GraphView>
  result_type operator()
    (Vertex element, PropMap& property_map, GraphView whole_graph)
  {
    size_t element_descriptor = element.descriptor();
    size_t element_nearest_node =
      element.property().property.get_nearest_neighbor();
    if (whole_graph[element_nearest_node].property()
         .property.get_nearest_neighbor() == element_descriptor) {
      // Nodes are to be clustered
      if (element_descriptor < element_nearest_node) {
        property_map[element_descriptor] = element_descriptor;
      } else {
        property_map[element_descriptor] = element_nearest_node;
      }
    } else {
      // Node is not to be clustered
      property_map[element_descriptor] = element_descriptor;
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Used to fill the graph used in @ref agglomerative_clustering
/// with starting points.
//////////////////////////////////////////////////////////////////////
struct fill_graph
{
  typedef void result_type;

  template<typename Vertex, typename Point>
  void operator()(Vertex element, Point point) {
    element.property() = my_vertex_property(point);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper class for agglomerative clustering algorithm
/// Class needed to store so as to contain typedef for return_type
//////////////////////////////////////////////////////////////////////
class agglomerative_clustering
{
private:
  typedef stapl::static_array<size_t> array_type;
  typedef array_view<array_type> array_vw;
  typedef graph<stapl::DIRECTED, stapl::MULTIEDGES,
    super_vertex_property<my_vertex_property>,
    super_edge_property<properties::no_property> > graph_type;
  typedef graph_view<graph_type> graph_vw;
  typedef graph_external_property_map<graph_vw, size_t, array_vw>
    graph_ext_prop_map;

public:
  typedef std::vector<graph_vw> return_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Agglomeratively clusters an array of points.
  /// @param inputView The view of the graph to be clustered.
  //////////////////////////////////////////////////////////////////////
  template<typename View>
  return_type cluster(View inputView) {
    graph_type* g = new graph_type(inputView.size());
    graph_vw g_vw(g);

    map_func(fill_graph(), g_vw, inputView);

    std::vector<graph_vw> g_level_vw;
    g_level_vw.push_back(g_vw);

    while (g_vw.size() > 1)
    {
      //Generates the array of nearest neighbors
      nc_map_func(find_nearest_node(), g_vw, make_repeat_view(g_vw));

      array_type group_array(g_vw.size());
      array_vw group_array_vw(group_array);
      nc_map_func(create_property_map(), g_vw, make_repeat_view(group_array_vw),
        make_repeat_view(g_vw));

      graph_ext_prop_map group_id_prop_map(g_vw, group_array_vw);

      g_level_vw.push_back(create_level(g_vw, group_id_prop_map,
        functor_merge_vertex(), empty_functor()));

      g_vw = g_level_vw.back();
    }
    return g_level_vw;
  }

  #ifdef SHOW_RESULTS
  //////////////////////////////////////////////////////////////////////
  /// @brief Converts a point to its RGB hex representation.
  //////////////////////////////////////////////////////////////////////
  std::string ConvertPointToRGBColor(my_point const& point) {
    char b[10];
    sprintf(&b[0], "%02X", (unsigned char)(point.m_x * 255));
    sprintf(&b[2], "%02X", (unsigned char)(point.m_y * 255));
    sprintf(&b[4], "%02X", (unsigned char)(point.m_z * 255));
    return std::string(b);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Writes the graph to a dot file (colors points using RGB)
  /// @todo Make more efficient version that does not redundantly build strings
  /// during the creation of dot file.
  //////////////////////////////////////////////////////////////////////
  template<typename Clusters>
  void printgraph2dotfile(Clusters clusters, char const* file="cluster.dot")
  {
    std::map<size_t, std::map<size_t, std::string> > mymap;

    do_once([&clusters, &mymap, file, this](void) {
      for (size_t lvl=0; lvl<clusters.size(); ++lvl) {
        for (size_t n=0; n<clusters[lvl].size(); ++n) {
          auto v = clusters[lvl][n];
          int v_id = v.descriptor();
          if (v.property().children.size() == 1) {
            mymap[lvl][v_id] =  mymap[lvl-1][v.property().children[0]];
            continue;
          }
          std::stringstream ss;
          ss << "subgraph cluster_" << lvl << "_" << v_id << " {\n";

          for (size_t k=0; k<v.property().children.size(); ++k) {
            ss << mymap[lvl-1][v.property().children[k]] << "\n";
          }
          if (lvl != 0) {
            ss << "label = \"";
          } else {
            ss << "\"[" << v_id<< "]\\n";
          }
          my_point p = v.property().property.get_Centroid();
          ss << p.m_x << "\\n" << p.m_y << "\\n" << p.m_z << "\";\n";
          ss << "style=\"filled\";fillcolor=\"#";
          ss << ConvertPointToRGBColor(p) << "\";\n}\n";

          mymap[lvl][v_id] = ss.str();
        }
      }
      std::ofstream ofile;
      ofile.open(file, std::fstream::out);
      ofile << "digraph pGraph {\n";
      ofile << mymap[clusters.size()-1][0];
      ofile << "}\n";
      ofile.close();
    });
    rmi_fence();
  }
  #endif
};



#endif /* STAPL_BENCHMARK_LONESTAR_AGGLOMERATIVECLUSTERING_HPP */
