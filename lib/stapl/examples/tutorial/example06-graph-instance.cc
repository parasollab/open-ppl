/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <stapl/graph.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/views/repeated_view.hpp>

struct add_edges
{
private:
  long m_n;

public:
  add_edges(long n)
    : m_n(n)
  { }

  // Function that modifies the property of each element in the view and
  // adds a predefined edge. The variable i is a proxy to a vertex contained
  // in the graph and j to a repeated view of the graph.
  template<typename T1, typename T2>
  void operator()(T1 i, T2 j)
  {
    // STL pseudo-random number generator.
    std::random_device rd;
    std::mt19937 mt(rd());

    // Store the descriptor of the vertex referenced by i.
    long m_desc = i.descriptor();

    // Assigns a random number between 0 and m_n-1 to the property of the
    // vertex in i.
    i.property() = mt() % m_n;

    // Adds an edge with start at the vertex referenced by i and end at the
    // vertex that follows (in the order of creation of the static graph) the
    // one referenced by i. If there is no vertex that follows, the first
    // vertex will be used. Note that this assumes all vertices with
    // descriptors [0, m_n) have been inserted into the graph prior to the call
    // to map_func.
    j.add_edge(m_desc, (m_desc + 1) % m_n);
  }

  // Function used to serialize work function instances as part
  // of sending them between locations.
  void define_type(stapl::typer& t)
  {
    t.member(m_n);
  }

};

struct update_properties
{
public:
  // Function that modifies the property of each element in the view.
  // The variable i is a proxy to a vertex contained in the graph.
  template<typename T>
  void operator()(T i)
  {
    // STL pseudo-random number generator.
    std::random_device rd;
    std::mt19937 mt(rd());

    // Change the default property of the vertex from 0 to some random
    // double number between 0 and 1000.
    if (i.property() == 0)
      i.property() = mt() % 1000 + (mt() % 100) * 0.001;
  }
};

stapl::exit_code stapl_main(int argc, char **argv)
{
  // Check for input parameter.
  if (argc < 2) {
    stapl::do_once(
      [] { std::cout << "usage: ./exec graph_size" << std::endl; }
    );
    return EXIT_FAILURE;
  }

  // Obtain the number of locations and the id of individual location.
  long num_loc = stapl::get_num_locations();
  long loc_id = stapl::get_location_id();

  // Number of vertices of stapl::graph and stapl::dynamic_graph at the
  // beginning of the program.
  long n = atol(argv[1]) * 2 * num_loc;

  // Define the type of a directed stapl::graph of integers that does not
  // allow multi edges.
  using graph_type = stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, int>;

  // Define the type of an undirected stapl::dynamic_graph of doubles
  // that allows multi edges.
  using dynamic_graph_type = stapl::dynamic_graph<stapl::UNDIRECTED,
                                                  stapl::MULTIEDGES,
                                                  double>;

  // Define the type of a vertex iterator for an undirected dynamic graph
  // of doubles that allows multi edges.
  using vertex_iterator_type = stapl::dynamic_graph<stapl::UNDIRECTED,
                               stapl::MULTIEDGES,
                               double>::vertex_iterator;

  // STL pseudo-random number generator.
  std::random_device rd;
  std::mt19937 mt(rd());

  // Construction of a stapl::graph and a stapl::dynamic_graph.
  graph_type s_g(n);
  dynamic_graph_type d_g(n);

  // Construction of two stapl::graph_view instances. The first view is
  // defined over a stapl::graph and the second over a stapl::dynamic_graph.
  stapl::graph_view<graph_type> vs_g(s_g);
  stapl::graph_view<dynamic_graph_type> vd_g(d_g);

  // Add vertices to the stapl::dynamic_graph in different ways.
  stapl::do_once(
    [&]{
      // Add vertices following the natural order of the vertices' descriptors.
      vd_g.add_vertex( n, 123.125);
      vd_g.add_vertex( n + 1, 456.25);

      // Add a vertex using the first parameter as descriptor and the second
      // as property.
      vd_g.add_vertex( n + 99 , 1080.5);

      // Add a vertex with automatically created descriptor. Note that it is
      // not required to specify the vertex property.
      vd_g.add_vertex( 1101.75 );
      vd_g.add_vertex();
    }
  );

  // Assigns properties random values and adds edges to the static graph s_g.
  stapl::map_func(add_edges(n), vs_g, stapl::make_repeat_view(vs_g));

  // Declares three iterators to add edges randomly to the dynamic graph d_g.
  // vi is used to iterate through the vertices, vi_end points past the final
  // vertex to stop the iteration and vi_tmp is used to point to a random
  // vertex.
  stapl::graph_view<dynamic_graph_type>::vertex_iterator vi = vd_g.begin();
  stapl::graph_view<dynamic_graph_type>::vertex_iterator vi_tmp;
  stapl::graph_view<dynamic_graph_type>::vertex_iterator vi_end = vd_g.end();

  // Warning: the num_vertices() method is expensive, as the location
  // calling it sends a message to all other locations to get the local size
  // from each and then sum them together.
  size_t d_g_num_vertices = d_g.num_vertices();

  // Randomly adds an edge to each vertex of the dynamic graph d_g.
  for (std::size_t i = loc_id; i < d_g_num_vertices; i += num_loc)
  {
    // Select a random vertex and add an edge to it from the current vertex
    // referenced by the iterator “vi + i”.
    vi_tmp = vi + mt()%d_g_num_vertices;
    vd_g.add_edge( (*(vi + i)).descriptor() ,  (*vi_tmp).descriptor());
  }

  // Changes the properties of vertices that are equal to zero to some double
  // between 0 and 1000.
  stapl::map_func(update_properties(), vd_g);

  // Print the current state of the stapl::graph "s_g" and the
  // stapl::dynamic_graph "d_g". Also, one edge of "d_g" is erased and
  // the result is shown.
  stapl::do_once(
    [&](){
      // Begin the printing of the state of the stapl::graph "s_g".
      // Print the number of vertices in "s_g".
      std::cout << "Num static graph vertices: " << n
                << std::endl;
      std::cout << "Printing degree, property and edges of each vertex in s_g."
                << std::endl;

      // Print the vertex descriptor, the number of edges, the property and
      // the list of edges of each vertex in "s_g". Note that we are accessing
      // each vertex with the operator [] according to the natural order used
      // during the creation of the vertices. Also, note that this print is
      // done only for the clarity of the output. The performance of this
      // approach is poor. One should use map_func and other parallel
      // algorithms to process the elements of a container.
      for (int i = 0; i < n; ++i)
      {
        std::cout << "Degree of Vertex [" << s_g[i].descriptor()
                  << "] = " << s_g[i].size() << ". Property = "
                  << s_g[i].property() << std::endl;

        // Print the edge list of the vertex.
        if ( s_g[i].size() > 0 )
        {
          // Declare an iterator to the beginning of the edge list.
          auto vi = s_g[i].begin();

          for (; vi != s_g[i].end(); ++vi)
          {
            // Print the target of the edge that starts at vertex s_g[i].
            std::cout << "Target: " << (*vi).target() << std::endl;
          }
        }
      }

      std::cout << std::endl;

      // Begin the printing of the state of the stapl::dynamic_graph "d_g".
      std::cout << "Num dynamic graph vertices: " << d_g_num_vertices
                << std::endl;
      std::cout << "Printing degree, property and edges of each vertex in d_g."
                << std::endl;

      // Get the begin and the end of the vertex list of "d_g".
      vertex_iterator_type vi = d_g.begin();
      vertex_iterator_type vi_end = d_g.end();

      // Print the vertex descriptor, the number of edges, the property and
      // the list of edges of each vertex in "d_g". Note the use of vertex
      // iterators in order to print the vertex list of a dynamic graph. Also,
      // note that this print using iterators is done only for the clarity of
      // the output. The performance of this approach is poor. One should use
      // map_func and other parallel algorithms to process the elements of a
      // container.
      for (; vi != vi_end; ++vi)
      {
        std::cout << "Degree of Vertex [" << (*vi).descriptor()
                  << "] = " << (*vi).size()
                  << ". Property: " << (*vi).property() << std::endl;

        if ( (*vi).size() > 0)
        {
          // Get the beginning of the edge list of the vertex referenced by
          // "vi".
          auto ei = (*vi).begin();

          // Print the list of edges of the vertex referenced by "vi".
          for (; ei != (*vi).end(); ++ei)
          {
            // Print the target vertex of the edge that begins in the vertex
            // referenced by "vi".
            std::cout << "Target: " << (*ei).target() << std::endl;
          }
        }
      }

      std::cout << std::endl;

      // Begin the erasing of the last vertex in the stapl::dynamic_graph
      // "d_g".
      std::cout << "Eliminating last vertex: Vertex["
                << (*(d_g.end() - 1)).descriptor()
                << "]" << std::endl;

      // Erase the last vertex stored in "d_g". The descriptor of the vertex
      // must be provided in order to erase a vertex.
      d_g.delete_vertex((*(d_g.end() - 1)).descriptor());

      // Get the begin of the vertex list of "d_g".
      vi = d_g.begin();

      // Print the vertex descriptor, the number of edges and the property of
      // each vertex in "d_g".
      for (; vi != d_g.end(); ++vi)
      {
        std::cout << "Degree of Vertex [" << (*vi).descriptor()
                  << "] in d_g = " << (*vi).size()
                  << " with property: " << (*vi).property() << std::endl;
      }
    }
  );

  return EXIT_SUCCESS;
}
