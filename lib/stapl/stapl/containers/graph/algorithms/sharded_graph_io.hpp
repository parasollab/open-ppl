/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/
#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_SHARDED_GRAPH_IO_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_SHARDED_GRAPH_IO_HPP

#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/utility/unravel_index.hpp>

#include <boost/lexical_cast.hpp>

#include <string>
#include <fstream>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Representation of how an input graph is sharded on disk.
//////////////////////////////////////////////////////////////////////
struct sharded_description
{
  /// The directory where the shards are stored
  std::string basedir;
  /// Total number of vertices
  std::size_t vertices;
  /// Total number of edges
  std::size_t edges;
  /// Total number of shards
  std::size_t total_shards;
  /// Shape of the nested directory structure
  std::vector<std::size_t> shape;

  sharded_description(std::string filename, std::size_t n, std::size_t m,
                      std::size_t t, std::vector<std::size_t> s)
    : vertices(n), edges(n), total_shards(t), shape(std::move(s))
  {
    auto last_slash = filename.find_last_of('/');
    basedir = filename.substr(0, last_slash);
  }

  void define_type(typer& t)
  {
    t.member(basedir);
    t.member(vertices);
    t.member(edges);
    t.member(total_shards);
    t.member(shape);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Read the metadata file from disk and create a description
///        of how the input graph is sharded.
///
/// @note This is a collective operation. One location will read the
///       file and broadcast its contents to all other locations.
/// @param filename Path to the metadata file
/// @return Description of the sharding
//////////////////////////////////////////////////////////////////////
sharded_description read_metadata_file(std::string const& filename)
{
  return stapl::do_once([&]() {
    std::ifstream file(filename, std::ios::in);

    if (!file.is_open())
      stapl::abort("Can't open metadata file");

    // Read in the first line
    std::size_t num_verts, num_edges, num_shards;
    file >> num_verts >> num_edges >> num_shards;

    // Read in the shape of the sharded space
    std::vector<std::size_t> shape;

    while (!file.eof())
    {
      std::string line;
      std::getline(file, line);

      if (line.size() == 0 || line[0] == '#')
        continue;

      shape.push_back(boost::lexical_cast<std::size_t>(line));
    }

    file.close();

    sharded_description desc{
      filename, num_verts, num_edges, num_shards, std::move(shape)
    };
    return desc;
  });
}

//////////////////////////////////////////////////////////////////////
/// @brief Work function that reads in a single shard and populates
///        the graph.
///
/// @tparam LineReader Functor to read in a single line
//////////////////////////////////////////////////////////////////////
template<typename LineReader>
class populate_graph_from_shards
{
  LineReader m_line_reader;
  sharded_description m_desc;

public:
  using result_type = void;

  populate_graph_from_shards(LineReader line_reader, sharded_description desc)
    : m_line_reader(std::move(line_reader)), m_desc(std::move(desc))
  { }

  template<typename Index, typename GraphView>
  result_type operator()(Index linear, GraphView g)
  {
    // Get multidimensional index
    auto unravelled = unravel_index(linear, m_desc.shape);

    // Build filename
    std::stringstream shard_name;
    shard_name << m_desc.basedir;

    for (auto x : unravelled)
      shard_name << '/' << x;

    // Open file
    std::ifstream file;
    file.open(shard_name.str().c_str());

    if (!file.is_open())
      stapl::abort("Can't open shard.");

    // Read this shard and populate the graph
    parallel_reader()(
      file, m_line_reader, g.get_container(),
      std::numeric_limits<size_t>::max(), true
    );

    file.close();
  }

  void define_type(typer& t)
  {
    stapl::abort("This work function should never be shipped.");
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Algorithm to read a graph that is sharded in a nested way.
///
/// Uses the user-provided reader to read each line and add the edges
/// from the line to the graph.
///
/// Input graph must be sharded using the nested sharder. A sharded file is
/// split into multiple input files and a metadata file that keeps
/// track of the number of vertices, edges, total shards and nested dimensions.
///
/// An example metadata file would be:
///   1000 1000000 128
///   2
///   100
///
/// Each line after the first line represents the shape of the space. The
/// files would then be in nested directories (e.g., base/1/24)
///
/// @param filename The name of the input metadata file.
/// @param line_reader The functor used to read-in a line of text from
/// file and add the appropriate edges to the provided aggregator,
/// LineReader is given a stream with the line and an Edge-Aggregator.
/// It should read the stream and add the appropriate edges to
/// the provided Aggregator.
/// LineReader should return '0' for success, '1' for failure.
///
/// @return A @ref graph_view over the output graph, populated with the
/// vertices and edges from the input file.
//////////////////////////////////////////////////////////////////////
template<typename Graph, typename LineReader>
graph_view<Graph> nested_sharded_graph_reader(std::string filename,
                                       LineReader line_reader)
{
  // Get description of the sharding
  sharded_description desc = read_metadata_file(filename);

  Graph* g = new Graph(desc.vertices);
  graph_view<Graph> vw(g);

  // Call the parallel reader on each shard
  populate_graph_from_shards<LineReader> wf{std::move(line_reader), desc};
  map_func(
    wf, counting_view<std::size_t>(desc.total_shards), make_repeat_view(vw)
  );

  return vw;
}

} // namespace stapl

#endif
