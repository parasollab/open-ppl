/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_P_GRAPH_IO_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_P_GRAPH_IO_HPP

#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/aggregator.hpp>
#include <stapl/views/native_view.hpp>
#include <stapl/skeletons/serial.hpp>
#include <string>
#include <sstream>
#include <fstream>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to print the property on a vertex/edge (if it has one).
///
/// The property must have the output operator (<<) defined for it.
/// @tparam P Type of the vertex or edge property.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename P>
struct print_property_struct
{
  template <typename T>
  void operator() (T t, std::ostream& os) const
  { os << t.property() << " "; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for functor to print the property on a
/// vertex/edge when it does not exist (stapl::properties::no_property).
///
/// stapl::properties::no_property is not printed.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<>
struct print_property_struct<properties::no_property>
{
  template <typename T>
  void operator() (T t, std::ostream& os) const
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to print a native-view of the input graph
/// to the given file.
///
/// Uses the user-provided output-function to print each vertex and
/// its adjacencies.
/// @tparam OutputFunctor The functor used to print a vertex and its
/// adjacencies.
/// @param filename The name of the output file. If the name is an empty
/// string, the graph is output to std::cout.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename OutputFunctor>
struct output_graph_to_file
{
protected:
  std::string   m_filename;
  OutputFunctor m_vertex_out_func;

public:
  typedef void result_type;

  output_graph_to_file()
    : m_filename(""), m_vertex_out_func()
  { }

  output_graph_to_file(std::string filename)
    : m_filename(filename), m_vertex_out_func()
  { }

  output_graph_to_file(std::string filename, OutputFunctor f)
    : m_filename(filename), m_vertex_out_func(f)
  { }

  template<typename Vw>
  result_type operator() (Vw const& v)
  {
    auto vi = v.begin(), vi_end = v.end();

    if (m_filename.size() == 0) {
      for (; vi != vi_end; ++vi)
        m_vertex_out_func(*vi, std::cout);
    } else {
      std::ofstream ofile;
      ofile.open(m_filename.c_str(), std::fstream::out | std::fstream::app);

      for (; vi != vi_end; ++vi)
        m_vertex_out_func(*vi, ofile);

      ofile.close();
    }
  }

  void define_type(typer& t)
  {
    t.member(m_filename);
    t.member(m_vertex_out_func);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A functor to print a vertex and its adjacencies.
///
/// The output is in the form of an adjacency-list, i.e.,
/// format:
/// @verbatim
/// source0 target00 target01 target02 ...
/// source1 target10 target11 target12 ...
/// :
/// @endverbatim
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct vertex_to_adjlist_func
{
  template<typename Vertex>
  void operator() (Vertex v, std::ostream& os)
  {
    os << v.descriptor() << " ";

    // print the edges...
    typedef typename Vertex::adj_edge_iterator aei_t;
    for (aei_t aei = v.begin(); aei !=  v.end(); ++aei) {
      os << (*aei).target() << " ";
    }
    os << std::endl;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A functor to print a vertex and its adjacencies.
///
/// The output is in the form of an adjacency-list, i.e.,
/// format:
/// @verbatim
/// VID VERTEXPROP #edges TGT WEIGHT TGT WEIGHT
/// VID VERTEX #edges TGT WEIGHT TGT WEIGHT
/// :
/// @endverbatim
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct vertex_to_adjlist_prop_func
{
  template<typename Vertex>
  void operator() (Vertex v, std::ostream& os)
  {
    //format: VID VERTEX #edges TGT WEIGHT TGT WEIGHT ...
    print_property_struct<typename Vertex::property_type> print_vertex_prop_f;
    os << v.descriptor() << " ";
    print_vertex_prop_f(v, os);
    os << v.size() << " ";

    // print the edges...
    typedef typename Vertex::adj_edge_iterator aei_t;
    print_property_struct<typename aei_t::value_type::property_type>
      print_edge_prop_f;
    for (aei_t aei = v.begin(); aei !=  v.end(); ++aei) {
      os << (*aei).target() << " ";
      print_edge_prop_f(*aei, os);
    }
    os << std::endl;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A functor to print a vertex and its adjacencies.
///
/// The output is in the form of an edge-list.
/// format:
/// @verbatim
/// SRC TGT
/// SRC TGT
///  ...
/// @endverbatim
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct vertex_to_edgelist_func
{
protected:
  bool m_is_directed;

public:
  vertex_to_edgelist_func(bool is_directed = false)
      : m_is_directed(is_directed)
  { }

  template<typename Vertex>
  void operator() (Vertex v, std::ostream& os)
  {
    //format: SRC TGT\nSRC TGT\n  ...
    for (typename Vertex::adj_edge_iterator aei = v.begin();
         aei != v.end(); ++aei) {
      if (m_is_directed || (*aei).source() <= (*aei).target() )
        os << (*aei).source() << "  " << (*aei).target() << std::endl;
    }
  }

  void define_type(typer& t)
  {
    t.member(m_is_directed);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A functor to print a vertex and its adjacencies in 1-based
///        form.
///
/// The output is in the form of an edge-list.
/// format:
/// @verbatim
/// SRC TGT
/// SRC TGT
///  ...
/// @endverbatim
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct vertex_to_edgelist_1based_func
{
protected:
  bool m_is_directed;

public:
  vertex_to_edgelist_1based_func(bool is_directed = false)
      : m_is_directed(is_directed)
  { }

  template<typename Vertex>
  void operator() (Vertex&& v, std::ostream& os)
  {
    auto v_end = v.end();
    //format: SRC TGT\nSRC TGT\n  ...
    for (typename Vertex::adj_edge_iterator aei = v.begin();
         aei != v_end; ++aei) {
      if (m_is_directed || (*aei).source() < (*aei).target() )
        os << (*aei).source() + 1 << "  " << (*aei).target() + 1 << std::endl;
    }
  }

  void define_type(typer& t)
  {
    t.member(m_is_directed);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A functor to print a vertex and its adjacencies in DOT format.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct vertex_to_DOT_func
{
protected:
  bool m_is_directed;
  std::string m_edge_connector;

public:
  vertex_to_DOT_func(bool is_directed = false)
    : m_is_directed(is_directed)
  {
    if (is_directed)
      m_edge_connector = " -> ";
    else
      m_edge_connector = " -- ";
  }

  template<typename Vertex>
  void operator() (Vertex v, std::ostream& os)
  {
    for (typename Vertex::adj_edge_iterator aei = v.begin();
         aei != v.end(); ++aei) {
      if (m_is_directed || (*aei).source() < (*aei).target() )
        os << "\t" << (*aei).source() << m_edge_connector
           << (*aei).target() << ";" << std::endl;
    }
  }

  void define_type(typer& t)
  {
    t.member(m_is_directed);
    t.member(m_edge_connector);
  }
};



//////////////////////////////////////////////////////////////////////
/// @brief Function to output the adjacency-list of a graph.
///
/// Format:
/// @verbatim
/// #vertices #edges
/// source0 target00 target01 target02 ...
/// source1 target10 target11 target12 ...
/// :
/// @endverbatim
/// @param g The input @ref graph_view to be printed.
/// @param filename The name of the file where the graph will be printed.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GraphVw>
void write_adj_list(GraphVw& g, std::string filename = "")
{
  do_once([&]() {
      std::ofstream ofile;
      ofile.open(filename.c_str(), std::fstream::out | std::fstream::app);
      ofile << g.num_vertices() << " " << g.num_edges() << std::endl;
      ofile.close();
    });

  // essentially, this call creates p tasks, one per location,
  // executed one-after-another
  // each task takes a native-view and dumps its graph to (append) the file
  serial(output_graph_to_file<vertex_to_adjlist_func>(filename),
         stapl::native_view(g));
}

//////////////////////////////////////////////////////////////////////
/// @brief Function to output the adjacency-list of a graph, when it
/// has both vertex and edge properties..
///
/// Format:
/// @verbatim
/// VID VERTEXPROP #edges TGT0 EDGEPROP0 TGT1 EDGEPROP1 ...
/// VID VERTEXPROP #edges TGT0 EDGEPROP0 TGT1 EDGEPROP1 ...
/// :
/// @endverbatim
/// @param g The input @ref graph_view to be printed.
/// @param filename The name of the file where the graph will be printed.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GraphVw>
void write_adj_list_prop(GraphVw& g, std::string filename = "")
{
  do_once([&]() {
      std::ofstream ofile;
      ofile.open(filename.c_str(), std::fstream::out | std::fstream::app);
      ofile << g.num_vertices() << " " << g.num_edges() << std::endl;
      ofile.close();
    });

  // essentially, this call creates p tasks, one per location,
  // executed one-after-another
  // each task takes a native-view and dumps its graph to (append) the file
  serial(output_graph_to_file<vertex_to_adjlist_prop_func>(filename),
         stapl::native_view(g));
}


//////////////////////////////////////////////////////////////////////
/// @brief Function to output the adjacency-list of a graph in Matrix
///        Market format.
///
/// Format:
/// @verbatim
/// TOTAL_ROWS TOTAL_COLUMNS TOTAL_EDGES
/// ROW        COLUMN
/// ROW        COLUMN
/// :
/// @endverbatim
/// @param g The input @ref graph_view to be printed.
/// @param filename The name of the file where the graph will be printed.
/// @param comments Comments added at the beginning of the file.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GraphVw>
void write_matrix_market(GraphVw& g, std::string filename = "",
                         std::string comments = "")
{
  do_once([&]() {
      std::ofstream ofile(filename.c_str());
      ofile << comments << std::endl;
      ofile << g.num_vertices() << "  " << g.num_vertices() << " "
            << g.num_edges() << std::endl;
      ofile.close();
    });

  // essentially, this call creates p tasks, one per location,
  // executed one-after-another
  // each task takes a native-view and dumps its graph (appends) to the file
  serial(output_graph_to_file<vertex_to_edgelist_1based_func>(filename,
                              vertex_to_edgelist_1based_func(g.is_directed())),
                              stapl::native_view(g));
}



//////////////////////////////////////////////////////////////////////
/// @brief Function to output the edge-list of a graph, without properties.
///
/// Format:
/// @verbatim
/// SRC0 TGT0
/// SRC1 TGT1
/// SRC2 TGT2
/// :
/// @endverbatim
/// @param g The input @ref graph_view to be printed.
/// @param filename The name of the file where the graph will be printed.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GraphVw>
void write_edge_list(GraphVw& g, std::string filename = "")
{
  do_once([&]() {
      std::ofstream ofile;
      ofile.open(filename.c_str(), std::fstream::out | std::fstream::app);
      ofile << g.num_vertices() << " " << g.num_edges() << std::endl;
      ofile.close();
    });

  // essentially, this call creates p tasks, one per location,
  // executed one-after-another
  // each task takes a native-view and dumps its graph to (append) the file
  serial(output_graph_to_file<vertex_to_edgelist_func>(
           filename, vertex_to_edgelist_func(g.is_directed())),
         stapl::native_view(g));
}


//////////////////////////////////////////////////////////////////////
/// @brief Function to output a DOT graph, without properties.
/// @param g The input @ref graph_view to be printed.
/// @param filename The name of the file where the graph will be printed.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GraphVw>
void write_dot(GraphVw& g, std::string filename)
{
  do_once([&]() {
      std::string header;
      if (g.is_directed())
        header = "digraph ";
      else
        header = "graph ";
      header += "pGraph {";
      std::ofstream ofile;
      ofile.open(filename.c_str(), std::fstream::out);
      ofile << header << std::endl;
      ofile.close();
    });


  // essentially, this call creates p tasks, one per location,
  // executed one-after-another
  // each task takes a native-view and dumps its graph to (append) the file
  serial(output_graph_to_file<vertex_to_DOT_func>(
           filename, vertex_to_DOT_func(g.is_directed())),
         stapl::native_view(g));

  do_once([&]() {
      std::ofstream ofile;
      ofile.open(filename.c_str(), std::fstream::out | std::fstream::app);
      ofile << "}" << std::endl;
      ofile.close();
    });
}


//////////////////////////////////////////////////////////////////////
/// @brief Function to output the graph in PMPL format.
/// @param g The input @ref graph_view to be printed.
/// @param filename The name of the file where the graph will be printed.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GraphVw>
void write_PMPL_graph(GraphVw& g, std::string filename = "")
{
  do_once([&]() {
      std::ofstream ofile;
      ofile.open(filename.c_str(), std::fstream::out | std::fstream::app);
      ofile << "#####GRAPHSTART#####" << std::endl
            << g.num_vertices() << " " << g.num_edges() << " "
            << g.num_vertices() << std::endl;
      ofile.close();
    });

  // essentially, this call creates p tasks, one per location,
  // executed one-after-another
  // each task takes a native-view and dumps its graph to (append) the file
  serial(output_graph_to_file<vertex_to_adjlist_func>(filename),
         stapl::native_view(g));

  do_once([&]() {
      std::ofstream ofile;
      ofile.open(filename.c_str(), std::fstream::out | std::fstream::app);
      ofile << std::endl << "#####GRAPHSTOP#####" << std::endl;
      ofile.close();
    });
}

namespace graph_io_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Default work-function to read header lines of a file.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct default_header_reader
{
  //////////////////////////////////////////////////////////////////////
  /// @param ifs The input file-stream.
  /// @return Returns the number of vertices of the graph.
  //////////////////////////////////////////////////////////////////////
  size_t operator()(std::ifstream& ifs)
  {
    size_t num_verts=0, num_edges=0;
    ifs >> num_verts >> num_edges;
    return num_verts;
  }
};

} // namespace graph_io_impl


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to read a graph from the given input file.
/// @ingroup pgraphAlgoDetails
///
/// Uses the user-provided reader to read each line and add the edges
/// from the line to the graph.
//////////////////////////////////////////////////////////////////////
struct parallel_reader
{
  //////////////////////////////////////////////////////////////////////
  /// @param ifs The input file-stream.
  /// @param line_reader The functor used to read-in a line of text from
  /// file and add the appropriate edges to the provided aggregator,
  /// LineReader is given a stream with the line and an Edge-Aggregator.
  /// It should read the stream and add the appropriate edges to
  /// the provided Aggregator.
  /// LineReader should return '0' for success, '1' for failure.
  /// @param g A pointer to the output graph. Vertices must have been added
  /// apriory.
  /// @param blk_sz The size of the block (in number of lines), in which
  /// the input file will be partitioned to be read.
  /// @param sharded Indicates if the graph has been sharded across multiple
  /// files.
  //////////////////////////////////////////////////////////////////////
  template<typename Graph, typename LineReader>
  void operator()(std::istream& ifs, LineReader const& line_reader, Graph* g,
                  size_t blk_sz = 4096, bool sharded = false)
  {
    size_t loc_id = g->distribution().get_location_id();
    size_t num_locs = g->distribution().get_num_locations();

    size_t lines_read =0, wrong_edges =0;

    constexpr bool has_edge_property = LineReader::has_edge_property::value;
    add_edge_aggregator<Graph, has_edge_property> edge_aggr(g);

    {
      size_t ixx=0;
      while (!ifs.eof()) {
        if (sharded || ixx == loc_id) {  // my block or my shard.
          for (size_t i=0; i<blk_sz; ++i) {
            if (ifs.eof())
              break;
            std::string line;
            std::getline(ifs, line);

            if (line.size() == 0 || line[0] == '#')
              continue;

            ++lines_read;
            std::stringstream ss(line);

            // LineReader is given a stream with the line and an
            // Edge-Aggregator.
            // It should read the stream and add the appropriate edges to
            // the provided Aggregator.
            // LineReader should return '0' for success, '1' for failure.
            wrong_edges += line_reader(ss, edge_aggr, g);
          }
        } else {  // not my block, skip these lines.
          for (size_t i=0; i<blk_sz; ++i) {
            if (ifs.eof())
              break;
            std::string line;
            std::getline(ifs, line);
          }
        }

        ++ixx;
        ixx = ixx % num_locs;
      }
    }

    if (wrong_edges > 0)
      std::cerr << loc_id << "> read " << lines_read << " lines, "
                << wrong_edges << " failed edges." << std::endl;
  }
};



//////////////////////////////////////////////////////////////////////
/// @brief Algorithm to read a graph from the given input file.
/// @ingroup pgraphAlgo
///
/// Uses the user-provided reader to read each line and add the edges
/// from the line to the graph.
/// @param filename The name of the input file.
/// @param line_reader The functor used to read-in a line of text from
/// file and add the appropriate edges to the provided aggregator,
/// LineReader is given a stream with the line and an Edge-Aggregator.
/// It should read the stream and add the appropriate edges to
/// the provided Aggregator.
/// LineReader should return '0' for success, '1' for failure.
/// @param blk_sz The size of the block (in number of lines), in which
/// the input file will be partitioned to be read.
/// @return A @ref graph_view over the output graph, populated with the
/// vertices and edges from the input file.
//////////////////////////////////////////////////////////////////////
template<typename Graph, typename LineReader>
graph_view<Graph> graph_reader(std::string filename, LineReader line_reader,
                               size_t blk_sz = 4096)
{
  using namespace graph_io_impl;
  std::ifstream ifs;
  ifs.open(filename.c_str());

  if (!ifs.is_open())
    stapl::abort("Failed to open input graph for reading.");

  size_t num_verts = default_header_reader()(ifs);

  Graph* g = new Graph(num_verts);

  // read the graph.
  parallel_reader()(ifs, line_reader, g, blk_sz);

  rmi_fence();  // needed for comm. generated by parallel-reader.
  ifs.close();
  return graph_view<Graph>(g);
}

//////////////////////////////////////////////////////////////////////
/// @copydoc graph_reader()
/// @param header_reader A custom header reader to parse the header of
/// a file and extract number of vertices.
//////////////////////////////////////////////////////////////////////
template<typename Graph, typename LineReader, typename HeaderReader>
graph_view<Graph> graph_reader(std::string filename, LineReader line_reader,
                          HeaderReader header_reader, size_t blk_sz = 4096)
{
  std::ifstream ifs;
  ifs.open(filename.c_str());

  if (!ifs.is_open())
    stapl::abort("Failed to open input graph for reading.");

  size_t num_verts = header_reader(ifs);

  Graph* g = new Graph(num_verts);

  // read the graph.
  parallel_reader()(ifs, line_reader, g, blk_sz);

  rmi_fence();  // needed for comm. generated by parallel-reader.
  ifs.close();
  return graph_view<Graph>(g);
}



//////////////////////////////////////////////////////////////////////
/// @brief Algorithm to read a sharded graph from the given input file.
/// @ingroup pgraphAlgo
///
/// Uses the user-provided reader to read each line and add the edges
/// from the line to the graph.
/// Input graph must be sharded using the sharder. A sharded file is
/// split into multiple input files and a metadata file that keeps
/// track of the number of shards, vertices and edges, etc.
/// @param filename The name of the input file.
/// @param line_reader The functor used to read-in a line of text from
/// file and add the appropriate edges to the provided aggregator,
/// LineReader is given a stream with the line and an Edge-Aggregator.
/// It should read the stream and add the appropriate edges to
/// the provided Aggregator.
/// LineReader should return '0' for success, '1' for failure.
/// @return A @ref graph_view over the output graph, populated with the
/// vertices and edges from the input file.
//////////////////////////////////////////////////////////////////////
template<typename Graph, typename LineReader>
graph_view<Graph> sharded_graph_reader(std::string filename,
                                       LineReader line_reader)
{
  size_t num_verts=0, num_edges=0, num_shards=0;
  std::string metadata_filename = filename + ".metadata";
  std::ifstream mfile;
  mfile.open(metadata_filename.c_str());

  if (!mfile.is_open())
    stapl::abort("Failed to open input graph metadata for reading.");

  mfile >> num_verts >> num_edges >> num_shards;
  mfile.close();

  Graph* g = new Graph(num_verts);

  // Create partition to balance the work of reading shards
  balanced_partition<indexed_domain<std::size_t>> part{
    {0, num_shards-1}, g->get_num_locations()
  };
  auto local_shards = part[g->get_location_id()];

  domain_map(local_shards, [&](std::size_t i) {
    std::ifstream ifile;
    std::stringstream shard_name;
    shard_name << filename << "." << i;
    ifile.open(shard_name.str().c_str());

    // read the graph.
    parallel_reader()(ifile, line_reader, g,
                      std::numeric_limits<size_t>::max(), true);

    ifile.close();
  });

  rmi_fence();  // needed for comm. generated by parallel-reader.
  return graph_view<Graph>(g);
}




//////////////////////////////////////////////////////////////////////
/// @brief Adjacency-list line-reader functor for the @ref graph_reader().
/// @ingroup pgraphAlgoDetails
///
/// Reads a vertex and its adjacencies from the input line (stream).
/// format:
/// @verbatim
/// #vertices #edges
/// source0 target00 target01 target02 ...
/// source1 target10 target11 target12 ...
/// :
/// @endverbatim
//////////////////////////////////////////////////////////////////////
struct read_adj_list_line
{
  /// Lines do not have properties for edges
  using has_edge_property = std::false_type;

  //////////////////////////////////////////////////////////////////////
  /// @param ss The stringstream representing a line from the input file.
  /// @param aggr The edge-aggregator to output the read edges to.
  /// @param g A pointer to the output graph. The edges may be added to the
  /// graph, or additional operations performed (i.e., setting properties
  /// for vertices and edges, etc.)
  /// @return '0' for success, '1' for failure.
  //////////////////////////////////////////////////////////////////////
  template<typename Aggr, typename Graph>
  size_t operator() (std::stringstream& ss, Aggr& aggr, Graph* g) const
  {
    size_t src, tgt;
    ss >> src;
    while (ss >> tgt) {
      if (src >= g->size() || tgt >= g->size())
        return 1;
      else
        aggr.add(typename Graph::edge_descriptor(src, tgt));
    }
    return 0;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Edge-list line-reader functor for the @ref graph_reader().
/// @ingroup pgraphAlgoDetails
///
/// Reads an edge (source and target) from the input line (stream).
/// EdgeList format: \#edge-source \#edge-target
/// format:
/// @verbatim
/// #vertices #edges
/// source0 target0
/// source1 target1
/// :
/// @endverbatim
//////////////////////////////////////////////////////////////////////
struct read_edge_list_line
{
  /// Lines do not have properties for edges
  using has_edge_property = std::false_type;

  //////////////////////////////////////////////////////////////////////
  /// @param ss The stringstream representing a line from the input file.
  /// @param aggr The edge-aggregator to output the read edges to.
  /// @param g A pointer to the output graph. The edges may be added to the
  /// graph, or additional operations performed (i.e., setting properties
  /// for vertices and edges, etc.)
  /// @return '0' for success, '1' for failure.
  //////////////////////////////////////////////////////////////////////
  template<typename Aggr, typename Graph>
  size_t operator() (std::stringstream& ss, Aggr& aggr, Graph* g) const
  {
    size_t src, tgt;
    ss >> src;
    ss >> tgt;
    if (src >= g->size() || tgt >= g->size())
      return 1;
    else {
      aggr.add(typename Graph::edge_descriptor(src, tgt));
      return 0;
    }
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Weighted edge list line-reader functor for the @ref graph_reader().
/// @ingroup pgraphAlgoDetails
///
/// Reads an edge (source target weight) from the input line (stream).
/// EdgeList format: \#edge-source \#edge-target \#weight
/// format:
/// @verbatim
/// #vertices #edges
/// source0 target0 weight00
/// source1 target1 weight11
/// :
/// @endverbatim
//////////////////////////////////////////////////////////////////////
struct read_weighted_edge_list_line
{
  /// Lines have properties for edges
  using has_edge_property = std::true_type;

  //////////////////////////////////////////////////////////////////////
  /// @param ss The stringstream representing a line from the input file.
  /// @param aggr The edge-aggregator to output the read edges to.
  /// @param g A pointer to the output graph. The edges may be added to the
  /// graph, or additional operations performed (i.e., setting properties
  /// for vertices and edges, etc.)
  /// @return '0' for success, '1' for failure.
  //////////////////////////////////////////////////////////////////////
  template<typename Aggr, typename Graph>
  size_t operator() (std::stringstream& ss, Aggr& aggr, Graph* g) const
  {
    using weight_type = typename Graph::edge_property;
    size_t src, tgt;
    weight_type weight;
    ss >> src;
    ss >> tgt;
    ss >> weight;

    if (src >= g->size() || tgt >= g->size())
      return 1;
    else {
      aggr.add({typename Graph::edge_descriptor(src, tgt), weight});
      return 0;
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Matrix Market line-reader functor for the @ref graph_reader().
/// The file format is described at
/// http://math.nist.gov/MatrixMarket/formats.html. Note that the vertex
/// numbering is one based.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct read_matrix_market_line
{
  /// Lines have properties for edges
  using has_edge_property = std::false_type;

  template<typename Aggr, typename Graph>
  size_t operator() (std::stringstream& ss, Aggr& aggr, Graph* g) const
  {
    size_t src, tgt;
    ss >> src;
    ss >> tgt;

    // The vertex numbering is 1 based
    src -= 1;
    tgt -= 1;

    if (src >= g->size() || tgt >= g->size())
      return 1;
    else {
      aggr.add(typename Graph::edge_descriptor(src, tgt));
      return 0;
    }
  }
};


/////////////////////////////////////////////////////////////////////
/// @brief Matrix Market header-reader for the @ref graph_reader().
/// @ingroup pgraphAlgoDetails
///
/// Reads the header lines from the input file-stream.
/// Skips over comment lines and reads number of vertices.
/// format:
/// @verbatim
/// % comments
/// % comments
/// rows cols nnz
/// @endverbatim
//////////////////////////////////////////////////////////////////////
struct read_matrix_market_header
{
  /// Lines do not have properties for edges
  using has_edge_property = std::false_type;

  //////////////////////////////////////////////////////////////////////
  /// @param ifs The input file-stream.
  /// @return Returns the number of vertices in the graph.
  //////////////////////////////////////////////////////////////////////
  size_t operator()(std::ifstream& ifs) const
  {
    size_t num_verts = 0;

    for (std::string line; std::getline(ifs, line);) {
      // Skip empty lines and comments
      if (line.empty() || line.front() == '%')
        continue;

      // Number of vertices is assumed to be the first number on the
      // first line after the comments
      std::stringstream ss(line);
      ss >> num_verts;
      break;
    }

    return num_verts;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Edge-list line-reader functor for the @ref graph_reader().
/// @ingroup pgraphAlgoDetails
///
/// Reads an edge (source and target) from the input line (stream).
/// Randomly shuffle the vertex IDs but maintain the right edges.
/// This is useful when a bad locality for the vertices has to be broken
/// EdgeList format: \#edge-source \#edge-target \#weight
/// format:
/// @verbatim
/// #vertices #edges
/// source0 target0
/// source1 target1
/// :
/// @endverbatim
//////////////////////////////////////////////////////////////////////
template <typename ShuffleMapperView>
struct read_edge_list_line_shuffle
{
private:
  ShuffleMapperView const& m_mapper;

public:
  /// Lines do not have properties for edges
  using has_edge_property = std::false_type;

  //////////////////////////////////////////////////////////////////////
  /// @param mapper A view over a stapl container which gives a one to one
  ///   mapping from one vertex id to a new id. No 2 ids must map to the
  ///   same new id
  //////////////////////////////////////////////////////////////////////
  read_edge_list_line_shuffle(ShuffleMapperView const& mapper)
    : m_mapper(mapper)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param ss The stringstream representing a line from the input file.
  /// @param aggr The edge-aggregator to output the read edges to.
  /// @param g A pointer to the output graph. The edges may be added to the
  /// graph, or additional operations performed (i.e., setting properties
  /// for vertices and edges, etc.)
  /// @return '0' for success, '1' for failure.
  //////////////////////////////////////////////////////////////////////
  template<typename Aggr, typename Graph>
  size_t operator() (std::stringstream& ss, Aggr& aggr, Graph* g) const
  {
    size_t src, tgt;
    ss >> src;
    ss >> tgt;
    if (src >= g->size() || tgt >= g->size())
      return 1;
    else {
      aggr.add(typename Graph::edge_descriptor(m_mapper[src], m_mapper[tgt]));
      return 0;
    }
  }

  void define_type(typer& t)
  {
    t.member(m_mapper);
  }
};


/////////////////////////////////////////////////////////////////////
/// @brief Dimacs line-reader for the @ref graph_reader().
/// @ingroup pgraphAlgoDetails
///
/// Reads an edge (source and target) from the input line (stream).
/// format:
/// @verbatim
///  c comments
///  p sp #vertices #edges
///  a source0 target0
///  a source1 target1
///  :
/// @endverbatim
/// @note Vertex id's in Dimacs files start with 1 (not zero).
//////////////////////////////////////////////////////////////////////
struct read_dimacs_line
{
  /// Lines do not have properties for edges
  using has_edge_property = std::false_type;

  //////////////////////////////////////////////////////////////////////
  /// @param ss The stringstream representing a line from the input file.
  /// @param aggr The edge-aggregator to output the read edges to.
  /// @param g A pointer to the output graph. The edges may be added to the
  /// graph, or additional operations performed (i.e., setting properties
  /// for vertices and edges, etc.)
  /// @return '0' for success, '1' for failure.
  //////////////////////////////////////////////////////////////////////
  template<typename Aggr, typename Graph>
  size_t operator()(std::stringstream& ss, Aggr& aggr, Graph* g) const
  {
    char prefix;
    ss >> prefix;
    if (prefix == 'a') {
      size_t src, tgt;
      ss >> src >> tgt;
      src -= 1;
      tgt -= 1;
      if (src < g->size() && tgt < g->size()) {
        aggr.add(typename Graph::edge_descriptor(src, tgt));
        return 0;
      }
    }
    return 1;
  }
};


/////////////////////////////////////////////////////////////////////
/// @brief Dimacs header-reader for the @ref graph_reader().
/// @ingroup pgraphAlgoDetails
///
/// Reads the header lines from the input file-stream.
/// Skips over comment lines and reads number of vertices.
/// format:
/// @verbatim
/// c comments
/// c comments
/// p sp #vertices #edges
/// @endverbatim
//////////////////////////////////////////////////////////////////////
struct read_dimacs_header
{
  /// Lines do not have properties for edges
  using has_edge_property = std::false_type;

  //////////////////////////////////////////////////////////////////////
  /// @param ifs The input file-stream.
  /// @return Returns the number of vertices in the graph.
  //////////////////////////////////////////////////////////////////////
  size_t operator()(std::ifstream& ifs) const
  {
    size_t num_verts = 0;
    while (ifs.good()) {
      if (ifs.eof())
        break;
      std::string line;
      std::getline(ifs, line);
      std::stringstream ss(line);
      char prefix;
      ss >> prefix;
      if (prefix == 'c')
        continue;
      if (prefix == 'p') {
        std::string skip;
        ss >> skip >> num_verts;
      }
      break;
    }
    return num_verts;
  }
};



//////////////////////////////////////////////////////////////////////
/// @brief Function to wrap around file-reader to support reading an
/// edge-list.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Graph>
graph_view<Graph> read_edge_list(std::string filename, size_t blk_sz = 4096)
{
  return graph_reader<Graph>(filename, read_edge_list_line(), blk_sz);
}

//////////////////////////////////////////////////////////////////////
/// @brief Function to wrap around file-reader to support reading a
/// weighted edge-list.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Graph>
graph_view<Graph> read_weighted_edge_list(std::string filename,
                                          size_t blk_sz = 4096)
{
  return graph_reader<Graph>(filename, read_weighted_edge_list_line(), blk_sz);
}

//////////////////////////////////////////////////////////////////////
/// @brief Function to wrap around file-reader to support reading an
/// edge-list and rename the vertices id when added to the graph view.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Graph, typename ShuffleMapperView>
graph_view<Graph> read_edge_list_shuffle(ShuffleMapperView const& mapper,
                                         std::string filename,
                                         size_t blk_sz = 4096)
{
  auto reader = read_edge_list_line_shuffle<ShuffleMapperView>(mapper);
  return graph_reader<Graph>(filename, reader, blk_sz);
}

//////////////////////////////////////////////////////////////////////
/// @brief Function to wrap around file-reader to support reading an
/// adjacency-list.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Graph>
graph_view<Graph> read_adj_list(std::string filename, size_t blk_sz = 4096)
{
  return graph_reader<Graph>(filename, read_adj_list_line(), blk_sz);
}

//////////////////////////////////////////////////////////////////////
/// @brief Function to wrap around file-reader to support reading a
/// DIMACS graph.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Graph>
graph_view<Graph> read_dimacs(std::string filename, size_t blk_sz = 4096)
{
  return graph_reader<Graph>(filename,
                             read_dimacs_line(),
                             read_dimacs_header(),
                             blk_sz);
}


//////////////////////////////////////////////////////////////////////
/// @brief Function to wrap around file-reader to support reading a
/// Matrix Market graph.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Graph>
graph_view<Graph> read_matrix_market(std::string filename, size_t blk_sz = 4096)
{
  return graph_reader<Graph>(std::move(filename),
                             read_matrix_market_line(),
                             read_matrix_market_header(),
                             blk_sz);
}

//////////////////////////////////////////////////////////////////////
/// @brief Function to shard an input file for efficient parallel reading.
///
/// Files sharded by the graph sharder can be read by
/// @ref sharded_graph_reader().
/// This function is not parallel and works from one location.
/// @param filename The name of the input file. The input file must
/// contain the number of vertices and edges in the graph in the first line.
/// The output shards created will be named with the provided filename as the
/// prefix, followed by the shard-ID. A metadata file with the name
/// filename.metadata will be created containing the metadata for the shards
/// (\#vertices, \#edges, \#shards, etc.).
/// @param shard_size The size (in number of lines) of each output shard.
/// @param removecomments Set to true for excluding commented lines from
/// shards, or false to keep the commented lines in the output shards.
/// @ingroup pgraphAlgo
/// @todo Find a suitable default for shard_size parameter.
//////////////////////////////////////////////////////////////////////
inline void graph_sharder(std::string filename, size_t shard_size,
                   bool removecomments = true)
{
  if (get_location_id() == 0) {
    size_t num_verts=0, num_edges=0, num_shards=0, shard_id = 0;

    std::ifstream input_file;
    input_file.open(filename.c_str());

    // Read num vertices and edges from input file.
    input_file >> num_verts >> num_edges;

    std::ofstream output_file;

    size_t lines_read = 0;
    bool new_shard = true;
    while (!input_file.eof()) {
      // Open a new shard, if requested.
      if (new_shard) {
        std::stringstream shard_name;
        shard_name << filename << "." << shard_id;
        output_file.open(shard_name.str().c_str());
        new_shard = false;
      }

      std::string line;
      // Read a line from input file
      std::getline(input_file, line);

      // If remove-comments flag is true, don't output commented lines.
      if (removecomments && (line.size() == 0 || line[0] == '#'))
        continue;
      ++lines_read;

      // Write the line to output shard file.
      output_file << line << std::endl;

      // Reached max number of lines allowed for this shard.
      // Close current shard and request new one.
      if (lines_read % shard_size == 0) {
        output_file.close();
        ++shard_id;
        new_shard = true;
      }
    }

    num_shards = shard_id + 1;
    // Output the metadata file.
    std::string metadata_filename = filename + ".metadata";
    std::ofstream mfile;
    mfile.open(metadata_filename.c_str());
    mfile << num_verts << " " << num_edges << " " << num_shards << std::endl;
    mfile.close();
  }
}

} // namespace stapl
#endif
