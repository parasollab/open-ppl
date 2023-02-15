/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_GRAPH_INPUT_OUTPUT_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_GRAPH_INPUT_OUTPUT_HPP

#include <string.h>
#include <iostream>
#include <fstream>

namespace stapl{
namespace sequential{

//////////////////////////////////////////////////////////////////////
/// @brief Reads a file that contains a graph in Dot format,
/// and creates the resulting graph.
/// @param _g The graph object where the graph will be stored.
/// @param _fname The filename.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph>
void read_vertices_edges_graph(Graph& _g, const char*  _fname)
{
  std::ifstream  myifstream(_fname);
  if (!myifstream) {
    std::cout << "\nIn ReadGraph: can't open infile: " << _fname ;
    return;
  }
  read_vertices_edges_graph(_g, myifstream);
  myifstream.close();
}

//////////////////////////////////////////////////////////////////////
/// @brief Reads from an input stream into a graph, assuming
/// that the input stream contains a representation of a graph in Dot format.
/// @param _g The graph object where the graph will be stored.
/// @param _myistream The input stream.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph>
void read_vertices_edges_graph(Graph& _g, std::istream& _myistream)
{
  typedef typename Graph::vertex_property vp_type;
  typename Graph::vertex_descriptor v1d, v2d;
  typename Graph::edge_descriptor ed;
  vp_type p = vp_type();

  char tagstring[100];
  size_t nv, ne;

  _myistream >> tagstring;
  if ( !strstr(tagstring,"GRAPHSTART") ) {
    std::cout << std::endl << "In ReadGraph: didn't read GRAPHSTART tag right";
    return;
  }

  if (_g.get_num_vertices() != 0) {
    _g.clear(); // empty graph before filling it in
  }

  _myistream >> nv;
  _myistream >> ne;

  //read vertices
  for (size_t i=0;i<nv;++i){
    _myistream >> v1d;
    _g.add_vertex(v1d,p);
  }

  //read edges
  for (size_t i=0;i<ne;++i){
    _myistream >> v1d;
    _myistream >> v2d;
    ed = _g.add_edge(v1d,v2d);
    if (ed.id() == (size_t)INVALID_VALUE) {
      std::cout<<"Error while trying to insert edge "<<v1d<<" "<<v2d<<std::endl;
    }
  }

  _myistream >> tagstring;
  if ( !strstr(tagstring,"GRAPHSTOP") ) {
    std::cout << std::endl << "In ReadGraph: didn't read GRAPHSTOP tag right";
    return;
  }
 }

//////////////////////////////////////////////////////////////////////
/// @brief Writes the graph info from an input graph into a
/// file in metis format.
/// @param _g The input graph.
/// @param _fname The filename.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph>
void write_vertices_edges_graph(Graph& _g, const char* _fname)
{
  std::ofstream  myofstream(_fname);
  if (!myofstream) {
    std::cout << "\nInWriteGraph: can't open outfile: " << _fname ;
  }
  write_vertices_edges_graph(_g,myofstream);
  myofstream.close();
}

//////////////////////////////////////////////////////////////////////
/// @brief Writes the graph info from an input graph into an
/// output stream in metis format.
/// @param _g The input graph.
/// @param _myostream The output stream.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph>
void write_vertices_edges_graph(Graph& _g, std::ostream& _myostream)
{
  _myostream<<"GRAPHSTART"<<std::endl;
  _myostream<<_g.get_num_vertices()<<" "<<_g.get_num_edges()<<std::endl;
  for (typename Graph::vertex_iterator vi = _g.begin(); vi != _g.end(); ++vi) {
    _myostream<<(*vi).descriptor()<<std::endl;
  }
  for (typename Graph::vertex_iterator vi = _g.begin(); vi != _g.end(); ++vi) {
    for (typename Graph::adj_edge_iterator ei =
          (*vi).begin();ei != (*vi).end();++ei){
      _myostream<<(*ei).source()<<" "<<(*ei).target()<<std::endl;
    }
  }
  _myostream<<"GRAPHSTOP"<<std::endl;
}


//////////////////////////////////////////////////////////////////////
/// @brief Writes a graph to an output stream in Motion
/// Planning Format.
/// @param _g The input graph.
/// @param _myostream The output stream.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph>
void write_graph(const Graph& _g, std::ostream& _myostream)
{
  _myostream << "#####GRAPHSTART#####";
  _myostream << std::endl << _g.get_num_vertices() << " "
             << _g.get_num_edges() << " " << _g.get_num_vertices();
  //format: VID VERTEX #edges VID WEIGHT VID WEIGHT ...
  for (typename Graph::const_vertex_iterator vi =
                _g.begin(); vi != _g.end(); ++vi) {
    _myostream << std::endl;
    _myostream << (*vi).descriptor() << " ";
    _myostream << (*vi).property() << " ";
    _myostream << (*vi).size() << " ";
    for (typename Graph::const_adj_edge_iterator ei =
              (*vi).begin(); ei != (*vi).end(); ei++) {
      _myostream << (*ei).target() <<" ";
      _myostream << (*ei).property() << " ";
      _myostream << " ";
    }
  }
  _myostream << std::endl << "#####GRAPHSTOP#####";
  _myostream << std::endl;
}

//////////////////////////////////////////////////////////////////////
/// @brief Writes a graph to a file in Motion Planning Format.
/// @param _g The input graph.
/// @param _fname The filename.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph>
void write_graph(const Graph& _g, const char* _fname)
{
  std::ofstream  myofstream(_fname);
  if (!myofstream) {
    std::cout << "\nInWriteGraph: can't open outfile: " << _fname ;
  }
  write_graph(_g, myofstream);
  myofstream.close();
}

//////////////////////////////////////////////////////////////////////
/// @brief Reads in a graph from an input stream in Motion
/// Planning Format, and creates the resulting graph.
/// @param _g The graph object where the graph will be stored.
/// @param _myistream The input stream.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph>
void read_graph(Graph& _g, std::istream& _myistream)
{
  typename Graph::vertex_descriptor v1id, v2id, maxVID;
  typename Graph::vertex_property   data;
  typename Graph::edge_property     weight;
  size_t nVerts, nEdges, nedges;
  char tagstring[100];

  _myistream >> tagstring;
  if ( !strstr(tagstring,"#####GRAPHSTART#####") ) {
    std::cout << std::endl << "In ReadGraph: didn't read GRAPHSTART tag right";
    return;
  }
  if (_g.get_num_vertices() != 0) {
    _g.clear(); // empty graph before filling it in
  }
  _myistream >> nVerts >> nEdges >> maxVID;
  for (size_t i = 0; i < nVerts; i++){
    _myistream >> v1id >> data;             // read and add vertex
    _g.add_vertex(v1id, data);

    _myistream >> nedges;               // read and add its edges
    for (size_t j = 0; j < nedges; j++){
      _myistream >> v2id >> weight;
      _g.add_edge(v1id, v2id, weight);
    }
  }
  _myistream >> tagstring;
  if ( !strstr(tagstring,"#####GRAPHSTOP#####") ) {
    std::cout << std::endl << "In ReadGraph: didn't read GRAPHSTOP tag right";
    return;
  }
}

//////////////////////////////////////////////////////////////////////
/// @brief Reads a graph from a file in Motion Planning Format,
/// and creates the resulting graph.
/// @param _g The graph object where the graph will be stored.
/// @param _fname The filename.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph>
void read_graph(Graph& _g, const char*  _fname)
{
  std::ifstream  myifstream(_fname);
  if (!myifstream) {
    std::cout << "\nIn ReadGraph: can't open infile: " << _fname ;
    return;
  }
  read_graph(_g, myifstream);
  myifstream.close();
}

//////////////////////////////////////////////////////////////////////
/// @brief Writes an input graph to an output stream in Dot
/// format.
/// @param _g The input graph.
/// @param _myostream The output stream.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph>
void write_dot_graph(const Graph& _g, std::ostream& _myostream)
{
  _myostream << "digraph G {\n";
  for (typename Graph::const_vertex_iterator vi =
                _g.begin(); vi != _g.end(); ++vi) {
    _myostream << (*vi).descriptor() << "\n";
  }

  for (typename Graph::const_vertex_iterator vi =
                _g.begin(); vi != _g.end(); ++vi) {
    for (typename Graph::const_adj_edge_iterator ei =
              (*vi).begin(); ei != (*vi).end(); ei++) {
      _myostream << (*ei).source() <<"->";
      _myostream << (*ei).target() << "\n";
    }
  }
  _myostream << std::endl << "}";
  _myostream << std::endl;
}

//////////////////////////////////////////////////////////////////////
/// @brief Writes an input graph to a file in Dot format.
/// @param _g The input graph.
/// @param _fname The filename.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph>
void write_dot_graph(const Graph& _g, const char* _fname)
{
  std::ofstream  myofstream(_fname);
  if (!myofstream) {
    std::cout << "\nInWriteGraph: can't open outfile: " << _fname ;
  }
  write_dot_graph(_g, myofstream);
  myofstream.close();
}


}//namespace sequential
}//namespace stapl

#endif
