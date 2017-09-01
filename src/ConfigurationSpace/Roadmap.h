#ifndef ROADMAP_H_
#define ROADMAP_H_

#include <unordered_map>

#include "RoadmapGraph.h"
#include "MPProblem/Environment/Environment.h"

#ifdef _PARALLEL
#include <containers/graph/algorithms/graph_io.hpp>
#else
#include <containers/sequential/graph/algorithms/graph_input_output.h>
#endif

////////////////////////////////////////////////////////////////////////////////
/// The Roadmap is essentially the graph G = (V, E) which is used to approximate
/// the planning space by sampling-based motion planning algorithms. The set of
/// vertices V are robot configurations, and the set of edges E are local plans
/// between them.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class Roadmap final {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType            CfgType;
    typedef typename MPTraits::WeightType         WeightType;
    typedef RoadmapGraph<CfgType, WeightType>     GraphType;
    typedef typename GraphType::vertex_descriptor VID;

    ///@}
    ///@name Construction
    ///@{

    Roadmap(Robot* const _r);

    Roadmap(const Roadmap& _r);

    Roadmap(Roadmap&& _r);

    ~Roadmap();

    ///@}
    ///@name Graph Accessors
    ///@{

    GraphType* GetGraph() noexcept;
    const GraphType* GetGraph() const noexcept;

    /// Set the graph object. The previous graph will be deleted, and the
    /// roadmap will take ownership of the new one.
    /// @param _g The new graph to use.
    void SetGraph(GraphType* const _g) noexcept;

    /// Copy the nodes and edges from another roadmap and append them to this.
    /// @param _r The roadmap to copy from.
    void AppendRoadmap(const Roadmap& _r);

    ///@}
    ///@name I/O
    ///@{

    /// Read in a roadmap (.map) file.
    /// @param _filename The name of the map file to read.
    void Read(const std::string& _filename);

    /// Write the current roadmap out to a roadmap (.map) file.
    /// @param _filename The name of the map file to write to.
    /// @param _env The environment for which this map was constructed.
    void Write(const std::string& _filename, Environment* _env);

    ///@}

  private:

    ///@name Internal State
    ///@{

    Robot* const m_robot;          ///< The robot this roadmap is for.
    GraphType* m_graph{nullptr};   ///< Graph of configurations and edges.

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
Roadmap<MPTraits>::
Roadmap(Robot* const _r) : m_robot(_r), m_graph(new GraphType()) { }


template <typename MPTraits>
Roadmap<MPTraits>::
Roadmap(const Roadmap& _r) : m_robot(_r.m_robot), m_graph(new GraphType()) {
  AppendRoadmap(_r);
}


template <typename MPTraits>
Roadmap<MPTraits>::
Roadmap(Roadmap&& _r) : m_robot(_r.m_robot), m_graph(std::move(_r.m_graph)) {
  _r.m_graph = nullptr;
}


template <typename MPTraits>
Roadmap<MPTraits>::
~Roadmap() {
  delete m_graph;
}

/*----------------------------- Graph Accessors ------------------------------*/

template <typename MPTraits>
typename Roadmap<MPTraits>::GraphType*
Roadmap<MPTraits>::
GetGraph() noexcept {
  return m_graph;
}


template <typename MPTraits>
const typename Roadmap<MPTraits>::GraphType*
Roadmap<MPTraits>::
GetGraph() const noexcept {
  return m_graph;
}


template <typename MPTraits>
void
Roadmap<MPTraits>::
SetGraph(GraphType* const _g) noexcept {
  delete m_graph;
  m_graph = _g;
}


template <typename MPTraits>
void
Roadmap<MPTraits>::
AppendRoadmap(const Roadmap& _r) {
  // Copy vertices and map the change in VIDs.
  std::unordered_map<VID, VID> oldToNew;
  for(auto vit = _r.m_graph->begin(); vit != _r.m_graph->end(); ++vit) {
    const VID oldVID = vit->descriptor();
    const VID newVID = m_graph->AddVertex(vit->property());
    oldToNew[oldVID] = newVID;
  }

  // Copy edges.
  for(auto vit = _r.m_graph->begin(); vit != _r.m_graph->end(); ++vit) {
    for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
      const VID source = oldToNew[eit->source()];
      const VID target = oldToNew[eit->target()];
      if(!m_graph->IsEdge(source, target))
        m_graph->AddEdge(source, target, eit->property());
    }
  }
}

/*----------------------------------- I/O ------------------------------------*/

template <class MPTraits>
void
Roadmap<MPTraits>::
Read(const std::string& _filename) {
  ifstream ifs(_filename.c_str());
  if(!ifs)
    throw ParseException(WHERE, "Cannot open file " + _filename + ".");

  std::string tag;
  bool headerParsed = false;
  int graphStart = 0;
  // Read the file until we find the GRAPHSTART tag.
  while(!headerParsed) {
    // Mark our position and read the next line.
    graphStart = ifs.tellg();
    if(!(ifs >> tag))
      throw ParseException(WHERE, "Error reading map file '" + _filename + "' - "
          "GRAPHSTART tag is missing.");

    // If we find the GRAPHSTART tag, we are done.
    if(tag.find("GRAPHSTART") != string::npos)
      headerParsed = true;
  }

  if(!m_robot)
    RunTimeException(WHERE, "Must specify robot when reading in roadmaps."
        " m_robot was null.");

  // Set the input robot for our edge class.
  /// @TODO this is a bad way to handle the fact that it's necessary to know
  /// the robot type (non/holonomic) when reading and writing.
  WeightType::inputRobot = m_robot;
  CfgType::inputRobot = m_robot;

  // Set ifs back to the line with the GRAPHSTART tag and read in the graph.
  ifs.seekg(graphStart, ifs.beg);
  typename GraphType::STAPLGraph& graph = *m_graph;
  stapl::sequential::read_graph(graph, ifs);

  // Unset the input robot for our edge class.
  WeightType::inputRobot = nullptr;
  CfgType::inputRobot = nullptr;
}


template <typename MPTraits>
void
Roadmap<MPTraits>::
Write(const std::string& _filename, Environment* _env) {
  std::ofstream ofs(_filename);
  ofs << "#####ENVFILESTART#####" << std::endl
      << _env->GetEnvFileName() << std::endl
      << "#####ENVFILESTOP#####" << std::endl;

#ifndef _PARALLEL
  stapl::sequential::write_graph(*m_graph, ofs);
#else
  ofs.close();
  stapl::graph_view<GraphType> gv(*m_graph);
  write_PMPL_graph(gv, _filename);
#endif

}

/*----------------------------------------------------------------------------*/

#endif
