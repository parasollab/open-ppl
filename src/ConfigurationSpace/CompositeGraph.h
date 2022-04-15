#ifndef PPL_COMPOSITE_GRAPH_H_
#define PPL_COMPOSITE_GRAPH_H_

#include "ConfigurationSpace/GenericStateGraph.h"
#include "MPProblem/RobotGroup/RobotGroup.h"
// #include "ConfigurationSpace/CompositeState.h"
// #include "ConfigurationSpace/CompositeEdge.h"

#include <containers/sequential/graph/algorithms/graph_input_output.h>

#ifndef INVALID_ED
#define INVALID_ED ED{std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max()}
#endif


////////////////////////////////////////////////////////////////////////////////
/// Represents a roadmap for a robot group.
///
/// Rather than duplicating the data for each robot, this object refers to an
/// individual roadmap for each robot. This object however does not own the
/// individual roadmaps - it merely points to them.
///
/// Note that VIDs in this object refer to GROUP configuration VIDs.
////////////////////////////////////////////////////////////////////////////////
template <typename Vertex, typename Edge> //<CompositeState<GraphType>, CompositeEdge<GraphType>>
class CompositeGraph : public GenericStateGraph<Vertex, Edge> {

  public:

    ///@name Local Types
    ///@{

    typedef GenericStateGraph<Vertex, Edge> BaseType;

    typedef typename Vertex::IndividualGraph      IndividualGraph;

    // typedef CompositeState<GraphType> Vertex;
    // typedef CompositeEdge<GraphType> Edge;

    // typedef typename BaseType::EID         ED;
    // typedef typename Cfg                   IndividualCfg;
    // typedef typename Edge::IndividualEdge  IndividualEdge;
    // typedef typename Vertex::VIDSet        VIDSet;
    // typedef GenericStateGraph<IndividualCfg, IndividualEdge> IndividualRoadmap;

    // typedef typename BaseType::adj_edge_iterator adj_edge_iterator;
    // typedef typename BaseType::edge_descriptor edge_descriptor;
    // typedef typename BaseType::vertex_iterator vertex_iterator;
    // typedef typename BaseType::vertex_descriptor vertex_descriptor;

    // using typename BaseType::CVI;
    // using typename BaseType::VI;
    // using typename BaseType::EI;
    // using typename BaseType::VID;
    // using typename BaseType::STAPLGraph;

    // using typename BaseType::VertexHook;
    // using typename BaseType::EdgeHook;
    // using typename BaseType::HookType;
    ///@}
    ///@name Construction
    ///@{

    /// Construct a group roadmap.
    CompositeGraph(RobotGroup* const _g, std::vector<IndividualGraph*> _graphs = {});

    // virtual ~CompositeGraph() = 0;

    ///@}
    ///@name Disabled Functions
    ///@{
    /// Move and copy are disabled because the group cfgs and edges need to know
    /// their group roadmap pointer. To implement these, we'll need to update
    /// the pointer on every component.

    CompositeGraph(const CompositeGraph&) = delete;
    CompositeGraph(CompositeGraph&&)      = delete;

    CompositeGraph& operator=(const CompositeGraph&) = delete;
    CompositeGraph& operator=(CompositeGraph&&)      = delete;

    ///@}
    ///@name Accessors
    ///@{

    /// Get the robot group.
    virtual RobotGroup* GetGroup();

    /// Get the individual roadmap for a robot in the group.
    /// @param _index The index of the desired robot.
    virtual IndividualGraph* GetRoadmap(const size_t _index);
    virtual const IndividualGraph* GetRoadmap(const size_t _index) const;

    /// Get the number of robots for the group this roadmap is for.
    virtual size_t GetNumRobots() const noexcept;

    ///@}
    ///@name Input/Output
    ///@{

    //virtual void Read(const std::string& _filename) override;

    // std::string PrettyPrint() const;

    ///@}
    ///@name Hooks
    ///@{

    /// Uninstall all hooks from each individual roadmap. Should only be used at
    /// the end of a library run to clean the roadmap object.
    virtual void ClearHooks() noexcept override;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    RobotGroup* const m_group; ///< The robot group.

    std::vector<IndividualGraph*> m_roadmaps; ///< The individual roadmaps.

    using BaseType::m_timestamp;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename Vertex, typename Edge>
CompositeGraph<Vertex, Edge>::
CompositeGraph(RobotGroup* const _g, std::vector<IndividualGraph*> _graphs) :
  GenericStateGraph<Vertex, Edge>(nullptr),
  m_group(_g), m_roadmaps(_graphs) {

    if (m_roadmaps.size() != m_group->Size())
      m_roadmaps.reserve(m_group->Size());
  }

/*-------------------------------- Accessors ---------------------------------*/

template <typename Vertex, typename Edge>
inline
RobotGroup*
CompositeGraph<Vertex, Edge>::
GetGroup() {
  return m_group;
}


template <typename Vertex, typename Edge>
inline
typename CompositeGraph<Vertex, Edge>::IndividualGraph*
CompositeGraph<Vertex, Edge>::
GetRoadmap(const size_t _index) {
  return m_roadmaps[_index];
}


template <typename Vertex, typename Edge>
inline
const typename CompositeGraph<Vertex, Edge>::IndividualGraph*
CompositeGraph<Vertex, Edge>::
GetRoadmap(const size_t _index) const {
  return m_roadmaps[_index];
}


template <typename Vertex, typename Edge>
size_t
CompositeGraph<Vertex, Edge>::
GetNumRobots() const noexcept {
  return m_group->Size();
}

/*-------------------------------Input/Output---------------------------------*/

// template <typename Vertex, typename Edge>
// std::string
// CompositeGraph<Vertex, Edge>::
// PrettyPrint() const {
//   std::ostringstream out;
//   out << "Number of group vertices: " << this->get_num_vertices() << std::endl;
//   out << "Vertices in each individual roadmap:" << std::endl << "| ";
//   for(size_t i = 0; i < GetNumRobots(); ++i) {
//     out << "(Robot " << i << ") " << GetRoadmap(i)->get_num_vertices() << " | ";
//   }
//   out << std::endl;

//   return out.str();
// }

/*----------------------------------- Hooks ----------------------------------*/

template <typename Vertex, typename Edge>
void
CompositeGraph<Vertex, Edge>::
ClearHooks() noexcept {
  BaseType::ClearHooks();

  for(IndividualGraph* const map : m_roadmaps)
    map->ClearHooks();
}

/*----------------------------------------------------------------------------*/

#endif
