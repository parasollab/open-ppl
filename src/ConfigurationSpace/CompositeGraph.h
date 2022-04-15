#ifndef PPL_COMPOSITE_GRAPH_H_
#define PPL_COMPOSITE_GRAPH_H_

#include "ConfigurationSpace/GenericStateGraph.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

#include <containers/sequential/graph/algorithms/graph_input_output.h>

#ifndef INVALID_ED
#define INVALID_ED ED{std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max()}
#endif


////////////////////////////////////////////////////////////////////////////////
/// Represents a composite graph for a robot group.
///
/// Rather than duplicating the data for each robot, this object refers to an
/// individual graph for each robot. This object however does not own the
/// individual graphs - it merely points to them.
///
/// Note that VIDs in this object refer to composite state VIDs.
////////////////////////////////////////////////////////////////////////////////
template <typename Vertex, typename Edge>
class CompositeGraph : public GenericStateGraph<Vertex, Edge> {

  public:

    ///@name Local Types
    ///@{

    typedef GenericStateGraph<Vertex, Edge>       BaseType;
    typedef typename Vertex::IndividualGraph      IndividualGraph;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a group graph.
    CompositeGraph(RobotGroup* const _g, std::vector<IndividualGraph*> _graphs = {});

    ///@}
    ///@name Disabled Functions
    ///@{
    /// Move and copy are disabled because the group cfgs and edges need to know
    /// their group graph pointer. To implement these, we'll need to update
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

    /// Get the individual graph for a robot in the group.
    /// @param _index The index of the desired robot.
    virtual IndividualGraph* GetRoadmap(const size_t _index);
    virtual const IndividualGraph* GetRoadmap(const size_t _index) const;

    /// Get the number of robots for the group this graph is for.
    virtual size_t GetNumRobots() const noexcept;

    ///@}
    ///@name Input/Output
    ///@{

    /// Write the graph using the current standard output rule.
    /// @param _filename The file to write to.
    /// @param _env The environment, to place in the graph.
    virtual void Write(const std::string& _filename, Environment* _env) const
        override;

    /// Write the graph as a Vizmo-compatible composite C-Space path.
    /// @param _filename The file to write to.
    /// @param _env The environment, to place in the graph.
    virtual void WriteCompositeGraph(const std::string& _filename,
                            Environment* const _env) const;

    ///@}
    ///@name Hooks
    ///@{

    /// Uninstall all hooks from each individual graph. Should only be used at
    /// the end of a library run to clean the graph object.
    virtual void ClearHooks() noexcept override;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    RobotGroup* const m_group; ///< The robot group.

    std::vector<IndividualGraph*> m_roadmaps; ///< The individual graphs.

    using BaseType::m_timestamp;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename Vertex, typename Edge>
CompositeGraph<Vertex, Edge>::
CompositeGraph(RobotGroup* const _g, std::vector<IndividualGraph*> _graphs) :
  GenericStateGraph<Vertex, Edge>(nullptr), m_group(_g), m_roadmaps(_graphs) {

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

template <typename Vertex, typename Edge>
void
CompositeGraph<Vertex, Edge>::
Write(const std::string& _filename, Environment* _env) const {
  /// For now, we only support composite C-Space output as vizmo is the only
  /// vizualiser that has support for groups/composite robots.
  WriteCompositeGraph(_filename, _env);
}


template <typename Vertex, typename Edge>
void
CompositeGraph<Vertex, Edge>::
WriteCompositeGraph(const std::string& _filename, Environment* _env) const {
  #ifndef VIZMO_MAP
    throw RunTimeException(WHERE, "Cannot use this method without the vizmo map"
                                  " option enabled in the Makefile!");
  #endif

  std::ofstream ofs(_filename);
  ofs << "#####ENVFILESTART#####" << std::endl
      << _env->GetEnvFileName() << std::endl
      << "#####ENVFILESTOP#####" << std::endl;

  stapl::sequential::write_graph(*this, ofs);
}

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
