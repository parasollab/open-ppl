#ifndef PMPL_WORKSPACE_SKELETON_H_
#define PMPL_WORKSPACE_SKELETON_H_

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/GenericStateGraph.h"
#include "ConfigurationSpace/Weight.h"
#include "Geometry/Boundaries/Boundary.h"

#include <containers/sequential/graph/directed_preds_graph.h>

#include "Vector.h"

#include <queue>
#include <set>
#include <unordered_map>
#include <string>


////////////////////////////////////////////////////////////////////////////////
/// Geometric skeleton of the workspace.
////////////////////////////////////////////////////////////////////////////////
class WorkspaceSkeleton : public GenericStateGraph<mathtool::Point3d, std::vector<mathtool::Point3d>> {

  public:

    ///@name Local Types
    ///@{

    typedef GenericStateGraph<mathtool::Point3d, std::vector<mathtool::Point3d>> BaseType;

    typedef BaseType::VID             VD;
    typedef BaseType::EID             ED;
    typedef typename BaseType::VI     vertex_iterator;
    typedef typename BaseType::VID    vertex_descriptor;
    typedef typename BaseType::EI     adj_edge_iterator;

    // Interface with CompositeState, CompositeEdge
    typedef mathtool::Point3d                        CfgType;
    typedef std::vector<mathtool::Point3d>           EdgeType;

    ///@}
    ///@name Construction
    ///@{

    WorkspaceSkeleton();

    ///@}
    ///@name Locators
    ///@{

    /// Helper function to find the skeleton vertex that is closest to a given
    /// workspace point.
    /// @param _target The workspace point.
    /// @return An iterator to the nearest skeleton vertex.
    vertex_iterator FindNearestVertex(const mathtool::Point3d& _target);

    /// Find a vertex iterator in the skeleton by descriptor.
    /// @param _vertexDescriptor The descriptor of the vertex to search for.
    /// @return An iterator to the desired vertex.
    vertex_iterator FindVertex(const VD _vertexDescriptor);

    /// Find an edge iterator in the skeleton by descriptor.
    /// @param _edgeDescriptor The descriptor of the edge to search for.
    /// @return An iterator to the desired edge.
    adj_edge_iterator FindEdge(const ED& _edgeDescriptor);

    ///@}
    ///@name Modifiers
    ///@{

    /// Direct the skeleton so that all edges point outwards from the vertex
    /// nearest to a given starting point.
    /// @param _start A workspace point from which the direction should eminate.
    /// @return A directed skeleton with flow pointing outward from _start.
    WorkspaceSkeleton Direct(const mathtool::Point3d& _start);

    /// Prune the skeleton by removing vertices and edges that are not reachable
    /// by back-tracking the edges from the vertex nearst to a goal point. Only
    /// makes sense after calling Direct.
    /// @param _goal A workspace goal point.
    void Prune(const mathtool::Point3d& _goal);

    /// Divide existing edges into sizes of at most _maxLength.
    void RefineEdges(double _maxLength);

    /// Double up the edges so that each one has a duplicate facing the opposite
    /// direction with reversed path and same edge ID. This does not make sense
    /// with Direct or Prune. If you want to use with RefineEdges you should
    /// call that first to ensure the edge counterparts match exactly.
    /// @note This function assumes that all edges currently in the skeleton
    ///       are unique and not reverse duplicates.
    void DoubleEdges();

    ///@}
    ///@name IO
    ///@{

    /// Writes the graph to a file.
    /// @param _file the output file name.

    void Write(const std::string& _file);

    /// Reads the graph from a file.
    /// @param _file the output file name.

    void Read(const std::string& _file);

    ///@}

  private:

    ///@name Internal State
    ///@{

    bool m_debug{false};  ///< Show debug messages?

    /// The 'start' vertex from which the skeleton was last directed.
    VD m_start{std::numeric_limits<VD>::max()};

    ///@}
};

#endif
