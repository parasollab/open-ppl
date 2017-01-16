#ifndef WORKSPACE_SKELETON_H_
#define WORKSPACE_SKELETON_H_

#include <queue>
#include <set>
#include <unordered_map>

using namespace std;

#include "ConfigurationSpace/Cfg.h"
#include <containers/sequential/graph/directed_preds_graph.h>
#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Boundaries/Boundary.h"

#include <Vector.h>
using namespace mathtool;

class ActiveMultiBody;
class Cfg;


////////////////////////////////////////////////////////////////////////////////
/// Geometric skeleton of the workspace
////////////////////////////////////////////////////////////////////////////////
class WorkspaceSkeleton {

  public:

    ///@name Local Types
    ///@{

    /// Graph type is a directed multiedge graph of points and paths.
    typedef stapl::sequential::directed_preds_graph<
        stapl::MULTIEDGES, Vector3d, vector<Vector3d>> GraphType;
    typedef GraphType::vertex_descriptor        FVD;
    typedef GraphType::edge_descriptor          FED;
    typedef typename GraphType::vertex_iterator vertex_iterator;


    ////////////////////////////////////////////////////////////////////////////
    /// Auxiliary data for regions includes skeleton edge descriptor, index
    /// along flow edge, and number of failed extentions.
    ////////////////////////////////////////////////////////////////////////////
    struct RegionData {

      FED edgeDescriptor;
      size_t edgeIndex;
      size_t failCount;

    };

    ///@}

    /// Helper function to find the nearest vertex to a given Vector3d type
    /// vertex in the flow graph
    /// @param _target the target vertex for which we want to find nearest
    vertex_iterator FindNearestVertex(const Vector3d& _target);

    /// Produces a directed version of workspace skeleton
    /// @param _start a workspace point near the start
    /// @return a directed version of worksspace skeleton
    // look at Utilities/ReebGraphConstruction.GetFlowGraph()
    WorkspaceSkeleton Direct(const Vector3d& _start);

    /// Creates regions arond the the vertex nearest to the start
    /// @param _start a workspace point near the start
    /// @param _regionRadius the region radius we want to use when creating
    // refer to MPLibrary/MPStrategies/DynamicRegionRRT.run()
    void InitRegions(const Vector3d& _start, const double _regionRadius);

    ///
    void CreateRegions(const Vector3d& _p, double _regionRadius);

    /// Get the set of active regions
    const vector<Boundary*>& GetRegions() const;

    /// Set the skeleton graph
    /// @param p_graph the flow graph we want to set for this skeleton
    void SetGraph(GraphType& _graph);

    /// Test regions for containment of new configuration and advance if
    /// necessary
    /// @param _cfg the new configuration
    void AdvanceRegions(const Cfg& _cfg);

    /// Test if a configuration is inside a region
    /// @param
    bool IsTouching(const Cfg& _cfg, const Boundary* const _region,
        const double _robotFactor = 1);

    /// Prune the flow graph, delete unnecessary vertices in the flow graph
    /// i.e., those do not show up when back tracing goal vertex
    /// @param _f the GraphType obj (flow graph) we want to prune
    void PruneFlowGraph(const Cfg& _goal);

    /// Push the flow graph to medial axis
    /// @param _f the flow graph we want to push
    //void FlowToMedialAxis(GraphType& _f) const;

    ///
    void MarkAllNodesUnvisited();

    ///
    void SetFailedAttempts(const Boundary* const _region, const size_t _num);

  private:

    ///@name Internal State
    ///@{

    vector<Boundary*> m_regions;       ///< All Regions
    GraphType m_graph;                 ///< The skeleton graph

    unordered_map<Boundary*, RegionData> m_regionData;
    unordered_map<FVD, bool> m_visited;

    ///@}

};

#endif
