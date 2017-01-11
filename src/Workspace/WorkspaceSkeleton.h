#ifndef WORKSPACE_SKELETON_H_                                                   
#define WORKSPACE_SKELETON_H_

#include <queue>
#include <unordered_map>

using namespace std;

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Boundaries/BoundingSphere.h"
#include <containers/sequential/graph/directed_preds_graph.h> 
#include <Vector.h>
#include <set>
#include "Geometry/Boundaries/Boundary.h"

#include "Geometry/Bodies/ActiveMultiBody.h"

using namespace mathtool;

class Cfg;
class ActiveMultiBody;

////////////////////////////////////////////////////////////////////////////////
/// Geometric skeleton of the workspace
////////////////////////////////////////////////////////////////////////////////
class WorkspaceSkeleton{
    
  public:
    //typedef Cfg CfgType;
    typedef Boundary* RegionPtr;
    typedef stapl::sequential::directed_preds_graph<                               
      stapl::MULTIEDGES, Vector3d, vector<Vector3d>                                
             > GraphType; ///< Flow graph is a directed multiedge graph of points and   
                          ///< paths. Is a flow of the embedded ReebGraph. 
    typedef GraphType::vertex_descriptor FVD;
    typedef GraphType::edge_descriptor FED;
    typedef typename GraphType::vertex_iterator vertex_itr;

    /// Helper function to find the nearest vertex to a given Vector3d type
    /// vertex in the flow graph 
    /// @param _target the target vertex for which we want to find nearest
    vertex_itr FindNearestVertex(const Vector3d& _target);
    
    /// Produces a directed version of workspace skeleton
    /// @param _start a workspace point near the start
    /// @return a directed version of worksspace skeleton
    // look at Utilities/ReebGraphConstruction.GetFlowGraph()
    // TODO test this function
    WorkspaceSkeleton Direct(const Vector3d& _start);
    
    /// Creates regions arond the the vertex nearest to the start
    /// @param _start a workspace point near the start
    /// @param _regionRadius the region radius we want to use when creating
    // refer to MPLibrary/MPStrategies/DynamicRegionRRT.run()
    // TODO test this function
    void InitRegions(const Vector3d& _start, const double _regionRadius);

    ///
    void CreateRegions(const Vector3d& _p, double _regionRadius);

    /// Get the set of active regions
    // TODO test this function
    vector<RegionPtr>& GetRegions();

    /// Set the skeleton graph
    /// @param p_graph the flow graph we want to set for this skeleton
    void SetGraph(GraphType& _graph);

    /// Test regions for containment of new configuration and advance if
    /// necessary
    /// @param _cfg the new configuration
    // @TODO test this function
    void AdvanceRegions(const Cfg& _cfg);

    /// Test if a configuration is inside a region
    /// @param 
    // @TODO should be correct, mostly copy and paste
    bool IsTouching(const Cfg& _cfg, RegionPtr _region, double _robotFactor = 1.0);

    /// Prune the flow graph, delete unnecessary vertices in the flow graph
    /// i.e., those do not show up when back tracing goal vertex
    /// @param _f the GraphType obj (flow graph) we want to prune
    void PruneFlowGraph(const Cfg& goalCfg);
    
    /// Push the flow graph to medial axis
    /// @param _f the flow graph we want to push
    //void FlowToMedialAxis(GraphType& _f) const;

    /// 
    void MarkAllNodesUnvisited();

    ///
    void SetFailedAttempts(RegionPtr _region, size_t _num);

  private:
    vector<RegionPtr> m_regions;         ///< All Regions
    GraphType m_graph;                 ///< The skeleton graph
    //Region structure stores tuple of flow edge descriptor,
    //index along flow edge, number of failed extentions
    unordered_map<RegionPtr, tuple<FED, size_t, size_t>> regions;
    unordered_map<FVD, bool> m_visited;
};

#endif
