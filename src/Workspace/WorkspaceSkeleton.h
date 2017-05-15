#ifndef WORKSPACE_SKELETON_H_
#define WORKSPACE_SKELETON_H_

#include <queue>
#include <set>
#include <unordered_map>

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Boundaries/Boundary.h"

#include <containers/sequential/graph/directed_preds_graph.h>

#include "Vector.h"

using namespace mathtool;
using namespace std;

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
        stapl::MULTIEDGES, Point3d, vector<Point3d>> GraphType;
    typedef GraphType::vertex_descriptor               VD;
    typedef GraphType::edge_descriptor                 ED;
    typedef typename GraphType::vertex_iterator        vertex_iterator;
    typedef typename GraphType::adj_edge_iterator      adj_edge_iterator;


    ////////////////////////////////////////////////////////////////////////////
    /// Auxiliary data for regions includes skeleton edge descriptor, index
    /// along flow edge, and number of failed extentions.
    ////////////////////////////////////////////////////////////////////////////
    struct RegionData {
      ED edgeDescriptor;     ///< Descriptor of the edge the region is traveling.
      size_t edgeIndex;      ///< Region is at this index on the skeleton edge.
      size_t successSamples; ///< Number of successful samples in this region.
      size_t failedSamples;  ///< Number of successful samples in this region.
      size_t totalSamples;   ///< Total samples in this region.
      double weight;         ///< Ratio of successful samples to total samples.
      double probability;    ///< Probability for selecting this region.
    };

    ///@}

    /// Helper function to find the skeleton vertex that is closest to a given
    /// workspace point.
    /// @param _target The workspace point.
    /// @return An iterator to the nearest skeleton vertex.
    vertex_iterator FindNearestVertex(const Point3d& _target);

    /// Find an edge iterator in the skeleton by descriptor.
    /// @param _edgeDescriptor The descriptor of the edge to search for.
    /// @return An iterator to the desired edge.
    adj_edge_iterator FindEdge(const ED& _edgeDescriptor);

    /// Find iterators to all edges inbound on a given vertex.
    /// @param _vertexDescriptor The descriptor for the vertex of interest.
    /// @return A set of iterators for each edge incident on _vertexDescriptor.
    std::vector<adj_edge_iterator> FindInboundEdges(const VD& _vertexDescriptor);

    /// Find iterators to all edges inbound on a given vertex.
    /// @param _vi An iterator to the vertex of interest.
    /// @return A set of iterators for each edge incident on _vi.
    std::vector<adj_edge_iterator> FindInboundEdges(const vertex_iterator& _vi);

    /// Produces a directed version of workspace skeleton
    /// @param _start a workspace point near the start
    /// @return a directed version of worksspace skeleton
    // look at Utilities/ReebGraphConstruction.GetFlowGraph()
    WorkspaceSkeleton Direct(const Point3d& _start);

    /// Creates regions arond the the vertex nearest to the start
    /// @param _start a workspace point near the start
    /// @param _regionRadius the region radius we want to use when creating
    // refer to MPLibrary/MPStrategies/DynamicRegionRRT.run()
    void InitRegions(const Point3d& _start, const double _regionRadius);

    ///
    void CreateRegions(const Point3d& _p, double _regionRadius);

    /// Get the set of active regions
    const vector<Boundary*>& GetRegions() const;

    /// Set the skeleton graph
    /// @param p_graph the flow graph we want to set for this skeleton
    void SetGraph(GraphType& _graph);

    /// Get the skeleton graph.
    GraphType& GetGraph() {return m_graph;}

    /// Test regions for containment of new configuration and advance if
    /// necessary
    /// @param _cfg the new configuration
    void AdvanceRegions(const Cfg& _cfg);

    /// Test if a configuration is inside a region
    /// @param
    bool IsTouching(const Cfg& _cfg, const Boundary* const _region,
        const double _robotFactor = 1);

    /// Prune the skeleton by removing vertices and edges that are not touched
    /// when back-tracking from the vertex nearst to a goal point.
    /// @param _goal A workspace point representing the goal.
    void Prune(const Point3d& _goal);

    /// Push the flow graph to medial axis
    /// @param _f the flow graph we want to push
    //void FlowToMedialAxis(GraphType& _f) const;

    ///

    const unordered_map<Boundary*, RegionData>& GetRegionMap() {
      return m_regionData;
    }

    RegionData& GetRegionData(Boundary* _boundary) {
      return m_regionData[_boundary];
    }

    // Select a region based on its ratio of successful samples to total samples.
    Boundary* SelectRegion();

    void IncrementSuccess(Boundary* _region);

  private:

    ///@name Helpers
    ///@{

    /// Mark all nodes as being unvisited.
    void MarkAllNodesUnvisited();

    /// Update the probability of selecting each sampling region.
    void ComputeProbability();

    ///@}
    ///@name Internal State
    ///@{

    vector<Boundary*> m_regions;  ///< The set of active regions.
    GraphType m_graph;            ///< The skeleton graph.

    double m_explore{0.5};  ///< Probability of explore.

    unordered_map<Boundary*, RegionData> m_regionData;
    unordered_map<VD, bool> m_visited;

    bool m_debug{false};  ///< Show debug messages?

    /// The 'start' vertex from which the skeleton was last directed.
    VD m_start{std::numeric_limits<VD>::max()};

    ///@}
};

#endif
