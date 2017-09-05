#ifndef PROPERTY_MAP_H_
#define PROPERTY_MAP_H_

#include <queue>
#include <unordered_map>

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Bodies/ActiveMultiBody.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "MPLibrary/MPBaseObject.h"
#include "WorkspaceSkeleton.h"

#include <containers/sequential/graph/directed_preds_graph.h>

#include "Vector.h"

using namespace mathtool;
using namespace std;


// Hash for edge descriptor
struct edgeHash {
  size_t operator()(const WorkspaceSkeleton::ED&  _ed) const {
    return hash<typename WorkspaceSkeleton::ED::edge_id_type>()(_ed.id());
  }
};


////////////////////////////////////////////////////////////////////////////////
/// Property map of the workspace skeleton
/// To use just edge property map, use default vertex property
////////////////////////////////////////////////////////////////////////////////
template<typename EdgeProperty, typename VertexProperty=void>
class PropertyMap {

  public:

    ///@name Local Types
    ///@{

    /// Graph type is a directed multiedge graph of points and paths.
    typedef WorkspaceSkeleton::GraphType  GraphType;
    typedef WorkspaceSkeleton::VD               VD;
    typedef WorkspaceSkeleton::ED                ED;
    typedef typename GraphType::vertex_iterator        vertex_iterator;
    typedef typename GraphType::adj_edge_iterator      adj_edge_iterator;

    // Filter function types. The filter function removes the properties or
    // skeleton elemets for which the function returns true
    typedef function<bool(EdgeProperty&)>     EdgeFilterFunction;
    typedef function<bool(VertexProperty&)>   VertexFilterFunction;

    typedef unordered_map<ED, EdgeProperty, edgeHash> EdgeMapType;
    typedef unordered_map<VD, VertexProperty> VertexMapType;

    ///@}
    ///@name Constructors
    ///@{

    PropertyMap() = default;
    PropertyMap(WorkspaceSkeleton* _ws) : m_skeleton(_ws)  {}

    ///@}
    ///@name Sub-graph Generators
    ///@{
    
    /// Compute the graph in terms of filter funtion
    /// then replace the old graph with the annotated one
    /// @param _f Filter functor: Delete edges for which function returns true
    /// @return A reference to the annotated workspace skeleton
    WorkspaceSkeleton* GetEdgeFilteredSkeleton(EdgeFilterFunction&& _f);

    /// Compute the graph in terms of filter funtion
    /// then replace the old graph with the annotated one
    /// @param _f Filter functor: Delete vertices for which function returns
    ///  true
    /// @return A reference to the annotated workspace skeleton
    WorkspaceSkeleton* GetVertexFilteredSkeleton(VertexFilterFunction&& _f);

    ///@}
    ///@name Accessors
    ///@{
    void SetSkeleton(WorkspaceSkeleton* _ws) { m_skeleton = _ws;}
    VertexMapType& GetVertexMap() { return m_vertexMap; }
    EdgeMapType& GetEdgeMap() { return m_edgeMap; }
    void SetVertexMap(VertexMapType& _vMap)  { m_vertexMap = _vMap; }
    void SetEdgeMap(EdgeMapType& _eMap)  { m_edgeMap = _eMap; }
	
    /// Individual accessors
    
    void SetVertexProperty(const VD& _v, const VertexProperty& _vp);
    void SetEdgeProperty(const ED& _e, const EdgeProperty& _ep);
    VertexProperty& GetVertexProperty(const VD& _v);
    EdgeProperty& GetEdgeProperty(const ED& _e);
    ///@}


  private:

    ///@name Internal State
    ///@{
    WorkspaceSkeleton* m_skeleton{nullptr};  ///< Original skeleton
    EdgeMapType m_edgeMap;  ///< Map for edge property
    VertexMapType m_vertexMap;  ///< Map for vertex property
    ///@}
};



/*------------------------------------------------------------------------*/


template<typename EdgeProperty, typename VertexProperty>
EdgeProperty&
PropertyMap<EdgeProperty, VertexProperty>::
GetEdgeProperty(const ED& _e) {
  return m_edgeMap.at(_e);
}

template<typename EdgeProperty, typename VertexProperty>
void
PropertyMap<EdgeProperty, VertexProperty>::
SetEdgeProperty(const ED& _e, const EdgeProperty& _ep) {
  m_edgeMap.emplace(_e, _ep);
}

template<typename EdgeProperty, typename VertexProperty>
VertexProperty&
PropertyMap<EdgeProperty, VertexProperty>::
GetVertexProperty(const VD& _v) {
  return m_vertexMap.at(_v);
}

template<typename EdgeProperty, typename VertexProperty>
void
PropertyMap<EdgeProperty, VertexProperty>::
SetVertexProperty(const VD& _v, const VertexProperty& _vp) {
  m_vertexMap.emplace(_v, _vp);
}

template<typename EdgeProperty, typename VertexProperty>
WorkspaceSkeleton*
PropertyMap<EdgeProperty, VertexProperty>:: 
GetVertexFilteredSkeleton(VertexFilterFunction&& _f)  {
  auto graph = m_skeleton->GetGraph();
  
  for(auto mit = m_vertexMap.begin(); 
	mit != m_vertexMap.end(); ++mit) {
    if(_f(mit->second)) {
      graph.delete_vertex(mit->first);
    }
  }
  m_skeleton->SetGraph(graph);

  return m_skeleton;
}

template<typename EdgeProperty, typename VertexProperty>
WorkspaceSkeleton*
PropertyMap<EdgeProperty, VertexProperty>:: 
GetEdgeFilteredSkeleton(EdgeFilterFunction&& _f)  {
  auto graph = m_skeleton->GetGraph();
  
  for(auto mit = m_edgeMap.begin(); 
	mit != m_edgeMap.end(); ++mit)  {
    if(_f(mit->second))  {
      graph.delete_edge(mit->first);
    }
  }
  m_skeleton->SetGraph(graph);

  return m_skeleton;
}

/*------------------ Clearance annotated skeleton ------------------------*/

/// Function to generate the annotated clearance skeleton
template <typename MPTraits>
PropertyMap<vector<double>,double>* 
ClearanceAnnotatedSkeleton(MPBaseObject<MPTraits>* _mp, WorkspaceSkeleton* _ws, 
       bool _boundary = true) {
  typedef typename MPTraits::CfgType CfgType;
  auto clearanceMap = new PropertyMap<vector<double>,double>(_ws);

  auto g = _ws->GetGraph();
  auto boundary = _mp->GetEnvironment()->GetBoundary();
  auto vc = _mp->GetValidityChecker("pqp_solid");

  auto pointRobot = _mp->GetMPProblem()->GetRobot("point");

  // Function to compute clearance for input point _p.
  auto getClearance = [&](const Point3d& _p) -> double {
    // Check against obstacles using a point robot.
    CfgType cfg(_p, pointRobot);
    CDInfo cdInfo(true);
    vc->IsValid(cfg, cdInfo, "Skeleton Clearance");

    // Check against boundary.
    if(_boundary)  {
      const double boundaryClearance = boundary->GetClearance(_p);
      if(boundaryClearance < cdInfo.m_minDist) {
        cdInfo.m_objectPoint = boundary->GetClearancePoint(_p);
        cdInfo.m_minDist = boundaryClearance;
      }
    }
    // Return the minimum clearance.
    return cdInfo.m_minDist;
  };

  // Graph vertices clearance.
  for(auto vit = g.begin(); vit != g.end(); ++vit)
    clearanceMap->SetVertexProperty(vit->descriptor(),
        getClearance(vit->property()));

  // Graph edges clearance.
  for(auto eit = g.edges_begin(); eit != g.edges_end(); ++eit)	{
    vector<double> clearances;
    for(auto pit = eit->property().begin(); pit < eit->property().end(); ++pit)
      clearances.push_back(getClearance(*pit));
    auto ed = WorkspaceSkeleton::ED(eit->source(), eit->target(), 
        eit->descriptor().id());
    clearanceMap->SetEdgeProperty(ed,clearances);
  }
  return clearanceMap;
}

/// Function to generate the skeleton with edges filtered based on a 
//tolerance value
template <typename MPTraits>
void 
ClearanceFilteredSkeleton(double _t, MPBaseObject<MPTraits>* _mp, 
                          WorkspaceSkeleton* _ws, bool _boundary = true) {

  // Function for filtering edges based on minimum clearance
  struct ClearanceFiltration {
    double m_min;   ///< Minimum clearance
    ClearanceFiltration(double _a)  { m_min = _a; }
    bool operator()(vector<double>& _i)  { 
      for(auto i: _i)
        if(i < m_min)
	  return true;
      return false; 
    }
  };
  
  auto clearanceMap = ClearanceAnnotatedSkeleton(_mp, _ws, _boundary);
  _ws = clearanceMap->GetEdgeFilteredSkeleton(ClearanceFiltration(_t));
}
/*------------------------------------------------------------------------*/
#endif
