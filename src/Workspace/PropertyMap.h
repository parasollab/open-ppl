#ifndef PROPERTY_MAP_H_
#define PROPERTY_MAP_H_

#include <queue>
#include <unordered_map>

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Bodies/MultiBody.h"
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
    typedef WorkspaceSkeleton                    GraphType;
    typedef WorkspaceSkeleton::VD                VD;
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

    ///@name IO
    ///@{

    /// Writes the graph to a file
    /// @param _file the output file name

    void Write(const std::string& _file);
    void WriteAnnotation(const std::string& _file);
    void ReadAnnotation(const std::string& _file);

    /// Reads the graph from a file
    /// @param _file the output file name

    void Read(const std::string& _file);
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

  for(auto mit = m_vertexMap.begin();
    mit != m_vertexMap.end(); ++mit) {
    if(_f(mit->second)) {
      m_skeleton->DeleteVertex(mit->first);
    }
  }

  return m_skeleton;
}

template<typename EdgeProperty, typename VertexProperty>
WorkspaceSkeleton*
PropertyMap<EdgeProperty, VertexProperty>::
GetEdgeFilteredSkeleton(EdgeFilterFunction&& _f)  {

  for(auto mit = m_edgeMap.begin();
    mit != m_edgeMap.end(); ++mit)  {
    if(_f(mit->second))  {
      m_skeleton->DeleteEdge(mit->first);
    }
  }

  return m_skeleton;
}

/*-------------------------- I/O helpers ---------------------------------*/

template<typename EdgeProperty, typename VertexProperty>
void
PropertyMap<EdgeProperty, VertexProperty>::
Write(const std::string& _file) {
  std::ofstream ofs(_file);

  ofs << m_vertexMap.size() <<" "<< m_edgeMap.size() << std::endl;
  for(auto vit : m_vertexMap){
    auto spk = this->GetVertexProperty(vit.first);
    ofs << vit.first <<" "<< spk.size() <<" ";
    for(auto s : spk)
      ofs << s <<" ";
    ofs << std::endl;
  }

  for(auto eit : m_edgeMap)	{
   auto spks = this->GetEdgeProperty(eit.first);

    ofs << eit.first.id() << " ";
    ofs << eit.first.source() << " " << eit.first.target() << " " << spks.size() << std::endl;

    for(auto spk : spks) {
      ofs << spk.size() << " ";
      for(auto s : spk) {
        ofs << s << " ";
      }
      ofs << std::endl;
    }
  }

  ofs.close();
}

template<typename EdgeProperty, typename VertexProperty>
void
PropertyMap<EdgeProperty, VertexProperty>::
Read(const std::string& _file) {
  typedef WorkspaceSkeleton::VD VD;
  typedef WorkspaceSkeleton::ED ED;

  std::fstream ifs(_file);

  if(!ifs) {
    std::cout << "File " << _file << " does not exist." << std::endl;
    return;
  }
  size_t nVerts, nEdges;

  ifs >> nVerts >> nEdges;

  for(size_t vit = 0; vit < nVerts; vit++) {
    size_t propSize;
    VD vid;
    std::vector<Point3d> vertexProperty;

    ifs >> vid >> propSize;
    for (size_t propit = 0; propit < propSize; propit++) {
      Point3d prop;
      ifs >> prop;
      vertexProperty.push_back(prop);
    }
    this->SetVertexProperty(vid, vertexProperty);
  }

  for(size_t eit = 0; eit < nEdges; eit++) {

    size_t proSize, eSource, eTarget, eDesID;
    std::vector<vector<Point3d> > intermediateWitnesses;

    ifs >> eDesID;
    ifs >> eSource >> eTarget >> proSize;
    for (size_t pit = 0; pit < proSize; pit++) {

      std::vector<Point3d> edgeProperty;
      size_t eSize;
      ifs >> eSize;
      for(size_t it = 0; it < eSize; it++) {
        Point3d prop;
        ifs >> prop;
        edgeProperty.push_back(prop);
      }

      intermediateWitnesses.push_back(edgeProperty);
    }

    auto ed =  ED(eSource, eTarget, eDesID);
    this->SetEdgeProperty(ed, intermediateWitnesses);
  }
}

template<typename EdgeProperty, typename VertexProperty>
void
PropertyMap<EdgeProperty, VertexProperty>::
WriteAnnotation(const std::string& _file) {
  std::ofstream ofs(_file);


  auto vMap = this->GetVertexMap();
  auto eMap = this->GetEdgeMap();

  ofs << vMap.size() <<" "<< eMap.size() << std::endl;

  for(auto vit : vMap){
    auto spk = this->GetVertexProperty(vit.first);
    ofs << vit.first <<" 1 ";
    ofs << spk << std::endl;
  }

  for(auto eit : eMap)	{
    auto spks = this->GetEdgeProperty(eit.first);
    ofs << eit.first.id() << " ";
    ofs << eit.first.source() << " " << eit.first.target() << " " << spks.size() << std::endl;

    for(auto spk : spks)
      ofs << " 1 " << spk << std::endl;
  }

  ofs.close();
}

template<typename EdgeProperty, typename VertexProperty>
void
PropertyMap<EdgeProperty, VertexProperty>::
ReadAnnotation(const std::string& _file) {
  typedef WorkspaceSkeleton::VD VD;
  typedef WorkspaceSkeleton::ED ED;

  std::fstream ifs(_file);
  size_t nVerts, nEdges;

  ifs >> nVerts >> nEdges;

  for(size_t vit = 0; vit < nVerts; vit++) {
    VD vid;
    size_t propSize;

    ifs >> vid >> propSize;
    for (size_t propit = 0; propit < propSize; propit++) {
      double prop;
      ifs >> prop;
      this->SetVertexProperty(vid, prop);
    }
  }

  for(size_t eit = 0; eit < nEdges; eit++) {

    size_t proSize, eSource, eTarget, eDesID;
    std::vector<double> intermediateWitnesses;

    ifs >> eDesID;
    ifs >> eSource >> eTarget >> proSize;
    for (size_t pit = 0; pit < proSize; pit++) {
      size_t eSize;
      ifs >> eSize;
      double prop;
      ifs >> prop;
      intermediateWitnesses.push_back(prop);
    }

    auto ed =  ED(eSource, eTarget, eDesID);
    this->SetEdgeProperty(ed, intermediateWitnesses);
  }
}

/*------------------ Clearance annotated skeleton ------------------------*/

/// Function to generate the annotated clearance skeleton
template <typename MPTraits>
PropertyMap<vector<double>,double>*
ClearanceAnnotatedSkeleton(MPBaseObject<MPTraits>* _mp, WorkspaceSkeleton* _ws,
       bool _boundary = true) {
  typedef typename MPTraits::CfgType CfgType;
  auto clearanceMap = new PropertyMap<vector<double>,double>(_ws);

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
  for(auto vit = _ws->begin(); vit != _ws->end(); ++vit)
    clearanceMap->SetVertexProperty(vit->descriptor(),
        getClearance(vit->property()));

  // Graph edges clearance.
  for(auto eit = _ws->edges_begin(); eit != _ws->edges_end(); ++eit)	{
    vector<double> clearances;
    for(auto pit = eit->property().begin() + 1; pit < eit->property().end(); ++pit)
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

/*--------------------Priority annotation skeleton-------------------------*/


template <typename MPTraits>
PropertyMap<vector<double>,double>*
PriorityAnnotatedSkeleton(MPBaseObject<MPTraits>* _mp, WorkspaceSkeleton* _ws) {
  typedef typename MPTraits::CfgType CfgType;
  auto lastPriorityMap = new PropertyMap<vector<double>,double>(_ws);

  for(auto vit = _ws->begin(); vit != _ws->end(); ++vit)
    lastPriorityMap->SetVertexProperty(vit->descriptor(), -1.0);

  for(auto eit = _ws->edges_begin(); eit != _ws->edges_end(); ++eit)	{
    vector<double> priorityVals;
    for(auto pit = eit->property().begin(); pit < eit->property().end(); ++pit)
      priorityVals.push_back(-1.0);
    auto ed = WorkspaceSkeleton::ED(eit->source(), eit->target(),
        eit->descriptor().id());
    lastPriorityMap->SetEdgeProperty(ed, priorityVals);
  }
  return lastPriorityMap;
}

/*------------------------------------------------------------------------*/

template <typename MPTraits>
PropertyMap<vector<double>,double>*
SuccessRateAnnotatedSkeleton(MPBaseObject<MPTraits>* _mp, WorkspaceSkeleton* _ws) {
  typedef typename MPTraits::CfgType CfgType;
  auto map = new PropertyMap<vector<double>,double>(_ws);

  for(auto eit = _ws->edges_begin(); eit != _ws->edges_end(); ++eit)	{
    auto ed = WorkspaceSkeleton::ED(eit->source(), eit->target(),
        eit->descriptor().id());

    // SetEdgeProperty only accepts an EdgeProperty
    vector<double> zero {0, 0}; 

    // zero.push_back(0); // nsucceses
    // zero.push_back(0); // nattempts

    map->SetEdgeProperty(ed, zero);
  }
  return map;
}

/*------------------ Distance annotated skeleton ------------------------*/

/// Function to generate the annotated distance skeleton
template <typename MPTraits>
PropertyMap<vector<double>,double>*
DistanceAnnotatedSkeleton(MPBaseObject<MPTraits>* _mp, WorkspaceSkeleton* _ws,
       string _distance_metric_label,
       size_t _goal_id) {
  typedef typename MPTraits::CfgType CfgType;

  // Create distance map to hold annotations.
  auto distanceMap = new PropertyMap<vector<double>,double>(_ws);

  // Not sure how to get a Point3D goal from MPBaseObject. Something like this?
  auto road_map = _mp->GetRoadmap();

  if (road_map == nullptr) {
    return nullptr;
  }

  // Declare a configuration for our goal pose.
  //  -GetVertex throws RunTimeException if requested node is not in roadmap.
  CfgType cfg_goal_point = road_map->GetVertex(_goal_id);

  // Get the pointer to the distance metric for euclidean distance?
  auto dist_metric  = _mp->GetDistanceMetric(_distance_metric_label);

  // Distance metric operates on Cfgs. Need a Cfg type like point robot.
  auto pointRobot = _mp->GetMPProblem()->GetRobot("point");

  // @TODO remove this code.
  // Declare a configuration for our goal pose.
  //CfgType cfg_goal_point(_goal_pos);

  // Function to compute clearance for input point _p.
  // Declare local lambda function for computing the distance. Will be used to
  //  loop over all the points in the skeleton graph.
  auto getDistance = [&](const Point3d& _p) -> double {
    CfgType cfg_skeleton_point(_p, pointRobot);
    return dist_metric->Distance(cfg_skeleton_point, cfg_goal_point);
  };

  // Graph vertices distance.
  for(auto vit = _ws->begin(); vit != _ws->end(); ++vit)
    distanceMap->SetVertexProperty(vit->descriptor(),
        getDistance(vit->property()));

  // Graph edges distance.
  for(auto eit = _ws->edges_begin(); eit != _ws->edges_end(); ++eit) {
    vector<double> distances;
    for(auto pit = eit->property().begin() + 1; pit < eit->property().end(); ++pit)
      distances.push_back(getDistance(*pit));
    auto ed = WorkspaceSkeleton::ED(eit->source(), eit->target(),
        eit->descriptor().id());
    distanceMap->SetEdgeProperty(ed,distances);
  }
  return distanceMap;
}
#endif
