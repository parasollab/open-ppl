#ifndef PMPL_DYNAMIC_REGIONS_PRM_H_
#define PMPL_DYNAMIC_REGIONS_PRM_H_

#include "MPStrategyMethod.h"

#include "Geometry/Boundaries/CSpaceBoundingSphere.h"
#include "MPLibrary/MPTools/MeanCurvatureSkeleton3D.h"
#include "MPLibrary/MPTools/ReebGraphConstruction.h"
#include "Utilities/MedialAxis2D.h"
#include "Utilities/XMLNode.h"
#include "Workspace/WorkspaceSkeleton.h"

#include <unordered_map>
#include <unordered_set>
#include <utility>

using EIT = WorkspaceSkeleton::adj_edge_iterator;
using VIT = WorkspaceSkeleton::vertex_iterator;

////////////////////////////////////////////////////////////////////////////////
/// Dynamic Regions PRM algorithm.
///
/// A PRM guided by a workspace skeleton.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////

class Region {
  public:
  EIT edgeIterator;
  size_t edgeIndex{0};
  double weight{0};      ///< Ratio of successful samples to total samples.
  Vector3d center{Vector3d(0.0,0.0,0.0)};
  double size{0.0};

  Region(const EIT& _eit = EIT()) : edgeIterator(_eit) {}

  Vector3d GetCenter() const {
    return center;
  }

  void SetCenter(Vector3d _c) {
    center =_c;
  }

  double GetSize() const {
    return size;
  }

  void SetSize(double _s) {
    size = _s;

  }

  EIT GetEdge() const {
    return edgeIterator;
  }

  void SetWeight(const size_t _success, const size_t _attempts) {
    weight = 1.0 * _success / _attempts;
  }

  double GetWeight() const {
    return weight;
  }

  virtual bool operator==(const Region& _key) const {
    return 0;
  }

  const Vector3d* GetNextSkeletonEdgePoint() {
    const std::vector<Point3d>& path = edgeIterator->property();
    auto currentIt = find(path.begin(), path.end(), center);
    if(currentIt >= path.end()-1)
      return nullptr;
    advance(currentIt,1);

    return &(*currentIt);
  }
};

class ExpansionRegion : public Region {
  public:
  mutable size_t componentID{0};
  virtual bool operator==(const ExpansionRegion& _key) const {
    return componentID == _key.componentID;
  }
  ExpansionRegion(const EIT& _eit = EIT()) : Region(_eit) {}
};

class ConnectionRegion : public Region {
  public:
  pair<size_t, size_t> componentIDs{make_pair(0,0)};

  virtual bool operator==(const ConnectionRegion& _key) const {
    return componentIDs == _key.componentIDs;
  }
  ConnectionRegion(const EIT& _eit = EIT()) : Region(_eit) {}
};

namespace std {
  template <>
    class hash<ExpansionRegion> {
      public:
      typedef ExpansionRegion ExpansionRegionType;
      size_t operator()(const ExpansionRegionType& _key) const {
        return _key.edgeIndex;
      }
    };
  template <>
    class hash<ConnectionRegion> {
      public:
      typedef ConnectionRegion ConnectionRegionType;
      size_t operator()(const ConnectionRegionType& _key) const {
        return _key.edgeIndex;
      }
    };
}


template <typename MPTraits>
class DynamicRegionsPRM : public MPStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::VertexSet VertexSet;


    ///@}
    ///@name Local Types
    ///@{

    /// Settings for a specific sampler.
    struct SamplerSetting {
      std::string label;
      size_t count;
      size_t attempts;
    };

    ///@}
    ///@name Construction
    ///@{

    DynamicRegionsPRM();

    DynamicRegionsPRM(XMLNode& _node);

    virtual ~DynamicRegionsPRM() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}

  protected:

    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Initialize() override;
    virtual void Iterate() override;

    ///@}
    ///@name Helpers
    ///@{


    /// Add start angn goals to the roadmap
    void AddQuery();

    /// Grow a tree from a vertex q
    /// Stop when q connects to the rest of the roadmap or is within the
    /// boundary of a skeleton vertex
    bool GrowRRT(VID _q, VIT& _vit);

    /// Extend a tree node towards a direction
    VID Extend(VID _nearVID, CfgType _dir, LPOutput<MPTraits>& _lp);

    /// Build topological skeleton
    void BuildSkeleton();

    /// Get clearance
    /// @return robot clearance from closest obstacle
    double GetClearance(Vector3d _v);

    /// Get region radius
    /// return radius = robot clearance - robot radius
    double GetRegionRadius(Vector3d _v);

    /// Make a boundary
    /// @return a boundary centered at _v of size _r
    Boundary* MakeBoundary(Vector3d _v, double _r);

    /// if the component covers the point, there is/was a region centered at the
    /// point recordered in the coverage map of the CC
    bool IsCovered(Vector3d _point, size_t _componentID);

    /// Sample and add configurations to the roadmap.
    /// @return The generated VIDs for the successful samples.
    std::vector<CfgType> Sample(const Boundary* _b, size_t& _attemptCount);

    /// Select a region based weighted success probabilities
    /// @return expansion region to be expanded
    ExpansionRegion* SelectExpansionRegion();

    /// Create a connection region between 2 components
    void CreateConnectionRegion(const EIT& _eit, Vector3d _p, size_t _ccA, size_t ccB);

    /// Create an expansion region for component _cc at skeleton vertex
    void CreateExpansionRegion(size_t _cc, VIT _vit);

    /// Select a connection region between two connected components
    /// @return expansion region to be expanded
    const ConnectionRegion* SelectConnectionRegion(size_t _ccA, size_t _ccB);

    /// Compute probabilities
    /// @return probabiliities based on expansion success
    vector<double> ComputeProbabilities();

    /// Return K nearest neighors from a set of candidates
    vector<Neighbor> FindNearestNeighbors(const CfgType& _cfg, const VertexSet* const _candidates);

    /// Attemot connections between a configuration and its neighbors
    /// @return the set of neighbors that successfully connect to _c
    bool AttemptConnection(CfgType _c1, CfgType _c2, LPOutput<MPTraits>& _lp);

   /// Expand a component in region r
   /// @return the list of new cfgs added to the component
    vector<CfgType> ExpandComponent(ExpansionRegion* _r);

    /// Advance a region along an edge
    /// @return true if the region is successfully advanced
    /// @return false if the region reaches end of the edge or an low clearance
    /// area
    bool AdvanceRegion(ExpansionRegion* _r);

    /// Check if region still covers a subset of the given samples set
    bool AreSamplesCovered(const ExpansionRegion* _region, const vector<CfgType> _samples);

    /// connect component to the rest of the graph using connection regions
    void ConnectComponent(size_t _componentID);
    ///@}
    ///@name Internal State
    ///@{

    /// Sampler labels with number and attempts of sampler.
    std::vector<SamplerSetting> m_samplers;
    /// Connector labels for node-to-node.
    std::vector<std::string> m_connectorLabels;

    /// An optional sampling boundary for fixing the base.
    std::unique_ptr<Boundary> m_samplingBoundary;

    std::string m_decompositionLabel; ///< The workspace decomposition label.
    double edgeClearance{0.};     ///< The edge clearance for that region.

    PropertyMap<vector<double>, double>* m_skeletonClearanceMap{nullptr};
    string m_annotationFile;

    std::string m_skeletonType{"reeb"};

    /// Weight of explore vs. exploit in region selection probabilities.
    /// Exploring is a uniform chance to select each region, while exploit
    /// favors successful regions.
    double m_explore{.5};
    ///@}
    std::string m_nfLabel;       ///< The neighborhood finder label.
    std::string m_lpLabel;       ///< The neighborhood finder label.
    std::string m_exLabel;       ///< The extender label.
    WorkspaceSkeleton m_skeleton;     ///< The workspace skeleton.

    std::unordered_map<size_t, std::unordered_set<mathtool::Vector3d> > m_skeletonCoverageMap;
    std::unordered_map<size_t, std::unordered_set<ExpansionRegion> > m_ccExpansionRegionMap;
    std::unordered_map<pair<size_t, size_t>, std::unordered_set<ConnectionRegion> > m_ccConnectionRegionMap;

    vector<ExpansionRegion> m_expansionRegions;
    vector<ConnectionRegion> m_connectionRegions;

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
DynamicRegionsPRM<MPTraits>::
DynamicRegionsPRM() {
  this->SetName("DynamicRegionsPRM");
}


template <typename MPTraits>
DynamicRegionsPRM<MPTraits>::
DynamicRegionsPRM(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  this->SetName("DynamicRegionsPRM");

  m_nfLabel = _node.Read("nfLabel", true, "", "Neighborhood Finder");
  m_lpLabel = _node.Read("lpLabel", true, "", "Local Planner");
  m_exLabel = _node.Read("extenderLabel", true, "", "Extender label");

  for(auto& child : _node) {
    if(child.Name() == "Sampler") {
      SamplerSetting s;
      s.label = child.Read("label", true, "", "Sampler Label");
      s.count = child.Read("number", true,
          1, 0, MAX_INT, "Number of samples");
      s.attempts = child.Read("attempts", false,
          1, 0, MAX_INT, "Number of attempts per sample");
      m_samplers.push_back(s);
    }
    else if(child.Name() == "Connector")
      m_connectorLabels.push_back(
          child.Read("label", true, "", "Connector Label"));
  }

  m_decompositionLabel = _node.Read("decompositionLabel", true, "",
      "The workspace decomposition to use.");
  m_skeletonType = _node.Read("skeletonType", false, "reeb",
      "the type of skeleton to use, Available options are reeb and mcs");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
Print(std::ostream& _os) const {
  MPStrategyMethod<MPTraits>::Print(_os);
  _os << "\tSamplers" << std::endl;
  for(const auto& sampler : m_samplers)
    _os << "\t\t" << sampler.label
      << "\tNumber:"   << sampler.count
      << "\tAttempts:" << sampler.attempts
      << std::endl;

  _os << "\tConnectors" << std::endl;
  for(const auto& label : m_connectorLabels)
    _os << "\t\t" << label << std::endl;
}

/*------------------------ MPStrategyMethod Overrides ------------------------*/

template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
Initialize() {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::Iterate");

  // build the skeleton first
  this->BuildSkeleton();
  // Save the set of created regions to return.
  std::vector<Boundary*> regions;
  auto& skeletonGraph = m_skeleton.GetGraph();
  auto g = this->GetRoadmap();

  // Check each skeleton node to see if a new region should be created.
  for(auto iter = skeletonGraph.begin(); iter != skeletonGraph.end(); ++iter) {
    // Create a new region for each outgoing edge of this skeleton node.
    if(this->m_debug)
      std::cout << "skeleton vertex: " << iter->descriptor() << std::endl;

    double regionSize = this->GetRegionRadius(iter->property());

    if(regionSize < 0) {
      if(this->m_debug)
        std::cout << " region size " << regionSize << " too small." << std::endl;
      continue;
    }

    //generate a valid configuration in the boundary centered at the skeleton
    //vertex with size regionSize
    auto boundary = MakeBoundary(iter->property(), regionSize);
    size_t attempts = 0;
    vector<CfgType> samples = this->Sample(boundary, attempts);
    if(samples.empty()) {
      if(this->m_debug)
        std::cout << "Could not find a valid sample at this vertex." << std::endl;
      continue;
    }

    auto s = samples[0];
    auto ccID = g->AddVertex(s);
    for(auto eit = iter->begin(); eit != iter ->end(); ++eit) {
      const std::vector<Point3d>& path = eit->property();
      if(IsCovered(path[0], ccID))
        continue;
      ExpansionRegion er(eit);
      er.componentID = ccID;
      er.SetCenter(path[0]);
      er.SetSize(regionSize);
      er.SetWeight(1, attempts);
      m_ccExpansionRegionMap[ccID].insert(er);
      m_expansionRegions.push_back(er);
      m_skeletonCoverageMap[ccID].insert(path[0]);
    }
      m_skeletonCoverageMap[ccID].insert(iter->property());

  }
  if(this->m_debug)
    std::cout << "Done initializing " << m_ccExpansionRegionMap.size()
              << " expansion regions." << endl;
  AddQuery();
}


template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
Iterate() {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::Iterate");

  auto region = SelectExpansionRegion();
  auto samples = this->ExpandComponent(region);
  if(region != nullptr) {
    if(!samples.empty()) {
      auto ccID = region->componentID;
      while (AdvanceRegion(region) && AreSamplesCovered(region, samples))
        continue;
      ConnectComponent(ccID);
    }
  }
  else {
    //get random component
    auto ccs = this->GetRoadmap()->GetCCTracker()->GetRepresentatives();
    auto randomIndex = ccs.begin();
    advance(randomIndex, LRand() % ccs.size());
    ConnectComponent(*randomIndex);
  }
}

/*--------------------------------- Helpers ----------------------------------*/

/*-------------------- Roadmap builders -------------------------------------*/

template<typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
AddQuery() {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::AddQuery");

  auto g = this->GetRoadmap();
  vector<VID> queryPoints;
  const VID start = this->GenerateStart(m_samplers.front().label);
  std::vector<VID> goals = this->GenerateGoals(m_samplers.front().label);
  queryPoints.push_back(start);
  queryPoints.insert(queryPoints.end(), goals.begin(), goals.end());

  VertexSet nodes;
  for(auto vi = g->begin(); vi != g->end(); ++vi)
    nodes.insert(VID(vi->descriptor()));

  for(auto q : queryPoints) {
    auto knn = FindNearestNeighbors(g->GetVertex(q), &(nodes));
    vector<pair<VID,LPOutput<MPTraits> > > targets;
    for(auto n : knn) {
      LPOutput<MPTraits> lp;
      if(AttemptConnection(g->GetVertex(q), g->GetVertex(n.target), lp))
        targets.push_back(make_pair(n.target, lp));
    }
    if(targets.empty()) {
      VIT vit;
      auto qIsConnectedToG = GrowRRT(q, vit);
      if(!qIsConnectedToG)
        CreateExpansionRegion(q, vit);
      continue;
    }
    auto closest = targets[0];
    g->AddEdge(q, closest.first, closest.second.m_edge);
  }
}


template<typename MPTraits>
bool
DynamicRegionsPRM<MPTraits>::
GrowRRT(VID _q, VIT& _vit) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::GrowRRT");

  auto g = this->GetRoadmap();
  auto& skeletonGraph = m_skeleton.GetGraph();
  VertexSet tree;
  tree.insert(_q);
  while(true) {
    size_t attempts;
    std::vector<CfgType> samples = Sample(this->GetEnvironment()->GetBoundary(), attempts);
    if(samples.empty())
      continue;
    CfgType q_rand = samples.front();
    auto neighbors = FindNearestNeighbors(q_rand, &tree);
    if(neighbors.empty())
      continue;
    auto q_near = neighbors[0].target;
    LPOutput<MPTraits> lp;
    auto q_new = Extend(q_near, q_rand, lp);
    if(q_new == INVALID_VID)
      continue;

    tree.insert(q_new);

    // Try to connect q_new to the roadmap
    auto representatives = g->GetCCTracker()->GetRepresentatives();
    for(auto rep : representatives) {
      if(g->GetCCTracker()->InSameCC(rep, q_new))
        continue;
      auto cc = g->GetCCTracker()->GetCC(rep);
      auto neighbors = FindNearestNeighbors(g->GetVertex(q_new), &cc);
      for(auto& n : neighbors) {
        LPOutput<MPTraits> lp2;
        if(AttemptConnection(g->GetVertex(q_new), g->GetVertex(n.target), lp2)) {
          g->AddEdge(q_new, n.target, lp2.m_edge);
          return true;
        }
      }
    }

    // Check each skeleton node to see if a new region should be created.
    for(auto iter = skeletonGraph.begin(); iter != skeletonGraph.end(); ++iter) {
      auto boundary = MakeBoundary(iter->property(), GetRegionRadius(iter->property()));
      if(g->GetVertex(q_new).InBounds(boundary)) {
        _vit = iter;
        return false;
      }
    }
  }
  return false;
}


template <typename MPTraits>
typename DynamicRegionsPRM<MPTraits>::VID
DynamicRegionsPRM<MPTraits>::
Extend(VID _nearVID, CfgType _dir, LPOutput<MPTraits>& _lp) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::Extend");

  auto e = this->GetExtender(m_exLabel);
  const CfgType& qNear = this->GetRoadmap()->GetVertex(_nearVID);
  CfgType qNew(this->GetTask()->GetRobot());

  const bool success = e->Extend(qNear, _dir, qNew, _lp);
  if(this->m_debug)
    std::cout << "Extending from VID " << _nearVID
              << "\n\tqNear: " << qNear.PrettyPrint()
              << "\n\tExtended "
              << std::setprecision(4) << _lp.m_edge.first.GetWeight()
              << " units."
              << std::endl;

  if(!success) {
    // The extension failed to exceed the minimum distance.
    if(this->m_debug)
      std::cout << "\tNode too close, not adding." << std::endl;
    return INVALID_VID;
  }

  // The extension succeeded. Try to add the node.
  const auto newVID = this->GetRoadmap()->AddVertex(qNew);

  // The node was ok. Add the edge.
  this->GetRoadmap()->AddEdge(_nearVID, newVID, _lp.m_edge);

  return newVID;
}


template <typename MPTraits>
std::vector<typename DynamicRegionsPRM<MPTraits>::CfgType>
DynamicRegionsPRM<MPTraits>::
Sample(const Boundary* _b, size_t& _attemptCount) {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::Sample");

  auto boundary = _b != NULL ? _b : this->GetEnvironment()->GetBoundary();

  // Generate nodes with each sampler.
  std::vector<CfgType> samples;
  std::vector<CfgType> collision;
  for(auto& sampler : m_samplers) {
    samples.clear();
    samples.reserve(sampler.count);

    auto s = this->GetSampler(sampler.label);
    s->Sample(sampler.count, sampler.attempts, boundary,
        std::back_inserter(samples), std::back_inserter(collision));

    _attemptCount += samples.size() + collision.size();

    if(this->m_debug)
      std::cout << "\tSampler '" << sampler.label << "' generated "
        << samples.size() << "/" << sampler.count << " configurations."
        << std::endl;
  }

  return samples;
}


template <typename MPTraits>
bool
DynamicRegionsPRM<MPTraits>::
AttemptConnection(CfgType _c1, CfgType _c2, LPOutput<MPTraits>& _lpOutput) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::AttemptConnection");

  auto lp = this->GetLocalPlanner(m_lpLabel);
  Environment* env = this->GetEnvironment();

  return (lp->IsConnected(_c1, _c2, &_lpOutput,
        env->GetPositionRes(), env->GetOrientationRes(), true, false));
}


template <typename MPTraits>
vector<Neighbor>
DynamicRegionsPRM<MPTraits>::
FindNearestNeighbors(const CfgType& _cfg, const VertexSet* const _candidates) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::FindNearestNeighbors");

  if(this->m_debug)
    std::cout << "Searching for nearest neighbors to " << _cfg.PrettyPrint()
      << " with '" << m_nfLabel << "' from "
      << (_candidates
          ? "a set of size " + std::to_string(_candidates->size())
          : "the full roadmap")
      << "."
      << std::endl;

  std::vector<Neighbor> neighbors;
  auto g = this->GetRoadmap();
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  if(_candidates)
    nf->FindNeighbors(g, _cfg, *_candidates, neighbors);
  else
    nf->FindNeighbors(g, _cfg, std::back_inserter(neighbors));

  if(neighbors.empty()) {
    stats->IncStat("BasicRRTStrategy::FailedNF");
    if(this->m_debug)
      std::cout << "\tFailed to find a nearest neighbor."
        << std::endl;
  }
  if(this->m_debug)
    cout << "Returning " << neighbors.size() << " nearest neighbors" << endl;
    return neighbors;
}


/*--------------------------- Regions Methods ------------------------------*/

template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
CreateExpansionRegion(size_t _cc, const VIT _vit) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::CreateExpansionRegion");

  size_t count = 0;
  for(auto eit = _vit->begin(); eit != _vit->end(); ++eit) {
    const std::vector<Point3d>& path = eit->property();
    if(IsCovered(path[0], _cc))
      continue;

    double regionSize = GetRegionRadius(path[0]);
    ExpansionRegion er(eit);
    er.componentID = _cc;
    er.SetCenter(path[0]);
    er.SetSize(regionSize);
    m_ccExpansionRegionMap[_cc].insert(er);
    m_expansionRegions.push_back(er);
    m_skeletonCoverageMap[_cc].insert(path[0]);
    count++;
  }
    m_skeletonCoverageMap[_cc].insert(_vit->property());
  if(this->m_debug) {
    std::cout << count << " new expansion regions added at skeleton vertex "
              << _vit->descriptor()
              << endl;
  }
}


template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
CreateConnectionRegion(const EIT& _eit, Vector3d _p, size_t _ccA, size_t _ccB) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::CreateConnectionRegion");

  ConnectionRegion cr (_eit);
  cr.componentIDs = make_pair(_ccA, _ccB);
  cr.SetCenter(_p);
  cr.SetSize(GetRegionRadius(_p));
  m_connectionRegions.push_back(cr);
  m_ccConnectionRegionMap[std::minmax(_ccA, _ccB)].insert(cr);
  if(this->m_debug) {
    std::cout << "New connection region added between CC " << _ccA
              << " and " << _ccB <<"."
              << " Total number of connection regions: "
              << m_connectionRegions.size()
              << endl;
  }
}


template <typename MPTraits>
bool
DynamicRegionsPRM<MPTraits>::
AdvanceRegion(ExpansionRegion* _r) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::AdvanceRegion");

  if(_r == nullptr) {
    cout << "Cannot advance whole environment. Why is this being called?" << endl;
    return false;
  }
  // mark current position as covered and spawn connection regions
  auto ccID = _r->componentID;
  auto position = _r->GetCenter();
  m_skeletonCoverageMap[ccID].insert(position);
  auto g = this->GetRoadmap();
  auto ccTracker = g->GetCCTracker();
  auto representatives = ccTracker->GetRepresentatives();
  for(auto & rep : representatives) {
    if(ccTracker->InSameCC(ccID, rep))
      continue;
    if(!IsCovered(position, rep))
      continue;

    CreateConnectionRegion(_r->edgeIterator, position, ccID, rep);
    if(this->m_debug)
      cout << rep << " and " << ccID << " overlap. Create a connection region." << endl;
  }

  //check for end of edge
  auto nextEdgePoint = _r->GetNextSkeletonEdgePoint();
  if(nextEdgePoint == nullptr) {
    //remove region from list of regions
    m_expansionRegions.erase(remove(m_expansionRegions.begin(), m_expansionRegions.end(), *_r), m_expansionRegions.end());

    VIT vit = m_skeleton.find_vertex(_r->edgeIterator->target());
    CreateExpansionRegion(ccID, vit);
  if(this->m_debug)
    cout << "region reached end of edge. New region created at vertex "
         << vit->descriptor()
         << " with component ID "
         << ccID
         << endl;
    return false;
  }
  //check for usable clearance
  double clearance = GetRegionRadius(*nextEdgePoint);
  if(clearance < 0) {
    if(this->m_debug)
      cout << "region clearance not usable. Cannot advance." << endl;
    m_expansionRegions.erase(remove(m_expansionRegions.begin(), m_expansionRegions.end(), *_r), m_expansionRegions.end());
    return false;
  }
  //move to the next position
  _r->SetCenter(*nextEdgePoint);
  _r->SetSize(clearance);
  m_skeletonCoverageMap[ccID].insert(*nextEdgePoint);
  if(this->m_debug)
    cout << "region advanced to edge point " << *nextEdgePoint
         << " with size " << clearance << endl;
  return true;
}


template<typename MPTraits>
Boundary*
DynamicRegionsPRM<MPTraits>::
MakeBoundary(Vector3d _v, double _r) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::MakeBoundary");

  vector<double> v{_v[0], _v[1], _v[2]};
  return new CSpaceBoundingSphere(v, _r);
}


template<typename MPTraits>
double
DynamicRegionsPRM<MPTraits>::
GetRegionRadius(Vector3d _v) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::GetRegionRadius");

  auto clearance = GetClearance(_v);
  double robotRadius = this->GetTask()->GetRobot()->GetMultiBody()->GetBoundingSphereRadius();
  return (clearance - robotRadius);
}


template<typename MPTraits>
double
DynamicRegionsPRM<MPTraits>::
GetClearance(Vector3d _v) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::GetClearance");

  auto pointRobot = this->GetMPProblem()->GetRobot("point");
  auto vc = this->GetValidityChecker("pqp_solid");
  string callee = this->GetNameAndLabel() + "::GetClearance";
  CfgType cfg(_v, pointRobot);
  CDInfo cd(true);
  vc->IsValid(cfg, cd, callee);
  return cd.m_minDist;
}


template <typename MPTraits>
bool
DynamicRegionsPRM<MPTraits>::
AreSamplesCovered(const ExpansionRegion* _region, const vector<CfgType> _samples) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::AreSamplesCovered");

  auto boundary = MakeBoundary(_region->GetCenter(), _region->GetSize());
  for(auto s: _samples) {
    if(s.InBounds(boundary))
      return true;
  }
  return false;
}


template<typename MPTraits>
ExpansionRegion*
DynamicRegionsPRM <MPTraits>::
SelectExpansionRegion() {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::SelectExpansionRegion");

  // Update all region probabilities.
  std::vector<double> probabilities = ComputeProbabilities();

  // Construct with random number generator with the region probabilities.
  static std::default_random_engine generator(0);
  std::discrete_distribution<size_t> distribution(probabilities.begin(),
      probabilities.end());

  const size_t index = distribution(generator);
  const bool envSelected = index == m_expansionRegions.size();

  if(this->m_debug) {
    std::cout << "Updated region selection probabilities ("
              << "last is whole env):\n\t";

    for(auto p : distribution.probabilities())
      std::cout << std::setprecision(4) << p << " ";

    std::cout << "\n\tSelected index " << index
              << (envSelected ? " (whole env)." : " ")
              << std::endl;
  }

  if(envSelected)
    return nullptr;

  // Get the selected region.
  auto it = m_expansionRegions.begin();
  advance(it, index);

  if(this->m_debug) {
    auto c = it->GetCenter();
    std::cout << "\tRegion is " << it->edgeIndex
              << " with center at "
              << Vector3d(c[0], c[1], c[2])
              << "." << std::endl;
  }

  return &(*it);
}


template<typename MPTraits>
const ConnectionRegion*
DynamicRegionsPRM <MPTraits>::
SelectConnectionRegion(size_t _ccA, size_t _ccB) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::SelectConnectionRegion");

  if(m_ccConnectionRegionMap.size() == 0) {
    if(this->m_debug)
      cout << "No connection regions available at this moment." << endl;
    return nullptr;
  }
  auto regions = m_ccConnectionRegionMap[std::minmax(_ccA, _ccB)];
  if(regions.size() == 0)
    return nullptr;
  //return a random connection region in that list
  auto index = regions.begin();
  size_t randomPos = LRand() % regions.size();

  if(this->m_debug)
    cout << "Selected connection region "
         << randomPos
         << " out of "
         << regions.size()
         << "between " << _ccA << " and " << _ccB
         << endl;

  advance(index, randomPos);
  return &(*index);
}


template<typename MPTraits>
std::vector<double>
DynamicRegionsPRM <MPTraits>::
ComputeProbabilities() {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::ComputeProbabilities");

  // Sum all weights of all current regions.
  double totalWeight = 0.;
  for(auto& regionInfo : m_expansionRegions)
    totalWeight += regionInfo.GetWeight();

  // Compute the probabilities for the current regions.
  std::vector<double> probabilities;
  probabilities.reserve(m_expansionRegions.size() + 1);

  const double explore = m_explore / (m_expansionRegions.size() + 1);

  for(auto& regionInfo : m_expansionRegions) {
    const auto& weight = regionInfo.GetWeight();
    const double exploit = (1 - m_explore) * weight / totalWeight;

    probabilities.emplace_back(exploit + explore);
  }

  // Get the probability for the whole environment.
  probabilities.emplace_back(explore);

  return probabilities;
}


/*------------------------- Connected Component Methods -------------------*/

template <typename MPTraits>
vector<typename DynamicRegionsPRM<MPTraits>::CfgType>
DynamicRegionsPRM<MPTraits>::
ExpandComponent(ExpansionRegion* _r) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::ExpandComponent");

  auto boundary = _r == nullptr ? this->GetEnvironment()->GetBoundary() :
                  MakeBoundary(_r->GetCenter(), _r->GetSize());
  size_t attempts = 0;
  auto samples = Sample(boundary, attempts);
  auto g = this->GetRoadmap();
  auto ccTracker = g->GetCCTracker();
  VertexSet nodes;
  if(_r == nullptr)
    for(auto vi = g->begin(); vi != g->end(); ++vi)
      nodes.insert(VID(vi->descriptor()));
  else
    nodes = ccTracker->GetCC(_r->componentID);

  for(auto s : samples) {
    auto knn = FindNearestNeighbors(s, &(nodes));
    vector<pair<VID, LPOutput<MPTraits> > > edgeTargets;
    for(auto n : knn) {
      LPOutput<MPTraits> lp;
      if(AttemptConnection(s, g->GetVertex(n.target), lp))
        edgeTargets.push_back(make_pair(n.target, lp));
    }
    if(edgeTargets.empty()) {
      samples.erase(remove(samples.begin(), samples.end(),s), samples.end());
      continue;
    }
    auto vid = g->AddVertex(s);
    for(auto target: edgeTargets) {
      g->AddEdge(vid, target.first, target.second.m_edge);
    }
  }
  if(_r != nullptr)
    _r->SetWeight(samples.size(), attempts);

  return samples;
}


template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
ConnectComponent(size_t _componentID) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::ConnectComponent");

  auto g = this->GetRoadmap();
  auto ccTracker = g->GetCCTracker();
  auto representatives = ccTracker->GetRepresentatives();
  if(this->m_debug) {
    cout << "Attempting to connect "
      << _componentID << " and " << representatives.size()
      << " connected components."
      << endl;
    for(auto & rep : representatives)
      cout << rep << ", ";
    cout << endl;
  }
  for(auto & rep : representatives) {
    if(ccTracker->InSameCC(_componentID, rep))
      continue;
    auto region = SelectConnectionRegion(_componentID, rep);
    if(region == nullptr)
      continue;
    auto boundary = MakeBoundary(region->GetCenter(), region->GetSize());
    size_t attempts = 0;
    auto samples = Sample(boundary, attempts);
    for(auto s : samples) {
      auto knnA = FindNearestNeighbors(s, &(ccTracker->GetCC(_componentID)));
      vector<pair<VID, LPOutput<MPTraits> > > edgeSetA;
      for(auto nA : knnA) {
        LPOutput<MPTraits> lpA;
        if(AttemptConnection(s, g->GetVertex(nA.target), lpA))
          edgeSetA.push_back(make_pair(nA.target, lpA));
      }
      auto knnB = FindNearestNeighbors(s, &(ccTracker->GetCC(rep)));
      vector<pair<VID, LPOutput<MPTraits> > > edgeSetB;
      for(auto nB : knnB) {
        LPOutput<MPTraits> lpB;
        if(AttemptConnection(s, g->GetVertex(nB.target), lpB))
          edgeSetB.push_back(make_pair(nB.target, lpB));
      }
      if(edgeSetA.empty() && edgeSetB.empty())
        continue;
      if(this->m_debug)
        cout << edgeSetA.size() << " edges connecting to CC " << _componentID
          << ", " << edgeSetB.size() << " edges connecting to CC " << rep << endl;
      if(!edgeSetA.empty() && !edgeSetB.empty()) {
        if(this->m_debug)
          cout << "Merging CCs "
            << _componentID << ", " << rep
            <<". Current #CCs: " << representatives.size()
            << endl;
        //clear regions that are superfluous after merging
        auto cRegions = m_ccConnectionRegionMap[std::minmax(rep, _componentID)];
        for(auto& cr : cRegions) {
          m_connectionRegions.erase(remove(m_connectionRegions.begin(),
                m_connectionRegions.end(), cr), m_connectionRegions.end());
        }
        auto er1 = m_ccExpansionRegionMap[_componentID];
        if(this->m_debug)
          cout << "CC " << _componentID << " had "
               << er1.size() << "expansion regions.";
        for(auto& er : er1) {
          const std::vector<Point3d>& path = er.GetEdge()->property();
          if(IsCovered(path[0], rep)) {
            m_expansionRegions.erase(remove(m_expansionRegions.begin(),
                  m_expansionRegions.end(), er), m_expansionRegions.end());
            m_ccExpansionRegionMap[_componentID].erase(er);
          }
        }
        if(this->m_debug)
          cout << " " << er1.size() << " left uncovered by CC " << rep << endl;

        auto er2 = m_ccExpansionRegionMap[rep];
        if(this->m_debug)
          cout << "CC " << rep << " had "
               << er2.size() << "expansion regions.";
        for(auto& er : er2) {
          const std::vector<Point3d>& path = er.GetEdge()->property();
          if(IsCovered(path[0], _componentID)) {
            m_expansionRegions.erase(remove(m_expansionRegions.begin(),
                  m_expansionRegions.end(), er), m_expansionRegions.end());
            m_ccExpansionRegionMap[rep].erase(er);
          }
          else {
            er.componentID = _componentID;
            m_ccExpansionRegionMap[_componentID].insert(er);
          }
        }
        er1 = m_ccExpansionRegionMap[_componentID];
        if(this->m_debug)
          cout << " " << er2.size() << " left uncovered by CC " << _componentID
               << "\n"
               << "New size after merging: "
               << er1.size()
               << endl;
      }
      VID vid = g->AddVertex(s);
      for(auto e1 : edgeSetA)
        g->AddEdge(vid, e1.first, e1.second.m_edge);
      for(auto e2 : edgeSetB)
        g->AddEdge(vid, e2.first, e2.second.m_edge);
    }
  }
}


template<typename MPTraits>
bool
DynamicRegionsPRM <MPTraits>::
IsCovered(Vector3d _point, size_t _componentID) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::IsCovered");

  auto g = this->GetRoadmap();
  auto ccTracker = g->GetCCTracker();
  for(auto iter = m_skeletonCoverageMap.begin(); iter != m_skeletonCoverageMap.end(); ++iter) {
    if(ccTracker->InSameCC(_componentID, iter->first)) {
      auto regions = iter->second;

      if(regions.empty())
        return false;

      for(auto& r : regions) {
        if(r == _point)
          return true;
      }
    }
  }
  return false;
}

/*---------------------------- Topological Skeleton --------------------------*/

template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
BuildSkeleton() {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::BuildSkeleton");

  // Determine if we need a 2d or 3d skeleton.
  auto env = this->GetEnvironment();
  auto robot = this->GetTask()->GetRobot();
  const bool threeD = robot->GetMultiBody()->GetBaseType() == Body::Type::Volumetric;

  if(threeD) {
    auto decomposition = this->GetMPTools()->GetDecomposition(m_decompositionLabel);
    if(m_skeletonType == "mcs") {
      if(this->m_debug)
        cout << "Building a Mean Curvature skeleton" << endl;
      MeanCurvatureSkeleton3D mcs;
      mcs.SetEnvironment(this->GetEnvironment());
      mcs.BuildSkeleton();

      // Create the workspace skeleton.
      auto sk = mcs.GetSkeleton();
      m_skeleton = sk.first;
    }
    else {
      // Create a workspace skeleton using a reeb graph.
      if(this->m_debug)
        cout << "Building a Reeb Graph skeleton" << endl;
      ReebGraphConstruction reeb;
      reeb.Construct(decomposition);

      // Create the workspace skeleton.
      m_skeleton = reeb.GetSkeleton();
    }
  }
  else {
    // Collect the obstacles we want to consider (all in this case).
    std::vector<GMSPolyhedron> polyhedra;
    for(size_t i = 0; i < env->NumObstacles(); ++i) {
      MultiBody* const obstacle = env->GetObstacle(i);
      for(size_t j = 0; j < obstacle->GetNumBodies(); ++j)
        polyhedra.emplace_back(obstacle->GetBody(j)->GetWorldPolyhedron());
    }

    // Build a skeleton from a 2D medial axis.
    MedialAxis2D ma(polyhedra, env->GetBoundary());
    ma.BuildMedialAxis();
    m_skeleton = get<0>(ma.GetSkeleton(1)); // 1 for free space.
  }
}

/*----------------------------------------------------------------------------*/

#endif
