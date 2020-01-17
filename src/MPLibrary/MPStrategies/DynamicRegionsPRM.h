#ifndef PMPL_DYNAMIC_REGIONS_PRM_H_
#define PMPL_DYNAMIC_REGIONS_PRM_H_

#include "MPStrategyMethod.h"

#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "Geometry/Boundaries/CSpaceBoundingSphere.h"
#include "MPLibrary/MPTools/MeanCurvatureSkeleton3D.h"
#include "MPLibrary/MPTools/ReebGraphConstruction.h"
#include "MPLibrary/MPTools/RegionKit.h"
#include "MPLibrary/MPTools/SkeletonClearanceUtility.h"
#include "Utilities/MedialAxis2D.h"
#include "Utilities/XMLNode.h"
#include "Workspace/WorkspaceSkeleton.h"
#include "Vector.h"

#include <iomanip>
#include <iostream>
#include <iterator>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <string>
#include <vector>

using VD = WorkspaceSkeleton::VD;
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
  mutable size_t successes{0};   ///< Number of valid samples in this region.
  mutable size_t attempts{1};    ///< Number of sampling attempts in this region.
  mutable double weight{0};      ///< Ratio of successful samples to total samples.
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

  void UpdateSuccessRate(const size_t _success, const size_t _attempts) {
    successes = _success;
    attempts = _attempts;

    weight = 1.0 * successes / attempts;
  }

  virtual bool operator==(const Region& _key) const {return 0;}

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
  size_t componentID{0};
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

    void BuildSkeleton();

    double GetClearance(Vector3d _v);

    double GetRegionRadius(Vector3d _v);

    Boundary* MakeBoundary(Vector3d _v, double _r);

    bool IsCovered(Vector3d _point, size_t _componentID);

    /// Initialize the object just in time by building a workspace skeleton and
    /// region kit.
    void LazyInitialize();

    /// Sample and add configurations to the roadmap.
    /// @return The generated VIDs for the successful samples.
    std::vector<CfgType> Sample(const Boundary* _b, size_t& _attemptCount);

    /// Connect nodes in the roadmap.
    /// @param _vids A set of node VIDs to connect to the rest of the roadmap.
    void Connect(const std::vector<VID>& _vids);

    /// Select a region based weighted success probabilities
    /// @return expansion region to be expanded
    ExpansionRegion* SelectExpansionRegion();

    void CreateConnectionRegion(const EIT& _eit, Vector3d _p, size_t _ccA, size_t ccB);
    void CreateExpansionRegion(size_t _cc, VIT _vit);
    /// Select a connection region between two connected components
    /// @return expansion region to be expanded
    ExpansionRegion* SelectConnectionRegion(size_t _ccA, size_t _ccB);

    /// ComputeProbabilities
    /// @return probabiliities based on expansion success
    vector<double> ComputeProbabilities();

    void IncrementSuccessFailure(const ExpansionRegion _region, const size_t _success, const size_t _failure);
    vector<Neighbor> FindNearestNeighbors(const CfgType& _cfg, const VertexSet* const _candidates);

    bool AttemptConnection(CfgType _c1, CfgType _c2);

    vector<CfgType> ExpandComponent(ExpansionRegion* _r);

    bool AdvanceRegion(ExpansionRegion* _r);

    bool AreSamplesCovered(const ExpansionRegion* _region, const vector<CfgType> _samples);

    void ConnectComponent(size_t _componentID);
    ///@}
    ///@name Internal State
    ///@{

    /// Sampler labels with number and attempts of sampler.
    std::vector<SamplerSetting> m_samplers;
    /// Connector labels for node-to-node.
    std::vector<std::string> m_connectorLabels;

    std::string m_inputMapFilename; ///< Input roadmap to initialize map

    ///@}
    ///@name Fix-base hacks
    ///@{
    /// To be removed when we properly implement path constraints or perhaps a
    /// sampler-method option.

    bool m_fixBase{false};  ///< Keep the base fixed to the start cfg?
    /// An optional sampling boundary for fixing the base.
    std::unique_ptr<Boundary> m_samplingBoundary;

    RegionKit m_regionKit;            ///< Manages regions following the skeleton.
    std::string m_decompositionLabel; ///< The workspace decomposition label.
    std::string m_scuLabel;           ///< The skeleton clearance utility label.
    double edgeClearance{0.};     ///< The edge clearance for that region.
    string m_clearanceValue;      ///< Indicates whether to bias by min or max clearance

    PropertyMap<vector<double>, double>* m_skeletonClearanceMap{nullptr};
    bool m_setClearanceAnnotation{false};
    string m_annotationFile;

    bool m_initialized{false};

    std::string m_skeletonType{"reeb"};

    typedef std::unordered_set<mathtool::Vector3d> CoveredMap;
    typedef std::unordered_set<ExpansionRegion> ExpansionRegionMap;

    std::unordered_map<size_t, CoveredMap> m_skeletonCoverageMap;
    std::unordered_map<size_t, ExpansionRegionMap > m_ccRegionMapp;

    std::unordered_map<size_t, CoveredMap> m_skeletonEdgeMap;

    vector<ExpansionRegion> m_expansionRegions;

    vector<ConnectionRegion> m_connectionRegions;

    /// Weight of explore vs. exploit in region selection probabilities.
    /// Exploring is a uniform chance to select each region, while exploit
    /// favors successful regions.
    double m_explore{.5};
    ///@}
    std::string m_nfLabel;       ///< The neighborhood finder label.
    std::string m_lpLabel;       ///< The neighborhood finder label.
    WorkspaceSkeleton m_skeleton;     ///< The workspace skeleton.


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

  m_inputMapFilename = _node.Read("inputMap", false, "",
      "filename of roadmap to start from");

  m_nfLabel = _node.Read("nfLabel", true, "", "Neighborhood Finder");

  m_lpLabel = _node.Read("lpLabel", true, "", "Local Planner");

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

  // Temporary hack for fixing the base.
  m_fixBase = _node.Read("fixBase", false, m_fixBase,
      "Fix the robot's base position and orientation to the start cfg?");
  m_setClearanceAnnotation = _node.Read("clearanceAnnotation", false, false, "annotate skeleton? with clearance");
  if(m_setClearanceAnnotation)
    m_clearanceValue = _node.Read("clearanceValue", true, "", "indicates if "
        "exploration should be biased by min or max clearance value, option: "
        "min, max" );
  m_decompositionLabel = _node.Read("decompositionLabel", true, "",
      "The workspace decomposition to use.");

  m_scuLabel = _node.Read("scuLabel", false, "", "The skeleton clearance utility "
      "to use. If not specified, we use the hack-fix from wafr16.");
  m_skeletonType = _node.Read("skeletonType", false, "reeb", "the type of skeleton to use, Available options are reeb and mcs");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
Print(std::ostream& _os) const {
  MPStrategyMethod<MPTraits>::Print(_os);
  _os << "\tInput Map: " << m_inputMapFilename << std::endl;

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
  m_initialized = false;
}

//TODO change Run() back to iterate after testing
template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
Iterate() {
  if(!m_initialized)
    LazyInitialize();

  auto region = SelectExpansionRegion();
  auto samples = this->ExpandComponent(region);
  if(region != nullptr) {
    if(!samples.empty()) {
      while (AdvanceRegion(region) && AreSamplesCovered(region, samples))
        continue;
      //ConnectComponent(region.ComponentID);
    }
    else {
      //get random component
      //connectcomponent(random component region)
    }
  }
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
vector<Neighbor>
DynamicRegionsPRM<MPTraits>::
FindNearestNeighbors(const CfgType& _cfg, const VertexSet* const _candidates) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, "DynamicRegionsPRM::FindNearestNeighbors");

  if(this->m_debug)
    std::cout << "Searching for nearest neighbors to " << _cfg.PrettyPrint()
      << " with '" << m_nfLabel << "' from "
      << (_candidates
          ? "a set of size " + std::to_string(_candidates->size())
          : "the full roadmap")
      << "."
      << std::endl;

  // Search for the nearest neighbors according to the NF.
  std::vector<Neighbor> neighbors;
  auto g = this->GetRoadmap();
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  if(_candidates)
    nf->FindNeighbors(g, _cfg, *_candidates, neighbors);
  else
    nf->FindNeighbors(g, _cfg, std::back_inserter(neighbors));

  // Check for no neighbors. We really don't want this to happen - if you see
  // high numbers for this, you likely have problems with parameter or algorithm
  // selection.
  if(neighbors.empty()) {
    stats->IncStat("BasicRRTStrategy::FailedNF");
    if(this->m_debug)
      std::cout << "\tFailed to find a nearest neighbor."
        << std::endl;
  }
    return neighbors;
}


template <typename MPTraits>
bool
DynamicRegionsPRM<MPTraits>::
AttemptConnection(CfgType _c1, CfgType _c2) {
  auto lp = this->GetLocalPlanner(m_lpLabel);
  Environment* env = this->GetEnvironment();
  LPOutput<MPTraits> lpOutput;
  if(lp->IsConnected(_c1, _c2, &lpOutput,
    env->GetPositionRes(), env->GetOrientationRes(), true, false))
    return true;

  return false;
}


template <typename MPTraits>
bool
DynamicRegionsPRM<MPTraits>::
AreSamplesCovered(const ExpansionRegion* _region, const vector<CfgType> _samples) {
  auto boundary = MakeBoundary(_region->GetCenter(), _region->GetSize());
  for(auto s: _samples) {
    if(s.InBounds(boundary))
      return true;
  }
  return false;
}


template <typename MPTraits>
bool
DynamicRegionsPRM<MPTraits>::
AdvanceRegion(ExpansionRegion* _r) {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::AdvanceRegion()" << endl;

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
         << vit->descriptor() << endl;
    return false;
  }
  //check for usable clearance
  double clearance = GetRegionRadius(*nextEdgePoint);
  if(clearance < 0) {
    m_expansionRegions.erase(remove(m_expansionRegions.begin(), m_expansionRegions.end(), *_r), m_expansionRegions.end());
    return false;
  }
  //move to the next position
  _r->SetCenter(*nextEdgePoint);
  _r->SetSize(clearance);
  if(this->m_debug)
    cout << "region advanced to edge point " << *nextEdgePoint
         << " with size " << clearance << endl;
  return true;
}


template <typename MPTraits>
vector<typename DynamicRegionsPRM<MPTraits>::CfgType>
DynamicRegionsPRM<MPTraits>::
ExpandComponent(ExpansionRegion* _r) {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::ExpandComponent()" << endl;

  auto boundary = _r == nullptr ? this->GetEnvironment()->GetBoundary() : MakeBoundary(_r->GetCenter(), _r->GetSize());
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
    //attempt connection between s and nearest neighbors in CC
    auto knn = FindNearestNeighbors(s, &(nodes));
    bool isConnected = false;
    VID vid1;
    for(auto n : knn) {
      isConnected = AttemptConnection(s, g->GetVertex(n.target));
      if(isConnected) {
        vid1 = n.target;
        continue;
      }
    }
    if(!isConnected) {
      samples.erase(remove(samples.begin(), samples.end(),s), samples.end());
      if(this->m_debug)
        cout << "sample could not connect. Remaining to test: " << samples.size() << endl;
      continue;
    }
    auto vid2 = g->AddVertex(s);
    LPOutput<MPTraits> lp;
    g->AddEdge(vid1, vid2, lp.m_edge);
    if(this->m_debug)
      cout << "sample " << vid1 << " and " << vid2
           << " connected successfully. Remaining to test: " << samples.size() << endl;
  }
  if(_r != nullptr)
    _r->UpdateSuccessRate(samples.size(), attempts);

  return samples;
}


template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
ConnectComponent(size_t _componentID) {
  auto g = this->GetRoadmap();
  auto ccTracker = g->GetCCTracker();
  auto representatives = ccTracker->GetRepresentatives();
  for(auto & rep : representatives) {
    if(ccTracker->InSameCC(_componentID, rep))
      continue;

  }
}


template <typename MPTraits>
std::vector<typename DynamicRegionsPRM<MPTraits>::CfgType>
DynamicRegionsPRM<MPTraits>::
Sample(const Boundary* _b, size_t& _attemptCount) {
  if(this->m_debug)
    std::cout << "Sampling new nodes..." << std::endl;

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
void
DynamicRegionsPRM<MPTraits>::
Connect(const std::vector<VID>& _vids) {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::Connect");

  if(this->m_debug)
    std::cout << "Connecting..." << std::endl;

  auto r = this->GetRoadmap();
  for(const auto& label : m_connectorLabels) {
    if(this->m_debug)
      std::cout << "\tUsing connector '" << label << "'." << std::endl;

    this->GetConnector(label)->Connect(r, _vids.begin(), _vids.end());
  }

  if(this->m_debug)
    std::cout << "\tGraph has "
      << r->get_num_edges() << " edges and "
      << r->GetCCTracker()->GetNumCCs() << " connected components."
      << std::endl;
}


template<typename MPTraits>
Boundary*
DynamicRegionsPRM<MPTraits>::
MakeBoundary(Vector3d _v, double _r) {
  vector<double> v{_v[0], _v[1], _v[2]};
  return new CSpaceBoundingSphere(v, _r);
}


template<typename MPTraits>
double
DynamicRegionsPRM<MPTraits>::
GetRegionRadius(Vector3d _v) {
  auto clearance = this->GetClearance(_v);
  double robotRadius = this->GetTask()->GetRobot()->GetMultiBody()->GetBoundingSphereRadius();
  //return min((robotRadius * 3), (clearance - robotRadius));
  return (clearance - robotRadius);
}


template<typename MPTraits>
double
DynamicRegionsPRM<MPTraits>::
GetClearance(Vector3d _v) {
  //place the base of the robot at v's position
  auto pointRobot = this->GetMPProblem()->GetRobot("point");
  auto vc = this->GetValidityChecker("pqp_solid");
  string callee = this->GetNameAndLabel() + "::GetClearance";

  CfgType cfg(_v, pointRobot);
  CDInfo cd(true);
  vc->IsValid(cfg, cd, callee);
  return cd.m_minDist;
}


template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
CreateExpansionRegion(size_t _cc, const VIT _vit) {
  for(auto eit = _vit->begin(); eit != _vit->end(); ++eit) {
    const std::vector<Point3d>& path = eit->property();
    if(IsCovered(path[0], _cc))
      continue;

    double regionSize = GetRegionRadius(path[0]);
    ExpansionRegion er(eit);
    er.componentID = _cc;
    er.SetCenter(path[0]);
    er.SetSize(regionSize);
    m_ccRegionMapp[_cc].insert(er);
    m_expansionRegions.push_back(er);
  }
  m_skeletonCoverageMap[_cc].insert(_vit->property());
}


template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
CreateConnectionRegion(const EIT& _eit, Vector3d _p, size_t _ccA, size_t _ccB) {
  ConnectionRegion cr (_eit);
  cr.componentIDs = make_pair(_ccA, _ccB);
  cr.SetCenter(_p);
  m_connectionRegions.push_back(cr);

  if(this->m_debug) {
    std::cout << "New connection region added between CC " << _ccA
              << " and " << _ccB <<". Total number of connection regions: "
              << m_connectionRegions.size() << endl;
  }
}


template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
LazyInitialize() {
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
      er.UpdateSuccessRate(1, attempts);
      m_ccRegionMapp[ccID].insert(er);
      m_expansionRegions.push_back(er);
    }
    m_skeletonCoverageMap[ccID].insert(iter->property());

  }
  if(this->m_debug)
    std::cout << "Done initializing " << m_ccRegionMapp.size() << " expansion regions." << endl;
}


template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
BuildSkeleton() {
  if(m_initialized)
    return;
  m_initialized = true;

  MethodTimer mt(this->GetStatClass(), "DynamicRegionsPRM::BuildSkeleton");

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


template<typename MPTraits>
bool
DynamicRegionsPRM <MPTraits>::
IsCovered(Vector3d _point, size_t _componentID) {
  //if the component covers the point, there is/was a region centered at the
  //point recordered in the coverage map of the CC

  auto regions = m_skeletonCoverageMap[_componentID];

  if(regions.empty())
    return false;

  for(auto& r : regions) {
    if(r == _point)
      return true;
  }
  return false;
}


template<typename MPTraits>
ExpansionRegion*
DynamicRegionsPRM <MPTraits>::
SelectExpansionRegion() {
  // Update all region probabilities.
  std::vector<double> probabilities = ComputeProbabilities();

  // Construct with random number generator with the region probabilities.
  static std::default_random_engine generator(0);
  std::discrete_distribution<size_t> distribution(probabilities.begin(),
      probabilities.end());

  const size_t index = distribution(generator);
  const bool envSelected = index == m_ccRegionMapp.size();

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
              << Vector3d(c[0], c[1], c[2]) << ", success rate so far "
              << it->successes << " / " << it->attempts
              << "." << std::endl;
  }

  // total samples increment
  ++it->attempts;

  return &(*it);
}


template<typename MPTraits>
std::vector<double>
DynamicRegionsPRM <MPTraits>::
ComputeProbabilities() {
  // Sum all weights of all current regions.
  double totalWeight = 0.;
  for(auto& cc : m_ccRegionMapp) {
    auto& regions = cc.second;
    for(auto& regionInfo : regions) {

      // Compute weight of this region.
      regionInfo.weight = regionInfo.successes / static_cast<double>(regionInfo.attempts);

      // Add to total.
      totalWeight += regionInfo.weight;
    }
  }

  // Compute the probabilities for the current regions.
  std::vector<double> probabilities;
  probabilities.reserve(m_expansionRegions.size() + 1);

  const double explore = m_explore / (m_expansionRegions.size() + 1);

  for(auto& cc : m_ccRegionMapp) {
    auto regions = cc.second;
    for(auto& regionInfo : regions) {
      const auto& weight = regionInfo.weight;
      const double exploit = (1 - m_explore) * weight / totalWeight;

      probabilities.emplace_back(exploit + explore);
    }
  }

  // Get the probability for the whole environment.
  probabilities.emplace_back(explore);

  return probabilities;
}


template<typename MPTraits>
void
DynamicRegionsPRM <MPTraits>::
IncrementSuccessFailure(const ExpansionRegion _region, const size_t _success, const size_t _failure) {
  // Ensure that this region exists.
  auto iter = std::find(m_expansionRegions.begin(), m_expansionRegions.end(), _region);

  iter->successes += _success;
  iter->attempts  += (_success + _failure);
}


/*----------------------------------------------------------------------------*/

#endif
