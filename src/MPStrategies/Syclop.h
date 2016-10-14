#ifndef SYCLOP_H_
#define SYCLOP_H_

#include <iomanip>
#include "MPStrategyMethod.h"
#include "MapEvaluators/RRTQuery.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief   This method is the 'Synergistic Combination of Layers of Planning'
///          technique that adds workspace guidance to RRT methods.
///
/// Paper reference:
///
/// Erion Plaku, Lydia E. Kavraki, Moshe Y. Vardi. "Synergistic Combination of
///   Layers of Planning". IEEE Transactions on Robotics. 2010.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class Syclop : public BasicRRTStrategy<MPTraits> {

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::CfgRef           CfgRef;
    typedef typename MPTraits::WeightType       WeightType;
    typedef typename MPTraits::MPProblemType    MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType   GraphType;
    typedef typename MPProblemType::VID         VID;

    ///@}
    ///\name Local Types
    ///@{

    typedef vector<VID>                         TreeType;
    typedef typename vector<TreeType>::iterator TreeIter;

    ///@}
    ///\name Construction
    ///@{

    Syclop();

    Syclop(MPProblemType* _problem, XMLNode& _node);

    virtual ~Syclop() = default;

    ///@}
    ///\name MPStrategy Overrides
    ///@{

    virtual void Initialize() override;
    virtual void Iterate() override;

    ///@}

  protected:

    ///\name Neighbor Helpers
    ///@{

    VID FindNearestNeighbor(const CfgType& _cfg, const TreeIter& _tree);

    ///@}
    ///\name Growth Helpers
    ///@{

    virtual VID Extend(const VID _nearVID, const CfgType& _qRand,
        const bool _lp = false);

    ///@}
    ///\name Tree Helpers
    ///@{

    virtual VID ExpandTree(const VID _nearestVID, const CfgType& _target);

    ///@}
    ///@name Syclop Functions
    ///@{

    typedef int* RegionPointer;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute a high-level plan (a sequence of regions).
    vector<RegionPointer> DiscreteLead();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute a set of potential regions from the discrete lead.
    vector<RegionPointer> AvailableRegions(vector<RegionPointer> _lead);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Select a region from a set of available regions.
    RegionPointer SelectRegion(vector<RegionPointer> _available);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Select a vertex from within a given region.
    VID SelectVertex(RegionPointer _r);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Find the workspace region that holds a given configuration.
    RegionPointer LocateRegion(const VID _v);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Update the coverage of the region containing a new vertex.
    void UpdateCoverage(const VID _v);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Update the connectivity information after connecting a vertex v1
    ///        in a region r1 to a vertex v2 in a different region r2.
    void UpdateConnections(const VID _v1, const VID _v2);

    ///@}
    ///@name Syclop Helpers
    ///@}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Find the number of occupied sub-cells in a given region.
    size_t Cov(RegionPointer _r);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Estimate the free state space volume of a given region.
    double FreeVol(RegionPointer _r);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Count the number of times that _r1 and _r2 have been selected as
    ///        part of a discrete lead.
    size_t Sel(RegionPointer _r1, RegionPointer _r2);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Estimate the progress made in connecting _r1 to _r2.
    void Conn(RegionPointer _r1, RegionPointer _r2);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute the edge weight in the region graph from _r1 to _r2.
    double Cost(RegionPointer _r1, RegionPointer _r2);

    ///@}
    ///@name Syclop State
    ///@{

    // Some kind of coverage map/structure

    // Some kind of connectivity map/structure

    ///@}
};

/*----------------------------- construction ---------------------------------*/

template<class MPTraits>
Syclop<MPTraits>::
Syclop() : BasicRRTStrategy<MPTraits>() {
  this->SetName("Syclop");
}


template<class MPTraits>
Syclop<MPTraits>::
Syclop(MPProblemType* _problem, XMLNode& _node) :
    BasicRRTStrategy<MPTraits>(_problem, _node) {
  this->SetName("Syclop");
}

/*-------------------------- MPStrategy overrides ----------------------------*/

template<class MPTraits>
void
Syclop<MPTraits>::
Initialize() {
  BasicRRTStrategy<MPTraits>::Initialize();
}


template<class MPTraits>
void
Syclop<MPTraits>::
Iterate() {
  BasicRRTStrategy<MPTraits>::Iterate();
}

/*---------------------------- Neighbor Helpers ------------------------------*/

template<class MPTraits>
typename Syclop<MPTraits>::VID
Syclop<MPTraits>::
FindNearestNeighbor(const CfgType& _cfg, const TreeIter& _tree) {
  this->GetStatClass()->StartClock("NeighborhoodFinding");

  vector<pair<VID, double>> neighbors;
  auto nf = this->GetNeighborhoodFinder(this->m_nfLabel);
  nf->FindNeighbors(this->GetRoadmap(),
      _tree->begin(), _tree->end(),
      _tree->size() == this->GetRoadmap()->GetGraph()->get_num_vertices(),
      _cfg, back_inserter(neighbors));
  VID nearestVID = neighbors[0].first;

  this->GetStatClass()->StopClock("NeighborhoodFinding");
  return nearestVID;
}

/*----------------------------- Growth Helpers -------------------------------*/

template<class MPTraits>
typename Syclop<MPTraits>::VID
Syclop<MPTraits>::
Extend(const VID _nearVID, const CfgType& _qRand, const bool _lp) {
  return BasicRRTStrategy<MPTraits>::Extend(_nearVID, _qRand, _lp);
}

/*------------------------------ Tree Helpers --------------------------------*/

template<class MPTraits>
typename Syclop<MPTraits>::VID
Syclop<MPTraits>::
ExpandTree(const VID _nearestVID, const CfgType& _target) {
  VID newVID = BasicRRTStrategy<MPTraits>::ExpandTree(_nearestVID, _target);
  return newVID;
}

/*------------------------------ Syclop Functions ----------------------------*/

template <typename MPTraits>
vector<typename Syclop<MPTraits>::RegionPointer>
Syclop<MPTraits>::
DiscreteLead() {
  return vector<RegionPointer>();
}


template <typename MPTraits>
vector<typename Syclop<MPTraits>::RegionPointer>
Syclop<MPTraits>::
AvailableRegions(vector<RegionPointer> _lead) {
  return vector<RegionPointer>();
}


template <typename MPTraits>
typename Syclop<MPTraits>::RegionPointer
Syclop<MPTraits>::
SelectRegion(vector<RegionPointer> _available) {
  return nullptr;
}


template <typename MPTraits>
typename Syclop<MPTraits>::VID
Syclop<MPTraits>::
SelectVertex(RegionPointer _r) {
  return INVALID_VID;
}


template <typename MPTraits>
typename Syclop<MPTraits>::RegionPointer
Syclop<MPTraits>::
LocateRegion(const VID _v) {
  return nullptr;
}


template <typename MPTraits>
void
Syclop<MPTraits>::
UpdateCoverage(const VID _v) {
}


template <typename MPTraits>
void
Syclop<MPTraits>::
UpdateConnections(const VID _v1, const VID _v2) {
}

/*------------------------------ Syclop Helpers ------------------------------*/

template <typename MPTraits>
size_t
Syclop<MPTraits>::
Cov(RegionPointer _r) {
  // Initially, just approximate the number of cells as number of vertices in
  // the region.
  return 0;
}


template <typename MPTraits>
double
Syclop<MPTraits>::
FreeVol(RegionPointer _r) {
  return 0;
}


template <typename MPTraits>
size_t
Syclop<MPTraits>::
Sel(RegionPointer _r1, RegionPointer _r2) {
  return 0;
}


template <typename MPTraits>
void
Syclop<MPTraits>::
Conn(RegionPointer _r1, RegionPointer _r2) {
}


template <typename MPTraits>
double
Syclop<MPTraits>::
Cost(RegionPointer _r1, RegionPointer _r2) {
  return 0;
}

/*----------------------------------------------------------------------------*/

#endif
