#ifndef PMPL_UTILITY_GUIDED_GENERATOR_H_
#define PMPL_UTILITY_GUIDED_GENERATOR_H_

#include "MPStrategyMethod.h"

#include <algorithm>


////////////////////////////////////////////////////////////////////////////////
/// An approximate model of c-space used by the utility guided sampling
/// strategy.
///
/// @todo This implementation is very bad. It takes O(n lg n) time for the
///       nearest-neighbor problem on each iteration. If we wish to use this as
///       a comparison method, we should first improve this implementation to a
///       more reasonable performance.
/// @ingroup MotionPlanningStrategyUtils
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ApproximateCSpaceModel {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType                CfgType;
    typedef typename MPTraits::MPLibrary              MPLibrary;
    typedef typename MPLibrary::DistanceMetricPointer DistanceMetricPointer;

    ///@}
    ///@name Local Types
    ///@{

    typedef typename std::pair<CfgType, double> ModelPair;

    ///@}
    ///@name Construction
    ///@{

    ApproximateCSpaceModel(DistanceMetricPointer _dm = nullptr);

    ///@}
    ///@name Model Interface
    ///@{

    /// Add a sample to the model.
    /// @param _c The sample to add.
    /// @param _valid The sample's validity.
    void AddSample(const CfgType& _c, const bool _valid);

    /// Estimate the probability that a sample is free based on the k-nearest
    /// neighbors according to the distance metric.
    /// @param _c The query sample.
    /// @param _k The k to use.
    /// @return Estimated probability that the sample is free, equal to the
    ///         average validity of the k-nearest samples.
    double FreeProbability(const CfgType& _c, const size_t _k);

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::vector<ModelPair> m_modelNodes;   ///< The nodes in the model.
    DistanceMetricPointer m_dm;            ///< The distance metric.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
ApproximateCSpaceModel<MPTraits>::
ApproximateCSpaceModel(DistanceMetricPointer _dm) : m_dm(_dm) {}

/*----------------------------------------------------------------------------*/

template <typename MPTraits>
void
ApproximateCSpaceModel<MPTraits>::
AddSample(const CfgType& _c, const bool _valid) {
  m_modelNodes.emplace_back(_c, _valid);
}


template <typename MPTraits>
double
ApproximateCSpaceModel<MPTraits>::
FreeProbability(const CfgType& _c, const size_t _k) {
  // Determine how many nodes will be used to estimate the probability that _c
  // is free.
  const size_t size = std::min(_k, m_modelNodes.size());
  if(size == 0)
    return 0.;

  // Sort the model nodes on distance to _c.
  /// @todo This should be replaced with something better, at minimum a linear
  ///       scan and priority-queue for the (size) best nodes for O(n lg size)
  ///       time.
  auto comparator = [&_c, this](const ModelPair& _p1, const ModelPair& _p2)
  {
    return this->m_dm->Distance(_c, _p1.first)
         < this->m_dm->Distance(_c, _p2.first);
  };
  std::sort(m_modelNodes.begin(), m_modelNodes.end(), comparator);

  // Return the average validity of the first (size) nodes.
  auto adder = [](const double _sum, const ModelPair& _p)
  {
    return _sum + _p.second;
  };
  return std::accumulate(m_modelNodes.begin(), m_modelNodes.begin() + size,
      0., adder) / size;
}

/*----------------------------------------------------------------------------*/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~ UtilitiyGuidedGenerator ~~~~~~~~~~~~~~~~~~~~~~~~~*/

////////////////////////////////////////////////////////////////////////////////
/// Attempts to generate configurations based on a midpoint rule and an
/// approximate model of c-space based on k-nearest neighbors.
///
/// Reference:
///   Brendan Burns and Oliver Brock. "Toward Optimal Configuration Space
///   Sampling". RSS 2005.
///
/// @note The paper seems to assume there will always be several connected
///       components and provides no contingency for when this isn't the case.
///       We use a uniform random sample from the environment bounary in this
///       case.
///
/// @todo This is really a strategy for sampling and needs to be moved to a
///       SamplerMethod class.
///
/// @warning This sampler really sucks when the roadmap is small. Tau needs to
///          be set VERY high to avoid problems where the midpoint between start
///          and goal cfgs/clusters is deep in obstacle space, which mitigates
///          the proposed benefits.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class UtilityGuidedGenerator : public MPStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::VertexSet VertexSet;

    ///@}
    ///@name Construction
    ///@{

    UtilityGuidedGenerator();
    UtilityGuidedGenerator(XMLNode& _node);
    virtual ~UtilityGuidedGenerator() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const;

    ///@}
    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Initialize() override;
    virtual void Iterate() override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    CfgType GenerateSample();
    CfgType UniformRandomSample();
    CfgType EntropyGuidedSample();

    ///@}
    ///@name Internal State
    ///@{

    std::string m_vcLabel;
    std::string m_nfLabel;
    std::string m_connectorLabel;

    double m_componentDist{10};
    double m_tau{5};
    size_t m_kNeighbors{10};
    size_t m_kSamples{5};

    ApproximateCSpaceModel<MPTraits> m_model;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
UtilityGuidedGenerator<MPTraits>::
UtilityGuidedGenerator() {
  this->SetName("UtilityGuidedGenerator");
}


template <typename MPTraits>
UtilityGuidedGenerator<MPTraits>::
UtilityGuidedGenerator(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  this->SetName("UtilityGuidedGenerator");

  m_vcLabel = _node.Read("vcLabel", true, "",
      "Validity Checker for sampling.");
  m_nfLabel = _node.Read("nfLabel", true, "",
      "Neighborhood Finder used in approximate c-space model.");
  m_connectorLabel = _node.Read("connectorLabel", true, "",
      "Connector for local planning.");

  m_componentDist = _node.Read("componentDist", false,
      m_componentDist, 0., std::numeric_limits<double>::max(),
      "Distance threshold between ccs");

  m_tau = _node.Read("tau", false,
      m_tau, 0., std::numeric_limits<double>::max(),
      "perturb amount");

  m_kNeighbors = _node.Read("kneighbors", false,
      m_kNeighbors, size_t(0), std::numeric_limits<size_t>::max(),
      "number of neighbors to look at when determining the probability a "
      "sample is free");

  m_kSamples = _node.Read("ksamples", false,
      m_kSamples, size_t(0), std::numeric_limits<size_t>::max(),
      "number of samples to select from during each round");
}

/*----------------------------------------------------------------------------*/

template <typename MPTraits>
void
UtilityGuidedGenerator<MPTraits>::
Print(std::ostream& _os) const {
  MPStrategyMethod<MPTraits>::Print(_os);
  _os << "\tValidity Checker: " << m_vcLabel
      << "\n\tNeighborhood Finder: " << m_nfLabel
      << "\n\tComponent Distance: " << m_componentDist
      << "\n\tTao: " << m_tau
      << "\n\tKNeighbors: " << m_kNeighbors
      << "\n\tKSamples: " << m_kSamples
      << "\n\tNode Connector: " << m_connectorLabel
      << std::endl;
}

/*----------------------------------------------------------------------------*/

template <typename MPTraits>
void
UtilityGuidedGenerator<MPTraits>::
Initialize() {
  // Initialize the approximate model.
  auto dm = this->GetDistanceMetric(this->GetNeighborhoodFinder(m_nfLabel)->
      GetDMLabel());
  m_model = ApproximateCSpaceModel<MPTraits>(dm);

  // Generate start and goal nodes if possible.
  this->GenerateStart();
  this->GenerateGoals();

  // Ensure the roadmap has at least one free sample.
  auto r = this->GetRoadmap();
  if(r->Size() >= 1)
    return;

  // Try up to 100 times to generate a valid sample.
  for(size_t i = 0; i < 100; ++i) {
    // Generate a sample.
    CfgType q = GenerateSample();

    // Add the sample to the model.
    const bool valid = q.GetLabel("VALID");
    m_model.AddSample(q, valid);

    if(valid) {
      r->AddVertex(q);
      return;
    }
  }

  throw RunTimeException(WHERE) << "Could not generate initial sample.";
}


template <typename MPTraits>
void
UtilityGuidedGenerator<MPTraits>::
Iterate() {
  CfgType q = GenerateSample();

  // Add the sample to the model.
  const bool valid = q.GetLabel("VALID");
  m_model.AddSample(q, valid);

  // If the sample wasn't free, quit.
  if(!valid)
    return;

  // Add q the roadmap and attempt connection.
  auto r = this->GetRoadmap();
  const VID vid = r->AddVertex(q);
  this->GetConnector(m_connectorLabel)->Connect(r, vid);
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
typename MPTraits::CfgType
UtilityGuidedGenerator<MPTraits>::
GenerateSample() {
  // If there are multiple CCs, use entropy-guided. Else use uniform.
  auto ccTracker = this->GetRoadmap()->GetCCTracker();
  const bool multipleCCs = ccTracker->GetNumCCs() > 1;
  CfgType (UtilityGuidedGenerator<MPTraits>::*makeSample)(void);
  makeSample = multipleCCs
             ? &UtilityGuidedGenerator<MPTraits>::EntropyGuidedSample
             : &UtilityGuidedGenerator<MPTraits>::UniformRandomSample;

  // Generate m_kSamples samples and keep the one with the highest estimated
  // probability of lying in free space.
  CfgType best;
  double bestProbability = -1;
  for(size_t i = 0; i < m_kSamples; ++i) {
    const CfgType q = (*this.*makeSample)();
    double probability = m_model.FreeProbability(q, m_kNeighbors);
    if(this->m_debug)
      std::cout << "\tq_" << i << " (" << probability << ") = "
                << q.PrettyPrint()
                << std::endl;

    if(probability > bestProbability) {
      if(this->m_debug)
        std::cout << "\t\tprobability greater, swapping to q_" << i
                  << std::endl;
      best = q;
      bestProbability = probability;
    }
  }

  // Validity check the sample.
  const std::string caller = this->GetNameAndLabel() + "::Iterate";
  const bool valid = this->GetValidityChecker(m_vcLabel)->IsValid(best, caller);
  if(this->m_debug)
    std::cout << "\tGenerated " << (valid ? "" : "in") << "valid sample "
              << best.PrettyPrint()
              << std::endl;

  return best;
}


template <typename MPTraits>
typename MPTraits::CfgType
UtilityGuidedGenerator<MPTraits>::
UniformRandomSample() {
  CfgType q(this->GetTask()->GetRobot());
  q.GetRandomCfg(this->GetEnvironment()->GetBoundary());

  if(this->m_debug)
    std::cout << "\t\tonly 1 cc, sampled new cfg"
              << std::endl;

  return q;
}


template <typename MPTraits>
typename MPTraits::CfgType
UtilityGuidedGenerator<MPTraits>::
EntropyGuidedSample() {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::EntropyGuidedSample");

  auto r = this->GetRoadmap();
  auto ccTracker = r->GetCCTracker();

  // Randomly select 2 ccs that are within a threshold m_componentDist of each
  // other. We will return a perturbation of their midpoint.
  if(this->m_debug)
    std::cout << "\t\tThere are " << ccTracker->GetNumCCs()
              << " ccs, looking for a pair less than " << m_componentDist
              << " apart"
              << std::endl;

  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  std::vector<Neighbor> neighbors;

  // Get a representative from each CC.
  VertexSet firstRepresentatives = ccTracker->GetRepresentatives(),
            allSecondRepresentatives = firstRepresentatives;

  // Try to find a pair of CCs where the closest nodes are within a threshold
  // m_componentDist of each other.
  std::pair<VID, VID> ccVIDs{INVALID_VID, INVALID_VID};
  double ccDistance = std::numeric_limits<double>::infinity();
  while(firstRepresentatives.size() and ccDistance > m_componentDist) {
    // Select a random first VID and remove it from the set of first reps.
    const VID firstVID = RandomElement(firstRepresentatives);
    firstRepresentatives.erase(firstVID);

    // Get the related CC.
    const VertexSet& firstCC = *ccTracker->GetCC(firstVID);

    // Get the representatives from all other CCs.
    allSecondRepresentatives.erase(firstVID);
    VertexSet secondRepresentatives = allSecondRepresentatives;

    // Try all of these.
    while(secondRepresentatives.size() and ccDistance > m_componentDist) {
      // Select a random second VID and remove it from the set of first reps.
      const VID secondVID = RandomElement(secondRepresentatives);
      secondRepresentatives.erase(secondVID);

      // Get the related CC.
      const VertexSet& secondCC = *ccTracker->GetCC(secondVID);

      // Search for the smallest distance between these.
      for(const VID vid : firstCC) {
        // Find neighbors to this vid.
        neighbors.clear();
        nf->FindNeighbors(r, r->GetVertex(vid), secondCC, neighbors);

        if(neighbors.empty())
          continue;

        const double distance = neighbors[0].distance;
        if(distance < ccDistance) {
          ccDistance = distance;
          ccVIDs = {vid, neighbors[0].target};

          // Quit if we're close enough.
          if(ccDistance <= m_componentDist)
            break;
        }
      }
    }
  }

  // Make sure we got valid VIDs.
  if(ccVIDs.first == INVALID_VID or ccVIDs.second == INVALID_VID)
    throw RunTimeException(WHERE) << "Could not find a valid pair of VIDs.";

  // Get the CC's for the selected VIDs.
  const VertexSet& cc1 = *ccTracker->GetCC(ccVIDs.first),
                 & cc2 = *ccTracker->GetCC(ccVIDs.second);

  if(this->m_debug)
    std::cout << "\t\t\tselected ccs " << &cc1 << ", " << &cc2
              << " with closest nodes "
              << ccVIDs.first << ", " << ccVIDs.second
              << " at distance " << ccDistance
              << std::endl;

  const CfgType& q1 = r->GetVertex(RandomElement(cc1));
  const CfgType& q2 = r->GetVertex(RandomElement(cc2));

  // Return perturbation of the midpoint between the two nodes.
  auto dm = this->GetDistanceMetric(nf->GetDMLabel());
  CfgType qn = (q1 + q2)/2;
  CfgType ray = qn;
  ray.GetRandomRay(DRand() * m_tau, dm);
  return qn + ray;
}

/*----------------------------------------------------------------------------*/

#endif
