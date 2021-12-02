#ifndef PPL_TERRAIN_SAMPLER_H_
#define PPL_TERRAIN_SAMPLER_H_

#include "SamplerMethod.h"

#include "Geometry/Boundaries/Range.h"

template <typename MPTraits>
class TerrainSampler : public SamplerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename SamplerMethod<MPTraits>::BoundaryMap;

    typedef typename std::vector<CfgType>::iterator InputIterator;
    typedef typename std::back_insert_iterator<std::vector<CfgType>> OutputIterator;
    typedef typename std::vector<GroupCfgType>::iterator GroupInputIterator;
    typedef typename std::back_insert_iterator<std::vector<GroupCfgType>> GroupOutputIterator;

    ///@}
    ///@name Construction
    ///@{

    TerrainSampler();

    TerrainSampler(XMLNode& _node);

    virtual ~TerrainSampler();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}

  protected:

    ///@name Individual Configuration Sampling
    ///@{

    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, OutputIterator _valid,
        OutputIterator _invalid);

    ///@}
    ///@name Group Configuration Sampling
    ///@{

    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, GroupOutputIterator _valid,
        GroupOutputIterator _invalid);

    ///@}
    ///@name Sampler Rule
    ///@{

    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        std::vector<CfgType>& _valid, std::vector<CfgType>& _invalid)
        override;

    virtual bool Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
        std::vector<GroupCfgType>& _valid, std::vector<GroupCfgType>& _invalid);

    virtual bool Sampler(GroupCfgType& _cfg, const BoundaryMap& _boundaryMap,
        std::vector<GroupCfgType>& _valid, std::vector<GroupCfgType>& _invalid);

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::string m_vcLabel;  ///< The validity checker to use.

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
TerrainSampler<MPTraits>::
TerrainSampler() : SamplerMethod<MPTraits>() {
  this->SetName("TerrainSampler");
}

template <typename MPTraits>
TerrainSampler<MPTraits>::
TerrainSampler(XMLNode& _node) : SamplerMethod<MPTraits>(_node) {
  this->SetName("TerrainSampler");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
}

template <typename MPTraits>
TerrainSampler<MPTraits>::
~TerrainSampler() = default;

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
TerrainSampler<MPTraits>::
Print(std::ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\tvcLabel = " << m_vcLabel
      << std::endl;
}

/*-------------------- Individual Configuration Sampling ---------------------*/

template <typename MPTraits>
void
TerrainSampler<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, OutputIterator _valid,
        OutputIterator _invalid) {
  // auto stats = this->GetStatClass();
  // MethodTimer mt(stats, this->GetNameAndLabel() + "::Sample");
  
  // Get environment
  auto envTerrains = this->GetEnvironment()->GetTerrains();

  auto robot = this->GetTask()->GetRobot();

  auto terrains = envTerrains.at(robot->GetCapability());

  // Sample terrain from set of terrains
  // Range<size_t> terrainRange = Range<size_t>(0, terrains.size());
  // auto terrainIdx = terrainRange.Sample();



  for(size_t i = 0; i < terrains.size(); ++i) {
    // Get terrain from set of terrains
    auto terrain = terrains.at(i).GetBoundaries()[0].get();
    SamplerMethod<MPTraits>::Sample(_numNodes * terrains.size(), _maxAttempts, terrain,
                                    _valid, _invalid);
  }

  // CfgType cfg(robot);
  // std::vector<CfgType> valid, invalid;
  // valid.reserve(_numNodes);

  // // Try to generate _numNodes samples, using up to _maxAttempts tries per
  // // sample.
  // for(size_t i = 0; i < _numNodes; ++i) {
  //   for(size_t attempts = 0; attempts < _maxAttempts; ++attempts) {
  //     cfg.GetRandomCfg(terrain.GetBoundaries()[0].get());
  //     if(this->Sampler(cfg, terrain.GetBoundaries()[0].get(), valid, invalid))
  //       break;
  //   }
  // }

  // stats->IncNodesGenerated(this->GetNameAndLabel(), valid.size());
  // stats->IncNodesAttempted(this->GetNameAndLabel(),
  //     valid.size() + invalid.size());

  // std::copy(valid.begin(), valid.end(), _valid);
  // std::copy(invalid.begin(), invalid.end(), _invalid);
}

/*-------------------- Group Configuration Sampling ---------------------*/

template <typename MPTraits>
void
TerrainSampler<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts,
    const Boundary* const _boundary,
    GroupOutputIterator _valid, GroupOutputIterator _invalid) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::Sample");

  GroupCfgType gcfg(this->GetGroupRoadmap());
  std::vector<GroupCfgType> valids, invalids;

  auto savedTask = this->GetGroupTask();

  // TODO::Separate the group into formations and remaining robots
  auto formations = this->GetGroupRoadmap()->GetActiveFormations();
  std::vector<Robot*> remainingRobots;
  
  for(auto robot : this->GetGroupRoadmap()->GetGroup()->GetRobots()) {
    bool inFormation = false;
    for(auto formation : formations) {
      for(auto r : formation->GetRobots()) {
        if(robot == r) {
          inFormation = true;
          break;
        }
      }
      if(inFormation)
        break;
    }
    if(!inFormation)
      remainingRobots.push_back(robot);
  }

  // Try to generate _numNodes samples, using up to _maxAttempts tries per
  // sample.
  for(size_t i = 0; i < _numNodes; i++) {
    for(size_t j = 0; j < _maxAttempts; j++) {
      bool valid = true;
      bool found = true;

      // Sample for formations 
      for(auto formation : formations) {
        // Sample leader in terrain
        auto leader = formation->GetLeader();

        MPTask dummyTask(leader);
        this->GetMPLibrary()->SetTask(&dummyTask);

        std::vector<CfgType> validSamples;
        std::vector<CfgType> invalidSamples;
        Sample(1,1,_boundary,
            std::back_inserter(validSamples),
            std::back_inserter(invalidSamples));

        Cfg cfg(leader);

        if(!validSamples.empty()) {
          cfg = validSamples[0];
        }
        else if(!invalidSamples.empty()) {
          cfg = invalidSamples[0];
          valid = false;
        }
        else {
          found = false;
          break;
        }

        // Convert cfg to CSpace boundary
        CSpaceBoundingBox bbx(cfg.DOF());
        bbx.ShrinkToPoint(cfg);

        // Sample rest of formation
        auto cfgs = formation->GetRandomFormationCfg(&bbx);
        if(cfgs.empty()) {
          found = false;
          break;
        }

        // Check validity
        for(auto& c : cfgs) {
          validSamples.clear();
          invalidSamples.clear();
          Sampler(c,_boundary,validSamples,invalidSamples);
          
          if(!validSamples.empty()) {
            gcfg.SetRobotCfg(c.GetRobot(),std::move(validSamples[0]));
            continue;
          }

          if(!invalidSamples.empty()) {
            gcfg.SetRobotCfg(c.GetRobot(),std::move(invalidSamples[0]));
            valid = false;
            continue;
          }
        }
      }

      // Sample for remaining robots        
      for(auto robot : remainingRobots) {

        MPTask dummyTask(robot);
        this->GetMPLibrary()->SetTask(&dummyTask);

        std::vector<CfgType> validSamples;
        std::vector<CfgType> invalidSamples;
        Sample(1,1,_boundary,
            std::back_inserter(validSamples),
            std::back_inserter(invalidSamples));

        if(!validSamples.empty()) {
          gcfg.SetRobotCfg(robot,std::move(validSamples[0]));
          continue;
        }

        if(!invalidSamples.empty()) {
          gcfg.SetRobotCfg(robot,std::move(invalidSamples[0]));
          valid = false;
          continue;
        }

        // Otherwise, no sample was found
        found = false;
        break;
      }
  
      // If a sample was found, save it
      if(!found)
        continue;

      if(valid)
        valids.push_back(gcfg);
      else 
        invalids.push_back(gcfg);
      break;
    }
  }

  this->GetMPLibrary()->SetTask(nullptr);
  this->GetMPLibrary()->SetGroupTask(savedTask);

  stats->IncNodesGenerated(this->GetNameAndLabel(), valids.size());
  stats->IncNodesAttempted(this->GetNameAndLabel(),
      valids.size() + invalids.size());

  std::copy(valids.begin(), valids.end(), _valid);
  std::copy(invalids.begin(), invalids.end(), _invalid);
}

/*------------------------------ Sampler Rule --------------------------------*/

template <typename MPTraits>
bool
TerrainSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    std::vector<CfgType>& _valid, std::vector<CfgType>& _invalid) {
  // Check Validity.
  auto vc = this->GetValidityChecker(m_vcLabel);
  const std::string callee = this->GetNameAndLabel() + "::Sampler";
  const bool isValid = vc->IsValid(_cfg, callee);

  // Store result.
  if(isValid)
    _valid.push_back(_cfg);
  else
    _invalid.push_back(_cfg);

  // Debug.
  if(this->m_debug) {
    std::cout << "Sampled Cfg: " << _cfg.PrettyPrint()
              << "\n\tBoundary: " << *_boundary
              << "\n\tValidity:  " << isValid
              << std::endl;

    VDClearAll();
    VDAddTempCfg(_cfg, isValid);
    VDComment("TerrainSampler::Cfg " + std::string(isValid ? "" : "in") +
        "valid");
  }

  return isValid;
}

template <typename MPTraits>
bool 
TerrainSampler<MPTraits>::
Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
        std::vector<GroupCfgType>& _valid, std::vector<GroupCfgType>& _invalid) {
  BoundaryMap emptyMap;
  return Sampler(_cfg, emptyMap, _valid, _invalid);
}

template <typename MPTraits>
bool
TerrainSampler<MPTraits>::
Sampler(GroupCfgType& _cfg, const BoundaryMap& _boundaryMap,
        std::vector<GroupCfgType>& _valid, std::vector<GroupCfgType>& _invalid) {

  // Check Validity.
  auto vc = this->GetValidityChecker(m_vcLabel);
  const std::string callee = this->GetNameAndLabel() + "::Sampler";
  const bool isValid = vc->IsValid(_cfg, callee);

  // Store result.
  if(isValid)
    _valid.push_back(_cfg);
  else
    _invalid.push_back(_cfg);

  // Debug.
  if(this->m_debug) {
    std::cout << "Sampled Cfg: " << _cfg.PrettyPrint()
              << "\n\tValidity:  " << isValid
              << std::endl;
  }

  return isValid;
}

/*----------------------------------------------------------------------------*/

#endif
