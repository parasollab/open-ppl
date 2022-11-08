#ifndef PPL_BRIDGE_TEST_SAMPLER_H_
#define PPL_BRIDGE_TEST_SAMPLER_H_

#include "SamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief This sampler validity checks the input sample and accepts it iff it 
/// passes the bridge test - i.e., a random, Gaussian ray places the sample
/// in the center of a ray and it only passes if the sample is valid but the 
/// ray's two endpoints are invalid. 
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class BridgeTestSampler : virtual public SamplerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;
    typedef std::map<Robot*, const Boundary*> BoundaryMap;
  
    ///@}
    ///@name Construction
    ///@{

    BridgeTestSampler(string _vcLabel = "", string _dmLabel = "",
        double _d = 0.5, bool _useBoundary = false);

    BridgeTestSampler(XMLNode& _node);

    virtual ~BridgeTestSampler() = default;

    ///@}
    ///@name MPBaseObjectOverrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name Parser
    ///@{

    void ParseXML(XMLNode& _node);

    ///@}
    ///@name Sampler Rule
    ///@{

    virtual bool Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
        vector<GroupCfgType>& _valid, vector<GroupCfgType>& _invalid);

    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        vector<CfgType>& _valid, vector<CfgType>& _invalid);

    virtual bool Sampler(GroupCfgType& _cfg, const BoundaryMap& _boundaryMap,
        std::vector<GroupCfgType>& _valid, std::vector<GroupCfgType>& _invalid);

    ///@}

  protected:

    ///@name Internal State
    ///@{ 

    double m_d;  ///< Gaussian d-value obtained from distribution
    bool m_useBoundary;  ///< use Bbox as obstacle?
    std::string m_vcLabel, m_dmLabel;  ///< The distance metric to use

    ///@}
    ///@name Helpers
    ///@{ 

    /// Check if all configurations within a group are inside their respective 
    /// boundaries
    bool GroupInBounds(GroupCfgType& _gcfg, const BoundaryMap& _boundaryMap);

    ///@}
};

/*---------------------------------- Construction ---------------------------*/

template <typename MPTraits>
BridgeTestSampler<MPTraits>::
BridgeTestSampler(std::string _vcLabel, std::string _dmLabel, double _d, bool _useBoundary) :
  m_d(_d), m_useBoundary(_useBoundary), m_vcLabel(_vcLabel), m_dmLabel(_dmLabel) {
  this->SetName("BridgeTestSampler");
}


template <typename MPTraits>
BridgeTestSampler<MPTraits>::
BridgeTestSampler(XMLNode& _node) : SamplerMethod<MPTraits>(_node) {
  this->SetName("BridgeTestSampler");
  ParseXML(_node);
}

/*------------------------------------- Parser --------------------------------*/

template <typename MPTraits>
void
BridgeTestSampler<MPTraits>::
ParseXML(XMLNode& _node) {
  m_d = _node.Read("d", true, 0.0, 0.0, 100.0, "bridge_d");
  m_useBoundary = _node.Read("useBBX", false, true,
      "Use the Boundary as an Obstacle");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
  m_dmLabel = _node.Read("dmLabel", true, "default", "Distance Metric Method");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
BridgeTestSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\td = " << m_d << endl;
  _os << "\tuseBoundary = " << m_useBoundary << endl;
  _os << "\tvcLabel = " << m_vcLabel << endl;
  _os << "\tdmLabel = " << m_dmLabel << endl;
}

/*-----------------------------  Sampler Rule --------------------------------*/

template <typename MPTraits>
bool
BridgeTestSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    vector<CfgType>& _valid, vector<CfgType>& _invalid) {

  std::string callee(this->GetNameAndLabel() + "::SampleImpl");
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel); 
  auto robot = this->GetTask()->GetRobot();
  bool generated = false; // Have we generated a sample that passes the test?

  if(this->m_debug)
    VDClearAll();
  
  if(vc->IsValid(_cfg, callee) and !(m_useBoundary and !_cfg.InBounds(_boundary))) {
    // If _cfg is valid extend Gaussian rays in opposite directions at length d/2
    CfgType mid = _cfg, incr(robot), cfg1(robot);
    incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d))/2, dm);
    cfg1 = mid - incr;

    if(this->m_debug)
      std::cout << "Input cfg is valid: " << mid << std::endl;

    // If cfg1 is invalid after adjustment, create cfg2
    if((m_useBoundary and !cfg1.InBounds(_boundary)) or !vc->IsValid(cfg1, callee)) {
      CfgType cfg2 = mid + incr;
      
      if(this->m_debug)
        std::cout << "Ray endpoint 1 is invalid: " << cfg1 << std::endl;

      // If cfg2 is also invalid, node generation successful
      if((m_useBoundary and !cfg2.InBounds(_boundary)) or !vc->IsValid(cfg2, callee)) {
        generated = true;
        _valid.push_back(mid);

        if(this->m_debug)
          std::cout << "Ray endpoint 2 is invalid: " << cfg2 << std::endl
                    << "Input cfg passes the bridge test." << mid << std::endl;
          
      } else {
        _invalid.push_back(mid);

        if(this->m_debug)
          std::cout << "Ray endpoint 2 is valid: " << cfg2 << std::endl
                    << "Input cfg fails the bridge test: " << mid << std::endl;
      }
    } else {
      if(this->m_debug)
        std::cout << "Ray endpoint 1 is valid: " << cfg1 << std::endl
                  << "Input cfg fails the bridge test: " << mid << std::endl;
    }
  } else {
    // If _cfg is invalid, already have good cfg1, extend Gaussian ray at length d
    CfgType cfg1 = _cfg, incr(robot), cfg2(robot);
    incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d)), dm);
    cfg2 = cfg1 + incr;
    
    if(this->m_debug)
      std::cout << "Input cfg is invalid: " << cfg1 << std::endl
                << "Using it as the endpoint of the ray." << std::endl;

    // If both cfg1 and cfg2 are invalid, create mid and pass test if mid is valid
    if((m_useBoundary and !cfg2.InBounds(_boundary)) or !vc->IsValid(cfg2, callee)) {

      if(this->m_debug)
        std::cout << "Ray endpoint 2 is invalid: " << cfg2 << std::endl;

      CfgType mid(robot);
      mid.WeightedSum(cfg1, cfg2, 0.5);

      if(vc->IsValid(mid, callee) and !(m_useBoundary and !mid.InBounds(_boundary))) {
        generated = true;
        _valid.push_back(mid);
        
        if(this->m_debug)
          std::cout << "Ray midpoint is valid, cfg passes the bridge test: "
                    << mid << std::endl;

      } else {
          _invalid.push_back(mid);

        if(this->m_debug)
          std::cout << "Ray midpoint is invalid, cfg fails the bridge test:"
                    << mid << std::endl;
      }
    } else {
      if(this->m_debug)
        std::cout << "Ray endpoint 2 is valid, bridge test fails: " 
                  << cfg2 << std::endl;
    }
  }
  return generated;
}


template <typename MPTraits>
bool
BridgeTestSampler<MPTraits>::
Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
    vector<GroupCfgType>& _valid, vector<GroupCfgType>& _invalid) {
    
  string callee(this->GetNameAndLabel() + "::SampleImpl");
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel); 
  auto groupRoadmap = this->GetGroupRoadmap();
  bool generated = false;
  GroupCfgType incrementors = _cfg;

  if(this->m_debug)
    VDClearAll();

  for(size_t i = 0; i < _cfg.GetNumRobots(); i++) {
    auto robot = _cfg.GetRobot(i);
    CfgType incr(robot);
    incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d))/2, dm);
    incrementors.SetRobotCfg(i, std::move(incr));
  }
  
  if(vc->IsValid(_cfg, callee) and !(m_useBoundary and !_cfg.InBounds(_boundary))) {
    // If _cfg is a valid configuration extend Gaussian rays in opposite 
    // directions at length d/2
    GroupCfgType mid = _cfg, cfg1(groupRoadmap);
    cfg1 = mid - incrementors;
    
    if(this->m_debug)
      std::cout << "Input cfg is valid: " << mid << std::endl;

    //If cfg1 is invalid after adjustment, create cfg2
    if((m_useBoundary and !cfg1.InBounds(_boundary)) or !vc->IsValid(cfg1, callee)) {
      GroupCfgType cfg2 = mid + incrementors;

      if(this->m_debug)
        std::cout << "Ray endpoint 1 is invalid: " << cfg1 << std::endl;

      // If cfg2 is also invalid, node generation successful
      if((m_useBoundary and !cfg2.InBounds(_boundary)) or !vc->IsValid(cfg2, callee)) {
        generated = true;
        _valid.push_back(mid);

        if(this->m_debug)
          std::cout << "Ray endpoint 2 is invalid: " << cfg2 << std::endl
                    << "Input cfg passes the bridge test." << mid << std::endl;
          
      } else {
        _invalid.push_back(mid);

        if(this->m_debug)
          std::cout << "Ray endpoint 2 is valid: " << cfg2 << std::endl
                    << "Input cfg fails the bridge test: " << mid << std::endl;
      }
    } else {
      if(this->m_debug)
        std::cout << "Ray endpoint 1 is valid: " << cfg1 << std::endl
                  << "Input cfg fails the bridge test: " << mid << std::endl;
    }
  } else {
    // If _cfg is invalid, already have good cfg1, extend Gaussian ray at length d
    GroupCfgType cfg1 = _cfg, cfg2(groupRoadmap);
    cfg2 = cfg1 + incrementors;

    if(this->m_debug)
      std::cout << "Input cfg is invalid: " << cfg1 << std::endl
                << "Using it as the endpoint of the ray." << std::endl;

    // If both cfg1 and cfg2 are invalid, create mid and pass test if mid is valid
    if((m_useBoundary and !cfg2.InBounds(_boundary)) or !vc->IsValid(cfg2, callee)) {
        
      if(this->m_debug)
        std::cout << "Ray endpoint 2 is invalid: " << cfg2 << std::endl;
        
      GroupCfgType mid(groupRoadmap);
      mid = (cfg1 + cfg2) * 0.5;

      if(vc->IsValid(mid, callee) and !(m_useBoundary and !mid.InBounds(_boundary))) {
        generated = true;
        _valid.push_back(mid);

        if(this->m_debug)
          std::cout << "Ray midpoint is valid, cfg passes the bridge test: "
                    << mid << std::endl;
          
      } else {
        _invalid.push_back(mid);

        if(this->m_debug)
          std::cout << "Ray midpoint is invalid, cfg fails the bridge test:"
                    << mid << std::endl;
      }
    } else {
      if(this->m_debug)
        std::cout << "Ray endpoint 2 is valid, bridge test fails: " 
                  << cfg2 << std::endl;
    }
  }
  return generated;
}


template <typename MPTraits>
bool
BridgeTestSampler<MPTraits>::
Sampler(GroupCfgType& _cfg, const BoundaryMap& _boundaryMap,
    vector<GroupCfgType>& _valid, vector<GroupCfgType>& _invalid) {
  
  string callee(this->GetNameAndLabel() + "::SampleImpl");
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel); 
  auto groupRoadmap = this->GetGroupRoadmap();
  bool generated = false;
  GroupCfgType incrementors = _cfg;

  if(this->m_debug)
    VDClearAll();

  for(size_t i = 0; i < _cfg.GetNumRobots(); i++) {
    auto robot = _cfg.GetRobot(i);
    CfgType incr(robot);
    incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d))/2, dm);
    incrementors.SetRobotCfg(i, std::move(incr));
  }
  
  if(vc->IsValid(_cfg, callee) and 
      !(m_useBoundary and !GroupInBounds(_cfg, _boundaryMap))) {
    // If _cfg is a valid configuration extend Gaussian rays in opposite 
    // directions at length d/2
    GroupCfgType mid = _cfg, cfg1(groupRoadmap);
    cfg1 = mid - incrementors;
    
    if(this->m_debug)
      std::cout << "Input cfg is valid: " << mid << std::endl;

    //If cfg1 is invalid after adjustment, create cfg2
    if((m_useBoundary and !GroupInBounds(_cfg, _boundaryMap)) or 
        !vc->IsValid(cfg1, callee)) {
      GroupCfgType cfg2 = mid + incrementors;

      if(this->m_debug)
        std::cout << "Ray endpoint 1 is invalid: " << cfg1 << std::endl;

      // If cfg2 is also invalid, node generation successful
      if((m_useBoundary and !GroupInBounds(_cfg, _boundaryMap)) or 
          !vc->IsValid(cfg2, callee)) {
        generated = true;
        _valid.push_back(mid);

        if(this->m_debug)
          std::cout << "Ray endpoint 2 is invalid: " << cfg2 << std::endl
                    << "Input cfg passes the bridge test." << mid << std::endl;
          
      } else {
        _invalid.push_back(mid);

        if(this->m_debug)
          std::cout << "Ray endpoint 2 is valid: " << cfg2 << std::endl
                    << "Input cfg fails the bridge test: " << mid << std::endl;
      }
    } else {
      if(this->m_debug)
        std::cout << "Ray endpoint 1 is valid: " << cfg1 << std::endl
                  << "Input cfg fails the bridge test: " << mid << std::endl;
    }
  } else {
    // If _cfg is invalid, already have good cfg1, extend Gaussian ray at length d
    GroupCfgType cfg1 = _cfg, cfg2(groupRoadmap);
    cfg2 = cfg1 + incrementors;

    if(this->m_debug)
      std::cout << "Input cfg is invalid: " << cfg1 << std::endl
                << "Using it as the endpoint of the ray." << std::endl;

    // If both cfg1 and cfg2 are invalid, create mid and pass test if mid is valid
    if((m_useBoundary and !GroupInBounds(_cfg, _boundaryMap)) or 
        !vc->IsValid(cfg2, callee)) {
        
      if(this->m_debug)
        std::cout << "Ray endpoint 2 is invalid: " << cfg2 << std::endl;
        
      GroupCfgType mid(groupRoadmap);
      mid = (cfg1 + cfg2) * 0.5;

      if(vc->IsValid(mid, callee) and 
          !(m_useBoundary and !GroupInBounds(_cfg, _boundaryMap))) {
        generated = true;
        _valid.push_back(mid);

        if(this->m_debug)
          std::cout << "Ray midpoint is valid, cfg passes the bridge test: "
                    << mid << std::endl;
          
      } else {
        _invalid.push_back(mid);

        if(this->m_debug)
          std::cout << "Ray midpoint is invalid, cfg fails the bridge test:"
                    << mid << std::endl;
      }
    } else {
      if(this->m_debug)
        std::cout << "Ray endpoint 2 is valid, bridge test fails: " 
                  << cfg2 << std::endl;
    }
  }
  return generated;
}


template <typename MPTraits>
bool
BridgeTestSampler<MPTraits>::
GroupInBounds(GroupCfgType& _gcfg, const BoundaryMap& _boundaryMap) {
  // Check if each robot is inside its individual boundary
  for(size_t i = 0; i < _gcfg.GetNumRobots(); i++) {
    CfgType cfg = _gcfg.GetRobotCfg(i);
    auto boundary = _boundaryMap.at(cfg.GetRobot());
    if(!cfg.InBounds(boundary))
      return false;
  }
  return true;
}

#endif
