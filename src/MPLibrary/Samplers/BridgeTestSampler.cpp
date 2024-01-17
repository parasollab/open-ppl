#include "BridgeTestSampler.h"

#include "MPLibrary/MPLibrary.h"

/*---------------------------------- Construction ---------------------------*/

BridgeTestSampler::
BridgeTestSampler(std::string _vcLabel, std::string _dmLabel, double _d, bool _useBoundary) :
  m_d(_d), m_useBoundary(_useBoundary), m_vcLabel(_vcLabel), m_dmLabel(_dmLabel) {
  this->SetName("BridgeTestSampler");
}


BridgeTestSampler::
BridgeTestSampler(XMLNode& _node) : SamplerMethod(_node) {
  this->SetName("BridgeTestSampler");
  ParseXML(_node);
}

/*------------------------------------- Parser --------------------------------*/

void
BridgeTestSampler::
ParseXML(XMLNode& _node) {
  m_d = _node.Read("d", true, 0.0, 0.0, 100.0, "bridge_d");
  m_useBoundary = _node.Read("useBBX", false, true,
      "Use the Boundary as an Obstacle");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
  m_dmLabel = _node.Read("dmLabel", true, "default", "Distance Metric Method");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

void
BridgeTestSampler::
Print(ostream& _os) const {
  SamplerMethod::Print(_os);
  _os << "\td = " << m_d << endl;
  _os << "\tuseBoundary = " << m_useBoundary << endl;
  _os << "\tvcLabel = " << m_vcLabel << endl;
  _os << "\tdmLabel = " << m_dmLabel << endl;
}

/*-----------------------------  Sampler Rule --------------------------------*/

bool
BridgeTestSampler::
Sampler(Cfg& _cfg, const Boundary* const _boundary,
    vector<Cfg>& _valid, vector<Cfg>& _invalid) {

  std::string callee(this->GetNameAndLabel() + "::SampleImpl");
  auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel); 
  auto robot = this->GetTask()->GetRobot();
  bool generated = false; // Have we generated a sample that passes the test?

  if(this->m_debug)
    VDClearAll();
  
  if(vc->IsValid(_cfg, callee) and !(m_useBoundary and !_cfg.InBounds(_boundary))) {
    // If _cfg is valid extend Gaussian rays in opposite directions at length d/2
    Cfg mid = _cfg, incr(robot), cfg1(robot);
    incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d))/2, dm);
    cfg1 = mid - incr;

    if(this->m_debug)
      std::cout << "Input cfg is valid: " << mid << std::endl;

    // If cfg1 is invalid after adjustment, create cfg2
    if((m_useBoundary and !cfg1.InBounds(_boundary)) or !vc->IsValid(cfg1, callee)) {
      Cfg cfg2 = mid + incr;
      
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
    Cfg cfg1 = _cfg, incr(robot), cfg2(robot);
    incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d)), dm);
    cfg2 = cfg1 + incr;
    
    if(this->m_debug)
      std::cout << "Input cfg is invalid: " << cfg1 << std::endl
                << "Using it as the endpoint of the ray." << std::endl;

    // If both cfg1 and cfg2 are invalid, create mid and pass test if mid is valid
    if((m_useBoundary and !cfg2.InBounds(_boundary)) or !vc->IsValid(cfg2, callee)) {

      if(this->m_debug)
        std::cout << "Ray endpoint 2 is invalid: " << cfg2 << std::endl;

      Cfg mid(robot);
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


bool
BridgeTestSampler::
Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
    vector<GroupCfgType>& _valid, vector<GroupCfgType>& _invalid) {
    
  string callee(this->GetNameAndLabel() + "::SampleImpl");
  auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel); 
  auto groupRoadmap = this->GetGroupRoadmap();
  bool generated = false;
  GroupCfgType incrementors = _cfg;

  if(this->m_debug)
    VDClearAll();

  for(size_t i = 0; i < _cfg.GetNumRobots(); i++) {
    auto robot = _cfg.GetRobot(i);
    Cfg incr(robot);
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


bool
BridgeTestSampler::
Sampler(GroupCfgType& _cfg, const BoundaryMap& _boundaryMap,
    vector<GroupCfgType>& _valid, vector<GroupCfgType>& _invalid) {
  
  string callee(this->GetNameAndLabel() + "::SampleImpl");
  auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel); 
  auto groupRoadmap = this->GetGroupRoadmap();
  bool generated = false;
  GroupCfgType incrementors = _cfg;

  if(this->m_debug)
    VDClearAll();

  for(size_t i = 0; i < _cfg.GetNumRobots(); i++) {
    auto robot = _cfg.GetRobot(i);
    Cfg incr(robot);
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


bool
BridgeTestSampler::
GroupInBounds(GroupCfgType& _gcfg, const BoundaryMap& _boundaryMap) {
  // Check if each robot is inside its individual boundary
  for(size_t i = 0; i < _gcfg.GetNumRobots(); i++) {
    Cfg cfg = _gcfg.GetRobotCfg(i);
    auto boundary = _boundaryMap.at(cfg.GetRobot());
    if(!cfg.InBounds(boundary))
      return false;
  }
  return true;
}
