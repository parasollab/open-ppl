#include "UniformObstacleBasedSampler.h"

#include "MPLibrary/MPLibrary.h"

/*------------------------------ Construction --------------------------------*/

UniformObstacleBasedSampler::
UniformObstacleBasedSampler(string _vcLabel, string _dmLabel, double _margin,
    bool _useBoundary) :
    m_margin(_margin), m_useBoundary(_useBoundary),
    m_vcLabel(_vcLabel), m_dmLabel(_dmLabel) {
  this->SetName("UniformObstacleBasedSampler");
}


UniformObstacleBasedSampler::
UniformObstacleBasedSampler(XMLNode& _node) : SamplerMethod(_node) {
  this->SetName("UniformObstacleBasedSampler");

  m_margin = _node.Read("d", true, 0.0, 0.0, MAX_DBL,
      "Set the length of line segments d away from obstacles"); 
  m_useBoundary = _node.Read("useBBX", true, false,
      "Use bounding box as obstacle");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity checker label");
  m_dmLabel =_node.Read("dmLabel", true, "default", "Distance metric label");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

void
UniformObstacleBasedSampler::
Print(ostream& _os) const {
  SamplerMethod::Print(_os);
  _os << "\tmargin = " << m_margin << endl
      << "\tuseBoundary = " << m_useBoundary << endl
      << "\tvcLabel = " << m_vcLabel << endl
      << "\tdmLabel = " << m_dmLabel << endl;
}

/*--------------------------- Sampler Interface ------------------------------*/

bool
UniformObstacleBasedSampler::
Sampler(Cfg& _cfg, const Boundary* const _boundary,
    std::vector<Cfg>& _result, std::vector<Cfg>& _invalid) {

  Environment* env = this->GetEnvironment();
  string callee(this->GetNameAndLabel() + "::SampleImpl()");

  //Check validity 
  auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);
  auto robot = this->GetTask()->GetRobot();

  bool generated = false;
  int attempts = 0;


  bool cfg1Free;
  double margin = m_margin != 0.
                ? m_margin
                : _cfg.GetMultiBody()->GetBase()->GetPolyhedron().GetMaxRadius();

  attempts++;

  //Generate first cfg
  Cfg& cfg1 = _cfg;

  cfg1Free = vc->IsValid(cfg1, callee);

  Cfg cfg2(robot);
  Cfg incr(robot);

  //Generate a random direction ray using margin and distance metric method.
  incr.GetRandomRay(margin, dm);  

  //Extend segment cfg1 with distance incr.
  cfg2 = cfg1 + incr;

  Cfg inter(robot);
  Cfg tick = cfg1;
  int nTicks;

  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();

  bool tempFree = cfg1Free;
  bool tickFree;
  Cfg temp = cfg1;

  inter.FindIncrement(cfg1, cfg2, &nTicks, positionRes, orientationRes);

  //Generate nTicks intermediate points along ray
  for(int i = 1; i < nTicks; i++) {
    tick += inter;
    tickFree = vc->IsValid(tick, callee);

    if(m_useBoundary)
      tickFree = tickFree && tick.InBounds(_boundary);

    if(tempFree == tickFree) {
      tempFree = tickFree;
      temp = tick;
    }
    else {	//tempFree != tickFree
      generated = true;

      //Store the vector created to _result 
      if(tempFree and temp.InBounds(_boundary)) {
        _result.push_back(temp);
        tempFree = tickFree;
        temp = tick;
      }
      else if(tickFree and tick.InBounds(_boundary)) { //tickFree
        _result.push_back(tick);
        tempFree = tickFree;
        temp = tick;
      }
    }
  }
  return generated;
}

/*----------------------------Group Sampler ---------------------*/

bool
UniformObstacleBasedSampler::
Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
    std::vector<GroupCfgType>& _result, std::vector<GroupCfgType>& _invalid) {

  Environment* env = this->GetEnvironment();
  string callee(this->GetNameAndLabel() + "::SampleImpl()");

  //Check validity 
  auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);
  auto groupRoadmap = this->GetGroupRoadmap();

  bool generated = false;
  int attempts = 0;

  bool cfg1Free;

  if (m_margin == 0) {
    throw RunTimeException(WHERE, "margin must not be equal to zero");
  }

  attempts++;

  //Generate first cfg
  GroupCfgType& cfg1 = _cfg;

  cfg1Free = vc->IsValid(cfg1, callee);

  GroupCfgType cfg2(groupRoadmap);
  GroupCfgType incr(groupRoadmap);

  //Generate a random direction ray using margin and distance metric method.
  //increment direction to group robots
  incr.GetRandomRay(m_margin, dm);

  //Extend segment cfg1 with distance incr.
  cfg2 = cfg1 + incr;

  GroupCfgType inter(groupRoadmap);
  GroupCfgType tick = cfg1;
  int nTicks;

  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();

  bool tempFree = cfg1Free;
  bool tickFree;
  GroupCfgType temp = cfg1;

  inter.FindIncrement(cfg1, cfg2, &nTicks, positionRes, orientationRes);

  //Generate nTicks intermediate points along ray
  for(int i = 1; i < nTicks; i++) {
    tick += inter;
    tickFree = vc->IsValid(tick, callee);

    if(m_useBoundary)
      tickFree = tickFree && tick.InBounds(_boundary);

    if(tempFree == tickFree) {
      tempFree = tickFree;
      temp = tick;
    }
    else {	//tempFree != tickFree
      generated = true;

      //Store the vector created to _result 
      if(tempFree and temp.InBounds(_boundary)) {
        _result.push_back(temp);
        tempFree = tickFree;
        temp = tick;
      }
      else if(tickFree and tick.InBounds(_boundary)) { //tickFree
        _result.push_back(tick);
        tempFree = tickFree;
        temp = tick;
      }
    }
  }
  return generated;
}

/*----------------------------------------------------------------------------*/
