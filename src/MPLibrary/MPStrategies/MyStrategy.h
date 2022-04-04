#ifndef MY_STRATEGY_H_
#define MY_STRATEGY_H_

#include "MPStrategyMethod.h"

template<class MPTraits>
class MyStrategy : public MPStrategyMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

    MyStrategy();
    MyStrategy(XMLNode& _node);

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();

  protected:

    std::string m_vcLabel;
    std::string m_dmLabel;
    std::string m_lpLabel;
    CfgType m_lastNode;
    double m_length;
    CfgType GenerateNode();
    bool ConnectNode(CfgType& _c);

};


template<class MPTraits>
MyStrategy<MPTraits>::
MyStrategy() : m_lastNode(), m_length(0.){
  this->SetName("MyStrategy");
}


template<class MPTraits>
MyStrategy<MPTraits>::
MyStrategy(XMLNode& _node)
  : MPStrategyMethod<MPTraits>(_node), m_lastNode(), m_length(0.) {
    this->SetName("MyStrategy");
    ParseXML(_node);
  }


template<class MPTraits>
void
MyStrategy<MPTraits>::
ParseXML(XMLNode& _node) {
  // Obtain the specified methods for planning or if non is specified use the default methods.
  m_vcLabel = _node.Read("vcLabel", false, "pqp_solid", "Validity Checker");
  m_lpLabel = _node.Read("lpLabel", false, "sl", "Local Planner");
  m_dmLabel = _node.Read("dmLabel", false, "euclidean", "Distance Metric");
}


template<class MPTraits>
void
MyStrategy<MPTraits>::
Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
}


template<class MPTraits>
void
MyStrategy<MPTraits>::
Initialize() {
  m_lastNode = CfgType(this->GetTask()->GetRobot());
}


template<class MPTraits>
void
MyStrategy<MPTraits>::
Run() {
  auto r = this->GetRoadmap();
  u_int numNodes = 4;
  while(r->get_num_vertices() < numNodes) {
    CfgType newCfg = GenerateNode();
    ConnectNode(newCfg);
  }
}


template<class MPTraits>
void
MyStrategy<MPTraits>::
Finalize() {
  cout << "MyStrategy Path Length: " << m_length << endl;
  string fileName = this->GetBaseFilename() + ".map";
  cout << this->GetBaseFilename() + ".map" << endl;
  this->GetRoadmap()->Write(fileName, this->GetEnvironment());
}

template<class MPTraits>
typename MPTraits::CfgType
MyStrategy<MPTraits>::
GenerateNode() {
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto env = this->GetEnvironment();
  CfgType newCfg(this->GetTask()->GetRobot());
  // Keep getting Cfg until a valid one is found.
  do {
    newCfg.GetRandomCfg(this->GetEnvironment()->GetBoundary());
  } while(!newCfg.InBounds(env) ||
  !vc->IsValid(newCfg, this->GetNameAndLabel()));
  return newCfg;
}

template<class MPTraits>
bool
MyStrategy<MPTraits>::
ConnectNode(CfgType& _c) {
  cout << "ConnectNode Working" << endl;
  auto lp = this->
  GetLocalPlanner(m_lpLabel);
  
  auto dm = this->
  GetDistanceMetric(m_dmLabel);

  auto r = this->GetRoadmap();
  auto env = this->GetEnvironment();

  CfgType blank(this->GetTask()->GetRobot());
  if(m_lastNode == blank) {
    r->AddVertex(_c);
    m_lastNode = _c;
    return true;
  } 

  // _c is not the first node, check if it collides and is connectable to the last node.
  LPOutput<MPTraits> lpOutput;
  auto robot = this->GetTask()->GetRobot();
  CfgType collisionCfg(robot);
  if(lp->IsConnected(m_lastNode, _c, collisionCfg, &lpOutput,
    env->GetPositionRes(), env->GetOrientationRes())) {
    m_length += dm->Distance(m_lastNode, _c);
    VID newNode = r->AddVertex(_c);
    r->AddEdge(r->GetVID(m_lastNode), newNode, lpOutput.m_edge);
    m_lastNode = _c;
    return true;
  }

  return false;
}


#endif

