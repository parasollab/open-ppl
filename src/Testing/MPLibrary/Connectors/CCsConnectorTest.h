#ifndef PPL_CCS_CONNECTOR_TEST_H_
#define PPL_CCS_CONNECTOR_TEST_H_

#include "MPLibrary/Connectors/CCsConnector.h"
#include "Testing/MPLibrary/Connectors/ConnectorMethodTest.h"

template <typename MPTraits>
class CCsConnectorTest : virtual public CCsConnector<MPTraits>,
                         public ConnectorMethodTest<MPTraits> {
  public: 
    
    ///@name Local Types
    ///@{
  
    typedef TestBaseObject::TestResult TestResult;

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;

    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename MPTraits::WeightType   WeightType;

    ///@}
    ///@name Construction
    ///@{
    
    CCsConnectorTest();

    CCsConnectorTest(XMLNode& _node);

    ~CCsConnectorTest();
    
    ///@}
  
  private:

    ///@name Test Interface Functions
    ///@{
    
    virtual TestResult IndividualRobotConnectTest() override;

    virtual TestResult RobotGroupConnectTest() override;
 
    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
CCsConnectorTest<MPTraits>::
CCsConnectorTest() : CCsConnector<MPTraits>() {}

template <typename MPTraits>
CCsConnectorTest<MPTraits>::
CCsConnectorTest(XMLNode& _node) : ConnectorMethod<MPTraits>(_node),
                                   CCsConnector<MPTraits>(_node) {}

template <typename MPTraits>
CCsConnectorTest<MPTraits>::
~CCsConnectorTest() {}

/*----------------------- Test Interface Functions ---------------------*/

template<typename MPTraits>
typename CCsConnectorTest<MPTraits>::TestResult
CCsConnectorTest<MPTraits>::
IndividualRobotConnectTest() {
  bool passed = true;
  std::string message = "";

  auto robot = this->GetMPProblem()->GetRobots()[0].get();

  /// Roadmap Construction
  RoadmapType* roadmap = new RoadmapType(robot);
  roadmap->SetCCTracker(nullptr);

  // Construct vertices and add to roadmap
  auto p1 = CfgType(robot);
  std::istringstream p1Stream("0 0 0 0 0 0 0");
  p1.Read(p1Stream);
  auto v1 = roadmap->AddVertex(p1);

  auto p2 = CfgType(robot);
  std::istringstream p2Stream("0 2 0 0 0 0 0");
  p2.Read(p2Stream);
  auto v2 = roadmap->AddVertex(p2);

  auto p3 = CfgType(robot);
  std::istringstream p3Stream("0 2 0 0 0.5 0 0");
  p3.Read(p3Stream);
  auto v3 = roadmap->AddVertex(p3);

  auto p4 = CfgType(robot);
  std::istringstream p4Stream("0 1 0 0 0.5 0 0");
  p4.Read(p4Stream);
  auto v4 = roadmap->AddVertex(p4);

  roadmap->AddEdge(v1, v2, WeightType("a", 1.0));
  roadmap->AddEdge(v2, v3, WeightType("b", 1.0));

  this->Connect(roadmap, v1);

  // Make sure that only the vertex from the other CC was connected
  if (roadmap->IsEdge(v1, v3) or !(roadmap->IsEdge(v1, v4))) {
    passed = false;
    message = "\n\tDid not connect the correct vertices.\n";
  }

  if (passed) {
    message = "IndividualRobotConnect::PASSED!\n";
  } else {
    message = "IndividualRobotConnect::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename CCsConnectorTest<MPTraits>::TestResult
CCsConnectorTest<MPTraits>::
RobotGroupConnectTest() {
  bool passed = true;
  std::string message = "";

  // TODO the connector logic is the same between individual and group
  // roadmaps

  if (passed) {
    message = "RobotGroupConnect::PASSED!\n";
  } else {
    message = "RobotGroupConnect::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

/*--------------------------------------------------------------------*/
#endif
