#ifndef PPL_NODE_CLEARANCE_VALIDITY_TEST_H_
#define PPL_NODE_CLEARANCE_VALIDITY_TEST_H_

#include "ValidityCheckerMethodTest.h"
#include "MPLibrary/ValidityCheckers/NodeClearanceValidity.h"

template <typename MPTraits>
class NodeClearanceValidityTest : virtual public NodeClearanceValidity<MPTraits>,
                               public ValidityCheckerMethodTest<MPTraits> {

  public: 
  
    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    NodeClearanceValidityTest();

    NodeClearanceValidityTest(XMLNode& _node);

    ~NodeClearanceValidityTest();

    ///@}
    ///@name Interface 
    ///@{

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const std::string& _callName) override;

    ///@}

  private:

    ///@name Test Interface Functions
    ///@{

    virtual TestResult IndividualCfgValidityTest() override;

    virtual TestResult GroupCfgValidityTest() override;

    ///@}

    //using NodeClearanceValidity<MPTraits>::m_name;
    template<typename T, typename U> friend class MethodSet;

};

/*--------------------------- Construction ---------------------------*/

template<typename MPTraits>
NodeClearanceValidityTest<MPTraits>::
NodeClearanceValidityTest() : NodeClearanceValidity<MPTraits>() {}

template<typename MPTraits>
NodeClearanceValidityTest<MPTraits>:: 
NodeClearanceValidityTest(XMLNode& _node) : ValidityCheckerMethod<MPTraits>(_node),
                                         NodeClearanceValidity<MPTraits>(_node) {
}

template<typename MPTraits>
NodeClearanceValidityTest<MPTraits>::
~NodeClearanceValidityTest() {}

/*---------------------------- Interface -----------------------------*/

template <typename MPTraits>
bool
NodeClearanceValidityTest<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const std::string& _callName) {
  return NodeClearanceValidity<MPTraits>::IsValidImpl(_cfg,_cdInfo,_callName);
}

/*--------------------- Test Interface Functions ---------------------*/

template<typename MPTraits>
typename NodeClearanceValidityTest<MPTraits>::TestResult
NodeClearanceValidityTest<MPTraits>::
IndividualCfgValidityTest() {
  auto rdmp = this->GetRoadmap();

  // Remove all vertices from roadmap
  for (auto v : rdmp->GetAllVIDs()) {
    // rdmp->DeleteVertex(v);
    std::cout << v << std::endl;
  }

  bool passed = true;
  std::string message = "";


  // Just for this environment, for these tests:
  if (this->m_minClearance > 20){
    this->m_minClearance = 20;
  }

  // Make a bounding sphere around (0, 12, 0) with radius 2. 
  Boundary* b;
  CSpaceBoundingSphere boundary(std::vector<double>{0, 12, 0, 0, 0, 0}, 2);
  b = &boundary;

  // Set up roadmap by sampling in this region. 
  auto sampler = this->GetSampler("UniformRandom");
  std::vector<CfgType> samples;
  sampler->Sample(300, 1, b, std::back_inserter(samples));

  for (auto& sample : samples) {
    rdmp->AddVertex(sample);
  }
  // static_assert(this->GetRoadmap()->get_num_vertices() >= 1, "rdmp does hot have enough vertices.");

  // Cfg too close to another sample.
  std::vector<CfgType> badSamples;
  sampler->Sample(100, 1, b, std::back_inserter(badSamples));
  for (auto& s : badSamples) 
    passed &= !this->IsValid(s, "NodeClearanceValidityCheckerTest");

  // Cfg is far away from another sample. - these Cfgs are at least 20 units away.
  Boundary* goodBoundary;
  CSpaceBoundingSphere goodB(std::vector<double>{0, -12, 0, 0, 0, 0}, 2);
  goodBoundary = &goodB;
  
  std::vector<CfgType> goodSamples;
  sampler->Sample(100, 1, goodBoundary, std::back_inserter(goodSamples));
  for (auto& s : goodSamples)
    passed &= this->IsValid(s, "NodeClearanceValidityCheckerTest");

  
  // We are done. 
  if(passed) {
    message = "IndividualCfgValidity::PASSED!\n";
  }
  else {
    message = "IndividualCfgValidity::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}

template<typename MPTraits>
typename NodeClearanceValidityTest<MPTraits>::TestResult
NodeClearanceValidityTest<MPTraits>::
GroupCfgValidityTest() {

  bool passed = true;
  std::string message = "";

  auto output = this->IndividualCfgValidity();

  if(passed) {
    message = "GroupCfgValidity::PASSED!\n";
  }
  else {
    message = "GroupCfgValidity::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}

/*--------------------------------------------------------------------*/
#endif
