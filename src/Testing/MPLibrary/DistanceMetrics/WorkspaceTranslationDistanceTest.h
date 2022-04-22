#ifndef PPL_WORKSPACETRANSLATION_DISTANCE_TEST_H_
#define PPL_WORKSPACETRANSLATION_DISTANCE_TEST_H_

#include "MPLibrary/DistanceMetrics/WorkspaceTranslationDistance.h"
#include "Testing/MPLibrary/DistanceMetrics/DistanceMetricMethodTest.h"

template <typename MPTraits>
class WorkspaceTranslationDistanceTest : virtual public WorkspaceTranslationDistance<MPTraits>,
                                         public DistanceMetricMethodTest<MPTraits> {

  public: 
  
    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::GroupCfgType  GroupCfgType;

    ///@}
    ///@name Construction
    ///@{

    WorkspaceTranslationDistanceTest();

    WorkspaceTranslationDistanceTest(XMLNode& _node);

    ~WorkspaceTranslationDistanceTest();

    ///@}

  private:

    ///@name Test Interface Functions
    ///@{

    virtual TestResult TestIndividualCfgDistance() override;

    virtual TestResult TestIndividualEdgeWeight() override;

    virtual TestResult TestIndividualScaleCfg() override;

    virtual TestResult TestGroupCfgDistance() override;

    virtual TestResult TestGroupEdgeWeight() override;

    virtual TestResult TestGroupScaleCfg() override;

    ///@}
    ///@name Helpers
    ///@{

    double TrueIndividualCfgDistance();

    ///@}

};

/*--------------------------- Construction ---------------------------*/

template<typename MPTraits>
WorkspaceTranslationDistanceTest<MPTraits>::
WorkspaceTranslationDistanceTest() : WorkspaceTranslationDistance<MPTraits>() {}

template<typename MPTraits>
WorkspaceTranslationDistanceTest<MPTraits>:: 
WorkspaceTranslationDistanceTest(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node),
                                                   WorkspaceTranslationDistance<MPTraits>(_node) {}

template<typename MPTraits>
WorkspaceTranslationDistanceTest<MPTraits>::
~WorkspaceTranslationDistanceTest() {}

/*--------------------- Test Interface Functions ---------------------*/

template<typename MPTraits>
typename WorkspaceTranslationDistanceTest<MPTraits>::TestResult
WorkspaceTranslationDistanceTest<MPTraits>::
TestIndividualCfgDistance() {
  bool passed = true;
  std::string message = "";

  double dist = this->IndividualCfgDistance();
  double trueDist = TrueIndividualCfgDistance();

  if (fabs(dist - trueDist) > 1e-7) {
    passed = false;
    message = message + "\n\tIncorrect distance returned between different configurations.\n";
  }

  if (passed) {
    message = "IndividualCfgSample::PASSED!\n";
  } else {
    message = "IndividualCfgSample::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename WorkspaceTranslationDistanceTest<MPTraits>::TestResult
WorkspaceTranslationDistanceTest<MPTraits>::
TestIndividualEdgeWeight() {
  bool passed = true;
  std::string message = "";

  double weight = this->IndividualEdgeWeight();
  double trueWeight = TrueIndividualCfgDistance();

  if (fabs(weight - trueWeight) > 1e-7) {
    passed = false;
    message = message + "\n\tIncorrect edge weight/distance between configurations.\n";
  }

  if (passed) {
    message = "IndividualEdgeWeight::PASSED!\n";
  } else {
    message = "IndividualEdgeWeight::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename WorkspaceTranslationDistanceTest<MPTraits>::TestResult
WorkspaceTranslationDistanceTest<MPTraits>::
TestIndividualScaleCfg() {
  bool passed = true;
  std::string message = "";

  CfgType c1 = this->GetIndividualCfg();
  CfgType c2 = this->IndividualScaleCfg();
  double newLength = this->Distance(c1, c2);

  if (fabs(newLength - 10.0) > 1) {
    passed = false;
    message = message + "\n\tScaled distance is not the correct magnitude.\n";
  }

  if (passed) {
    message = "IndividualScaleCfg::PASSED!\n";
  } else {
    message = "IndividualScaleCfg::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename WorkspaceTranslationDistanceTest<MPTraits>::TestResult
WorkspaceTranslationDistanceTest<MPTraits>::
TestGroupCfgDistance() {
  bool passed = true;
  std::string message = "";

  GroupCfgType gcfg = this->GetGroupCfg();

  double dist = this->GroupCfgDistance();
  double trueDist = TrueIndividualCfgDistance() * gcfg.GetNumRobots();

  if (fabs(dist - trueDist) > 1e-7) {
    passed = false;
    message = message + "\n\tIncorrect distance returned between different configurations.\n";
  }

  if (passed) {
    message = "GroupCfgSample::PASSED!\n";
  } else {
    message = "GroupCfgSample::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename WorkspaceTranslationDistanceTest<MPTraits>::TestResult
WorkspaceTranslationDistanceTest<MPTraits>::
TestGroupEdgeWeight() {
  bool passed = true;
  std::string message = "";

  //TODO::This method does not exist in distance metric method yet.

  if (passed) {
    message = "GroupEdgeWeight::PASSED!\n";
  } else {
    message = "GroupEdgeWeight::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename WorkspaceTranslationDistanceTest<MPTraits>::TestResult
WorkspaceTranslationDistanceTest<MPTraits>::
TestGroupScaleCfg() {
  bool passed = true;
  std::string message = "";

  //TODO::This method does not exist in distance metric method yet.

  if (passed) {
    message = "GroupScaleCfg::PASSED!\n";
  } else {
    message = "GroupScaleCfg::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

/*--------------------------- Helpers --------------------------------*/

template <typename MPTraits>
double
WorkspaceTranslationDistanceTest<MPTraits>::
TrueIndividualCfgDistance() {
  // Given a cfg at the origin and a cfg with each of the position DOFs 
  // increased by 5, and all others increased by 0.5, we will get a workspace 
  // translation distance of 8.6602540378444
  double distance = 8.6602540378444;
  
  return distance;
}

/*--------------------------------------------------------------------*/
#endif
