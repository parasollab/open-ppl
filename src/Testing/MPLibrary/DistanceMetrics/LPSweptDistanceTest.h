#ifndef PPL_LPSWEPT_DISTANCE_TEST_H_
#define PPL_LPSWEPT_DISTANCE_TEST_H_

#include "MPLibrary/DistanceMetrics/LPSweptDistance.h"
#include "Testing/MPLibrary/DistanceMetrics/DistanceMetricMethodTest.h"

template <typename MPTraits>
class LPSweptDistanceTest : virtual public LPSweptDistance<MPTraits>,
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

    LPSweptDistanceTest(string _lpLabel="sl", double _positionRes = 0.1, 
      double _orientationRes = 0.1, bool _bbox = false);

    LPSweptDistanceTest(XMLNode& _node);

    ~LPSweptDistanceTest();

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
LPSweptDistanceTest<MPTraits>::
LPSweptDistanceTest(string _lpLabel, double _positionRes, 
  double _orientationRes, bool _bbox) :
  LPSweptDistance<MPTraits>(_lpLabel, _positionRes, _orientationRes, _bbox) {}

template<typename MPTraits>
LPSweptDistanceTest<MPTraits>:: 
LPSweptDistanceTest(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node),
                                      LPSweptDistance<MPTraits>(_node) {}

template<typename MPTraits>
LPSweptDistanceTest<MPTraits>::
~LPSweptDistanceTest() {}

/*--------------------- Test Interface Functions ---------------------*/

template<typename MPTraits>
typename LPSweptDistanceTest<MPTraits>::TestResult
LPSweptDistanceTest<MPTraits>::
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
typename LPSweptDistanceTest<MPTraits>::TestResult
LPSweptDistanceTest<MPTraits>::
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
typename LPSweptDistanceTest<MPTraits>::TestResult
LPSweptDistanceTest<MPTraits>::
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
typename LPSweptDistanceTest<MPTraits>::TestResult
LPSweptDistanceTest<MPTraits>::
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
typename LPSweptDistanceTest<MPTraits>::TestResult
LPSweptDistanceTest<MPTraits>::
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
typename LPSweptDistanceTest<MPTraits>::TestResult
LPSweptDistanceTest<MPTraits>::
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
LPSweptDistanceTest<MPTraits>::
TrueIndividualCfgDistance() {
  // LP Swept Distance will be sum of vertex displacements along LP 
  // path between two configs, where each pos component goes from 0. to 5. along
  // path and each ori component goes from 0 to 0.5 along path, with 
  // pos resolution of 0.1 and ori resolution of 0.1, without using bounding box
  //
  // For the straight line local planner, this value is 8.9778381120884.
  if (this->m_lpLabel == "sl")
    return 8.9778381120884;
  else 
    throw RunTimeException(WHERE, "Invalid lpLabel for CfgDistance");
  // As more local planners are added, their ground truth values will be
  // included below
}

/*--------------------------------------------------------------------*/
#endif
