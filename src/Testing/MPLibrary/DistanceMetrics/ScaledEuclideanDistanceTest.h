#ifndef PPL_SCALED_EUCLIDEAN_DISTANCE_TEST_H_
#define PPL_SCALED_EUCLIDEAN_DISTANCE_TEST_H_

#include "MPLibrary/DistanceMetrics/ScaledEuclideanDistance.h"
#include "Testing/MPLibrary/DistanceMetrics/DistanceMetricMethodTest.h"

template <typename MPTraits>
class ScaledEuclideanDistanceTest : virtual public ScaledEuclideanDistance<MPTraits>,
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

    ScaledEuclideanDistanceTest();

    ScaledEuclideanDistanceTest(XMLNode& _node);

    ~ScaledEuclideanDistanceTest();

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
ScaledEuclideanDistanceTest<MPTraits>::
ScaledEuclideanDistanceTest() : ScaledEuclideanDistance<MPTraits>() {}

template<typename MPTraits>
ScaledEuclideanDistanceTest<MPTraits>:: 
ScaledEuclideanDistanceTest(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node),
                                              ScaledEuclideanDistance<MPTraits>(_node) {}

template<typename MPTraits>
ScaledEuclideanDistanceTest<MPTraits>::
~ScaledEuclideanDistanceTest() {}

/*--------------------- Test Interface Functions ---------------------*/

template<typename MPTraits>
typename ScaledEuclideanDistanceTest<MPTraits>::TestResult
ScaledEuclideanDistanceTest<MPTraits>::
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
typename ScaledEuclideanDistanceTest<MPTraits>::TestResult
ScaledEuclideanDistanceTest<MPTraits>::
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
typename ScaledEuclideanDistanceTest<MPTraits>::TestResult
ScaledEuclideanDistanceTest<MPTraits>::
TestIndividualScaleCfg() {
  bool passed = true;
  std::string message = "";

  CfgType c1 = this->GetIndividualCfg();
  CfgType c2 = this->IndividualScaleCfg();
  double newLength = this->Distance(c1, c2);

  // Note: Fails if threshold is 1e-3 or above.
  if (fabs(newLength - 10.0) > 1e-2) {
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
typename ScaledEuclideanDistanceTest<MPTraits>::TestResult
ScaledEuclideanDistanceTest<MPTraits>::
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
typename ScaledEuclideanDistanceTest<MPTraits>::TestResult
ScaledEuclideanDistanceTest<MPTraits>::
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
typename ScaledEuclideanDistanceTest<MPTraits>::TestResult
ScaledEuclideanDistanceTest<MPTraits>::
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
ScaledEuclideanDistanceTest<MPTraits>::
TrueIndividualCfgDistance() {
  CfgType cfg2 = this->GetIndividualCfg();
  // Scaled euclidean distance should be (m_scale * 5^3 * PosDOF + (1 - m_scale) * 0.5^3 * OriDOF)^(1 / 3)
  double trueDist;
  trueDist = this->m_scale * std::pow(5, 3.0) * cfg2.PosDOF();
  trueDist += (1 - this->m_scale) * std::pow(0.5, 3.0) * (cfg2.DOF() - cfg2.PosDOF());
  trueDist = std::pow(trueDist, 1.0 / 3.0);

  return trueDist;
}

/*--------------------------------------------------------------------*/
#endif
