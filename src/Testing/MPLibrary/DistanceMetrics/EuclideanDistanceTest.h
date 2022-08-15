#ifndef PPL_EUCLIDEAN_DISTANCE_TEST_H_
#define PPL_EUCLIDEAN_DISTANCE_TEST_H_

#include "MPLibrary/DistanceMetrics/EuclideanDistance.h"
#include "Testing/MPLibrary/DistanceMetrics/DistanceMetricMethodTest.h"

template <typename MPTraits>
class EuclideanDistanceTest : virtual public EuclideanDistance<MPTraits>,
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

    EuclideanDistanceTest(bool _normalize = false);

    EuclideanDistanceTest(XMLNode& _node);

    ~EuclideanDistanceTest();

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
EuclideanDistanceTest<MPTraits>::
EuclideanDistanceTest(bool _normalize) : EuclideanDistance<MPTraits>(_normalize) {}

template<typename MPTraits>
EuclideanDistanceTest<MPTraits>:: 
EuclideanDistanceTest(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node),
                                        EuclideanDistance<MPTraits>(_node) {}

template<typename MPTraits>
EuclideanDistanceTest<MPTraits>::
~EuclideanDistanceTest() {}

/*--------------------- Test Interface Functions ---------------------*/

template<typename MPTraits>
typename EuclideanDistanceTest<MPTraits>::TestResult
EuclideanDistanceTest<MPTraits>::
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
typename EuclideanDistanceTest<MPTraits>::TestResult
EuclideanDistanceTest<MPTraits>::
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
typename EuclideanDistanceTest<MPTraits>::TestResult
EuclideanDistanceTest<MPTraits>::
TestIndividualScaleCfg() {
  bool passed = true;
  std::string message = "";

  CfgType c1 = this->GetIndividualCfg();
  CfgType c2 = this->IndividualScaleCfg();
  double newLength = this->Distance(c1, c2);

  if (fabs(newLength - 10.0) > 1e-7) {
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
typename EuclideanDistanceTest<MPTraits>::TestResult
EuclideanDistanceTest<MPTraits>::
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
typename EuclideanDistanceTest<MPTraits>::TestResult
EuclideanDistanceTest<MPTraits>::
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
typename EuclideanDistanceTest<MPTraits>::TestResult
EuclideanDistanceTest<MPTraits>::
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
EuclideanDistanceTest<MPTraits>::
TrueIndividualCfgDistance() {
  CfgType cfg1 = this->GetIndividualCfg();
  CfgType cfg2 = this->GetIndividualCfg();

  // Euclidean distance should be sqrt(5^2 * PosDOF + 0.5^2 * OriDOF)
  double trueDist;
  if (this->m_normalize) {
    const double diagonal = this->GetEnvironment()->GetBoundary()->GetMaxDist(
        2, 0.5);
    trueDist = std::pow(5 / diagonal, 2) * cfg2.PosDOF();
  } else {
    trueDist = std::pow(5, 2) * cfg2.PosDOF();
  }
  trueDist += std::pow(0.5, 2) * (cfg2.DOF() - cfg2.PosDOF());
  trueDist = std::pow(trueDist, 0.5);

  return trueDist;
}

/*--------------------------------------------------------------------*/
#endif
