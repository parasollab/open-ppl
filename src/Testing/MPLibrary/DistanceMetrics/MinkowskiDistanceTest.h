#ifndef PPL_MINKOWSKI_DISTANCE_TEST_H_
#define PPL_MINKOWSKI_DISTANCE_TEST_H_

#include "MPLibrary/DistanceMetrics/MinkowskiDistance.h"
#include "Testing/MPLibrary/DistanceMetrics/DistanceMetricMethodTest.h"

template <typename MPTraits>
class MinkowskiDistanceTest : virtual public MinkowskiDistance<MPTraits>,
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

    MinkowskiDistanceTest(double _r1 = 3, double _r2 = 3, double _r3 = 1. / 3,
        bool _normalize = false);

    MinkowskiDistanceTest(XMLNode& _node);

    ~MinkowskiDistanceTest();

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
MinkowskiDistanceTest<MPTraits>::
MinkowskiDistanceTest(double _r1, double _r2, double _r3, bool _normalize) :
    MinkowskiDistance<MPTraits>(_r1, _r2, _r3, _normalize) {}

template<typename MPTraits>
MinkowskiDistanceTest<MPTraits>:: 
MinkowskiDistanceTest(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node),
                                        MinkowskiDistance<MPTraits>(_node) {}

template<typename MPTraits>
MinkowskiDistanceTest<MPTraits>::
~MinkowskiDistanceTest() {}

/*--------------------- Test Interface Functions ---------------------*/

template<typename MPTraits>
typename MinkowskiDistanceTest<MPTraits>::TestResult
MinkowskiDistanceTest<MPTraits>::
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
typename MinkowskiDistanceTest<MPTraits>::TestResult
MinkowskiDistanceTest<MPTraits>::
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
typename MinkowskiDistanceTest<MPTraits>::TestResult
MinkowskiDistanceTest<MPTraits>::
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
typename MinkowskiDistanceTest<MPTraits>::TestResult
MinkowskiDistanceTest<MPTraits>::
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
typename MinkowskiDistanceTest<MPTraits>::TestResult
MinkowskiDistanceTest<MPTraits>::
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
typename MinkowskiDistanceTest<MPTraits>::TestResult
MinkowskiDistanceTest<MPTraits>::
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
MinkowskiDistanceTest<MPTraits>::
TrueIndividualCfgDistance() {
  CfgType cfg1 = this->GetIndividualCfg();
  CfgType cfg2 = this->GetIndividualCfg();

  // Minkowski distance should be (5^r1 * PosDOF + 0.5^r2 * OriDOF)^r3
  double trueDist;
  if (this->m_normalize) {
    const double diagonal = this->GetEnvironment()->GetBoundary()->GetMaxDist(
        this->m_r1, this->m_r3);
    trueDist = std::pow(5 / diagonal, this->m_r1) * cfg2.PosDOF();
  } else {
    trueDist = std::pow(5, this->m_r1) * cfg2.PosDOF();
  }
  trueDist += std::pow(0.5, this->m_r2) * (cfg2.DOF() - cfg2.PosDOF());
  trueDist = std::pow(trueDist, this->m_r3);

  return trueDist;
}

/*--------------------------------------------------------------------*/
#endif
