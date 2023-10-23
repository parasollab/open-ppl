#ifndef PPL_SCALED_EUCLIDEAN_DISTANCE_TEST_H_
#define PPL_SCALED_EUCLIDEAN_DISTANCE_TEST_H_

#include "MPLibrary/DistanceMetrics/ScaledEuclideanDistance.h"  //src
#include "DistanceMetricMethodTest.h"

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
    std::cout << "\n\tIncorrect distance returned between different configurations." << std::endl;
  }

  message = "\tFINISHED TestIndividualCfgDistance";
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
    std::cout << "\n\tIncorrect edge weight/distance between configurations." << std::endl;
  }

  message = "\tFINISHED TestIndividualEdgeWeight";
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
    std::cout << "\n\tScaled distance is not the correct magnitude." << std::endl;
  }

  message = "\tFINISHED TestIndividualScaleCfg";
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
    std::cout << "\n\tIncorrect distance returned between different configurations." << std::endl;
  }

  message = "\tFINISHED TestGroupCfgDistance";
  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename ScaledEuclideanDistanceTest<MPTraits>::TestResult
ScaledEuclideanDistanceTest<MPTraits>::
TestGroupEdgeWeight() {
  bool passed = true;
  std::string message = "";

  //TODO::This method does not exist in distance metric method yet.

  message = "\tFINISHED TestGroupEdgeWeight";
  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename ScaledEuclideanDistanceTest<MPTraits>::TestResult
ScaledEuclideanDistanceTest<MPTraits>::
TestGroupScaleCfg() {
  bool passed = true;
  std::string message = "";

  //TODO::This method does not exist in distance metric method yet.

  message = "\tFINISHED TestGroupScaleCfg";
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
