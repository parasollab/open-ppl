#ifndef PPL_WEIGHTED_EUCLIDEAN_DISTANCE_TEST_H_
#define PPL_WEIGHTED_EUCLIDEAN_DISTANCE_TEST_H_

#include "MPLibrary/DistanceMetrics/WeightedEuclideanDistance.h"
#include "Testing/MPLibrary/DistanceMetrics/DistanceMetricMethodTest.h"
#include <iostream>

template <typename MPTraits>
class WeightedEuclideanDistanceTest : virtual public WeightedEuclideanDistance<MPTraits>,
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

    WeightedEuclideanDistanceTest();

    WeightedEuclideanDistanceTest(XMLNode& _node);

    WeightedEuclideanDistanceTest(const double _pos, const double _rot,
        const double _vel, const double _avl);

    ~WeightedEuclideanDistanceTest();

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
WeightedEuclideanDistanceTest<MPTraits>::
WeightedEuclideanDistanceTest() : WeightedEuclideanDistance<MPTraits>() {}

template<typename MPTraits>
WeightedEuclideanDistanceTest<MPTraits>:: 
WeightedEuclideanDistanceTest(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node),
                                                WeightedEuclideanDistance<MPTraits>(_node) {}
template<typename MPTraits>
WeightedEuclideanDistanceTest<MPTraits>::
WeightedEuclideanDistanceTest(const double _pos, const double _rot, const double _vel, const double _avl) :
    WeightedEuclideanDistance<MPTraits>(_pos, _rot, _vel, _avl) {}

template<typename MPTraits>
WeightedEuclideanDistanceTest<MPTraits>::
~WeightedEuclideanDistanceTest() {}

/*--------------------- Test Interface Functions ---------------------*/

template<typename MPTraits>
typename WeightedEuclideanDistanceTest<MPTraits>::TestResult
WeightedEuclideanDistanceTest<MPTraits>::
TestIndividualCfgDistance() {
  bool passed = true;
  std::string message = "";

  double dist = this->IndividualCfgDistance();
  double trueDist = TrueIndividualCfgDistance();

  std::cout << trueDist << "\n";
  std::cout << dist << "\n";
  std::cout << "------------------" << "\n";

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
typename WeightedEuclideanDistanceTest<MPTraits>::TestResult
WeightedEuclideanDistanceTest<MPTraits>::
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
typename WeightedEuclideanDistanceTest<MPTraits>::TestResult
WeightedEuclideanDistanceTest<MPTraits>::
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
typename WeightedEuclideanDistanceTest<MPTraits>::TestResult
WeightedEuclideanDistanceTest<MPTraits>::
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
typename WeightedEuclideanDistanceTest<MPTraits>::TestResult
WeightedEuclideanDistanceTest<MPTraits>::
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
typename WeightedEuclideanDistanceTest<MPTraits>::TestResult
WeightedEuclideanDistanceTest<MPTraits>::
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
WeightedEuclideanDistanceTest<MPTraits>::
TrueIndividualCfgDistance() {
  CfgType cfg1 = this->GetIndividualCfg();
  CfgType cfg2 = this->GetIndividualCfg();
  double defaultWeight = 0.25;

  // Weighted euclidean distance should be sqrt(defaultWeight * 5^2 * PosDOF + defaultWeight * 0.5^2 * OriDOF)
  double trueDist;
  trueDist = defaultWeight * std::pow(5, 2) * cfg2.PosDOF();
  trueDist += defaultWeight * std::pow(0.5, 2) * (cfg2.DOF() - cfg2.PosDOF());
  trueDist = std::pow(trueDist, 0.5);

  return trueDist;
}

/*--------------------------------------------------------------------*/
#endif
