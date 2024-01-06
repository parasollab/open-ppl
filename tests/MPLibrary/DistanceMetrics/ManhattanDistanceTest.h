#ifndef PPL_MANHATTAN_DISTANCE_TEST_H_
#define PPL_MANHATTAN_DISTANCE_TEST_H_

#include "MPLibrary/DistanceMetrics/ManhattanDistance.h"    //src
#include "DistanceMetricMethodTest.h"

template <typename MPTraits>
class ManhattanDistanceTest : virtual public ManhattanDistance<MPTraits>,
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

    ManhattanDistanceTest(bool _normalize = false);

    ManhattanDistanceTest(XMLNode& _node);

    ~ManhattanDistanceTest();

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
ManhattanDistanceTest<MPTraits>::
ManhattanDistanceTest(bool _normalize) : ManhattanDistance<MPTraits>(_normalize) {}

template<typename MPTraits>
ManhattanDistanceTest<MPTraits>:: 
ManhattanDistanceTest(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node),
                                        ManhattanDistance<MPTraits>(_node) {}

template<typename MPTraits>
ManhattanDistanceTest<MPTraits>::
~ManhattanDistanceTest() {}

/*--------------------- Test Interface Functions ---------------------*/

template<typename MPTraits>
typename ManhattanDistanceTest<MPTraits>::TestResult
ManhattanDistanceTest<MPTraits>::
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
typename ManhattanDistanceTest<MPTraits>::TestResult
ManhattanDistanceTest<MPTraits>::
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
typename ManhattanDistanceTest<MPTraits>::TestResult
ManhattanDistanceTest<MPTraits>::
TestIndividualScaleCfg() {
  bool passed = true;
  std::string message = "";

  CfgType c1 = this->GetIndividualCfg();
  CfgType c2 = this->IndividualScaleCfg();
  double newLength = this->Distance(c1, c2);

  if (fabs(newLength - 10.0) > 1e-7) {
    passed = false;
    std::cout << "\n\tScaled distance is not the correct magnitude." << std::endl;
  }

  message = "\tFINISHED TestIndividualScaleCfg";
  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename ManhattanDistanceTest<MPTraits>::TestResult
ManhattanDistanceTest<MPTraits>::
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
typename ManhattanDistanceTest<MPTraits>::TestResult
ManhattanDistanceTest<MPTraits>::
TestGroupEdgeWeight() {
  bool passed = true;
  std::string message = "";

  //TODO::This method does not exist in distance metric method yet.

  message = "\tFINISHED TestGroupEdgeWeight";
  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename ManhattanDistanceTest<MPTraits>::TestResult
ManhattanDistanceTest<MPTraits>::
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
ManhattanDistanceTest<MPTraits>::
TrueIndividualCfgDistance() {
  CfgType cfg1 = this->GetIndividualCfg();
  CfgType cfg2 = this->GetIndividualCfg();

  // Manhattan distance should be 5 * PosDOF + 0.5 * OriDOF
  double trueDist;
  if (this->m_normalize) {
    const double diagonal = this->GetEnvironment()->GetBoundary()->GetMaxDist(
        1, 1);
    trueDist = std::pow(5 / diagonal, 1) * cfg2.PosDOF();
  } else {
    trueDist = std::pow(5, 1) * cfg2.PosDOF();
  }
  trueDist += std::pow(0.5, 1) * (cfg2.DOF() - cfg2.PosDOF());
  trueDist = std::pow(trueDist, 1);

  return trueDist;
}

/*--------------------------------------------------------------------*/
#endif
