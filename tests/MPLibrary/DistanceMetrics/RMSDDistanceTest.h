#ifndef PPL_RMSD_DISTANCE_TEST_H_
#define PPL_RMSD_DISTANCE_TEST_H_

#include "MPLibrary/DistanceMetrics/RMSDDistance.h"     //src
#include "DistanceMetricMethodTest.h"

template <typename MPTraits>
class RMSDDistanceTest : virtual public RMSDDistance<MPTraits>,
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

    RMSDDistanceTest();

    RMSDDistanceTest(XMLNode& _node);

    ~RMSDDistanceTest();

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
RMSDDistanceTest<MPTraits>::
RMSDDistanceTest() : RMSDDistance<MPTraits>() {}

template<typename MPTraits>
RMSDDistanceTest<MPTraits>:: 
RMSDDistanceTest(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node),
                                   RMSDDistance<MPTraits>(_node) {}

template<typename MPTraits>
RMSDDistanceTest<MPTraits>::
~RMSDDistanceTest() {}

/*--------------------- Test Interface Functions ---------------------*/

template<typename MPTraits>
typename RMSDDistanceTest<MPTraits>::TestResult
RMSDDistanceTest<MPTraits>::
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
typename RMSDDistanceTest<MPTraits>::TestResult
RMSDDistanceTest<MPTraits>::
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
typename RMSDDistanceTest<MPTraits>::TestResult
RMSDDistanceTest<MPTraits>::
TestIndividualScaleCfg() {
  bool passed = true;
  std::string message = "";

  // TODO: This method does not exist for RMSDDistance yet

  message = "\tFINISHED TestIndividualScaleCfg";
  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename RMSDDistanceTest<MPTraits>::TestResult
RMSDDistanceTest<MPTraits>::
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
typename RMSDDistanceTest<MPTraits>::TestResult
RMSDDistanceTest<MPTraits>::
TestGroupEdgeWeight() {
  bool passed = true;
  std::string message = "";

  //TODO::This method does not exist in distance metric method yet.

  message = "\tFINISHED TestGroupEdgeWeight";
  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename RMSDDistanceTest<MPTraits>::TestResult
RMSDDistanceTest<MPTraits>::
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
RMSDDistanceTest<MPTraits>::
TrueIndividualCfgDistance() {
  // The distance between two cfgs of a one-body system is always 0
  double distance = 0;
  return distance;
}

/*--------------------------------------------------------------------*/
#endif
