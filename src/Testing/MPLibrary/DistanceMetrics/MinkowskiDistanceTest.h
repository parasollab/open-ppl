#ifndef PPL_MINKOWSKI_DISTANCE_TEST_H_
#define PPL_MINKOWSKI_DISTANCE_TEST_H_

#include "Testing/MPLibrary/DistanceMetrics/DistanceMetricMethodTest.h"
#include "MPLibrary/DistanceMetrics/MinkowskiDistance.h"

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

    MinkowskiDistanceTest();
    MinkowskiDistanceTest(XMLNode& _node);
    ~MinkowskiDistanceTest();

    ///@}
    ///@name Interface 
    ///@{

    virtual double Distance(const CfgType& _c1, const CfgType& _c2) override;

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
};

/*--------------------------- Construction ---------------------------*/

template<typename MPTraits>
MinkowskiDistanceTest<MPTraits>::
MinkowskiDistanceTest() : MinkowskiDistance<MPTraits>() {}

template<typename MPTraits>
MinkowskiDistanceTest<MPTraits>:: 
MinkowskiDistanceTest(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node),
                                        MinkowskiDistance<MPTraits>(_node) {}

template<typename MPTraits>
MinkowskiDistanceTest<MPTraits>::
~MinkowskiDistanceTest() {}

/*---------------------------- Interface -----------------------------*/

template <typename MPTraits>
double
MinkowskiDistanceTest<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  return MinkowskiDistance<MPTraits>::Distance(_c1, _c2);
}

/*--------------------- Test Interface Functions ---------------------*/

template<typename MPTraits>
typename MinkowskiDistanceTest<MPTraits>::TestResult
MinkowskiDistanceTest<MPTraits>::
TestIndividualCfgDistance() {
  bool passed = true;
  std::string message = "";

  auto cfg1 = this->GetIndividualCfg();
  auto cfg2 = this->GetIndividualCfg();

  // Minkowski distance should be 5 * PosDOF + 0.5 * OrientationDOF
  double trueDist = 5 * cfg2.PosDOF() + 0.5 * (cfg2.DOF() - cfg2.PosDOF());
  auto dist = this->IndividualCfgDistance();
  if (fabs(dist - trueDist) > 1e-7) {
    passed = false;
    message = message + "\n\tIncorrect non-zero distance returned between different configurations.\n";
  }

  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename MinkowskiDistanceTest<MPTraits>::TestResult
MinkowskiDistanceTest<MPTraits>::
TestIndividualEdgeWeight() {
  return std::make_pair(true, "");
}

template<typename MPTraits>
typename MinkowskiDistanceTest<MPTraits>::TestResult
MinkowskiDistanceTest<MPTraits>::
TestIndividualScaleCfg() {
  return std::make_pair(true, "");
}

template<typename MPTraits>
typename MinkowskiDistanceTest<MPTraits>::TestResult
MinkowskiDistanceTest<MPTraits>::
TestGroupCfgDistance() {
  return std::make_pair(true, "");
}

template<typename MPTraits>
typename MinkowskiDistanceTest<MPTraits>::TestResult
MinkowskiDistanceTest<MPTraits>::
TestGroupEdgeWeight() {
  return std::make_pair(true, "");
}

template<typename MPTraits>
typename MinkowskiDistanceTest<MPTraits>::TestResult
MinkowskiDistanceTest<MPTraits>::
TestGroupScaleCfg() {
  return std::make_pair(true, "");
}

/*--------------------------------------------------------------------*/
#endif
