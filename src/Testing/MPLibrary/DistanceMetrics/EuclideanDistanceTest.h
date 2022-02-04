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
EuclideanDistanceTest<MPTraits>::
EuclideanDistanceTest(bool _normalize) : EuclideanDistance<MPTraits>(_normalize) {}

template<typename MPTraits>
EuclideanDistanceTest<MPTraits>:: 
EuclideanDistanceTest(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node),
                                        EuclideanDistance<MPTraits>(_node) {}

template<typename MPTraits>
EuclideanDistanceTest<MPTraits>::
~EuclideanDistanceTest() {}

/*---------------------------- Interface -----------------------------*/

template <typename MPTraits>
double
EuclideanDistanceTest<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  return EuclideanDistance<MPTraits>::Distance(_c1, _c2);
}

/*--------------------- Test Interface Functions ---------------------*/

template<typename MPTraits>
typename EuclideanDistanceTest<MPTraits>::TestResult
EuclideanDistanceTest<MPTraits>::
TestIndividualCfgDistance() {
  bool passed = true;
  std::string message = "";

  // todo test normalized also

  auto cfg1 = this->GetIndividualCfg();
  auto cfg2 = this->GetIndividualCfg();

  // Euclidean distance should be 5 * PosDOF + 0.5 * OrientationDOF
  double trueDist = std::pow(5, this->m_r1) * cfg2.PosDOF() + std::pow(0.5, this->m_r2) * (cfg2.DOF() - cfg2.PosDOF());
  trueDist = std::pow(trueDist, this->m_r3);
  auto dist = this->IndividualCfgDistance();
  if (fabs(dist - trueDist) > 1e-7) {
    passed = false;
    message = message + "\n\tIncorrect non-zero distance returned between different configurations.\n";
  }

  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename EuclideanDistanceTest<MPTraits>::TestResult
EuclideanDistanceTest<MPTraits>::
TestIndividualEdgeWeight() {
  return std::make_pair(true, "");
}

template<typename MPTraits>
typename EuclideanDistanceTest<MPTraits>::TestResult
EuclideanDistanceTest<MPTraits>::
TestIndividualScaleCfg() {
  return std::make_pair(true, "");
}

template<typename MPTraits>
typename EuclideanDistanceTest<MPTraits>::TestResult
EuclideanDistanceTest<MPTraits>::
TestGroupCfgDistance() {
  return std::make_pair(true, "");
}

template<typename MPTraits>
typename EuclideanDistanceTest<MPTraits>::TestResult
EuclideanDistanceTest<MPTraits>::
TestGroupEdgeWeight() {
  return std::make_pair(true, "");
}

template<typename MPTraits>
typename EuclideanDistanceTest<MPTraits>::TestResult
EuclideanDistanceTest<MPTraits>::
TestGroupScaleCfg() {
  return std::make_pair(true, "");
}

/*--------------------------------------------------------------------*/
#endif
