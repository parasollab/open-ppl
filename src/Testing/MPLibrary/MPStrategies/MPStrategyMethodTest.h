#ifndef PPL_MP_STRATEGY_METHOD_TEST_H_
#define PPL_MP_STRATEGY_METHOD_TEST_H_

#include "MPLibrary/MPStrategies/MPStrategyMethod.h"
#include "Testing/TestBaseObject.h"

template <typename MPTraits>
class MPStrategyMethodTest : virtual public MPStrategyMethod<MPTraits>,
                             public TestBaseObject {

  public: 
    ///@name Local Types
    ///@{

    typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    MPStrategyMethodTest();

    ~MPStrategyMethodTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}

  protected:
    ///@name Interface Test Function
    ///@{

    // This will vary for each method, so this is a generic test method
    virtual TestResult TestStrategy() = 0;

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
MPStrategyMethodTest<MPTraits>::
MPStrategyMethodTest() : MPStrategyMethod<MPTraits>() { }

template <typename MPTraits>
MPStrategyMethodTest<MPTraits>::
~MPStrategyMethodTest() { }

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>
typename MPStrategyMethodTest<MPTraits>::TestResult
MPStrategyMethodTest<MPTraits>::
RunTest() {
  
  bool passed = true;
  std::string message = "";

  auto result = TestStrategy();
  passed = passed and result.first;
  message = message + result.second;

  return std::make_pair(passed,message);
}

/*--------------------------------------------------------------------*/

#endif
