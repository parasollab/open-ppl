#ifndef PPL_BASIC_EXTENDER_TEST_H_
#define PPL_BASIC_EXTENDER_TEST_H_

#include "MPLibrary/Extenders/BasicExtender.h"
#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/LocalPlanners/GroupLPOutput.h"
#include "Testing/MPLibrary/Extenders/ExtenderMethodTest.h"

template <typename MPTraits>
class BasicExtenderTest : virtual public BasicExtender<MPTraits>,
                          public ExtenderMethodTest<MPTraits> {

  public:

    ///@name Local Types
    ///@{
    
    typedef TestBaseObject::TestResult TestResult;
    
    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;

    ///@}
    ///@name Construction
    ///@{
    
    BasicExtenderTest();

    BasicExtenderTest(XMLNode& _node);

    virtual ~BasicExtenderTest();
    
    ///@}

  protected:

    ///@name Interface Test Function 
    ///@{
    
    virtual TestResult IndividualRobotExtendTests() override;

    virtual TestResult RobotGroupExtendTests() override;
    
    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
BasicExtenderTest<MPTraits>::
BasicExtenderTest() : BasicExtender<MPTraits>() {}

template<typename MPTraits>
BasicExtenderTest<MPTraits>:: 
BasicExtenderTest(XMLNode& _node) : ExtenderMethod<MPTraits>(_node),
                                    BasicExtender<MPTraits>(_node) {}

template <typename MPTraits>
BasicExtenderTest<MPTraits>::
~BasicExtenderTest() {}

/*--------------------- Test Interface Functions ---------------------*/

template<typename MPTraits>
typename BasicExtenderTest<MPTraits>::TestResult
BasicExtenderTest<MPTraits>::
IndividualRobotExtendTests() {
  bool passed = true;
  std::string message = "";

  // New CfgType and lpOutput for storing the result of Extend
  CfgType newCfg;
  LPOutput<MPTraits> lpOuput;

  // Extend by 10 in the x direction
  bool result = this->IndividualRobotExtend(newCfg, lpOuput);

  // Check that the newCfg is correct
  double maxDist = std::min(this->m_maxDist, 10.0);

  if (!result) {
    passed = false;
    message = message + "\n\tThe configuration was not successfully extended.\n";
  } else if (fabs(newCfg[0] - maxDist) > 1e-7) {
    passed = false;
    message = message + "\n\tThe configuration was not extended the correct distance.\n";
  }

  if (passed) {
    message = "IndividualRobotExtend::PASSED!\n";
  } else {
    message = "IndividualRobotExtend::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename BasicExtenderTest<MPTraits>::TestResult
BasicExtenderTest<MPTraits>::
RobotGroupExtendTests() {
  bool passed = true;
  std::string message = "";

  // TODO Need GroupCfg support

  if (passed) {
    message = "RobotGroupExtend::PASSED!\n";
  } else {
    message = "RobotGroupExtend::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

#endif
