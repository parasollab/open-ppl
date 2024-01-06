#ifndef PPL_ROTATION_THEN_TRANSLATION_TEST_H_
#define PPL_ROTATION_THEN_TRANSLATION_TEST_H_

#include "MPLibrary/Extenders/RotationThenTranslation.h"
#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/LocalPlanners/GroupLPOutput.h"
#include "Testing/MPLibrary/Extenders/ExtenderMethodTest.h"

template <typename MPTraits>
class RotationThenTranslationTest : virtual public RotationThenTranslation<MPTraits>,
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
    
    RotationThenTranslationTest();

    RotationThenTranslationTest(XMLNode& _node);

    virtual ~RotationThenTranslationTest();
    
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
RotationThenTranslationTest<MPTraits>::
RotationThenTranslationTest() : RotationThenTranslation<MPTraits>() {}

template<typename MPTraits>
RotationThenTranslationTest<MPTraits>:: 
RotationThenTranslationTest(XMLNode& _node) : ExtenderMethod<MPTraits>(_node),
                                    RotationThenTranslation<MPTraits>(_node) {}

template <typename MPTraits>
RotationThenTranslationTest<MPTraits>::
~RotationThenTranslationTest() {}

/*--------------------- Test Interface Functions ---------------------*/

template<typename MPTraits>
typename RotationThenTranslationTest<MPTraits>::TestResult
RotationThenTranslationTest<MPTraits>::
IndividualRobotExtendTests() {
  bool passed = true;
  std::string message = "";

  auto dm = this->GetDistanceMetric("euclidean");
  CfgType c = this->GetIndividualCfg();

  // TEST: Test scenario where rotation first causes collision
  c[0] = -10.6;
  c[1] = 0.9;
  c[5] = 0.0;
  const CfgType startCfg = CfgType(c);
  c[0] = 10.6;
  c[1] = 0.9;
  c[5] = 0.5;
  const CfgType endCfg = CfgType(c);
  //New CfgType and lpOutput for storing the result of Extend

  CfgType newCfg;
  LPOutput<MPTraits> lpOuput;

  // Rotate and Extend from startCfg to endCfg
  bool result = this->Extend(startCfg, endCfg, newCfg, lpOuput);
  
  // Check that the configuration 
  bool reached = result && dm->Distance(newCfg, endCfg) < 0.01;

  if (reached) {
    passed = false;
    message = message + "\n\tThe configuration successfully rotated and extended when it was not supposed to extend successfully.\n";
  }

  // TEST: Test scenario where simultaneous rotation and translation (or translation then rotation) would
  // result in collision with object
  c[0] = 10.6;
  c[1] = 0.0;
  c[5] = 0.0;
  const CfgType startCfg2 = CfgType(c);
  c[0] = 10.6;
  c[1] = 3.0;
  c[5] = 0.5;
  const CfgType endCfg2 = CfgType(c);
  result = this->Extend(startCfg2, endCfg2, newCfg, lpOuput);
  
  // Check that the configuration reached the goal
  reached = result && dm->Distance(newCfg, endCfg2) < 0.01;

  if (!reached) {
    passed = false;
    message = message + "\n\tThe configuration successfully rotated and extended when it was not supposed to rotate successfully.\n";
  }

  if (passed) {
    message = "IndividualRobotExtend::PASSED!\n";
  } else {
    message = "IndividualRobotExtend::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename RotationThenTranslationTest<MPTraits>::TestResult
RotationThenTranslationTest<MPTraits>::
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
