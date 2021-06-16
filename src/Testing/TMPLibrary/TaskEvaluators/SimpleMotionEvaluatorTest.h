#ifndef PPL_SIMPLE_MOTION_EVALUATOR_TEST_H_
#define PPL_SIMPLE_MOTION_EVALUATOR_TEST_H_

#include "Testing/TestBaseObject.h"
#include "TMPLibrary/TaskEvaluators/SimpleMotionEvaluator.h"

class SimpleMotionEvaluatorTest : public SimpleMotionEvaluator, public TestBaseObject {
  public: 
    ///@name LocalTypes
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    SimpleMotionEvaluatorTest();

    SimpleMotionEvaluatorTest(XMLNode& _node);

    virtual ~SimpleMotionEvaluatorTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}
};

#endif
