#include "SimpleMotionEvaluatorTest.h"

/*--------------------------- Construction ---------------------------*/

SimpleMotionEvaluatorTest::
SimpleMotionEvaluatorTest() {}

SimpleMotionEvaluatorTest::
SimpleMotionEvaluatorTest(XMLNode& _node) : SimpleMotionEvaluator(_node) {}

SimpleMotionEvaluatorTest::
~SimpleMotionEvaluatorTest() {}

/*----------------------------- Interface ----------------------------*/

typename SimpleMotionEvaluatorTest::TestResult
SimpleMotionEvaluatorTest::
RunTest() {
  return TestResult();
}

/*--------------------------------------------------------------------*/
