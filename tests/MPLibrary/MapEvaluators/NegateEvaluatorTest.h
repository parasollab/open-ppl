#ifndef PPL_NEGATE_EVALUATOR_TEST_H_
#define PPL_NEGATE_EVALUATOR_TEST_H_

#include "MPLibrary/MapEvaluators/NegateEvaluator.h"
#include "Testing/MPLibrary/MapEvaluators/MapEvaluatorMethodTest.h"

template <typename MPTraits>
class NegateEvaluatorTest : virtual public NegateEvaluator<MPTraits>,
                            public MapEvaluatorMethodTest<MPTraits> {
  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    NegateEvaluatorTest();

    NegateEvaluatorTest(XMLNode& _node);

    ~NegateEvaluatorTest();

    ///@}

  private:

    ///@name Interface Test Function
    ///@{

    virtual TestResult MainFunctionTest() override;

    ///@}
};

/*--------------------------- Construction ---------------------------*/
template <typename MPTraits>
NegateEvaluatorTest<MPTraits>::
NegateEvaluatorTest() : NegateEvaluator<MPTraits>() {}

template <typename MPTraits>
NegateEvaluatorTest<MPTraits>::
NegateEvaluatorTest(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node), 
                                      NegateEvaluator<MPTraits>(_node) {}

template <typename MPTraits>
NegateEvaluatorTest<MPTraits>::
~NegateEvaluatorTest() {}

/*----------------------Interface Test Functions ---------------------*/

template <typename MPTraits>
typename NegateEvaluatorTest<MPTraits>::TestResult
NegateEvaluatorTest<MPTraits>::
MainFunctionTest() {

  bool passed = true;
  std::string message = "";

  // Get the result of running the base map evaluator.
  auto me = this->GetMapEvaluator(this->m_meLabel);
  auto result = (*me)();

  // Make sure that the result of the negate evaluator is the opposite.
  passed = this->operator()() != result;

  if (passed) {
    message = "MainFunctionTest::PASSED!\n";
  } else {
    message = "MainFunctionTest::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

#endif
