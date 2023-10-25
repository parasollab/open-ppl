#ifndef PPL_MP_PROBLEM_TESTS_H_
#define PPL_MP_PROBLEM_TESTS_H_

#include "MPProblem/MPProblem.h"    //src

#include <string>
#include <utility>

class MPProblemTests : public MPProblem {
  public:
    ///@name LocalTypes
    ///@{

    typedef std::pair<bool,std::string> TestResult;

    ///@}
    ///@name Construction
    ///@{

    MPProblemTests();

    MPProblemTests(const std::string& _xmlFile);

    virtual ~MPProblemTests();

    ///@}
    ///@name Interface
    ///@{

    TestResult RunTest();

    ///@}
};

#endif
