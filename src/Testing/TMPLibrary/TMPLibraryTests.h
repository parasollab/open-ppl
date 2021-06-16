#ifndef PPL_TMP_LIBRARY_TESTS_H_
#define PPL_TMP_LIBRARY_TESTS_H_

#include "Testing/TestBaseObject.h"
#include "TMPLibrary/TMPLibrary.h"

class TMPLibraryTests : public TMPLibrary, public TestBaseObject {
  public:
    ///@name LocalTypes
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    TMPLibraryTests();

    TMPLibraryTests(const std::string& _xmlFile);

    virtual ~TMPLibraryTests();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}
};

#endif
