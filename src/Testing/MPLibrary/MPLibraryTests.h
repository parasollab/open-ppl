#ifndef PPL_M_P_LIBRARY_TEST_H_
#define PPL_M_P_LIBRARY_TEST_H_

#include "MPLibrary/MPLibrary.h"
#include "Testing/TestBaseObject.h"

template <typename TestTraits>
class MPLibraryTests : public MPLibraryType<TestTraits>, public TestBaseObject {
  public:
    ///@LocalTypes
    ///@{

    typedef MPLibraryType<TestTraits>    MPLibrary;
    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Method Set Types
    ///@{

    ///@}
    ///@name Construction
    ///@{

    MPLibraryTests();

    MPLibraryTests(const std::string& _xmlFile);

    virtual ~MPLibraryTests();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}

};

/*--------------------------- Construction ---------------------------*/

template <typename TestTraits>
MPLibraryTests<TestTraits>::
MPLibraryTests() {}

template <typename TestTraits>
MPLibraryTests<TestTraits>::
MPLibraryTests(const std::string& _xmlFile) : MPLibraryType<TestTraits>(_xmlFile) {}

template <typename TestTraits>
MPLibraryTests<TestTraits>::
~MPLibraryTests() {}

/*----------------------------- Interface ----------------------------*/

template <typename TestTraits>
typename MPLibraryTests<TestTraits>::TestResult
MPLibraryTests<TestTraits>::
RunTest() {
  return TestResult();
}

/*--------------------------------------------------------------------*/
#endif
