#ifndef PPL_M_P_LIBRARY_TEST_H_
#define PPL_M_P_LIBRARY_TEST_H_

#include "MPLibrary/MPLibrary.h"
#include "Testing/TestBaseObject.h"

template <typename MPTraits>
class MPLibraryTests : public MPLibraryType<MPTraits>, public TestBaseObject {
  public:
    ///@LocalTypes
    ///@{

    typedef MPLibraryType<MPTraits>    MPLibrary;
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

template <typename MPTraits>
MPLibraryTests<MPTraits>::
MPLibraryTests() {}

template <typename MPTraits>
MPLibraryTests<MPTraits>::
MPLibraryTests(const std::string& _xmlFile) : MPLibraryType<MPTraits>(_xmlFile) {}

template <typename MPTraits>
MPLibraryTests<MPTraits>::
~MPLibraryTests() {}

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>
typename MPLibraryTests<MPTraits>::TestResult
MPLibraryTests<MPTraits>::
RunTest() {
  return TestResult();
}

/*--------------------------------------------------------------------*/
#endif
