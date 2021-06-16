#include "TMPLibraryTests.h"

/*--------------------------- Construction ---------------------------*/

TMPLibraryTests::
TMPLibraryTests() {}

TMPLibraryTests::
TMPLibraryTests(const std::string& _xmlFile) {
  //TODO::Parse XML and construct test classes for everything included.
}

TMPLibraryTests::
~TMPLibraryTests() {}

/*----------------------------- Interface ----------------------------*/

typename TMPLibraryTests::TestResult
TMPLibraryTests::
RunTest() {
  return TestResult();
}

/*--------------------------------------------------------------------*/
