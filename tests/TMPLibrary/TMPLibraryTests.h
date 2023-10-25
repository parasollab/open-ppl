#ifndef PPL_TMP_LIBRARY_TESTS_H_
#define PPL_TMP_LIBRARY_TESTS_H_

#include "TMPLibrary/TMPLibrary.h"  //src

#include "TaskAllocators/TaskAllocatorMethodTest.h"
#include "TaskDecomposers/TaskDecomposerMethodTest.h"
#include "TaskEvaluators/TaskEvaluatorMethodTest.h"

class TMPLibraryTests : public TMPLibrary {

  public:

    ///@name LocalTypes
    ///@{

    typedef std::pair<bool,std::string> TestResult;

    ///@}
    ///@name Method Set Types
    ///@{

    typedef TMPMethodSet<TaskAllocatorMethodTest>      TaskAllocatorMethodTestSet;
    typedef TMPMethodSet<TaskDecomposerMethodTest>     TaskDecomposerMethodTestSet;
    typedef TMPMethodSet<TaskEvaluatorMethodTest>      TaskEvaluatorMethodTestSet;
    //typedef TMPMethodSet<TMPStrategyMethod>        TMPStrategyMethodTestSet;
    //typedef TMPMethodSet<PoIPlacementMethod>       PoIPlacementMethodTestSet;
    //typedef TMPMethodSet<StateGraph>               StateGraphSet;

    ///@}
    ///@name Construction
    ///@{

    TMPLibraryTests();

    TMPLibraryTests(const std::string& _xmlFile);

    virtual ~TMPLibraryTests();

    ///@}

  public:

    ///@name Helper Functions
    ///@{

    void InitializeMethodSets();

    ///@}
    ///@name XML Helpers
    ///@{

    void ReadXMLFile(const std::string& _filename);

    bool ParseChild(XMLNode& _node);

    ///@} 
    ///@name Internal State
    ///@{

    bool verbose{true};

    ///@}
    ///@name TMPMethod Sets
    ///@{
    /// Method sets hold and offer access to the tmp planning objects of the
    /// corresponding type.

    TaskAllocatorMethodTestSet*   m_taskAllocatorTests;
    TaskDecomposerMethodTestSet*  m_taskDecomposerTests;
    TaskEvaluatorMethodTestSet*   m_taskEvaluatorTests;
    //TMPStrategyMethodTestSet*     m_tmpStrategieTests;
    //PoIPlacementMethodTestSet*    m_poiPlacementMethodTests;
    //StateGraphTestSet*            m_stateGraphTests;

    ///@}
};

#endif
