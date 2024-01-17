#ifndef PPL_MPLIBRARY_TEST_H_
#define PPL_MPLIBRARY_TEST_H_

#include "MPLibrary/MPLibrary.h"    //src

#include "DistanceMetrics/DistanceMetricMethodTest.h"
#include "ValidityCheckers/ValidityCheckerMethodTest.h"
#include "NeighborhoodFinders/NeighborhoodFinderMethodTest.h"
#include "Samplers/SamplerMethodTest.h"
#include "LocalPlanners/LocalPlannerMethodTest.h"
#include "Extenders/ExtenderMethodTest.h"
#include "PathModifiers/PathModifierMethodTest.h"
#include "Connectors/ConnectorMethodTest.h"
#include "Metrics/MetricMethodTest.h"
#include "MapEvaluators/MapEvaluatorMethodTest.h"

#include "ValidityCheckers/CollisionDetection/CollisionDetectionMethodTest.h"
#include "ValidityCheckers/CollisionDetection/BoundingSpheresCollisionDetectionTest.h"
#include "ValidityCheckers/CollisionDetection/InsideSpheresCollisionDetectionTest.h"

#include "MPStrategies/MPStrategyMethodTest.h"
#include "MPStrategies/DynamicRegionRRTTest.h"


template <typename MPTraits>
class MPLibraryTests : public MPLibraryType<MPTraits> {
  public:
    ///@LocalTypes
    ///@{

    typedef MPLibraryType<MPTraits>    MPLibrary;
    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Method Set Types
    ///@{

    typedef MethodSet<MPTraits, DistanceMetricMethodTest<MPTraits>> DistanceMetricTestSet;
    typedef MethodSet<MPTraits, ValidityCheckerMethodTest<MPTraits>>
      ValidityCheckerTestSet;
    typedef MethodSet<MPTraits, NeighborhoodFinderMethodTest<MPTraits>>
      NeighborhoodFinderTestSet;
    typedef MethodSet<MPTraits, SamplerMethodTest<MPTraits>>        SamplerTestSet;
    typedef MethodSet<MPTraits, LocalPlannerMethodTest<MPTraits>>   LocalPlannerTestSet;
    typedef MethodSet<MPTraits, ExtenderMethodTest<MPTraits>>       ExtenderTestSet;
    typedef MethodSet<MPTraits, PathModifierMethodTest<MPTraits>>   PathModifierTestSet;
    typedef MethodSet<MPTraits, ConnectorMethodTest<MPTraits>>      ConnectorTestSet;
    typedef MethodSet<MPTraits, MetricMethodTest<MPTraits>>         MetricTestSet;
    typedef MethodSet<MPTraits, MapEvaluatorMethodTest<MPTraits>>   MapEvaluatorTestSet;

    ///@}
    ///@name Construction
    ///@{

    MPLibraryTests();

    MPLibraryTests(const std::string& _xmlFile);

    virtual ~MPLibraryTests();

    ///@}

  public:

    ///@name Helper Functions
    ///@{

    void InitializeMethodSets();

    void InitializeCollisionDetectionMethodTests();

    void InitializeMPStrategyMethodTests();


    ///@name XML Helpers
    ///@{

    void ReadXMLFile(const std::string& _filename);

    bool ParseChild(XMLNode& _node);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_xmlFile{""};
    bool verbose{true};
    std::map<std::string, CollisionDetectionMethodTest*> m_collisionDetectionTests;
    std::map<std::string, MPStrategyMethodTest<MPTraits>*> m_mpStrategyTests;

    ///@}
    ///@name Method Sets
    ///@{
    /// Method sets hold and offer access to the motion planning objects of the
    /// corresponding type.

    DistanceMetricTestSet*     m_distanceMetricTests{nullptr};
    ValidityCheckerTestSet*    m_validityCheckerTests{nullptr};
    NeighborhoodFinderTestSet* m_neighborhoodFinderTests{nullptr};
    SamplerTestSet*            m_samplerTests{nullptr};
    LocalPlannerTestSet*       m_localPlannerTests{nullptr};
    ExtenderTestSet*           m_extenderTests{nullptr};
    PathModifierTestSet*       m_pathModifierTests{nullptr};
    ConnectorTestSet*          m_connectorTests{nullptr};
    MetricTestSet*             m_metricTests{nullptr};
    MapEvaluatorTestSet*       m_mapEvaluatorTests{nullptr};

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
MPLibraryTests<MPTraits>::MPLibraryTests() {
  InitializeMethodSets();
}

template <typename MPTraits>
MPLibraryTests<MPTraits>::MPLibraryTests(const std::string& _xmlFile) : MPLibraryType<MPTraits>(_xmlFile),
  m_xmlFile(_xmlFile) {
    InitializeMethodSets();
    ReadXMLFile(_xmlFile);
  }

template <typename MPTraits>
MPLibraryTests<MPTraits>::~MPLibraryTests() {}

/*----------------------------- Interface ----------------------------*/

/*-------------------------- Helper Functions ------------------------*/

template <typename MPTraits>
void MPLibraryTests<MPTraits>::InitializeMethodSets() {
  m_distanceMetricTests = new DistanceMetricTestSet(this,
      typename MPTraits::DistanceMetricMethodList(), "DistanceMetrics");
  m_validityCheckerTests = new ValidityCheckerTestSet(this,
      typename MPTraits::ValidityCheckerMethodList(), "ValidityCheckers");
  m_neighborhoodFinderTests = new NeighborhoodFinderTestSet(this,
      typename MPTraits::NeighborhoodFinderMethodList(), "NeighborhoodFinders");
  m_samplerTests = new SamplerTestSet(this,
      typename MPTraits::SamplerMethodList(), "Samplers");
   m_localPlannerTests = new LocalPlannerTestSet(this,
       typename MPTraits::LocalPlannerMethodList(), "LocalPlanners");
  m_extenderTests = new ExtenderTestSet(this,
      typename MPTraits::ExtenderMethodList(), "Extenders");
  m_pathModifierTests = new PathModifierTestSet(this,
      typename MPTraits::PathModifierMethodList(), "PathModifiers");
  m_connectorTests = new ConnectorTestSet(this,
      typename MPTraits::ConnectorMethodList(), "Connectors");
  m_metricTests = new MetricTestSet(this,
      typename MPTraits::MetricMethodList(), "Metrics");
  m_mapEvaluatorTests = new MapEvaluatorTestSet(this,
      typename MPTraits::MapEvaluatorMethodList(), "MapEvaluators");
}

template<typename MPTraits>
void MPLibraryTests<MPTraits>::InitializeCollisionDetectionMethodTests() {
  // TODO: add addition collision detection methods here
  BoundingSpheresCollisionDetectionTest* boundingSpheres = nullptr;
  boundingSpheres = new BoundingSpheresCollisionDetectionTest(this->GetMPProblem());
  m_collisionDetectionTests["Bounding Spheres"] = boundingSpheres;

  InsideSpheresCollisionDetectionTest* insideSpheres = nullptr;
  insideSpheres = new InsideSpheresCollisionDetectionTest(this->GetMPProblem());
  m_collisionDetectionTests["Inside Spheres"] = insideSpheres;

}

template<typename MPTraits>
void
MPLibraryTests<MPTraits>::
InitializeMPStrategyMethodTests() {
  // TODO: add additional MPStrategy methods here
  XMLNode drrrt(m_xmlFile, "DynamicRegionRRT");
  DynamicRegionRRTTest<MPTraits>* dynamicRegionRRT = nullptr;
  dynamicRegionRRT = new DynamicRegionRRTTest<MPTraits>(drrrt);
  m_mpStrategyTests["DynamicRegionRRT"] = dynamicRegionRRT;

}

/*---------------------------- XML Helpers -----------------------------------*/

template <typename MPTraits>
void
MPLibraryTests<MPTraits>::
ReadXMLFile(const std::string& _filename) {
  // Open the XML and get the root node.
  XMLNode mpNode(_filename, "MotionPlanning");

  // Find the 'MPLibrary' node.
  XMLNode* planningLibrary = nullptr;
  for(auto& child : mpNode)
    if(child.Name() == "Library")
      planningLibrary = &child;

  // Throw exception if we can't find it.
  if(!planningLibrary)
    throw ParseException(WHERE) << "Cannot find MPLibrary node in XML file '"
      << _filename << "'.";

  // Parse the library node to set algorithms and parameters.
  for(auto& child : *planningLibrary)
    ParseChild(child);

}


template <typename MPTraits>
bool
MPLibraryTests<MPTraits>::
ParseChild(XMLNode& _node) {
  if(_node.Name() == "DistanceMetrics") {
    m_distanceMetricTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "ValidityCheckers") {
    m_validityCheckerTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "NeighborhoodFinders") {
    m_neighborhoodFinderTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Samplers") {
    m_samplerTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "LocalPlanners") {
    m_localPlannerTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Extenders") {
    m_extenderTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "PathModifiers") {
    m_pathModifierTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Connectors") {
    m_connectorTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Metrics") {
    m_metricTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "MapEvaluators") {
    m_mapEvaluatorTests->ParseXML(_node);
    return true;
  }
  else
    return false;
}

/*-------------------------------- Debugging ---------------------------------*/
/*--------------------------------------------------------------------*/
#endif
