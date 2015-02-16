#ifndef MP_BASE_OBJECT_H_
#define MP_BASE_OBJECT_H_

#include <iostream>
#include <string>
using namespace std;

#include "Utilities/IOUtils.h"

class Environment;
template<class MPTraits> class Roadmap;
class StatClass;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningUniverse
/// @brief Base class of all algorithm abstractions in PMPL.
///
/// The MPBaseObject is an abstract class which all algorithm abstractions in
/// PMPL extend themselves off of. It essentially composes a class name
/// @c m_name, a unique label @c m_label, and provides access to the MPProblem.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class MPBaseObject {

  public:

    typedef typename MPTraits::MPProblemType MPProblemType;

    MPBaseObject(MPProblemType* _problem = NULL, const string& _label = "", const string& _name = "", bool _debug = false) :
      m_name(_name), m_debug(_debug), m_label(_label), m_problem(_problem) {};
    MPBaseObject(MPProblemType* _problem, XMLNodeReader& _node, const string& _name="") :
      m_name(_name), m_problem(_problem) {
        ParseXML(_node);
      };
    virtual ~MPBaseObject() {}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Parse XML node
    /// @param _node XML node
    ///
    /// Parse XML node. By default every MPBaseObject requires a label and
    /// optionally loads a debug parameter.
    virtual void ParseXML(XMLNodeReader& _node) {
      m_label = _node.stringXMLParameter("label", true, "", "Label Identifier");
      m_debug = _node.boolXMLParameter("debug", false, false,
          "Run-time debug on(true)/off(false)");
    };

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Print values of object
    /// @param _os ostream to print values to
    ///
    /// Print values of object to ostream. By default name and label are output.
    virtual void Print(ostream& _os) const {
      _os << this->GetNameAndLabel() << endl;
    };

    ////////////////////////////////////////////////////////////////////////////
    /// @return MPProblem object
    MPProblemType* GetMPProblem() const {return m_problem;}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _m MPProblem object
    virtual void SetMPProblem(MPProblemType* _m) {m_problem = _m;}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Get unique string identifier to object
    /// @return unique identifier "m_name::m_label"
    string GetNameAndLabel() const {return m_name + "::" + m_label;}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _s label
    void SetLabel(const string& _s) {m_label = _s;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return debug value
    bool GetDebug() const {return m_debug;}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _d debug value
    void SetDebug(bool _d) {m_debug = _d;}

    ////////////////////////////////////////////////////////////////////////////
    // Quick Accessors to MPPrblem objects
    ////////////////////////////////////////////////////////////////////////////

    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::SamplerPointer SamplerPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::ExtenderPointer ExtenderPointer;
    typedef typename MPProblemType::PathModifierPointer PathModifierPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
    typedef typename MPProblemType::MetricPointer MetricPointer;
    typedef typename MPProblemType::MapEvaluatorPointer MapEvaluatorPointer;
    typedef typename MPProblemType::MPStrategyPointer MPStrategyPointer;

    ////////////////////////////////////////////////////////////////////////////
    /// @return Environment pointer
    Environment* GetEnvironment() {return m_problem->GetEnvironment();}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Roadmap pointer
    RoadmapType* GetRoadmap() {return m_problem->GetRoadmap();}
    ////////////////////////////////////////////////////////////////////////////
    /// @return BlockRoadmap pointer
    RoadmapType* GetBlockRoadmap() {return m_problem->GetBlockRoadmap();}
    ////////////////////////////////////////////////////////////////////////////
    /// @return StatClass pointer
    StatClass* GetStatClass() {return m_problem->GetStatClass();}

    ////////////////////////////////////////////////////////////////////////////
    /// @param _dm label
    /// @return DistanceMetric pointer
    DistanceMetricPointer GetDistanceMetric(const string& _dm) {return m_problem->GetDistanceMetric(_dm);}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _vc label
    /// @return ValidityChecker pointer
    ValidityCheckerPointer GetValidityChecker(const string& _vc) {return m_problem->GetValidityChecker(_vc);}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _nf label
    /// @return NeighborhoodFinder pointer
    NeighborhoodFinderPointer GetNeighborhoodFinder(const string& _nf) {return m_problem->GetNeighborhoodFinder(_nf);}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _s label
    /// @return Sampler pointer
    SamplerPointer GetSampler(const string& _s) {return m_problem->GetSampler(_s);}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _lp label
    /// @return LocalPlanner pointer
    LocalPlannerPointer GetLocalPlanner(const string& _lp) {return m_problem->GetLocalPlanner(_lp);}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _e label
    /// @return Extender pointer
    ExtenderPointer GetExtender(const string& _e) {return m_problem->GetLocalPlanner(_e);}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _pm label
    /// @return PathModifier pointer
    PathModifierPointer GetPathModifier(const string& _pm) {return m_problem->GetLocalPlanner(_pm);}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _c label
    /// @return Connector pointer
    ConnectorPointer GetConnector(const string& _c) {return m_problem->GetConnector(_c);}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _m label
    /// @return Metric pointer
    MetricPointer GetMetric(const string& _m) {return m_problem->GetMetric(_m);}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _me label
    /// @return MapEvaluator pointer
    MapEvaluatorPointer GetMapEvaluator(const string& _me) {return m_problem->GetMapEvaluator(_me);}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _mps label
    /// @return MPStrategy pointer
    MPStrategyPointer GetMPStrategy(const string& _mps) {return m_problem->GetMPStrategy(_mps);}

  protected:

    ////////////////////////////////////////////////////////////////////////////
    /// @return label
    const string& GetLabel() const {return m_label;}

    ////////////////////////////////////////////////////////////////////////////
    /// @param _s class name
    void SetName(const string& _s) {m_name  = _s;}

    string m_name; ///< Class name
    bool m_debug; ///< Debug statements on or off

    template<typename T, typename U> friend class MethodSet;

  private:

    string m_label; ///< Unique identifier of object
    MPProblemType* m_problem; ///< Shared pointer to MPProblem object
};

#endif
