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
/// @tparam MPTraits Motion planning universe
///
/// The MPBaseObject is an abstract class which all algorithm abstractions in
/// PMPL extend themselves off of. It essentially composes a class name
/// @c m_name, a unique label @c m_label, and provides access to the MPProblem.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class MPBaseObject {

  public:

    ////////////////////////////////////////////////////////////////////////////
    /// @name MPBaseObject Typedefs
    /// @{

    typedef typename MPTraits::MPProblemType MPProblemType;
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

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name Constructor, Destructor
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @param _problem Global MPProblem
    /// @param _label ID of the object, i.e., user defined label
    /// @param _name Name of the object, i.e., derived class name
    /// @param _debug Turn debug output on or off
    MPBaseObject(MPProblemType* _problem = NULL, const string& _label = "",
        const string& _name = "", bool _debug = false) :
      m_name(_name), m_debug(_debug), m_label(_label), m_problem(_problem) {
      };
    ////////////////////////////////////////////////////////////////////////////
    /// @param _problem Global MPProblem
    /// @param _node XMLNode to parse for this object
    MPBaseObject(MPProblemType* _problem, XMLNode& _node) :
      m_problem(_problem) {
        ParseXML(_node);
      };

    virtual ~MPBaseObject() = default;

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name I/O
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Parse XML node
    /// @param _node XML node
    ///
    /// Parse XML node. By default every MPBaseObject requires a label and
    /// optionally loads a debug parameter.
    virtual void ParseXML(XMLNode& _node) {
      m_label = _node.Read("label", true, "", "Label Identifier");
      m_debug = _node.Read("debug", false, false,
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

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name MPBaseObject Accessors

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

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name MPBaseObject Accessors to MPProblem Objects
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @return Environment pointer
    Environment* GetEnvironment() const {return m_problem->GetEnvironment();}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Roadmap pointer
    RoadmapType* GetRoadmap() const {return m_problem->GetRoadmap();}
    ////////////////////////////////////////////////////////////////////////////
    /// @return BlockRoadmap pointer
    RoadmapType* GetBlockRoadmap() const {return m_problem->GetBlockRoadmap();}
    ////////////////////////////////////////////////////////////////////////////
    /// @return StatClass pointer
    StatClass* GetStatClass() const {return m_problem->GetStatClass();}

    ////////////////////////////////////////////////////////////////////////////
    /// @param _dm Label
    /// @return DistanceMetric pointer
    DistanceMetricPointer GetDistanceMetric(const string& _dm) const {
      return m_problem->GetDistanceMetric(_dm);
    }
    ////////////////////////////////////////////////////////////////////////////
    /// @param _vc Label
    /// @return ValidityChecker pointer
    ValidityCheckerPointer GetValidityChecker(const string& _vc) const {
      return m_problem->GetValidityChecker(_vc);
    }
    ////////////////////////////////////////////////////////////////////////////
    /// @param _nf Label
    /// @return NeighborhoodFinder pointer
    NeighborhoodFinderPointer GetNeighborhoodFinder(const string& _nf) const {
      return m_problem->GetNeighborhoodFinder(_nf);
    }
    ////////////////////////////////////////////////////////////////////////////
    /// @param _s Label
    /// @return Sampler pointer
    SamplerPointer GetSampler(const string& _s) const {
      return m_problem->GetSampler(_s);
    }
    ////////////////////////////////////////////////////////////////////////////
    /// @param _lp Label
    /// @return LocalPlanner pointer
    LocalPlannerPointer GetLocalPlanner(const string& _lp) const {
      return m_problem->GetLocalPlanner(_lp);
    }
    ////////////////////////////////////////////////////////////////////////////
    /// @param _e Label
    /// @return Extender pointer
    ExtenderPointer GetExtender(const string& _e) const {
      return m_problem->GetExtender(_e);
    }
    ////////////////////////////////////////////////////////////////////////////
    /// @param _pm Label
    /// @return PathModifier pointer
    PathModifierPointer GetPathModifier(const string& _pm) const {
      return m_problem->GetPathModifier(_pm);
    }
    ////////////////////////////////////////////////////////////////////////////
    /// @param _c Label
    /// @return Connector pointer
    ConnectorPointer GetConnector(const string& _c) const {
      return m_problem->GetConnector(_c);
    }
    ////////////////////////////////////////////////////////////////////////////
    /// @param _m Label
    /// @return Metric pointer
    MetricPointer GetMetric(const string& _m) const {
      return m_problem->GetMetric(_m);
    }
    ////////////////////////////////////////////////////////////////////////////
    /// @param _me Label
    /// @return MapEvaluator pointer
    MapEvaluatorPointer GetMapEvaluator(const string& _me) const {
      return m_problem->GetMapEvaluator(_me);
    }
    ////////////////////////////////////////////////////////////////////////////
    /// @param _mps Label
    /// @return MPStrategy pointer
    MPStrategyPointer GetMPStrategy(const string& _mps) const {
      return m_problem->GetMPStrategy(_mps);
    }

    /// @}
    ////////////////////////////////////////////////////////////////////////////

  protected:

    ////////////////////////////////////////////////////////////////////////////
    /// @return Label
    const string& GetLabel() const {return m_label;}

    ////////////////////////////////////////////////////////////////////////////
    /// @param _s Class name
    void SetName(const string& _s) {m_name  = _s;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return base file name from MPProblem
    const string& GetBaseFilename() const {return m_problem->GetBaseFilename();}

    string m_name;            ///< Class name
    bool m_debug;             ///< Debug statements on or off

  private:

    string m_label;           ///< Unique identifier of object
    MPProblemType* m_problem; ///< Shared pointer to MPProblem object

    template<typename T, typename U> friend class MethodSet;
};

#endif
