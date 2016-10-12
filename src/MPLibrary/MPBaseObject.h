#ifndef MP_BASE_OBJECT_H_
#define MP_BASE_OBJECT_H_

#include <iostream>
#include <string>
using namespace std;

#include "Utilities/IOUtils.h"
#include "Utilities/MethodSet.h"

class Environment;
template <typename MPTraits> class Roadmap;
class StatClass;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningUniverse
/// @brief Base class of all algorithm abstractions in PMPL.
///
/// The MPBaseObject is an abstract class which all algorithm abstractions in
/// PMPL extend themselves off of. It essentially composes a class name
/// @c m_name, a unique label @c m_label, and provides access to the MPProblem.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MPBaseObject {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::MPProblemType                  MPProblemType;
    typedef typename MPTraits::MPLibraryType            MPLibraryType;

    typedef typename MPProblemType::RoadmapType               RoadmapType;

    typedef typename MPLibraryType::SamplerPointer      SamplerPointer;
    typedef typename MPLibraryType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPLibraryType::ExtenderPointer     ExtenderPointer;
    typedef typename MPLibraryType::PathModifierPointer PathModifierPointer;
    typedef typename MPLibraryType::ConnectorPointer    ConnectorPointer;
    typedef typename MPLibraryType::MetricPointer       MetricPointer;
    typedef typename MPLibraryType::MapEvaluatorPointer MapEvaluatorPointer;
    typedef typename MPLibraryType::MPStrategyPointer   MPStrategyPointer;
    typedef typename MPLibraryType::DistanceMetricPointer
                                                        DistanceMetricPointer;
    typedef typename MPLibraryType::ValidityCheckerPointer
                                                        ValidityCheckerPointer;
    typedef typename MPLibraryType::NeighborhoodFinderPointer
                                                        NeighborhoodFinderPointer;

    ///@}
    ///@name Construction
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @param _label ID of the object, i.e., user defined label
    /// @param _name Name of the object, i.e., derived class name
    /// @param _debug Turn debug output on or off
    MPBaseObject(const string& _label = "", const string& _name = "",
        bool _debug = false) :
        m_name(_name), m_debug(_debug), m_label(_label) { }

    ////////////////////////////////////////////////////////////////////////////
    /// @param _node XMLNode to parse for this object
    MPBaseObject(XMLNode& _node) {
      ParseXML(_node);
    }

    virtual ~MPBaseObject() = default;

    ///@}
    ///@name I/O
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Parse XML node
    /// @param _node XML node
    ///
    /// Parse XML node. By default every MPBaseObject requires a label and
    /// optionally loads a debug parameter.
    void ParseXML(XMLNode& _node) {
      m_label = _node.Read("label", true, "", "Label Identifier");
      m_debug = _node.Read("debug", false, false,
          "Run-time debug on(true)/off(false)");
    }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Print values of object
    /// @param _os ostream to print values to
    ///
    /// Print values of object to ostream. By default name and label are output.
    virtual void Print(ostream& _os) const {
      _os << this->GetNameAndLabel() << endl;
    }

    ///@}
    ///@name Accessors
    ///@{

    void SetMPLibrary(MPLibraryType* _p) {m_library = _p;}

    MPLibraryType* GetMPLibrary() const {return m_library;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return MPProblem object
    MPProblemType* GetMPProblem() const {return m_problem;}

    ////////////////////////////////////////////////////////////////////////////
    /// @param _m MPProblem object
    virtual void SetMPProblem(MPProblemType* _p) {m_problem = _p;}

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

    ///@}
    ///@name MPProblem Accessors
    ///@{

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

    string GetQueryFilename() const {return m_problem->GetQueryFilename();}

    void SetQueryFilename(const string& _n) {
      return m_problem->SetQueryFilename(_n);
    }

    ///@}
    ///@name MPLibrary Accessors
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @param _dm Label
    /// @return DistanceMetric pointer
    DistanceMetricPointer GetDistanceMetric(const string& _dm) const {
      return m_library->GetDistanceMetric(_dm);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// @param _vc Label
    /// @return ValidityChecker pointer
    ValidityCheckerPointer GetValidityChecker(const string& _vc) const {
      return m_library->GetValidityChecker(_vc);
    }

    void ToggleValidity() {m_library->ToggleValidity();}

    ////////////////////////////////////////////////////////////////////////////
    /// @param _nf Label
    /// @return NeighborhoodFinder pointer
    NeighborhoodFinderPointer GetNeighborhoodFinder(const string& _nf) const {
      return m_library->GetNeighborhoodFinder(_nf);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// @param _s Label
    /// @return Sampler pointer
    SamplerPointer GetSampler(const string& _s) const {
      return m_library->GetSampler(_s);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// @param _lp Label
    /// @return LocalPlanner pointer
    LocalPlannerPointer GetLocalPlanner(const string& _lp) const {
      return m_library->GetLocalPlanner(_lp);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// @param _e Label
    /// @return Extender pointer
    ExtenderPointer GetExtender(const string& _e) const {
      return m_library->GetExtender(_e);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// @param _pm Label
    /// @return PathModifier pointer
    PathModifierPointer GetPathModifier(const string& _pm) const {
      return m_library->GetPathModifier(_pm);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// @param _c Label
    /// @return Connector pointer
    ConnectorPointer GetConnector(const string& _c) const {
      return m_library->GetConnector(_c);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// @param _m Label
    /// @return Metric pointer
    MetricPointer GetMetric(const string& _m) const {
      return m_library->GetMetric(_m);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// @param _me Label
    /// @return MapEvaluator pointer
    MapEvaluatorPointer GetMapEvaluator(const string& _me) const {
      return m_library->GetMapEvaluator(_me);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// @param _mps Label
    /// @return MPStrategy pointer
    MPStrategyPointer GetMPStrategy(const string& _mps) const {
      return m_library->GetMPStrategy(_mps);
    }

    ///@}

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

    string m_label;                          ///< Unique identifier of object
    MPProblemType* m_problem{nullptr};       ///< The current MPProblem object.
    MPLibraryType* m_library{nullptr}; ///< The planning library object.

    template<typename T, typename U> friend class MethodSet;
};

#endif
