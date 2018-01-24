#ifndef WEIGHT_H_
#define WEIGHT_H_

#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#ifdef _PARALLEL
#include "views/proxy.h"
#endif

#include "MPProblem/Robot/Actuator.h"
#include "MPProblem/Robot/Control.h"
#include "Utilities/MPUtils.h"
#include "Geometry/Boundaries/Range.h"


////////////////////////////////////////////////////////////////////////////////
/// Default weight class for roadmap edges. Defined as a value and a set of
/// intermediate configurations.
///
/// Weight is the concept for what is stored on the graph edges. Essentially,
/// edges are defined as polygonal chains I = {q_1, q_2, ..., q_n}
/// through @cspace. They have two essential properties, a weight value
/// representing some idea of distance between the two end points of the edge
/// and a set of intermediate configurations defining the polygonal chain (not
/// including the start and goal configurations).
/// @ingroup Weights
////////////////////////////////////////////////////////////////////////////////
template <class CfgType>
class DefaultWeight {

  public:

    ///@name Construction
    ///@{

    DefaultWeight(const std::string& _label = "", const double _w = 0,
        const std::vector<CfgType>& _intermediates = std::vector<CfgType>());

    virtual ~DefaultWeight() = default;

    ///@}
    ///@name Assignment
    ///@{

    virtual const DefaultWeight& operator=(const DefaultWeight& _w);

    ///@}
    ///@name Ordering and Equality
    ///@{

    virtual bool operator==(const DefaultWeight& _w) const noexcept;
    virtual bool operator!=(const DefaultWeight& _w) const noexcept;

    virtual bool operator<(const DefaultWeight& _other) const noexcept;

    ///@}
    ///@name Properties
    ///@{

    const std::string& GetLPLabel() const noexcept;
    void SetLPLabel(const std::string& _lpLabel) noexcept;

    std::vector<CfgType>& GetIntermediates() noexcept;
    const std::vector<CfgType>& GetIntermediates() const noexcept;

    void SetIntermediates(const std::vector<CfgType>& _intermediates) noexcept;
    void SetIntermediates(std::vector<CfgType>&& _intermediates) noexcept;

    double GetWeight() const noexcept;
    void SetWeight(const double _w) noexcept;

    void SetControl(const Control& _c) noexcept;
    const ControlSet& GetControlSet() const noexcept;
    ControlSet& GetControlSet() noexcept;
    void SetControlSet(const ControlSet& _c) noexcept;

    //For disassembly planning, we need the bodies (first one must be the body
    // rotated about) that were moved to create the edge.
    void SetActiveBodies(const std::vector<unsigned int>& _bodies) {
      m_activeBodies = _bodies;
    }
    std::vector<unsigned int> GetActiveBodies() {return m_activeBodies;}

    /// Call this so that Path::FullCfgs() doesn't reproduce intermediates for
    /// this edge (only used for disassembly planning right now). This creates
    /// INVALID roadmaps, and should only be used for visualization purposes.
    void SetSkipEdge() { m_skipEdge = true; }
    bool SkipEdge() { return m_skipEdge; }

    bool IsChecked(const int _mult) const noexcept;
    void SetChecked(const int _mult) noexcept;

    bool HasClearance() const noexcept;
    double GetClearance() const noexcept;
    void SetClearance(const double _c) noexcept;

    // The number of timesteps that the local plan on this edge cares about.
    size_t GetTimeSteps() const noexcept;
    void SetTimeSteps(std::size_t _steps) noexcept;


    ///@}
    ///@name I/O
    ///@{

    /// Read an edge in from an input stream.
    /// @param _is The input stream to read from.
    virtual void Read(std::istream& _is);

    /// Write an edge out to an output stream.
    /// @param _is The output stream to write to.
    virtual void Write(std::ostream& _os) const;

    /// A static pointer to the current robot, which is needed to parse any
    /// intermediate configurations in the edge.
    static Robot* inputRobot;

    /// Clear all of the contents of object to a reinitialized state.
    void Clear();

    ///@}
    ///@name Stuff for stapl graph interface
    ///@{

    /// This only adds weights, it doesn't take intermediates into account.
    virtual DefaultWeight operator+(const DefaultWeight& _other) const ;

    double Weight() const noexcept; // For GraphAlgo interface
    static DefaultWeight MaxWeight() noexcept; // For Dijkstra's Alg

    ///@}

  protected:

    ///@name Internal State
    ///@{

    std::string m_lpLabel;   ///< Label of local planner that built this edge.

    double m_weight{0.};                  ///< The edge length.
    std::vector<CfgType> m_intermediates; ///< Intermediate configurations.

    // In order to reproduce correct intermediates of a subassembly's path for
    // disassembly planning, we need the moved parts, with the body num that was
    // rotated about in the first element:
    std::vector<unsigned int> m_activeBodies;

    bool m_skipEdge{false}; ///< Don't compute intermediates in Path::FullCfgs()

    int m_checkedMult;
    bool m_hasClearance{false};
    double m_clearance;

    // For nonholonomic robots.
    ControlSet m_controls;   ///< The controls used.
    size_t m_timeSteps{0};   ///< The number of timesteps to apply the controls.

    /// A vector of safe intervals, not in collision with Dynamic Obstacles
    /// of known path, for this cfg.
    std::vector<Range<double>> m_safeIntervals;

    ///@}

  public:

#ifdef _PARALLEL
    void define_type(stapl::typer &t)
    {
      t.member(m_weight);
      t.member(m_lpLabel);
      t.member(m_intermediates);
    }
#endif
};

/*------------------------------ Construction --------------------------------*/

template <typename CfgType>
DefaultWeight<CfgType>::
DefaultWeight(const std::string& _label, const double _w,
    const std::vector<CfgType>& _intermediates) :
    m_lpLabel(_label), m_weight(_w), m_intermediates(_intermediates),
    m_checkedMult(numeric_limits<int>::max()), m_hasClearance(false) { }

/*------------------------------- Assignment ---------------------------------*/

template <typename CfgType>
const DefaultWeight<CfgType>&
DefaultWeight<CfgType>::
operator=(const DefaultWeight<CfgType>& _w) {
  if(this != &_w) {
    m_lpLabel = _w.GetLPLabel();
    m_weight = _w.GetWeight();
    m_intermediates = _w.GetIntermediates();
    m_checkedMult = _w.m_checkedMult;
    m_hasClearance = _w.HasClearance();
    m_clearance = _w.GetClearance();
    m_controls = _w.GetControlSet();
    m_timeSteps = _w.GetTimeSteps();
  }
  return *this;
}

/*-------------------------- Equality and Ordering ---------------------------*/

template <typename CfgType>
bool
DefaultWeight<CfgType>::
operator==(const DefaultWeight& _w) const noexcept {
  return m_lpLabel == _w.GetLPLabel() && m_weight == _w.GetWeight();
}


template <typename CfgType>
bool
DefaultWeight<CfgType>::
operator!=(const DefaultWeight& _w) const noexcept {
  return !(*this == _w);
}


template <typename CfgType>
bool
DefaultWeight<CfgType>::
operator<(const DefaultWeight& _w) const noexcept {
  return m_weight < _w.m_weight;
}

/*----------------------------------------------------------------------------*/

template <typename CfgType>
const std::string&
DefaultWeight<CfgType>::
GetLPLabel() const noexcept {
  return m_lpLabel;
}


template <typename CfgType>
void
DefaultWeight<CfgType>::
SetLPLabel(const std::string& _label) noexcept {
  m_lpLabel = _label;
}


template <typename CfgType>
std::vector<CfgType>&
DefaultWeight<CfgType>::
GetIntermediates() noexcept {
  return m_intermediates;
}


template <typename CfgType>
const std::vector<CfgType>&
DefaultWeight<CfgType>::
GetIntermediates() const noexcept {
  return m_intermediates;
}


template <typename CfgType>
void
DefaultWeight<CfgType>::
SetIntermediates(const std::vector<CfgType>& _intermediates) noexcept {
  m_intermediates = _intermediates;
}


template <typename CfgType>
void
DefaultWeight<CfgType>::
SetIntermediates(std::vector<CfgType>&& _intermediates) noexcept {
  m_intermediates = std::move(_intermediates);
}


template <typename CfgType>
double
DefaultWeight<CfgType>::
GetWeight() const noexcept {
  return m_weight;
}


template <typename CfgType>
void
DefaultWeight<CfgType>::
SetWeight(const double _w) noexcept {
  m_weight = _w;
}


//------------------------- Control Set Modifiers ------------------------------

//This is a wrapper for SetControlSet(), because right now the only use case is
// single controls:
template <typename CfgType>
void
DefaultWeight<CfgType>::
SetControl(const Control& _c) noexcept {
  SetControlSet({_c});
}

template <typename CfgType>
const ControlSet&
DefaultWeight<CfgType>::
GetControlSet() const noexcept {
  return m_controls;
}

template <typename CfgType>
ControlSet&
DefaultWeight<CfgType>::
GetControlSet() noexcept {
  return m_controls;
}


// We can potentially have multiple controls to get from one configuration to
// the next, so the entire set of those controls can be set here.
template <typename CfgType>
void
DefaultWeight<CfgType>::
SetControlSet(const ControlSet& _c) noexcept {
  m_controls = _c;
}


template <typename CfgType>
std::size_t
DefaultWeight<CfgType>::
GetTimeSteps() const noexcept {
  return m_timeSteps;
}

template <typename CfgType>
void
DefaultWeight<CfgType>::
SetTimeSteps(std::size_t _steps) noexcept {
  m_timeSteps = _steps;
}


template <typename CfgType>
bool
DefaultWeight<CfgType>::
IsChecked(const int _mult) const noexcept {
  return m_checkedMult <= _mult;
}


template <typename CfgType>
void
DefaultWeight<CfgType>::
SetChecked(const int _mult) noexcept {
  m_checkedMult = min(m_checkedMult, _mult);
}


template <typename CfgType>
bool
DefaultWeight<CfgType>::
HasClearance() const noexcept {
  return m_hasClearance;
}


template <typename CfgType>
double
DefaultWeight<CfgType>::
GetClearance() const noexcept {
  return m_clearance;
}


template <typename CfgType>
void
DefaultWeight<CfgType>::
SetClearance(const double _c) noexcept {
  m_hasClearance = true;
  m_clearance = _c;
}

/*----------------------------------- I/O ------------------------------------*/

/// Clears out the contents of all vectors and zeros out all member values.
/// Intended to reinitialize an object to its default-constructed state.
/// This was found to be required by Read(), due to stapl's reuse of a temp obj.
template <typename CfgType>
void
DefaultWeight<CfgType>::
Clear() {
  // Reset the initial state variables of this object:
  m_lpLabel.clear();
  m_weight = 0.;
  m_intermediates.clear();
  m_controls.clear();

  m_checkedMult = 0;
  m_hasClearance = false;
  m_clearance = 0.;

  m_timeSteps = 0;
}

// Initialize the input robot (a placeholder for not knowing the robot when
// reading in cfgs from a file) to nullptr, as it is to be set externally.
template <typename CfgType>
Robot* DefaultWeight<CfgType>::inputRobot = nullptr;


// Reads in an edge from a standard input stream, including
// control signals for nonholonomic robots.
// This function is symmetric with Write().
template <typename CfgType>
void
DefaultWeight<CfgType>::
Read(std::istream& _is) {
  if(!inputRobot)
    throw RunTimeException(WHERE, "Need to tell the weight class what robot "
        "we are reading for. Use inputRobot member to specify.");

  Clear(); //Necessary, as stapl seems to use the same temp weight object

  // Read the data for robots that are either holonomic or not:
  size_t numIntermediates;
  _is >> numIntermediates;
  m_intermediates.clear();

  CfgType tmp(inputRobot);
  for(size_t i = 0; i < numIntermediates; ++i) {
    _is >> tmp;
    m_intermediates.push_back(tmp);
  }
  _is >> m_weight;

  #ifndef VIZMO_MAP
    // Read the data specifically for nonholonomic robots if necessary:
    if(inputRobot->IsNonholonomic()) {
      //read the # timesteps used for this edge and the # actuators/controls
      _is >> m_timeSteps;
      std::size_t numControls;
      _is >> numControls;

      //Now loop through each individual control, outputting the full signal
      // and label for that control:
      for(std::size_t i = 0; i < numControls; i++) {
        Control con;
        std::string label;
        _is >> label;

        /// @TODO an easy optimization would be to make coast controls not print
        /// the 0's signal, it could reduce some roadmaps significantly, I'd bet

        //If the label is "Coast" then we need to put a nullptr for the actuator
        // since that's what is used for a coast control.
        if(label.compare("Coast") == 0) // Note compare() returns 0 if equal.
          con.actuator = nullptr;
        else
          con.actuator = inputRobot->GetActuator(label);

        //Loop through and get all of the control signal's values.
        //The number of values in the signal is equal to the robot's DOF.
        std::size_t signalLength = inputRobot->GetMultiBody()->DOF();
        con.signal.resize(signalLength, 0);
        for(std::size_t j = 0; j < signalLength; j++) {
          double val;
          _is >> val;
          con.signal[j] = val;
        }

        //It wouldn't be a bad idea to put in a check here to ensure that if
        // the actuator is a nullptr, then all of the signal is 0.

        //finally push back the read-in control signal:
        m_controls.push_back(con);
      }
    }
  #endif
}


// Writes to a standard output stream all of the data for the edge, including
// control signals for nonholonomic robots.
// This function is symmetric with Write().
template <typename CfgType>
void
DefaultWeight<CfgType>::
Write(std::ostream& _os) const {
  /// @TODO Now that we read/write the control signals for nonholonomic, we
  ///  should remove the writing of intermediates for conciseness.

  //Write the data that's needed whether it's a holonomic robot or not:
  _os << m_intermediates.size() << " ";
  for(auto&  cfg : m_intermediates)
    _os << cfg;
  _os << m_weight;

  #ifndef VIZMO_MAP
    //If nonholonomic, print extra data needed (holonomic will have no controls)
    if(!m_controls.empty()) {
      //output the # timesteps used for this edge and the # actuators/controls
      _os << " " << m_timeSteps << " " << m_controls.size() << " ";

      //Now loop through each individual control, outputting the full signal
      // and label for that control:
      for(auto control : m_controls) {
        //Output the actuator label:
        if(control.actuator)
          _os << control.actuator->GetLabel() << " ";
        else
          _os << "Coast ";// if actuator is nullptr, it's a coast control.

        /// @TODO an easy optimization would be to make coast controls not print
        /// the 0 signal, it could reduce some roadmaps significantly, I'd bet.
        //Write each of the control signal's values:
        for (double& val : control.signal)
          _os << val << " ";
      }
    }
  #endif
}



template <typename CfgType>
std::ostream&
operator<<(std::ostream& _os, const DefaultWeight<CfgType>& _w) {
  _w.Write(_os);
  return _os;
}


template <typename CfgType>
std::istream&
operator>>(std::istream& _is, DefaultWeight<CfgType>& _w) {
  _w.Read(_is);
  return _is;
}

/*---------------------- stapl graph interface helpers -----------------------*/

template <typename CfgType>
DefaultWeight<CfgType>
DefaultWeight<CfgType>::
operator+(const DefaultWeight& _other) const {
  return DefaultWeight(m_lpLabel, m_weight + _other.m_weight);
}


template <typename CfgType>
double
DefaultWeight<CfgType>::
Weight() const noexcept {
  return GetWeight();
}


template <typename CfgType>
DefaultWeight<CfgType>
DefaultWeight<CfgType>::
MaxWeight() noexcept {
  static constexpr double max = numeric_limits<double>::max();
  return DefaultWeight("INVALID", max);
}

/*----------------------------------------------------------------------------*/

#endif
