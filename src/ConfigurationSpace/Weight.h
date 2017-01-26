#ifndef WEIGHT_H_
#define WEIGHT_H_

#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#ifdef _PARALLEL
#include "views/proxy.h"
#endif

#include "MPProblem/Robot/Control.h"
#include "Utilities/MPUtils.h"


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

    DefaultWeight(const std::string& _label = "", const double _w = 1,
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

    bool IsChecked(const int _mult) const noexcept;
    void SetChecked(const int _mult) noexcept;

    bool HasClearance() const noexcept;
    double GetClearance() const noexcept;
    void SetClearance(const double _c) noexcept;

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
    static Robot* m_inputRobot;

    /// Set the robot for which we are currently reading edges.
    /// @param _r The robot to read for.
    static void SetInputRobot(Robot* const _r) noexcept;

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

    std::string m_lpLabel;

    double m_weight;
    std::vector<CfgType> m_intermediates;
    Control m_control;

    int m_checkedMult;
    bool m_hasClearance;
    double m_clearance;

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


template <typename CfgType>
void
DefaultWeight<CfgType>::
SetControl(const Control& _c) noexcept {
  m_control = _c;
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

template <typename CfgType>
Robot* DefaultWeight<CfgType>::m_inputRobot = nullptr;

template <typename CfgType>
void
DefaultWeight<CfgType>::
Read(std::istream& _is) {
  if(!m_inputRobot)
    throw RunTimeException(WHERE, "Need to tell the weight class what robot "
        "we are reading for. Use SetInputRobot to specify.");

  size_t numIntermediates;
  _is >> numIntermediates;
  m_intermediates.clear();

  CfgType tmp(m_inputRobot);
  for(size_t i = 0; i < numIntermediates; ++i) {
    _is >> tmp;
    m_intermediates.push_back(tmp);
  }
  _is >> m_weight;
}


template <typename CfgType>
void
DefaultWeight<CfgType>::
Write(std::ostream& _os) const {
  /// @TODO Read/write control signals and timesteps for non-holo instead of
  ///       printing intermediates.
  //const bool nonholonomic = !m_control.signal.empty();

  //if(!nonholonomic) {
    _os << m_intermediates.size() << " ";
    for(auto&  cfg : m_intermediates)
      _os << cfg;
    _os << m_weight;
  //}
  //if(nonholonomic) {
  //  _os << " " << m_control.signal.size() << " ";
  //  for(auto val : m_control.signal)
  //    _os << val << " ";
  //  _os << m_numSteps;
  //}
}


template <typename CfgType>
void
DefaultWeight<CfgType>::
SetInputRobot(Robot* const _r) noexcept {
  m_inputRobot = _r;
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
