#ifndef DH_PARAMETERS_H_
#define DH_PARAMETERS_H_

#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include <Transformation.h>
using namespace mathtool;

#include "Cfg/Cfg.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief Denavit-Hartenberg Parameters.
///
/// Denavit-Hartenberg Parameters for describing joint connections. Following is
/// a description of DH Parameters:
///   - The \f$z\f$ vector of any link frame is on a joint axis.
///   - \f$d\f$ is the algebraic distance along axis \f$z_{i-1}\f$ to the point
///     where the common perpendicular intersects axis \f$z_{i-1}\f$.
///   - \f$a\f$ is the length of the common perpendicular.
///   - \f$\theta\f$ is the angle, about \f$z_{i-1}\f$, that the common
///     perpendicular makes with vector \f$x_{i-1}\f$.
///   - \f$\alpha\f$ is the angle, about \f$x_i\f$, that vector \f$z_i\f$ makes
///     with vector \f$z_{i-1}\f$.
/// Here \f$x_i\f$, \f$z_i\f$ is \f$x\f$ and \f$z\f$ direction of current link.
/// \f$x_{i-1}\f$ and \f$z_{i-1}\f$ is \f$x\f$ and \f$z\f$ direction of previous
/// link.
////////////////////////////////////////////////////////////////////////////////
class DHParameters {
  public:
    ////////////////////////////////////////////////////////////////////////////
    /// @param _alpha Alpha
    /// @param _a A
    /// @param _d D
    /// @param _theta Theta

    ////////////////////////////////////////////////////////////////////////////
    /// @param _is Input stream
    /// @param _d DHParameters
    friend istream& operator>>(istream& _is, DHParameters& _d);
    ////////////////////////////////////////////////////////////////////////////
    /// @param _os Output stream
    /// @param _d DHParameters
    friend ostream& operator<<(ostream& _os, const DHParameters& _d);

    ////////////////////////////////////////////////////////////////////////////
    /// @return Transformation for DH frame
    Transformation GetTransformation() const;

  private:
    //Only this class and class that handles initial reading allowed to change, 
    //constant throughout execution
    pair<double, double> m_parameterRanges[4];
    double m_parameterValues[4];  ///< Initiate to midpoint of range

  public:

    //Helpers
    bool IsFixed(size_t _index) const;
    bool InRange(const vector<double>& _cfg, size_t _dof, size_t index) const;
    pair<double, double> GetAlphaRange() const {return m_parameterRanges[0];}  ///< Angle between two x axis
    pair<double, double> GetARange() const {return m_parameterRanges[1];}      ///< Distance between two z axis
    pair<double, double> GetDRange() const {return m_parameterRanges[2];}      ///< Algebraic distance along z axis
    pair<double, double> GetThetaRange() const {return m_parameterRanges[3];}  ///< Angle between two z axis
    double GetAlpha() const {return m_parameterValues[0];}
    double GetA() const {return m_parameterValues[1];}
    double GetD() const {return m_parameterValues[2];}
    double GetTheta() const {return m_parameterValues[3];}
    void SetDHParameters(size_t _index, double _v);
};

#endif
