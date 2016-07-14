#ifndef DH_PARAMETERS_H_
#define DH_PARAMETERS_H_

#include <iostream>
#include <fstream>
using namespace std;

#include <Transformation.h>
using namespace mathtool;

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
    DHParameters(double _alpha = 0.0, double _a = 0.0,
        double _d = 0.0, double _theta = 0.0);

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

    double m_alpha;   ///< Angle between two x axis
    double m_a;       ///< Distance between two z axis
    double m_d;       ///< Algebraic distance along z axis
    double m_theta;   ///< Angle between two z axis
};

#endif
