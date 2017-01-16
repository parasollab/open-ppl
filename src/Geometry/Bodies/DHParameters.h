#ifndef DH_PARAMETERS_H_
#define DH_PARAMETERS_H_

#include <iostream>
#include <fstream>
using namespace std;

#include <Transformation.h>
using namespace mathtool;

////////////////////////////////////////////////////////////////////////////////
/// Denavit-Hartenberg Parameters for describing joint connections.
///
/// @details A description of DH Parameters:
///   - The z vector of any link frame is on a joint axis.
///   - d is the algebraic distance along axis z_{i-1} to the point
///     where the common perpendicular intersects axis z_{i-1}.
///   - a is the length of the common perpendicular.
///   - \theta is the angle, about z_{i-1}, that the common
///     perpendicular makes with vector x_{i-1}.
///   - \alpha is the angle, about x_i, that vector z_i makes
///     with vector z_{i-1}.
/// Here x_i, z_i is x and z direction of current link.
/// x_{i-1} and z_{i-1} is x and z direction of previous
/// link.
/// @ingroup Geometry
////////////////////////////////////////////////////////////////////////////////
class DHParameters {

  public:

    ///@name Construction
    ///@{

    /// Construct a set of DH params from explicit values.
    /// @param _alpha Alpha
    /// @param _a A
    /// @param _d D
    /// @param _theta Theta
    DHParameters(double _alpha = 0.0, double _a = 0.0,
        double _d = 0.0, double _theta = 0.0);

    ///@}
    ///@name Conversion
    ///@{

    /// Convert the DH parameter representation into a standard transformation.
    Transformation GetTransformation() const;

    ///@}
    ///@name I/O
    ///@{

    /// Read a set of DH params from an instream.
    /// @param _is Input stream
    /// @param _d DHParameters
    friend istream& operator>>(istream& _is, DHParameters& _d);

    /// Write a set of DH params to an outstream.
    /// @param _os Output stream
    /// @param _d DHParameters
    friend ostream& operator<<(ostream& _os, const DHParameters& _d);

    ///@}
    ///@name Internal State
    ///@{

    double m_alpha;   ///< Angle between two x axis
    double m_a;       ///< Distance between two z axis
    double m_d;       ///< Algebraic distance along z axis
    double m_theta;   ///< Angle between two z axis

    ///@}

};

#endif
