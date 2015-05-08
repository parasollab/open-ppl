#ifndef DH_parameters_h
#define DH_parameters_h

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
class DHparameters {
  public:
    DHparameters(double _alpha = 0.0, double _a = 0.0, double _d = 0.0, double _theta = 0.0);

    ///Read alpha, a, d, and theta one by one from _is.
    friend istream& operator>>(istream&, DHparameters&);
    ///Output alpha, a, d, and theta one by one to _os.
    friend ostream& operator<<(ostream&, const DHparameters&);

    bool operator==(const DHparameters& dh) const;

    Transformation GetTransformation() const;

    double m_alpha;   ///<Angle between two x axis
    double m_a;       ///<distance between two z axis
    double m_d;       ///<algebraic distance along z axis
    double m_theta;   ///<Angle between two z axis
};

#endif
