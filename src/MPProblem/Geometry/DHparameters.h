/*DHparameters.h
 *Denavit-Hartenberg Parameters.
 *Following is description of DH Parameter.
 *The z vector of any link frame is on a joint axis.
 * - d is the algebraic distance along axis zi-1 to the point 
 *   where the common perpendicular intersects axis zi-1. 
 * - a is the length of the common perpendicular. 
 * - theta is the angle, about zi-1, that the common perpendicular makes with vector xi-1. 
 * - alpha is the angle, about xi, that vector zi makes with vector zi-1. 
 *here xi, zi is x and z direction of current link.
 *xi-1 and zi-1 is x and z direction of previous link.
 *
 *Denavit-Hartenberg Parameters are used to connection 
 */

/////////////////////////////////////////////////////////////////////////////////////////
#ifndef DHparameters_h
#define DHparameters_h

/////////////////////////////////////////////////////////////////////////////////////////
//Include general headers
#include <iostream>
#include <fstream>

using namespace std;

class DHparameters {
  public:
    DHparameters(double _alpha = 0.0, double _a = 0.0, double _d = 0.0, double _theta = 0.0);
    virtual ~DHparameters();

    ///Read alpha, a, d, and theta one by one from _is.
    friend istream& operator>>(istream&, DHparameters&);
    ///Output alpha, a, d, and theta one by one to _os.
    friend ostream& operator<<(ostream&, const DHparameters&);

    bool operator==(const DHparameters& dh) const;

    double alpha;   ///<Angle between two x axis
    double a;       ///<distance between two z axis
    double d;       ///<algebraic distance along z axis
    double theta;   ///<Angle between two z axis
};

#endif
