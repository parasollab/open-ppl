// $Id$
/////////////////////////////////////////////////////////////////////
//  DHparameters.h
//
//  Created   2/25/98 Aaron Michalk
/////////////////////////////////////////////////////////////////////

#ifndef DHparameters_h
#define DHparameters_h

#include <fstream.h>
#include "Transformation.h"

class Transformation;

class DHparameters {
public:
    //===============================================================
    //  Constructors and Destructor
    //===============================================================
    DHparameters(double _alpha = 0.0, double _a = 0.0, double _d = 0.0, double _theta = 0.0);
    DHparameters(Transformation & _t);
    ~DHparameters();
    //===============================================================
    //  Virtual Functions
    //===============================================================
    virtual void Read(ifstream & _is);
    virtual void Write(ostream & _os);
    //===============================================================
    //  Data
    //===============================================================
    double alpha;
    double a;
    double d;
    double theta;
protected:
private:
};

#endif
