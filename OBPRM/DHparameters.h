// $Id$

/**@file DHparameters.h
   @date 2/25/98
   @author Aaron Michalk
*/

/////////////////////////////////////////////////////////////////////////////////////////
#ifndef DHparameters_h
#define DHparameters_h

/////////////////////////////////////////////////////////////////////////////////////////
//Include general headers
#include <iostream.h>
#include <fstream.h>

/////////////////////////////////////////////////////////////////////////////////////////
//Forward Declaration
class Transformation;

/**Denavit-Hartenberg Parameters.
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
class DHparameters {
public:
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    //-----------------------------------------------------------
    /**@name  Constructors and Destructor.*/
    //-----------------------------------------------------------
    //@{

    ///Create a instance of DHparameters by given alpha, a, d and theta values.
    DHparameters(double _alpha = 0.0, double _a = 0.0, double _d = 0.0, double _theta = 0.0);
    ///Decompose a given Transformation into alpha, a, d, and theta.
    DHparameters(Transformation & _t);
    ///Destrcutor. Do nothing
    ~DHparameters();

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    I/O
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Input and Ouput methods*/
    //@{
    ///Read alpha, a, d, and theta one by one from _is.
    virtual void Read(ifstream & _is);
    ///Output alpha, a, d, and theta one by one to _os.
    virtual void Write(ostream & _os);
    //@}

    //===============================================================
    //  Data
    //===============================================================
    /**@name DH Parameters*/
    //@{
    double alpha;   ///<Angle between two x axis
    double a;       ///<distance between two z axis
    double d;       ///<algebraic distance along z axis
    double theta;   ///<Angle between two z axis
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected Data Members and MEmber Methods Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

protected:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private Data Members and MEmber Methods Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

private:
};

#endif
