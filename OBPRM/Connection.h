// $Id$
////////////////////////////////////////////////////////////////////
/**@file  Connection.h
  *
  *@author Aaron Michalk
  *date  2/25/98 
  */
/////////////////////////////////////////////////////////////////////

#ifndef Connection_h
#define Connection_h

#include <fstream.h>
#include "Input.h"
#include "DHparameters.h"
#include "Transformation.h"

class Input;
class Body;

/**This class stores information about connection for one body to another one.
  *The information stored in this class includes:
  *	- Connection type
  *	- 2 Body instances
  *	- Tansformation instances
  *	- DHparameters
  */
class Connection {
public:
    //---------------------------------------------------------------
    //  Enumerations
    //---------------------------------------------------------------
    enum ConnectionType {
        Revolute,
	Prismatic
    };
    //---------------------------------------------------------------
    /**@name  Constructors and Destructor
      *@todo The documentation of these function are not complete.
      *I can't really understand what these parameters for.
      */
    //---------------------------------------------------------------
    //@{
    
    /** Constructor with Body1 and Body2 (Body2 is optional).
      *Get might be called to retrieve values for unset data member.
      *@param _body1 Body1.
      *@param _body2 Body2.
      */
    Connection(Body * _body1, Body * _body2 = 0);  // Second argument is optional
    
    /**You need to provide all information for Constructor.
      *@param _body1
      *@param _body1
      *@param _transformationToBody2
      *@param _dhparameters
      *@param _transformationToDHFrame
      **/
    Connection(Body * _body1, Body * _body2, const Transformation & _transformationToBody2, 
	 const DHparameters & _dhparameters, const Transformation & _transformationToDHFrame);
	 
    ///Destructor
    ~Connection();
    //@}
    
    //---------------------------------------------------------------
    /**@name  Access Methods*/
    //---------------------------------------------------------------
    //@{
    
    /**Check if a give body is first body or not.
      *@param _body A point to Body instance. Address is checked
      * with the address of first body.
      *@return True if _body is first body. Otherwise False is returned.
      */
    int IsFirstBody(Body * _body);
    
    /**Get Previouse Body.
      *@note only body[0] is returned.
      **/
    Body * GetPreviousBody();    
    
    /**Get Next Body.
      *@note only body[1] is returned.
      **/
    Body * GetNextBody();
    
    ///Get the type of connection used in this Connection instance.
    ConnectionType GetConnectionType();

    ///Get a refecence of DHparameters used in this Connection instance.   
    DHparameters & GetDHparameters();
    
    ///Get a refecence of transformation which is a transformation from body1 to body2. (Not sure)
    Transformation & GetTransformationToBody2();
    
    ///Get a refecence of transformation which is a transformation from body1 to DHFrame of body2. (Not sure)
    Transformation & GetTransformationToDHFrame();
    
    /** Read values for data member from Input instance.
      * Get information about Body2, transformationToBody2, and transformationToDHFrame
      * from a given Input instance.
      * @param _input Pointer to Input instance. Most information is stored here.
      * @param _multibodyIndex  Used to find out index of body2 from _input.
      * @param _connectionIndex Used to find out the type of connection from _input.
      */
    void Get(Input * _input, int _multibodyIndex, int _connectionIndex);
    
    ///Write inforamtion about Body2, transformationToBody2, and transformationToDHFrame to output stream.
    virtual void Write(ostream & _os);
    //@}
    
protected:

///////////////////////////////////////////////////////////////////////////////////////
// PRIVATE METHOD and DATA MEMBERS
// NO DOCUMENTATION YET
///////////////////////////////////////////////////////////////////////////////////////
private:
    //---------------------------------------------------------------
    //  Data
    //---------------------------------------------------------------
    Body *body[2];
    Transformation transformationToBody2;
    Transformation transformationToDHFrame;
    DHparameters dhparameters;
    ConnectionType type;
    int IsActuator;
};

//===================================================================
//  Inline Functions
//===================================================================

//-------------------------------------------------------------------
//  GetDHparameters
//-------------------------------------------------------------------
inline DHparameters & Connection::GetDHparameters() {
    return dhparameters;
}

//-------------------------------------------------------------------
//  IsFirstBody
//-------------------------------------------------------------------
inline int Connection::IsFirstBody(Body * _body) {
    return body[0] == _body;
}

//-------------------------------------------------------------------
//  GetNextBody
//-------------------------------------------------------------------
inline Body * Connection::GetNextBody() {
    return body[1];
}

//-------------------------------------------------------------------
//  GetPreviousBody
//-------------------------------------------------------------------
inline Body * Connection::GetPreviousBody() {
    return body[0];
}

//-------------------------------------------------------------------
//  GetTransformationToBody2
//-------------------------------------------------------------------
inline Transformation & Connection::GetTransformationToBody2() {
    return transformationToBody2;
}

//-------------------------------------------------------------------
//  GetTransformationToDHFrame
//-------------------------------------------------------------------
inline Transformation & Connection::GetTransformationToDHFrame() {
    return transformationToDHFrame;
}

//-------------------------------------------------------------------
//  GetConnectionType
//-------------------------------------------------------------------
inline Connection::ConnectionType Connection::GetConnectionType() {
    return type;
}

#endif


