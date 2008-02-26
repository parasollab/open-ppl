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

////////////////////////////////////////////////////////////////////////////////////////////
//include OBPRM headers
#include "DHparameters.h"
#include "Transformation.h"

/////////////////////////////////////////////////////////////////////
class Body;
/////////////////////////////////////////////////////////////////////

/**This class stores information about connection from one body to another one.
  *The information stored in this class includes:
  * - Connection type
  * - 2 Body instances
  * - Tansformation instances
  * - DHparameters
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


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    //---------------------------------------------------------------
    /**@name  Constructors and Destructor
      *@todo The documentation of these function are not complete.
      *I can't really understand what these parameters for.
      */
    //---------------------------------------------------------------
    //@{
    
    /**Constructor with Body1 and Body2 (Body2 is optional).
      *Get might be called to retrieve values for unset data member.
      *@param _body1 One of this _body1's end joints is this connection.
      *@param _body2 One of this _body2's start joints is this connection.
      */
    Connection(Body * _body1, Body * _body2 = 0);  // Second argument is optional
    
    /**You need to provide all information for Constructor.
      *@param _body1
      *@param _body2
      *@param _transformationToDHFrame Transform from frame of body1 to DH-Frame
      *@param _dhparameters DHParameter
      *@param _transformationToBody2 Transform from DH-Frame to frame of body1
      **/
    Connection(Body * _body1, Body * _body2, const Transformation & _transformationToBody2, 
    const DHparameters & _dhparameters, const Transformation & _transformationToDHFrame);
     
    /**Destructor.
      *call first body's RemoveForwardConnection and second's RemoveBackwardConnection
      *@see Body::RemoveForwardConnection and Body::RemoveBackwardConnection
      */
    virtual ~Connection();

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    //---------------------------------------------------------------
    /**@name  Access Methods*/
    //---------------------------------------------------------------
    //@{
    
    /**Check if a give body is the first body of this connection instance or not.
      *@param _body A point to Body instance. Address is checked
      * with the address of first body.
      *@return True if _body is first body. Otherwise False is returned.
      */
    int IsFirstBody(Body * _body);
    
    /**Get Previouse Body. (i.e first body)
      *@note only body[0] is returned.
      **/
    Body * GetPreviousBody();    
    
    /**Get Next Body. (i.e second body)
      *@note only body[1] is returned.
      */
    Body * GetNextBody();
    
    /**Get the type of connection used in this Connection instance.
      *@see ConnectionType
      */
    ConnectionType GetConnectionType();

    ///Get a refecence of DHparameters used in this Connection instance.   
    DHparameters & GetDHparameters();
    
    ///Get a refecence of transformation which is a transformation from body1 to body2. (Not sure)
    Transformation & GetTransformationToBody2();
    
    ///Get a refecence of transformation which is a transformation from body1 to DHFrame of body2. (Not sure)
    Transformation & GetTransformationToDHFrame();
    
    /** Set values for data member
      */
    void Read(Body* body1, Body* body2,
	      const Vector3D& transformPosition, const Orientation& transformOrientation,
	      const Vector3D& positionToDHFrame, const Orientation& orientationToDHFrame,
	      const DHparameters& _dhparameters, const ConnectionType& connectionType);
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    I/O
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name I/O Methods. Use these method to read in/write out internal state*/
    //@{

    /**Write inforamtion about Body2, transformationToBody2, and transformationToDHFrame 
      *to output stream.
      */
    virtual void Write(ostream & _os);

    //@}
    
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

protected:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  // PRIVATE METHOD and DATA MEMBERS
  // NO DOCUMENTATION YET
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  Implementation of Connection
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

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


