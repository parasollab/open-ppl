/*This class stores information about connection from one body to another one.
 *The information stored in this class includes:
 * Connection type
 * 2 Body instances
 * _transformationToDHFrame Transform from frame of body1 to DH-Frame
 * _dhparameters DHParameter
 * _transformationToBody2 Transform from DH-Frame to frame of body2
 */

#ifndef Connection_h
#define Connection_h

////////////////////////////////////////////////////////////////////////////////////////////
//include OBPRM headers
#include "DHparameters.h"
#include "Transformation.h"
#include "Robot.h"
#include "boost/shared_ptr.hpp"
using boost::shared_ptr;


/////////////////////////////////////////////////////////////////////
class Body;
class MultiBody;


class Connection {
public:
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
    Connection(MultiBody* _owner);//_body1, const shared_ptr<Body>& _body2 = shared_ptr<Body>());  // Second argument is optional
    Connection(const shared_ptr<Body>& _body1, const shared_ptr<Body>& _body2 = shared_ptr<Body>());  // Second argument is optional
    
    /**You need to provide all information for Constructor.
      *@param _body1
      *@param _body2
      *@param _transformationToDHFrame Transform from frame of body1 to DH-Frame
      *@param _dhparameters DHParameter
      *@param _transformationToBody2 Transform from DH-Frame to frame of body1
      **/
    Connection(const shared_ptr<Body>& _body1, const shared_ptr<Body>& _body2, const Transformation & _transformationToBody2, 
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
    bool IsFirstBody(const shared_ptr<Body>& _body) const;
    
    /**Get Previouse Body. (i.e first body)
      *@note only body[0] is returned.
      **/
    shared_ptr<Body> GetPreviousBody();    
    
    /**Get Next Body. (i.e second body)
      *@note only body[1] is returned.
      */
    shared_ptr<Body> GetNextBody();
    
    /**Get the type of connection used in this Connection instance.
      *@see ConnectionType
      */
    Robot::JointType GetConnectionType() const;

    ///Get a refecence of DHparameters used in this Connection instance.   
    DHparameters & GetDHparameters();
    
    ///Get a refecence of transformation which is a transformation from body1 to body2. (Not sure)
    Transformation & GetTransformationToBody2();
    
    ///Get a refecence of transformation which is a transformation from body1 to DHFrame of body2. (Not sure)
    Transformation & GetTransformationToDHFrame();
    

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    I/O
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    friend ostream& operator<<(ostream& _os, const Connection& _c);
    friend istream& operator>>(istream& _is, Connection& _c);

  bool operator==(const Connection& c) const;

  friend class MultiBody;

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
    MultiBody* m_multibody;                  ///<Owner of this Connection
    shared_ptr<Body> body[2];
    Transformation transformationToBody2;
    Transformation transformationToDHFrame;
    DHparameters dhparameters;
    Robot::JointType type;
    size_t m_prevBodyIdx, m_nextBodyIdx;
};

#endif


