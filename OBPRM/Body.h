// $Id$
/////////////////////////////////////////////////////////////////////
/**@file Body.h
  *
  *@author Aaron Michalk, Wookho Son
  *@date 2/25/98
  *Created   2/25/98 Aaron Michalk
  *Modified  7/31/98 Wookho Son
  */  

////////////////////////////////////////////////////////////////////////////////////////////

#ifndef Body_h
#define Body_h

////////////////////////////////////////////////////////////////////////////////////////////
//include standard headers
#include <fstream.h>

////////////////////////////////////////////////////////////////////////////////////////////
//include CD headers
#ifdef USE_CSTK
#include <cstkSmallAPI.h>
#include <cstk_global.h>
#endif
#ifdef USE_VCLIP
#include <vclip.h>
#endif
#ifdef USE_RAPID
#include <RAPID.H>
#endif

////////////////////////////////////////////////////////////////////////////////////////////
//include OBPRM headers
#include "SwitchDefines.h"
#include "DHparameters.h"
#include "GMSPolyhedron.h"
#include "Connection.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////
class MultiBody;
class Input;
class Transformation;
///////////////////////////////////////////////////////////////////////////////////////////////////////////

/**class Body is a high level represented object in workspace.
  *Body is an abstract class with many basic functions provided.
  *Body instance could read (write) geometirc data from (to) specified file.
  *More information about bounding box and collision detection library internal models
  *could be computed and accessed from member methods.
  *if this Body instance is a link in articulate multibody,
  *connection information is also provided.
  */
class Body {
public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor*/
    //@{
    /**Constructor. Set the owner of this instance to _owner and initialize data members to 
      *0 ,NULL, and false.
      */
    Body(MultiBody * _owner);
    /**Constructor. Set owner and geometric information of this instance of Body. 
      *initialize data members to 0 ,NULL, and false.
      */
    Body(MultiBody * _owner, GMSPolyhedron & _polyhedron);
    ///Destructor. Free Forward (backward) connection information here.
    ~Body();
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Access Methods. Use these method to acess or change internal state*/
    //@{

        ///Find out if this is a fixed body in work space. Abstract method.
        virtual int IsFixedBody() = 0;

        ///Return transformation of this body in world coordinate. Abstract method
        virtual Transformation & GetWorldTransformation() = 0;

        ///Set world transformation of this body as _worldTransformation);
        void PutWorldTransformation(Transformation & _worldTransformation);

        /**Return world transformation of this body.
          *If worldTransformation has been calculated(updated), this method should be used
          *to avoid redundant calculation.
          *@todo what's the difference between WorldTransformation and GetWorldTransformation
          */
        Transformation & WorldTransformation();

        /**This function returns a Polyhedron whose vertices and normals are represented in 
          *world coordinate.
          */
        virtual GMSPolyhedron & GetWorldPolyhedron();

        ///Update internal data, worldPolyhedron, accroding current world transformation of this body.
        virtual void ChangeWorldPolyhedron();

        ///Return the local coordinates of the body
        virtual GMSPolyhedron & GetPolyhedron();

        ///Get the owner of this Body which is set in constrcutor.
        MultiBody * GetMultiBody();

        /**Return a bounding box of this instance of Body.
          *@see FindBoundingBox
          */
        double * GetBoundingBox();

        /**to get center of mass, you don't need to additionally call the above: ComputeCenterOfMass
          *because GetCenterOfMass will check if it is necessary to call this method.
          *@see ComputeCenterOfMass
          */
        Vector3D GetCenterOfMass();

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

    ///Output Polyhedron file name to ouput stream.
    virtual void Write(ostream & _os);

    /**Read data from given filename. Call GMSPolyhedron::Read and calculate the bounding box.
      *@see GMSPolyhedron::Read, FindBoundingBox
      */
    void Read(char * _fileName);

    /**CAll Read(char *) and buildCDstructure
      *@see buildCDstructure
      */
    void Read(Input*, char * _fileName);

    /**Read BYU format data from given inpustream. 
      *Call GMSPolyhedron::ReadBYU, calculate the bounding box, and then call buildCDstructure
      *to create auxilary data structure for collision detection.
      *@see GMSPolyhedron::ReadBYU, FindBoundingBox, buildCDstructure
      */
    void ReadBYU(cd_predefined cdtype, istream & _is);

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Helper methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    /**@name Helper Methods. Use these method to calculate auxilary infomation*/
    //@{

    ///Increment the contact count
    void AddContactCount();

    ///Initialize or Reinitialize contact count (to 0)
    void InitializeContact();

    ///Calculate Bounding Box of Polyhedron (in world coordinates).
    void FindBoundingBox();

    /**Calculate center of mass of given polyhedron (in world coordinates).
      *
      *This function is automatically calld by GetCenterOfMass()
      *if it has never been computed. After computing it,
      *this function will not be called again: rigid body.
      *
      *This way of computing center of mass is physically not true.
      *This assumes that each vertex carries the same mass, and
      *edges are weightless. To be more accurate, we need to
      *be modify this to consider the length of edges, which is
      *still an approximation.
      *
      *if GetCenterOfMass() is called for the first time,
      *this function calculates it and set 'available' flag 
      */
    void ComputeCenterOfMass();  

    /**Check if two Body share same joint(adjacent) for a robot
      *(by comparing all its forward and backward connection)
      *Facilitate robot self collision checking.
      */
    bool isAdjacent(Body *);

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Collision Detection Model methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    
    /**@name Collision Detection Model Methods. Use these method to build, access CD models*/
    //@{
    
    /**This method build collision detection model for specified collision detection library.
      *Each collision detection library has its own internal representation.
      *@param cdtype Which kind of collision detection library and model will be used.
      *@param nprocs used when USE_CSTK is specified in cdtype. Numer of times calling From_GMS_to_cstk.
      */
    void buildCDstructure(cd_predefined cdtype, int nprocs = 1);

#ifdef USE_CSTK
        void * GetCstkBody();
        void * GetCstkBody(int);
        void * From_GMS_to_CSTK();  // Convert from GMS format to CSTK format.
        void ** From_GMS_to_cstk(); // Convert from GMS format to CSTK format.
#endif
#ifdef USE_VCLIP
        PolyTree * GetVclipBody();  ///<Return VCLIP internal model
#endif
#ifdef USE_RAPID
        RAPID_model * GetRapidBody(); ///<Return RAPID internal model
#endif
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Connection methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    /**@name Connection methods */
    //@{

    ///Number of Forward connection (# of Body connected with this Body in Forward direction)
    int ForwardConnectionCount();
    ///Number of Backward connection (# of Body connected with this Body in Backward direction)
    int BackwardConnectionCount();

    ///Get Connection by index in forward direction
    Connection * GetForwardConnection(int _index);
    ///Get Connection by index in backward direction
    Connection * GetBackwardConnection(int _index);

    ///Set up a forward connectionship with the given connection
    void AddForwardConnection(Connection * _connection);
    ///Set up a backward connectionship with the given connection
    void AddBackwardConnection(Connection * _connection);

    /**Delete a given connection from forward connections.
      *if _delete is true, then this connection will be deallocated
      */
    void RemoveForwardConnection(Connection * _connection, int _delete);
    /**Delete a given connection from backward connections.
      *if _delete is true, then this connection will be deallocated
      */
    void RemoveBackwardConnection(Connection * _connection, int _delete);


    /**Create a connection use these given info.
      *@param _otherBody The next (forward) body connected with this instance.
      *@see Connection, Link(Connection *)
      */
    void Link(Body * _otherBody, const Transformation & _transformationToBody2,
    const DHparameters & _dhparameters, const Transformation &_transformationToDHFrame);
    /*Add this connection to forward connection list in this body
     *and add this connection to backward connection list in next body (defined in connection)
     */
    void Link(Connection * _c);
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:

    MultiBody * multibody;                  ///<Owner of this Body
    Transformation worldTransformation;     ///<World Transformation

    /*@name Geometry Related Data*/
    //@{
    GMSPolyhedron polyhedron;               ///<Geometry of this Body defined in local coordinate system.
    GMSPolyhedron worldPolyhedron;          ///<Geometry of this Body defined in world coordinate system.
    char polyhedronFileName[32];            ///<File name of geometric data
    bool CenterOfMassAvailable;             ///<Is the center of mass valid?
    Vector3D CenterOfMass;                  ///<The center of mass of a given polyhedron
    double boundingBox[6];                  ///<The Box which enclose every sinlge point on this Body
    //@}

    /*@name Connection Related Data*/
    //@{
    int forwardConnectionCount;             ///<Number of Forward connection
    Connection ** forwardConnection;        ///<Forward Connections
    int backwardConnectionCount;            ///<Number of Backward connection
    Connection ** backwardConnection;       ///<Backward Connections
    int contactCount;
    //@}

    //int IsConvex;
    //Material material;
    //Uncertainty uncertainty;


#ifdef USE_CSTK
    void * cstkBody[MAXPROCS];  ///<CSTK internal model (maybe?!)
#endif
#ifdef USE_VCLIP
    PolyTree *vclipBody;    ///<VCLIP internal model
#endif
#ifdef USE_RAPID
    RAPID_model *rapidBody; ///<RAPID internal model
#endif

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Friend info
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
friend class MultiBody; //Owner
friend class Connection;
};

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  Implementation of Body
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

//===================================================================
//  Inline functions
//===================================================================

/** isAdjacent:
 to check if two Body share same joint(adjacent) for a robot. */
inline bool Body::isAdjacent(Body * otherBody) {
    for(int i=0; i < forwardConnectionCount; i++)
        if(forwardConnection[i]->GetNextBody() == otherBody)
            return true;
    for(int j=0; j < backwardConnectionCount; j++)
        if(backwardConnection[j]->GetPreviousBody() == otherBody)
            return true;

    return (this == otherBody); // if the two are the same, return true too.
}

/** WorldTransformation

 If worldTransformation has been calculated(updated), this method should be used
 to avoid redundant calculation.
*/ 
inline Transformation & Body::WorldTransformation() {
    return worldTransformation;
}


///  GetBoundingBox
inline double * Body::GetBoundingBox(){
    return boundingBox;
}

///  GetCenterOfMass
inline Vector3D Body::GetCenterOfMass(){
    if (!CenterOfMassAvailable) {
        ComputeCenterOfMass();
    }
    return CenterOfMass;
}

///  GetMultiBody
inline MultiBody * Body::GetMultiBody() {
    return multibody;
}

//  ContactCount
//inline int Body::GetContactCount() {
//    return contactCount;
//}

///  ForwardConnectionCount
inline int Body::ForwardConnectionCount() {
    return forwardConnectionCount;
}

///  BackwardConnectionCount
inline int Body::BackwardConnectionCount() {
    return backwardConnectionCount;
}

//  GetContact
//inline Contact * Body::GetContact(int _index) {
//    if (_index < contactCount)
//        return contact[_index];
//    else
//        return 0;
//}

///  GetForwardConnection
inline Connection * Body::GetForwardConnection(int _index) {
    if (_index < forwardConnectionCount)
        return forwardConnection[_index];
    else
        return 0;
}

///  GetBackwardConnection
inline Connection * Body::GetBackwardConnection(int _index) {
    if (_index < backwardConnectionCount)
        return backwardConnection[_index];
    else
        return 0;
}

/**  AddContactCount
  Function: Increment the contact count
*/  
inline void Body::AddContactCount() {
  contactCount++;
}

/**
  PutWorldTransformation

  Function: Assign the given transformation as a transformation
            w.r.t the world for "this" body
*/
inline void Body::PutWorldTransformation(Transformation & _worldTransformation){
  worldTransformation = _worldTransformation;
}

#endif
