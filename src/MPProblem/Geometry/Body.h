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
//include CD headers
#ifdef USE_VCLIP
#include <vclip.h>
#endif
#ifdef USE_PQP
#include <PQP.h>
#endif
#ifdef USE_RAPID
#include <RAPID.H>
#endif
#ifdef USE_SOLID
#include <SOLID.h>
#include "DT_Polyhedron.h"
#include "DT_Polytope.h"
#endif

////////////////////////////////////////////////////////////////////////////////////////////
//include OBPRM headers
#include "GMSPolyhedron.h"
#include "Connection.h"
#include "Robot.h"
#include "boost/shared_ptr.hpp"
using boost::shared_ptr;

///////////////////////////////////////////////////////////////////////////////////////////////////////////
class MultiBody;
class Transformation;
class DHparameters;
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
    Body(MultiBody* _owner);
    /**Constructor. Set owner and geometric information of this instance of Body. 
      *initialize data members to 0 ,NULL, and false.
      */
    Body(MultiBody* _owner, GMSPolyhedron & _polyhedron);
    ///Destructor. Free Forward (backward) connection information here.

    Body(const Body& b);

    virtual ~Body();
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
       virtual GMSPolyhedron & GetWorldBoundingBox();

        ///Update internal data, worldPolyhedron, accroding current world transformation of this body.
        virtual void ChangeWorldPolyhedron();

        ///Return the local coordinates of the body
        virtual GMSPolyhedron & GetPolyhedron();
        virtual GMSPolyhedron & GetBoundingBoxPolyhedron();


        ///Get the owner of this Body which is set in constrcutor.
        MultiBody* GetMultiBody();

        /**Return a bounding box of this instance of Body.
          *@see FindBoundingBox
          */
        double * GetBoundingBox();

        /**to get center of mass, you don't need to additionally call the above: ComputeCenterOfMass
          *because GetCenterOfMass will check if it is necessary to call this method.
          *@see ComputeCenterOfMass
          */
        Vector3D GetCenterOfMass();

        bool IsBase() { return isBase; };
        Robot::Base GetBase() { return baseType; };
        Robot::BaseMovement GetBaseMovement() { return baseMovementType; };
        void SetBase(Robot::Base _baseType) { baseType = _baseType; };
        void SetBaseMovement(Robot::BaseMovement _baseMovementType) { baseMovementType = _baseMovementType; };

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
    void Read(string _fileName);

    /**Read BYU format data from given inpustream. 
      *Call GMSPolyhedron::ReadBYU, calculate the bounding box, and then call buildCDstructure
      *to create auxilary data structure for collision detection.
      *@see GMSPolyhedron::ReadBYU, FindBoundingBox, buildCDstructure
      */
    void ReadBYU(istream & _is);

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
    bool isAdjacent(shared_ptr<Body> otherBody);
    bool isWithinI(shared_ptr<Body> otherBody,int i);
    bool isWithinIHelper(Body* body1, Body* body2, int i,Body* prevBody);
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
      */
    void buildCDstructure(cd_predefined cdtype);

#ifdef USE_VCLIP
        shared_ptr<PolyTree> GetVClipBody();  ///<Return VCLIP internal model
#endif
#ifdef USE_RAPID
        shared_ptr<RAPID_model> GetRapidBody(); ///<Return RAPID internal model
#endif
#ifdef USE_PQP
        shared_ptr<PQP_Model> GetPQPBody(); ///<Return PQP internal model
#endif
#ifdef USE_SOLID
	shared_ptr<DT_ObjectHandle> GetSolidBody(); ///<Return SOLID internal model
#endif

#ifdef USE_SOLID
	void UpdateVertexBase(); ///<Changes the VertexBase 
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
    int ForwardConnectionCount() const;
    ///Number of Backward connection (# of Body connected with this Body in Backward direction)
    int BackwardConnectionCount() const;

    ///Get Connection by index in forward direction
    Connection & GetForwardConnection(size_t _index);
    ///Get Connection by index in backward direction
    Connection & GetBackwardConnection(size_t _index);

    ///Set up a forward connectionship with the given connection
    void AddForwardConnection(Connection _connection);
    ///Set up a backward connectionship with the given connection
    void AddBackwardConnection(Connection _connection);

    /**Create a connection use these given info.
      *@param _otherBody The next (forward) body connected with this instance.
      *@see Connection, Link(Connection *)
      */
    void Link(const shared_ptr<Body>& _otherBody, const Transformation & _transformationToBody2,
    const DHparameters & _dhparameters, const Transformation &_transformationToDHFrame);
    /*Add this connection to forward connection list in this body
     *and add this connection to backward connection list in next body (defined in connection)
     */
    void Link(Connection _c);
    //@}

    bool operator==(const Body& b) const;
    bool operator!=(const Body& b) const { return !(*this == b); }

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:

    string m_filename;
    MultiBody* multibody;                  ///<Owner of this Body
    Transformation worldTransformation;     ///<World Transformation
    bool isBase;                           ///<Is this a base?
    Robot::Base baseType;                  ///<If its a base, this needs to be set later
    Robot::BaseMovement baseMovementType;  ///<If its a base, this also needs to be set
    /*@name Geometry Related Data*/
    //@{
    GMSPolyhedron polyhedron;               ///<Geometry of this Body defined in local coordinate system.
    GMSPolyhedron worldPolyhedron;          ///<Geometry of this Body defined in world coordinate system.
    bool CenterOfMassAvailable;             ///<Is the center of mass valid?
    Vector3D CenterOfMass;                  ///<The center of mass of a given polyhedron
    double boundingBox[6];                  ///<The Box which enclose every sinlge point on this Body
    GMSPolyhedron bb_polyhedron;
    GMSPolyhedron bb_world_polyhedron;
    //@}

    /*@name Connection Related Data*/
    //@{
    vector<Connection> forwardConnection;
    vector<Connection> backwardConnection;
    //@}

#ifdef USE_VCLIP
    shared_ptr<PolyTree> vclipBody;    ///<VCLIP internal model
#endif
#ifdef USE_RAPID
    shared_ptr<RAPID_model> rapidBody; ///<RAPID internal model
#endif
#ifdef USE_PQP
    shared_ptr<PQP_Model> pqpBody; ///<PQP internal model
#endif
#ifdef USE_SOLID
    shared_ptr<DT_ObjectHandle> solidBody; ///<SOLID internal model
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

#ifdef USE_SOLID
    DT_VertexBaseHandle base;			//handles SOLID vertices
    MT_Point3* vertex;				//the actual SOLID vertex positions
#endif

};

#endif
