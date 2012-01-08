/**
  * @file MultiBody.h
  * @date 2/25/98
  * @author Aaron Michalk
  */
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef MultiBody_h
#define MultiBody_h

////////////////////////////////////////////////////////////////////////////////////////////
//include OBPRM headers
#include "FixedBody.h"
#include "FreeBody.h"
#include "Graph.h"
 

///////////////////////////////////////////////////////////////////////////////////////////////////////
class Environment;
class Transformation;

//////////////////////////////////////////////////////////////////////////////////////////////////////
//typedef pair<stapl::VID,int> RANGE_TYPE;

//////////////////////////////////////////////////////////////////////////////////////////////////////
/**A MultiBody represent a Obstacle or a Robot in workspace.
  *MultiBody contain one or more Bodys, either Fixed or Free Bodys.
  *Many access methods are implemented to allow client access internal information
  *about MultiBody instance, like number of Bodys, Fixed and Free, Bounding box,
  *center of mass, surface area size, and bounding sphere radius.
  *
  *@see Body, FreeBody, and FixedBody
  */
class MultiBody {
public:
    //-----------------------------------------------------------
    //  Static Methods
    //-----------------------------------------------------------

    /**@todo Check this!! ComputePUMAInverseKinematics ??
     */
    static void ComputePUMAInverseKinematics(Transformation & _t, double _a2, double _d3, double _a3, double _d4, double theta[8][6]);


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor*/
    //@{

    ///Constructor. Set _owner as the owner of this MultiBody instance.
    MultiBody();

    ///Destrucot. Free memory allocated to all Bodys added to this multiBody.
    virtual ~MultiBody();

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

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Get/Set Free Body info
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    
    //added by Xinyu Tang
    //To say whether this multibody is external (fake obstacle);
    bool IsInternal() const
    {
      return bInternal;
    } 

    ///Return a free body accroding to the given index. the index should be in [0,GetFreeBodyCount())
    shared_ptr<FreeBody> GetFreeBody(int _index) const;
    ///Number of free body in this mutilbody.
    int GetFreeBodyCount() const;
    ///Search index for given FreeBody, _b, if _b is not in this multibody, -1 is returned.
    int GetFreeBodyIndex(const FreeBody& _b) const;
    int GetFreeBodyIndex(const shared_ptr<FreeBody>& _b) const;
    ///Add a Free Body
    void AddBody(const FreeBody& _body);
    void AddBody(const shared_ptr<FreeBody>& _body);


    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Get/Set Fixed Body info
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    ///Return a fixed body accroding to the given index. the index should be in [0,GetFixedBodyCount())
    shared_ptr<FixedBody> GetFixedBody(int _index) const;
    ///Number of fixed body in this mutilbody.
    int GetFixedBodyCount() const;
    ///Search index for given FixedBody, _b, if _b is not in this multibody, -1 is returned.
    int GetFixedBodyIndex(const FixedBody& _b) const;
    int GetFixedBodyIndex(const shared_ptr<FixedBody>& _b) const;
    ///Add a Fixed Body
    void AddBody(const FixedBody& _body);
    void AddBody(const shared_ptr<FixedBody>& _body);

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Get/Set Body info
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    /**Return a free body accroding to the given index.
      *_index should in [0, GetFreeBodyCount()+GetFixedBodyCount())
      *if _index is smaller than GetFixedBodyCount(), then fixed body will be returned.
      *if _index is larger than GetFixedBodyCount(), then free body with (_index-GetFixedBodyCount())
      *index will be returned.
      */
    shared_ptr<Body> GetBody(int _index) const; // new
    ///return GetFreeBodyCount()+GetFixedBodyCount()
    int GetBodyCount() const; // new
    /**Get the very first body in a MultiBody.
     *It is a fixed base body, if the MultiBody is a manipualtor
     *Otherwise, it is the body itself.
     */
    shared_ptr<Body> GetFirstBody() const;

    ///Get number of of links in "this" MultiBody by checking forward connection.
    int GetNumberOfLinks() const;

    /**Determine if the MultiBody at hand is a manipulator.
      *If there is no free body attached to it,
      *it is considered to be a manipulator
      */
    bool IsManipulator() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Get/Set Range info (bounding box, bounding sphere...etc.
    //
    //////////////////////////////////////////////////////////////////////////////////////////


    /**to get center of mass, you don't need to additionally call the above: ComputeCenterOfMass
      *because GetCenterOfMass will check if it is necessary to call this method.
      *@see ComputeCenterOfMass
      */
    Vector3D GetCenterOfMass();

    /**Return Max Axis Range, which is computed during finding bounding box.
      *Max Axis Range is max distance in X, Y, or Z direction in bounding box.
      *@see FindBoundingBox
      */
    double GetMaxAxisRange() const;

    /*Return bounding box of this multibody
     *@see FindBoundingBox
     */
    const double * GetBoundingBox() const;

    /**Compute and return the maximum size of this multibody.
      *The maximum size is computed by (Radius of first link+ 2*Radius of second link+ ... )
      *@see GMSPolyhedron::maxRadius
      */
    double GetBoundingSphereRadius() const;

    /**Compute and return the minimum size of the multibody.
     *@see GMSPolyhedron::minRadius
     */
    double GetInsideSphereRadius() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Get/Set Area info.
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    /**Get total area of fixed bodys in this instance. (computed in Get method)
      *@see Get, GetFixAreas
      */
    double GetFixArea() const;

    /**Get a list of areas of fixed bodys in this instance. 
      *(computed in Get method)
      *@see Get, GetFixArea
      */
    vector<double> GetFixAreas() const;

    /**Get total area of free bodys in this instance. (computed in Get method)
      *@see Get, GetFreeAreas
      */
    double GetFreeArea() const;

    /**Get a list of areas of free bodys in this instance. 
      *(computed in Get method)
      *@see Get, GetFreeArea
      */
    vector<double> GetFreeAreas() const;

    /**Get total area of free bodys and fixed bodys in this instance. 
      *(computed in Get method)
      *@see Get, GetFixArea, and GetFreeArea
      */
    double GetArea() const;

    /**Set area variables
     */
    void CalculateArea();

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Get/Set Area info.
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    /**Get user input information.
      *Number of bodys (fixed and free), areas, bounding box, and center of mass are all computed
      *in here.
      *@param _multibodyIndex index for the owner of this fixed body in input
      *@see FreeBody::Read, FixedBody::Read, Connection, FreeBody::Link, Connection::Read,
      *FixedBody::Link ,FindBoundingBox, and ComputeCenterOfMass
      */
    virtual void Read(istream& is, int action, const char* descDir, bool _debug = false);
    void buildCDstructure(cd_predefined cdtype);

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

    /**Write information about this MultiBody instance to outputstream.
      *First the tag "MultiBody was output, and then calls Fixed and (or) Free Bodys'
      *write and Connnection's write.
      *@see FixedBody::Write, FreeBody::Write, and Connection::Write
      */
    void Write(ostream & _os);

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Helpers
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Help Methods*/
    //@{

    /**Configure the joint by the given amount of displacement.
      *@param _dof Number of Freebody that is going to be reconfigured (moved)
      *@param _s An array of displacement value. The length of _s is _dof.
      */
    void ConfigureJoint(double * _s, int _dof);

    /**if GetCenterOfMass() is called for the first time, then
      *ComputeCenterOfMass() is called automatically, and the
      *computed value is stored in this class for the next time.
      *@see Body::ComputeCenterOfMass
      */
    void ComputeCenterOfMass();

    /**Calculate bounding box by FreeBodys and FixedBodys in this instance.
      *maxAxisRange is its byproduct...
      */
    void FindBoundingBox();

#ifdef USE_SOLID
    void UpdateVertexBase();
#endif

    //polygonal approximation
    void PolygonalApproximation(vector<Vector3D>& result);

    //@}

    void PolygonalApproximation();
 
    bool operator==(const MultiBody& mb) const;
    bool operator!=(const MultiBody& mb) const { return !(*this == mb); }


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

protected:    
  // Area Stuff
  double fixArea;             ///< Area of FixedBodies
  double freeArea;            ///< Area of FreeBodies
  double area;                ///< Total Area of Bodies
  vector<double> fixAreas;    ///< Vector of Areas of FixedBodies
  vector<double> freeAreas;   ///< Vector of Areas of FreeBodies

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

private:
    //-----------------------------------------------------------
    ///  Data
    //-----------------------------------------------------------
  //added by Xinyu Tang
  // to say whether this multibody is Internal(fake obsbacle);
  bool bInternal;

  vector<shared_ptr<FixedBody> > fixedBody;
  vector<shared_ptr<FreeBody> > freeBody;
  
  Vector3D CenterOfMass;
  bool CenterOfMassAvailable;
  
  double boundingBox[6];
  double maxAxisRange;
};

#endif
