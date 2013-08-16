/**A MultiBody represent a Obstacle or a Robot in workspace.
  *MultiBody contain one or more Bodys, either Fixed or Free Bodys.
  *Many access methods are implemented to allow client access internal information
  *about MultiBody instance, like number of Bodys, Fixed and Free, Bounding box,
  *center of mass, surface area size, and bounding sphere radius.
  */

#ifndef MULTIBODY_H_
#define MULTIBODY_H_

#include "MPProblem/Geometry/FixedBody.h"
#include "MPProblem/Geometry/FreeBody.h"
#include "Graph.h"
#include "MPProblem/Robot.h"

class Environment;

enum BodyType{ACTIVE, PASSIVE, SURFACE, INTERNAL};

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
    ///Constructor. Set _owner as the owner of this MultiBody instance.
    MultiBody();

    ///Destructor. Free memory allocated to all Bodys added to this multiBody.
    virtual ~MultiBody();

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Get/Set Free Body info
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    void Initialize(string _modelFile, const Transformation& _where = Transformation(), BodyType _type=PASSIVE);

    void SetBodyType(BodyType _newType){m_bodyType = _newType;}
    BodyType GetBodyType() const{return m_bodyType;}

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

    /**Return a free body according to the given index.
      *_index should be in [0, GetFreeBodyCount()+GetFixedBodyCount())
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
      */
    Vector3d GetCenterOfMass();

    /**Return Max Axis Range, which is computed during finding bounding box.
      *Max Axis Range is max distance in X, Y, or Z direction in bounding box.
      */
    double GetMaxAxisRange() const;

    /*Return bounding box of this multibody
     */
    const double * GetBoundingBox() const;

    /**Compute and return the maximum size of this multibody.
      *The maximum size is computed by (Radius of first link+ 2*Radius of second link+ ... )
      */
    double GetBoundingSphereRadius() const;

    /**Compute and return the minimum size of the multibody.
     */
    double GetInsideSphereRadius() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Get/Set Area info.
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    /**Get total area of fixed bodys in this instance. (computed in Get method)
      */
    double GetFixArea() const;

    /**Get a list of areas of fixed bodys in this instance.
      *(computed in Get method)
      */
    vector<double> GetFixAreas() const;

    /**Get total area of free bodys in this instance. (computed in Get method)
      */
    double GetFreeArea() const;

    /**Get a list of areas of free bodys in this instance.
      *(computed in Get method)
      */
    vector<double> GetFreeAreas() const;

    /**Get total area of free bodys and fixed bodys in this instance.
      *(computed in Get method)
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
      */
    virtual void Read(istream& is, bool _debug = false);

    void buildCDstructure(cd_predefined cdtype);

    bool IsInternal() const;

    bool IsSurface() const;

    bool IsActive() const;

    bool IsPassive() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    I/O
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**Write information about this MultiBody instance to outputstream.
     *First the tag "MultiBody was output, and then calls Fixed and (or) Free Bodys'
     *write and Connnection's write.
     */
    void Write(ostream & _os);

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Helpers
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    /**Configure the joint by the given amount of displacement.
     */
    void ConfigureJoint(double * _s, int _dof);

    /**if GetCenterOfMass() is called for the first time, then
     *ComputeCenterOfMass() is called automatically, and the
     *computed value is stored in this class for the next time.
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
    void PolygonalApproximation(vector<Vector3d>& result);

    //@}

    void PolygonalApproximation();

    bool operator==(const MultiBody& mb) const;
    bool operator!=(const MultiBody& mb) const { return !(*this == mb); }

    Robot::JointMap& GetJointMap() {return jointMap;}
    void SetMultirobot(bool _m){m_multirobot = _m;}

    string GetLabel() { return m_label; }
    void SetLabel(string _label) { m_label = _label; }
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

    //does the multibody contain more than one robot
    bool m_multirobot;
    bool CenterOfMassAvailable;

    BodyType m_bodyType; //ACTIVE, PASSIVE, SURFACE, INTERNAL

    vector<shared_ptr<FixedBody> > fixedBody;
    vector<shared_ptr<FreeBody> > freeBody;

    Vector3d CenterOfMass;

    double boundingBox[6];
    double maxAxisRange;

    Robot::JointMap jointMap;

    string m_label;
};

#endif
