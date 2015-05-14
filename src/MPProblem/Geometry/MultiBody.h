#ifndef MULTI_BODY_H_
#define MULTI_BODY_H_

#include "MPProblem/Geometry/FixedBody.h"
#include "MPProblem/Geometry/FreeBody.h"

class Boundary;
class Environment;

enum DofType {POS, ROT, JOINT};

enum BodyType {ACTIVE, PASSIVE, SURFACE, INTERNAL};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief A collection of geometries in workspace reprenting, e.g., robots
///
/// A MultiBody represent a Obstacle or a Robot in workspace. MultiBody contain
/// one or more Body s, either FixedBody s or FreeBody s. Many access methods
/// are implemented to allow client access internal information about MultiBody
/// instance, like number of Body s, Fixed and Free, Bounding box, center of
/// mass, surface area size, bounding sphere radius, etc.
////////////////////////////////////////////////////////////////////////////////
class MultiBody {
  public:

    typedef shared_ptr<Connection> Joint;
    typedef vector<Joint> JointMap;
    typedef JointMap::iterator JointIT;

    //-----------------------------------------------------------
    //  Static Methods
    //-----------------------------------------------------------

    /**@todo Check this!! ComputePUMAInverseKinematics ??
    */
    static void ComputePUMAInverseKinematics(Transformation & _t,
        double _a2, double _d3, double _a3, double _d4, double _theta[8][6]);


    ///Constructor. Set _owner as the owner of this MultiBody instance.
    MultiBody();

    MultiBody(const MultiBody&) = delete;
    MultiBody& operator=(const MultiBody&) = delete;

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
    void Initialize(string _modelFile,
        const Transformation& _where = Transformation(),
        BodyType _type=PASSIVE);

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
    void AddBody(const shared_ptr<FreeBody>& _body);


    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Get/Set Fixed Body info
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    ///Return a fixed body accroding to the given index. the index should be in [0,GetFixedBodyCount())
    shared_ptr<FixedBody> GetFixedBody(int _index) const;
    ///Add a Fixed Body
    void AddBody(const shared_ptr<FixedBody>& _body);

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Get/Set Body info
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    int GetBodyIndex(shared_ptr<Body>& _body) const;
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

    /**Get user input information.
     *Number of bodys (fixed and free), areas, bounding box, and center of mass are all computed
     *in here.
     */
    void Read(istream& is, CountingStreamBuffer& _cbs);

    void InitializeDOFs(ostream* _os = NULL);

    void buildCDstructure(cd_predefined cdtype);

    bool IsInternal() const;

    bool IsSurface() const;

    bool IsActive() const;

    bool IsPassive() const;

    /**Write information about this MultiBody instance to outputstream.
     *First the tag "MultiBody was output, and then calls Fixed and (or) Free Bodys'
     *write and Connnection's write.
     */
    void Write(ostream & _os);

    /**Configure the joint by the given amount of displacement.
    */
    void ConfigureJoint(double * _s, int _dof);

    /**if GetCenterOfMass() is called for the first time, then
     *ComputeCenterOfMass() is called automatically, and the
     *computed value is stored in this class for the next time.
     */
    void ComputeCenterOfMass();

    /**Calculate bounding box by FreeBodys and FixedBodys in this instance.
     *m_maxAxisRange is its byproduct...
     */
    void FindBoundingBox();

#ifdef USE_SOLID
    void UpdateVertexBase();
#endif

    //polygonal approximation
    void PolygonalApproximation(vector<Vector3d>& result);

    void PolygonalApproximation();

    JointMap& GetJointMap() {return m_joints;}
    size_t NumJoints() const {return m_joints.size();}
    string GetLabel() { return m_label; }
    void SetLabel(string _label) { m_label = _label; }


    size_t m_baseIndex; //free body index for base
    shared_ptr<Body> m_baseBody;
    Body::Base m_baseType;
    Body::BaseMovement m_baseMovement;

    JointMap m_joints;

    const vector<DofType>& GetDOFTypes() const {return m_dofTypes;}
    size_t DOF() const {return m_dofTypes.size();}
    size_t PosDOF() const;

    void Configure(const vector<double>& _v);
    vector<double> GetRandomCfg(shared_ptr<Boundary>& _boundary);

  private:

    string m_modelDataDir; //directory where environment file is stored

    //does the multibody contain more than one robot
    bool m_comAvailable;

    BodyType m_bodyType; //ACTIVE, PASSIVE, SURFACE, INTERNAL

    vector<shared_ptr<FixedBody> > fixedBody;
    vector<shared_ptr<FreeBody> > freeBody;

    Vector3d CenterOfMass;

    double m_boundingBox[6];
    double m_maxAxisRange;

    string m_label;
    vector<DofType> m_dofTypes;
};

#endif
