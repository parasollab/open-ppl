#ifndef ACTIVE_MULTI_BODY_H_
#define ACTIVE_MULTI_BODY_H_

#include "MultiBody.h"

class Boundary;
class FreeBody;

enum class DofType {Positional, Rotational, Joint};

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
class ActiveMultiBody : public MultiBody {
  public:
    typedef shared_ptr<Connection> Joint;
    typedef vector<Joint> JointMap;
    typedef JointMap::iterator JointIT;

    ActiveMultiBody();

    ActiveMultiBody(const ActiveMultiBody&) = delete;
    ActiveMultiBody& operator=(const ActiveMultiBody&) = delete;

    //virtual ~ActiveMultiBody();

    ///Return a free body accroding to the given index. the index should be in [0,GetFreeBodyCount())
    shared_ptr<FreeBody> GetFreeBody(size_t _index) const;
    ///Number of free body in this mutilbody.
    size_t GetFreeBodyCount() const;
    ///Search index for given FreeBody, _b, if _b is not in this multibody, -1 is returned.
    size_t GetFreeBodyIndex(const FreeBody& _b) const;
    size_t GetFreeBodyIndex(const shared_ptr<FreeBody>& _b) const;
    ///Add a Free Body
    void AddBody(const shared_ptr<FreeBody>& _body);

    shared_ptr<Body> GetFirstBody() const;

    ///Get number of of links in "this" MultiBody by checking forward connection.
    size_t GetNumberOfLinks() const;

    /**Determine if the MultiBody at hand is a manipulator.
     *If there is no free body attached to it,
     *it is considered to be a manipulator
     */
    bool IsManipulator() const;

    void InitializeDOFs(ostream* _os = NULL);

    //polygonal approximation
    void PolygonalApproximation(vector<Vector3d>& result);

    /**Configure the joint by the given amount of displacement.
    */
    void ConfigureJoint(double * _s, size_t _dof);

    JointMap& GetJointMap() {return m_joints;}
    size_t NumJoints() const {return m_joints.size();}

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

    virtual void Read(istream& is, CountingStreamBuffer& _cbs);
    virtual void Write(ostream & _os);

  private:

    vector<shared_ptr<FreeBody> > freeBody;
    vector<DofType> m_dofTypes;

};

#endif
