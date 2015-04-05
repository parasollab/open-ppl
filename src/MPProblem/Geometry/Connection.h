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

#include <Transformation.h>
using namespace mathtool;

#include "MPProblem/Geometry/DHparameters.h"
#include "MPProblem/Robot.h"

class Body;
class MultiBody;

class Connection {
  public:
    enum JointType {REVOLUTE, SPHERICAL, NONACTUATED}; //1dof vs 2dof rotational joints

    Connection(MultiBody* _owner);
    Connection(const shared_ptr<Body>& _body1, const shared_ptr<Body>& _body2 = shared_ptr<Body>());
    Connection(const shared_ptr<Body>& _body1, const shared_ptr<Body>& _body2,
        const Transformation & _transformationToBody2,
        const DHparameters & _dhparameters,
        const Transformation & _transformationToDHFrame);

    virtual ~Connection();

    static JointType GetJointTypeFromTag(const string _tag);
    static string GetTagFromJointType(const JointType& _jt);
    /**Check if a give body is the first body of this connection instance or not.
     */
    bool IsFirstBody(const shared_ptr<Body>& _body) const {return _body == m_bodies[0];}

    //connection index
    size_t GetGlobalIndex() const {return m_globalIndex;}

    //connection type and limits
    JointType GetConnectionType() const {return m_jointType;}
    void SetConnectionType(JointType _jt) {m_jointType=_jt;}
    const pair<double, double>& GetJointLimits(int _i) const {return m_jointLimits[_i];}

    //body indices
    shared_ptr<Body> GetPreviousBody() const {return m_bodies[0];}
    size_t GetPreviousBodyIndex() const {return m_bodyIndices.first;}
    shared_ptr<Body> GetNextBody() const {return m_bodies[1];}
    size_t GetNextBodyIndex() const {return m_bodyIndices.second;}

    //Joint transformations
    DHparameters& GetDHparameters() {return m_dhParameters;}
    const DHparameters& GetDHparameters() const {return m_dhParameters;}
    Transformation& GetTransformationToBody2() {return m_transformationToBody2;}
    const Transformation& GetTransformationToBody2() const {return m_transformationToBody2;}
    Transformation& GetTransformationToDHFrame() {return m_transformationToDHFrame;}
    const Transformation& GetTransformationToDHFrame() const {return m_transformationToDHFrame;}

    friend ostream& operator<<(ostream& _os, const Connection& _c);
    friend istream& operator>>(istream& _is, Connection& _c);

    bool operator==(const Connection& c) const;

    friend class MultiBody;

  private:
    MultiBody* m_multibody; // Owner of this Connection
    shared_ptr<Body> m_bodies[2];
    Transformation m_transformationToBody2;
    Transformation m_transformationToDHFrame;
    DHparameters m_dhParameters;

    size_t m_globalIndex;
    JointType m_jointType;
    pair<size_t, size_t> m_bodyIndices; //(previous body, next body)
    pair<double, double> m_jointLimits[2]; //valid range within [-1,1)

    static size_t m_globalCounter; //global counter for connection indices
};

#endif


