#ifndef MULTI_BODY_H_
#define MULTI_BODY_H_

#include "MPProblem/Geometry/Body.h"

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

    enum class BodyType {Active, Passive, Surface, Internal};


    ///Constructor. Set _owner as the owner of this MultiBody instance.
    MultiBody();

    MultiBody(const MultiBody&) = delete;
    MultiBody& operator=(const MultiBody&) = delete;

    ///Destructor. Free memory allocated to all Bodys added to this multiBody.
    virtual ~MultiBody();

    static BodyType GetBodyTypeFromTag(const string& _tag, const string& _where);
    static string GetTagFromBodyType(const BodyType& _b);

    void SetBodyType(BodyType _newType){m_bodyType = _newType;}
    BodyType GetBodyType() const{return m_bodyType;}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Get/Set Body info
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    //size_t GetBodyIndex(shared_ptr<Body>& _body) const;
    /**Return a free body according to the given index.
     *_index should be in [0, GetFreeBodyCount()+GetFixedBodyCount())
     *if _index is smaller than GetFixedBodyCount(), then fixed body will be returned.
     *if _index is larger than GetFixedBodyCount(), then free body with (_index-GetFixedBodyCount())
     *index will be returned.
     */
    shared_ptr<Body> GetBody(size_t _index) const; // new
    ///return GetFreeBodyCount()+GetFixedBodyCount()
    size_t GetBodyCount() const; // new
    void AddBody(const shared_ptr<Body>& _body);

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

    void BuildCDStructure(cd_predefined _cdtype);

    bool IsInternal() const;

    bool IsSurface() const;

    bool IsActive() const;

    bool IsPassive() const;

    /**Get user input information.
     *Number of bodys (fixed and free), areas, bounding box, and center of mass are all computed
     *in here.
     */
    virtual void Read(istream& is, CountingStreamBuffer& _cbs) = 0;

    /**Write information about this MultiBody instance to outputstream.
     *First the tag "MultiBody was output, and then calls Fixed and (or) Free Bodys'
     *write and Connnection's write.
     */
    virtual void Write(ostream & _os) = 0;

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

  private:

    string m_modelDataDir; //directory where environment file is stored

    bool m_comAvailable;

    vector<shared_ptr<Body>> m_bodies;

    BodyType m_bodyType; //Active, Passive, Surface, Internal

    Vector3d CenterOfMass;

    double m_boundingBox[6];
    double m_maxAxisRange;
};

#endif
