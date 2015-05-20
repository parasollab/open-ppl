#ifndef STATIC_MULTI_BODY_H_
#define STATIC_MULTI_BODY_H_

#include "MultiBody.h"

class FixedBody;

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
class StaticMultiBody : public MultiBody {
  public:

    StaticMultiBody();

    StaticMultiBody(const StaticMultiBody&) = delete;
    StaticMultiBody& operator=(const StaticMultiBody&) = delete;

    ///Return a fixed body accroding to the given index. the index should be in [0,GetFixedBodyCount())
    shared_ptr<FixedBody> GetFixedBody(size_t _index) const;
    ///Add a Fixed Body
    void AddBody(const shared_ptr<FixedBody>& _body);

    virtual void Read(istream& is, CountingStreamBuffer& _cbs);
    virtual void Write(ostream & _os);
  private:

    vector<shared_ptr<FixedBody> > fixedBody;

    string m_surfaceLabel;
};

#endif
