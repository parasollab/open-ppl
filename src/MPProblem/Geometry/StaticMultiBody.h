#ifndef STATIC_MULTI_BODY_H_
#define STATIC_MULTI_BODY_H_

#include "MultiBody.h"

class FixedBody;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief A collection of geometries in workspace reprenting obstacles
///
/// StaticMultiBody stores obstacles. Currently only a single obstacle is
/// supported in PMPL per StaticMultiBody.
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

  protected:
    vector<shared_ptr<FixedBody>> fixedBody;
};

#endif
