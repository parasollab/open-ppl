#ifndef SURFACE_MULTI_BODY_H_
#define SURFACE_MULTI_BODY_H_

#include "StaticMultiBody.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief A collection of geometries representing a surface
///
/// SurfaceMultiBody stores surfaces. Currently only a single surface is
/// supported in PMPL per SurfaceMultiBody.
////////////////////////////////////////////////////////////////////////////////
class SurfaceMultiBody : public StaticMultiBody {
  public:

    SurfaceMultiBody();

    SurfaceMultiBody(const SurfaceMultiBody&) = delete;
    SurfaceMultiBody& operator=(const SurfaceMultiBody&) = delete;

    const string& GetLabel() const {return m_surfaceLabel;}

    virtual void Read(istream& is, CountingStreamBuffer& _cbs);
    virtual void Write(ostream & _os);

  private:
    string m_surfaceLabel;
};

#endif
