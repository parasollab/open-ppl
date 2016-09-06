#include "WorkspacePortal.h"
#include "WorkspaceDecomposition.h"
#include "WorkspaceRegion.h"

/*--------------------------- Construction -----------------------------------*/

WorkspacePortal::
WorkspacePortal() : m_decomposition(nullptr), m_sourceIndex(-1),
    m_targetIndex(-1) { }


WorkspacePortal::
WorkspacePortal(WorkspaceDecomposition* const _wd, const size_t _s,
    const size_t _t) : m_decomposition(_wd), m_sourceIndex(_s),
    m_targetIndex(_t) { }

/*-------------------------------- Accessors ---------------------------------*/

const WorkspaceRegion&
WorkspacePortal::
GetSource() const {
  return m_decomposition->GetRegion(m_sourceIndex);
}


const WorkspaceRegion&
WorkspacePortal::
GetTarget() const {
  return m_decomposition->GetRegion(m_targetIndex);
}

/*-------------------------------- Queries -----------------------------------*/

const vector<Point3d>
WorkspacePortal::
FindPoints() const {
  return GetSource().FindSharedPoints(GetTarget());
}


const vector<const WorkspacePortal::Facet*>
WorkspacePortal::
FindFacets() const {
  return GetSource().FindSharedFacets(GetTarget());
}

/*----------------------------------------------------------------------------*/
