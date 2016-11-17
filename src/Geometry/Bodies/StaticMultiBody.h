#ifndef STATIC_MULTI_BODY_H_
#define STATIC_MULTI_BODY_H_

#include "MultiBody.h"

class FixedBody;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Geometry
/// @brief A collection of geometries in workspace reprenting obstacles
///
/// StaticMultiBody stores obstacles. Currently only a single obstacle is
/// supported in PMPL per StaticMultiBody.
////////////////////////////////////////////////////////////////////////////////
class StaticMultiBody : public MultiBody {

  public:

    ///@name Construction
    ///@{

    StaticMultiBody(MultiBodyType _m);

    StaticMultiBody(const StaticMultiBody&) = delete;            ///< No copy
    StaticMultiBody& operator=(const StaticMultiBody&) = delete; ///< No assign

    virtual ~StaticMultiBody() = default;

    ///@}
    ///@name MultiBody Info
    ///@{

    using MultiBody::MultiBodyType;

    /// Get the type for this MultiBody.
    virtual MultiBodyType GetType() const noexcept override {
      return IsInternal() ? MultiBodyType::Internal : MultiBodyType::Passive;
    }

    ///@}

    /// Initialize with an object file and transform.
    /// @param _modelFileName path to .obj file that specifies obstacle geometry.
    /// @param _where a 6 dof vector specifying position and orientation
    void Initialize(const string& _modelFileName, const Transformation& _where);

    /// Is this MultiBody an internal type?
    bool IsInternal() const;

    /// Get a fixed sub-body by index.
    /// @param[in] _index The index of the sub-body to fetch.
    shared_ptr<FixedBody> GetFixedBody(size_t _index) const;

    /// @param _body Fixed Body to add
    void AddBody(const shared_ptr<FixedBody>& _body);

    virtual void Read(istream& _is, CountingStreamBuffer& _cbs);
    virtual void Write(ostream& _os);

  protected:

    vector<shared_ptr<FixedBody>> m_fixedBody; ///< All fixed body
};

#endif
