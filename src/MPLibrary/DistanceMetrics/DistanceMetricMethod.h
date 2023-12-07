#ifndef PMPL_DISTANCE_METRIC_METHOD_H
#define PMPL_DISTANCE_METRIC_METHOD_H

#include "MPLibrary/MPBaseObject.h"


////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for \ref DistanceMetrics.
///
/// DistanceMetricMethod has two important methods: @c Distance and @c ScaleCfg.
///
/// @c Distance takes as input two configurations \f$c_1\f$ and \f$c_2\f$ and
/// returns the computed transition distance between them.
/// @usage
/// @code
/// auto dm = this->GetDistanceMetric(m_dmLabel);
/// Cfg c1, c2;
/// double dist = dm->Distance(c1, c2);
/// @endcode
///
/// @c ScaleCfg is purposed to scale a \f$d\f$-dimensional ray in @cspace to a
/// certain magnitude based upon a general @dm.
/// @usage
/// @code
/// auto dm = this->GetDistanceMetric(m_dmLabel);
/// CfgType ray, origin;
/// double length;
/// dm->ScaleCfg(length, ray, origin);
/// @endcode
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
class DistanceMetricMethod  : public MPBaseObject {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID        VID;
    typedef typename MPBaseObject::GroupCfgType GroupCfgType;
    typedef typename GroupCfgType::Formation Formation;

    ///@}
    ///@name Construction
    ///@{

    DistanceMetricMethod() = default;
    DistanceMetricMethod(XMLNode& _node);
    virtual ~DistanceMetricMethod() = default;

    ///@}
    ///@name Distance Interface
    ///@{

    /// Compute a distance between two configurations.
    /// @param _c1 The first configuration.
    /// @param _c2 The second configuration.
    /// @return The computed distance between _c1 and _c2.
    virtual double Distance(const Cfg& _c1, const Cfg& _c2) = 0;

    /// This version is for group configurations. The default implementation
    /// returns the summed individual distances.
    /// @overload
    virtual double Distance(const GroupCfgType& _c1, const GroupCfgType& _c2);
    ///@example DistanceMetric_UseCase.cpp
    /// This is an example of how to use the distance metric methods.

    /// Compute the weight of an existing roadmap edge according to this metric.
    /// @param _r    The containing roadmap.
    /// @param _edge The edge iterator.
    /// @return The total edge weight computed through each intermediate of the
    ///         edge.
    /// @todo We need to make this work for group roadmaps. Probably it needs to
    ///       take the roadmap as a templated parameter and not be virtual
    ///       (there is really only one way to do this).
    double EdgeWeight(const RoadmapType* const _r,
        const typename RoadmapType::CEI& _edge) noexcept;

    /// Compute the weight of an existing roadmap edge according to this metric.
    /// @param _r The containing roadmap.
    /// @param _source The source vertex descriptor.
    /// @param _target The target vertex descriptor.
    /// @return The total edge weight computed through each intermediate of the
    ///         edge (_source, _target) in _r.
    /// @throws If the edge (_source, _target) does not exist in _r.
    double EdgeWeight(const RoadmapType* const _r, const VID _source,
        const VID _target) noexcept;

    ///@}
    ///@name Configuration Scaling
    ///@{
    /// These functions rescale a configuration vector based on this distance
    /// metric.

    /// Scale a directional configuration to a certain magnitude.
    /// @param _length Desired magnitude.
    /// @param _c Configuration to be scaled.
    /// @param _o Origin of scaling.
    virtual void ScaleCfg(double _length, Cfg& _c, const Cfg& _o);

    /// This version uses the default origin.
    /// @overload
    void ScaleCfg(double _length, Cfg& _c);

    /// This version is for group configurations.
    /// @overload
    virtual void ScaleCfg(double _length, GroupCfgType& _c,
        const GroupCfgType& _o);

    /// This version is for group configurations with default origin.
    /// @overload
    void ScaleCfg(double _length, GroupCfgType& _c);

    ///@}

};

#endif
