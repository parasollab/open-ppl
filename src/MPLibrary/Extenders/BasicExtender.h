#ifndef PMPL_BASIC_EXTENDER_H_
#define PMPL_BASIC_EXTENDER_H_

#include "ExtenderMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Basic straight-line extension.
///
/// Extends in straight-line through @cspace until either the target is reached,
/// @cobst is encountered, or the max distance is exceeded.
/// @ingroup Extenders
///
/// @ingroup Extenders
////////////////////////////////////////////////////////////////////////////////
class BasicExtender : virtual public ExtenderMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::GroupCfgType    GroupCfgType;
    typedef typename MPBaseObject::GroupWeightType GroupWeightType;
    typedef typename GroupCfgType::Formation   Formation;

    ///@}
    ///@name Construction
    ///@{

    BasicExtender();

    BasicExtender(XMLNode& _node);

    virtual ~BasicExtender() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name ExtenderMethod Overrides
    ///@{

    virtual bool Extend(const Cfg& _start, const Cfg& _end,
        Cfg& _new, LPOutput& _lp) override;

    virtual bool Extend(const Cfg& _start, const Cfg& _end,
        Cfg& _new, LPOutput& _lp, CDInfo& _cdInfo) override;

    virtual bool Extend(const GroupCfgType& _start, const GroupCfgType& _end,
        GroupCfgType& _new, GroupLPOutput& _lp,
        const Formation& _robotIndexes = Formation()) override;

    virtual bool Extend(const GroupCfgType& _start, const GroupCfgType& _end,
        GroupCfgType& _new, GroupLPOutput& _lp, CDInfo& _cdInfo,
        const Formation& _robotIndexes = Formation()) override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Basic utility for "extend" a RRT tree. Assumed to be given a start node
    /// and a goal node to grow towards. Resulting node extended towards the
    /// goal is passed by reference and modified.
    /// @param _start  Cfg to grow from.
    /// @param _end    Cfg to grow toward.
    /// @param _newCfg Return for newly created cfg.
    /// @param _delta  Maximum distance to grow
    /// @return True if the extension produced a valid configuration that is at
    ///         least the minimum distance away from the starting point.
    bool Expand(const Cfg& _start, const Cfg& _end, Cfg& _newCfg,
        double _delta, LPOutput& _lp, double _posRes, double _oriRes);
    bool Expand(const Cfg& _start, const Cfg& _end, Cfg& _newCfg,
        double _delta, LPOutput& _lp, CDInfo& _cdInfo,
        double _posRes, double _oriRes);


    /// GroupCfg Overrides
    bool Expand(const GroupCfgType& _start, const GroupCfgType& _end,
        GroupCfgType& _newCfg, double _delta, GroupLPOutput& _lp,
        double _posRes, double _oriRes,
        const Formation& _robotIndexes = Formation());
    bool Expand(const GroupCfgType& _start, const GroupCfgType& _end,
        GroupCfgType& _newCfg, double _delta, GroupLPOutput& _lp,
        CDInfo& _cdInfo, double _posRes, double _oriRes,
        const Formation& _robotIndexes = Formation());

    ///@}
    ///@name Internal State
    ///@{

    std::string m_dmLabel;          ///< The distance metric to use.
    std::string m_vcLabel;          ///< The validity checker to use.
    bool m_randomOrientation{true}; ///< Setting this to false fixes orientation.

    ///@}
};

#endif
