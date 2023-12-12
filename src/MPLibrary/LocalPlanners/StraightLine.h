#ifndef PMPL_STRAIGHT_LINE_H_
#define PMPL_STRAIGHT_LINE_H_

#include "LocalPlannerMethod.h"
#include "GroupLPOutput.h"
#include "LPOutput.h"


////////////////////////////////////////////////////////////////////////////////
/// Check a straight-line path in c-space for valididty.
///
/// This local planner validates straight line paths which is a direct linear
/// interpolation between two configurations in @cspace.
/// @ingroup LocalPlanners
////////////////////////////////////////////////////////////////////////////////
class StraightLine : virtual public LocalPlannerMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::WeightType       WeightType;
    typedef typename MPBaseObject::GroupCfgType     GroupCfgType;
    typedef typename MPBaseObject::GroupRoadmapType GroupRoadmapType;
    typedef typename GroupCfgType::Formation        Formation;

    ///@}
    ///@name Construction
    ///@{

    StraightLine(const std::string& _vcLabel = "", bool _binary = false,
        bool _saveIntermediates = false);

    StraightLine(XMLNode& _node);

    virtual ~StraightLine() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name LocalPlannerMethod Overrides
    ///@{

    virtual bool IsConnected(
        const Cfg& _c1, const Cfg& _c2, Cfg& _col,
        LPOutput* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false) override;

    virtual bool IsConnected(
        const GroupCfgType& _c1, const GroupCfgType& _c2, GroupCfgType& _col,
        GroupLPOutput* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false,
        const Formation& _robotIndexes = Formation()) override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Default for non closed chains
    bool IsConnectedFunc(
        const Cfg& _c1, const Cfg& _c2, Cfg& _col,
        LPOutput* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    /// Check if two Cfgs could be connected by straight line.
    /// This method implements straight line connection local planner
    /// by checking collision of each Cfg along the line.
    /// If the is any Cfg causes Robot collides with any obstacle,
    /// false will be returned.
    virtual bool IsConnectedSLSequential(
        const Cfg& _c1, const Cfg& _c2, Cfg& _col,
        LPOutput* _lpOutput, int& _cdCounter,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    /// Check if two Cfgs could be connected by straight line
    /// This method uses binary search to check clearances of Cfgs between _c1
    /// and _c2.
    virtual bool IsConnectedSLBinary(
        const Cfg& _c1, const Cfg& _c2, Cfg& _col,
        LPOutput* _lpOutput, int& _cdCounter,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_dmLabel;          ///< The metric for measuring edge length.
    std::string m_vcLabel;          ///< The validity checker.
    bool m_binaryEvaluation{false}; ///< Use binary search?

		double m_selfEdgeSteps{1};

    ///@}

};

#endif
