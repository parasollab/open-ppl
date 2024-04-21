#ifndef MEDIAL_AXIS_LP_H_
#define MEDIAL_AXIS_LP_H_

#include "LocalPlannerMethod.h"
#include "StraightLine.h"
#include "MPLibrary/ValidityCheckers/MedialAxisClearanceValidity.h"

#include <memory>


////////////////////////////////////////////////////////////////////////////////
/// Plan along the medial axis between two medial axis configurations
///
/// This class defines the medial axis local planner which performs a
/// push of the pathway connecting two medial axis configurations along
/// the medial axis. This algorithm provides three methods of medial axis path
/// generation:
///   - recursive - push midpoint to medial axis, and validate
///     epsilon-closeness,
///   - iterative - step along medial axis, and
///   - binary - push midpoint to medial axis upto resolution
/// @ingroup LocalPlanners
////////////////////////////////////////////////////////////////////////////////
class MedialAxisLP : public LocalPlannerMethod {
  public:
    enum class Controller {Recursive, Iterative, Binary};

    typedef typename MPBaseObject::WeightType   WeightType;
    typedef typename MPBaseObject::GroupCfgType GroupCfgType;

    typedef MethodSet<ValidityCheckerMethod> ValidityCheckerSet;
    typedef typename ValidityCheckerSet::MethodPointer ValidityCheckerPointer;

    MedialAxisLP(MedialAxisUtility _medialAxisUtility =
        MedialAxisUtility(),
        double _macEpsilon = 0.01, size_t _maxIter = 2);
    MedialAxisLP(XMLNode& _node);
    virtual ~MedialAxisLP() = default;

    virtual void Print(ostream& _os) const;
    MedialAxisUtility& GetMedialAxisUtility() {
      return m_medialAxisUtility;
    }

    virtual bool IsConnected(
        const Cfg& _c1, const Cfg& _c2, Cfg& _col,
        LPOutput* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = true);

  private:
    void Init();

    bool IsConnectedRec(
        const Cfg& _c1, const Cfg& _c2, Cfg& _col,
        LPOutput* _lpOutput,
        double _posRes, double _oriRes, size_t _itr = 0);

    bool EpsilonClosePath(
        const Cfg& _c1, const Cfg& _c2, Cfg& _mid,
        LPOutput* _lpOutput,
        double _posRes, double _oriRes);

    bool IsConnectedIter(
        const Cfg& _c1, const Cfg& _c2, Cfg& _col,
        LPOutput* _lpOutput,
        double _posRes, double _oriRes);

    bool IsConnectedBin(
        const Cfg& _c1, const Cfg& _c2, Cfg& _col,
        LPOutput* _lpOutput,
        double _posRes, double _oriRes);

    void RemoveBranches(LPOutput* _lpOutput);

    void ReduceNoise(const Cfg& _c1, const Cfg& _c2,
        LPOutput* _lpOutput, double _posRes, double _oriRes);

    Controller m_controller{Controller::Iterative};

    MedialAxisUtility m_medialAxisUtility; //stores operations and
    //variables for medial axis
    double m_macEpsilon; //some epsilon
    size_t m_maxIter; //maximum depth of recursion
    size_t m_maxFailures; //maximum number of failures that can ocure while pushing.
    size_t m_maxProgress; //maximum number of no progress failures that can occure.
    double m_resFactor{1.0}; //factor of the resolution


    MedialAxisClearanceValidity* m_macVC{nullptr};  //mac validity checker
    StraightLine m_envLP, m_macLP;     //straight line local planners

    std::string m_pathModifier;

    bool m_macVCAdded{false};

    string m_dmLabel{""};

    bool m_initialized{false};
};

#endif