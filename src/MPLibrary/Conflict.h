#include "MPLibrary/MPBaseObject.h"
#include "MPTools/SafeIntervalTool.h"
#include "ConfigurationSpace/RoadmapGraph.h"


template <typename MPTraits>
class Conflict  : public MPBaseObject<MPTraits> {

	typedef typename MPTraits::CfgType    CfgType;
	public:
		enum Type{Vertex,Edge};

		Robot* r1;
		Robot* r2;
		size_t id1;
		size_t id2;
		// size_t vid1; ///In conflict occurs on a vertex, vid1 is the VID on robot1's roadmap. 
		// std::pair<size_t,size_t> eid1; ///In conflict occurs on a edge, eid1 contains the edge VIDs on robot1's roadmap. 
		// size_t vid2;
		// std::pair<size_t,size_t> eid2;
		CfgType conflictCfg;
		Type t1;
		double conflictTimestep;
		bool emptyConflict;

		Conflict() : r1(nullptr), r2(nullptr), id1(-1), id2(-1), t1(Type::Vertex), conflictTimestep(-1), emptyConflict(true) {}
};

