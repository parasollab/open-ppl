// $Id$

////////////////////////////////////////////////////////////////////////////////////////////
/**@file Query.h
  *
  * This is the main OBPRM class which contains data and methods
  * to manipulate the environment with specified moving bodies
  * (ie, robot(s)) and the corresponding roadmap.
  *
  * This file contains the prototype declarations for the class. 
  * Definitions are in the file "Roadmap.cpp".
  *
  * @date 08/18/98
  * @author Nancy Amato
  */
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef Query_h
#define Query_h

////////////////////////////////////////////////////////////////////////////////////////////
#include "QueryCmds.h"     
#include "Roadmap.h"     
#include "util.h"

class Query {

public:

	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//	Constructors and Destructor
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	/**@name Constructors and Destructor */
	//@{

		/**
		 *Default Constructor. Set Every thing to NULL.
		 *This is useful when client needs to init data member by themself
		 *for example instead of read map from file, client could set roadmap
		 *directly.
		 */
		Query();

		/**
		  *Preferred*, fills input & inits.
		  *Initialize roadmap (#rdmp). 
		  *Read roadmap data from file.
		  *@see Roadmap::InitRoadmap
		  */
		Query
		(Input *, QueryCmds*, CollisionDetection*, DistanceMetric*, LocalPlanners*,ConnectMapNodes*);  

		///Destructor. Free memory.
		~Query();

	//@}

	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//	Helpe Methods
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
    /**@name Help methods*/
	//@{

		/**Default initializations.
		  *Set set_id for #lpsetid, #cdsetid, and #dmsetid from
		  *given ConnectMapNodes instance.
		  *If there is no user specified local planner set id then
		  *SL_R5_AD69 will be used.
		  */
		virtual void initDefaultSetIDs(ConnectMapNodes * cn);

		/**Try to connect given Cfg to given connected component.
		  *This method tries to connect every Cfgs in given 
		  *connected component to this given Cfg.
		  *(Cfgs in cc are sorted before connecting)
		  *
		  *@param _cfg connect this Cfg to connected component.
		  *@param _cc A list of Cfgs in same connected component.
		  *@param _vid if Connection is made then this parameter will
		  *be set as the id of vertex (Cfg) in the roadmap graph. If
		  *no connection is made, this parameter will be set as 
		  *INVALID_VID.
		  *@param _ci Connected path will be put in here.
		  *
		  *@see ConnectMapNodes::SortByDistFromCfg
		  */
		virtual bool CanConnectToCC(Cfg _cfg, 
			                        CollisionDetection *cd,
									ConnectMapNodes *cn, 
									LocalPlanners *lp,
									DistanceMetric *dm,
									vector<Cfg> _cc, 
									SID _lpsid, 
									VID *_vid,
									LPInfo *_ci);

		/**Get path between two given Cfgs by "re"-planning.
		  *This method finds local planner which connect these
		  *Cfgs in connection time and tries to connect them 
		  *again using same planner in query time.
		  *
		  *@param _c1 Start Cfg.
		  *@param _c2 Goal Cfg.
		  *@param _weight This parameter stores information about planner
		  *which connected _c1 and _c2 in connection time.
		  *@param _ci Generated path will be stored in here.
		  *
		  *@note Acturally, this method will try to connect from _c1 to _c2
		  *-AND- from _c2 to c1. If no connection will be made or no correct
		  *planner could be found, query process will be terminated.
		  *@return Ture if path is found. Otherwise false will be returned.
		  *(?! It looks like there is no way to return false ?!)
		  */
		virtual bool GetPathSegment(Cfg _c1, 
			                        Cfg _c2, 
									CollisionDetection *cd,
									LocalPlanners * lp,
									DistanceMetric * dm,
									WEIGHT _weight, 
									LPInfo* _ci);

	//@}

	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//	Query Methods
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
    /**@name Query methods*/
	//@{

		/**Query path(s) for Cfgs in #query.
		  *This method will read Cfg list, #query, and make connection for all Cfgs
		  *inside this list.
		  *Messages will be output to standard ouput to tell client
		  *what current status is.
		  *
		  *@return true if path is found. Otherwise false will be returned.
		  *@note path will be stored in #path.
		  */
		virtual bool PerformQuery
		(CollisionDetection *cd, ConnectMapNodes *cn, LocalPlanners * lp,DistanceMetric * dm);

		/**Query path for two given Cfgs.
		  *Algorithm:
		  *	-# 	For every connected component, cc, in roadmap.
		  *		-# if start and goal could connect to this cc
		  *			-# insert path from start to connection point, cc_start, in cc 
		  *            into return_path.
		  *			-# search path from cc_start to cc_end and insert it to return_path
		  *			-# insert path from connection point, cc_end, in cc 
		  *            to goal into return_path.
		  *			-# expand cc by adding start and goal as new vertices and paths
		  *            (start->cc_start) and (cc_end->goal) as new edges.
          *		-# else
		  *			cc = next connected component in roadmap
		  *		-# end if
		  *	-# end for
		  *	-# return false
		  *
		  *@param _start start configuration of Robot.
		  *@param _goal goal configuration for Robot.
		  *@param _lpsid Which local planer will used to connect start and goal to
		  *connected components.
		  *@path _path Result.
		  *
		  *@return true if path is found. Otherwise false will be returned.
		  *@sa GetPathSegment
		  */
		virtual bool PerformQuery(Cfg _start, 
			                      Cfg _goal, 
								  CollisionDetection*,
								  ConnectMapNodes*, 
								  LocalPlanners*, 
								  DistanceMetric*, 
								  SID _lpsid, 
								  vector<Cfg>* _path);

	//@}

	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//	I/O (Display, Input, Output)
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	/**@name I/O*/
	//@{

		///Read query Cfgs from given file.
		virtual void ReadQuery(const char* _filename);

		///Write path data to #outputPathFile.
		virtual void WritePath();

		///Write path data to given file.
		virtual void WritePath(char* _filename);

	//@}

	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//	Constructors and Destructor
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////

	Roadmap * pRdmp;

	vector<Cfg> query;      ///< start,[intermediate nodes],goal 
	vector<Cfg> path;       ///< output paths

	char *outputPathFile;	///< File name for path data

	SID lpsetid;		///< Local planner set id for query
	SID cdsetid;		///< Collision Detection set id for query
	SID dmsetid;		///< Distance Metrics set id for query
};

#endif
