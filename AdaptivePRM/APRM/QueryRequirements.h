////////////////////////////////////////////////////
//
//  QueryRequirements.h
//
/////////////////////////////////////////////////////
#ifndef QueryRequirements_h
#define QueryRequirements_h


#include"Query.h"
#include<iostream.h>


////////////////////////////////////////////////////
//
//  IQueryReq
//
////////////////////////////////////////////////////
//interface
class IQueryReq {
public:
  virtual void init(char* filename) = 0;

  virtual bool nodeValid(Cfg& node, Environment* env,
			 CollisionDetection* cd, SID cdsetid) = 0;
  virtual bool edgeValid(vector<Cfg> &cfgs, Environment* env,
			 CollisionDetection* cd, SID cdsetid) = 0;

  virtual void Print(ostream& os) = 0;
};


////////////////////////////////////////////////////
//
//  BasicQueryReq : IQueryReq
//
////////////////////////////////////////////////////
class BasicQueryReq : public IQueryReq {
public:
  BasicQueryReq();

  virtual void init(char* filename);

  virtual bool nodeValid(Cfg& node, Environment* env,
			 CollisionDetection* cd, SID cdsetid);
  virtual bool edgeValid(vector<Cfg> &cfgs, Environment* env,
			 CollisionDetection* cd, SID cdsetid);

  virtual void Print(ostream& os);

protected:
   // flags:
   bool checkResolution;
   bool checkClearance;
   bool checkRotation;
   bool checkTilting;
   bool checkPotential;

   // values:
   double resolution;
   double clearance;
   double rotation;
   double minTilt;
   double maxTilt;
   double potential;
};


////////////////////////////////////////////////////
//
//  DefaultQueryReqFactory
//
////////////////////////////////////////////////////
class QueryRequirementsObject;
class DefaultQueryReqFactory {
  friend class QueryRequirementsObject;

protected:
  virtual bool Create(IQueryReq ** ppIQueryReq) const;
};


////////////////////////////////////////////////////
//
//  QueryRequirementsObject
//
////////////////////////////////////////////////////
class QueryRequirementsObject {
public:
  QueryRequirementsObject();
  QueryRequirementsObject(char* filename);
  ~QueryRequirementsObject() {};

  virtual inline IQueryReq* GetIQueryReq();
  virtual inline const IQueryReq* GetIQueryReq() const;

  static const DefaultQueryReqFactory* GetDefaultQueryReqFactory();
  static void SetQueryReqFactory(DefaultQueryReqFactory* fact);

  bool isNodeValid(Cfg& node, Environment* env, CollisionDetection* cd, SID cdsetid);
  bool isEdgeValid(vector<Cfg>& cfgs, Environment* env, CollisionDetection* cd, SID cdsetid);

  void PrintValues(ostream& os);

protected:
  static DefaultQueryReqFactory* m_pFactory;
  IQueryReq* m_pIQueryReq;
};

#endif
