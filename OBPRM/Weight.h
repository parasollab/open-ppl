// Weight.h: interface for the Weight class.
//
//////////////////////////////////////////////////////////////////////
/*********************************************************************
 *@file Weight.h
 *@author Jyh-Ming
 *
 *This file constains weights classes and interfaces.
 * - IWeight Interface for weight data and basic functions.
 *   All weight class should derive from this class.
 * - IntWeight Default implementation of IWeight.
 * - DblWeight Another implementation of IWeight.
 * - DefaulWeightFactory Default WeightFactory which generate IntWeight
 *   instnaces. The only method of this class is Create which create 
 *	 IWeight instance for clients. 
 * - WeightObject This class contains a pointer to DefaultWeightFactory
 *   which could be changed to user defined WeightFactory.
 *
 * User who wants have their own customized weight should first have
 * weight class derived from IWeight interface and second have their
 * own WeightFactory class derived from DefaultWeightFactory. Customized
 * WeightFactory then generates customized weight for WeightObject.
 * Next, don't forget to set user's WeightFactory to WeightObject by
 * SetWeightFactory.
 *
 *@date   4/4/01
 *********************************************************************/

#if !defined(_OBPRM_WEIGHT_H_)
#define _OBPRM_WEIGHT_H_

#include <iostream.h>
#include <vector.h>

/**
* This is an interface for all weight classes, like IntWeight and DblWeight.
* Clients who want ot use a new weight for graph should derive form this
* iterface and assign clients' dervied class to Weight instance.
*/
class IWeight {
	
public:
	
	/**
	 *Create an IWeight object which is "equal" to this instance.
	 *This means this->equal will return true if created new IWeight object 
	 *is the input parameter.
	 */
	virtual inline IWeight* clone() const =0;

	/**@name comparison*/
	//@{
		//check if tmp has same values as this instance
		virtual inline bool equal( const IWeight * tmp ) const;
	//@}
	
	/**@name I/O*/
	//@{
		///Read in data
		virtual inline void Input( istream& in );
		///Write out data
		virtual inline void Output( ostream& out ) const;
	//@}
	
	/**@name Access methods*/
	//@{
		virtual inline double& Weight();
		virtual inline int& LP();
	//@}
	
protected:
	int lp;
	double weight;
};

/////////////////////////////////////////////////////////////
//
//	IntWeight : pub IWeight
//
/////////////////////////////////////////////////////////////

/**Represention for edge weights in roadmap graph.
*"normal" edges.
*/
class IntWeight : public IWeight 
{ 
public:
	
	/**@name Constructor and Destructor*/
	//@{
		/**Default contructor.
		*This constructor sets nticks to 1 and other data members to invalid value.
		*/
		IntWeight();
	//@}

	virtual inline IWeight* clone() const;
};

/////////////////////////////////////////////////////////////
//
//	DblWeight : pub IWeight
//
/////////////////////////////////////////////////////////////

/**Represention for edge weights in roadmap graph.
*"special" edges, for example, for protein folding.
*/
class DblWeight  : public IWeight  {
public:
	
	/**@name Constructor and Destructor*/
	//@{
		/**Default contructor.
		*This constructor sets its data members to invalid value.
		*/
		DblWeight();
	//@}

	virtual inline IWeight* clone() const;
};

/////////////////////////////////////////////////////////////
//
//	WeightFactory
//
/////////////////////////////////////////////////////////////
class WeightObject;
class DefaulWeightFactory{

	friend class WeightObject;

protected:
	virtual bool Create( IWeight ** ppIWeight /*in/out*/ ) const;
};

/////////////////////////////////////////////////////////////
//
//	WeightObject
//
/////////////////////////////////////////////////////////////

class WeightObject{

	//these two variable init in util.cpp
	static double WO_MAX_WEIGHT;
	static int    WO_INVALID_LP;

public:

	/////////////////////////////////////////////////////////////
	//
	//	Constructors and Destructor
	//	These functions create default IWeight if on clients
	//	speicifed their own IWeight.
	//
	/////////////////////////////////////////////////////////////
	
	WeightObject();
	WeightObject( int lpID );
	WeightObject( int lpID, double weight );
	WeightObject( const WeightObject & WeightObj );
	WeightObject( IWeight * pIWeight );
	
	virtual ~WeightObject();
	
	/////////////////////////////////////////////////////////////
	//
	//	Graph.h Interface
	//
	/////////////////////////////////////////////////////////////
	
	//A static function returns invalid weight
	static WeightObject InvalidWeight();
	static double & MaxWeight(); ///< For Dijkstra's Alg
	
	///Copy values from antoher given IntWeight instance.
	bool operator== (const WeightObject &tmp) const;
	const WeightObject & operator=(const WeightObject & WeightObj);
	
    ///Read values of datamember to given input stream.
	friend ostream& operator<< (ostream& _os, const WeightObject& w);  // in util.c
	
    ///Read values of datamember to given input stream.
    friend istream& operator>> (istream& _is, WeightObject& w);        // in util.c
	
	/////////////////////////////////////////////////////////////
	//
	//	Access Methods
	//
	/////////////////////////////////////////////////////////////
	
	virtual inline IWeight * GetIWeight();
	virtual inline const IWeight * GetIWeight() const;

	static const DefaulWeightFactory * GetWeightFactory();
	static void SetWeightFactory(DefaulWeightFactory * fact);

	virtual double& Weight(){
		return (m_pIWeight==NULL)?WO_MAX_WEIGHT:m_pIWeight->Weight();
	}

	virtual int& LP(){
		return (m_pIWeight==NULL)?WO_INVALID_LP:m_pIWeight->LP();
	}
	
	/////////////////////////////////////////////////////////////
	//
	//	Private Data
	//
	/////////////////////////////////////////////////////////////
protected:
	static DefaulWeightFactory * m_pFactory; //init in util.cpp
	IWeight * m_pIWeight;
};

#endif // !defined(_OBPRM_WEIGHT_H_)
